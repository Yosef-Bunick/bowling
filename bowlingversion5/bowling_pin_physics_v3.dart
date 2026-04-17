// ============================================================
// Bowling Pin Physics Engine v3 - Regulation Ten-Pin
//
// Grounded improvements in this version:
//   - Exact USBC station diameters are used for the pin body profile.
//   - Center of gravity is set to the USBC target value.
//   - Radius of gyration squared is used for transverse inertia.
//   - The pin's vertical spin inertia is estimated by integrating the
//     regulation shape as a solid of revolution.
//   - Tipping now uses a base-edge pivot model with gravity torque and a
//     critical angle, instead of simply falling because of linear gravity.
//   - Rack spacing uses the exact equilateral-triangle relation.
//
// Notes:
//   - Ball/pin and pin/pin restitution values are still approximations.
//   - This remains a compact gameplay-oriented simulation, not a full rigid
//     body solver.
// ============================================================

import 'dart:math';

// ── Unit helpers ─────────────────────────────────────────────

const double inchesToMeters = 0.0254;
const double poundsToKg = 0.45359237;
const double ouncesToKg = 0.028349523125;

double inch(double value) => value * inchesToMeters;

// ── Regulation constants (USBC ten-pin targets) ─────────────

const double pinMass = 3.0 * poundsToKg + 8.0 * ouncesToKg; // 3 lb 8 oz target
const double pinHeight = 15.0 * inchesToMeters;
const double pinBaseRadius = (2.031 / 2.0) * inchesToMeters;
const double pinMaxRadius = (4.766 / 2.0) * inchesToMeters;
const double pinSpacing = 12.0 * inchesToMeters;
const double pinCogHeight = 5.781 * inchesToMeters; // exact USBC target
const double pinRgSquared = 13.90 * inchesToMeters * inchesToMeters;

// Approximations that remain tunable.
const double restitutionPinPin = 0.62;
const double restitutionBallPin = 0.60;
const double rollingBallPinFriction = 0.10;
const double laneSlideFriction = 0.18;
const double angularDamping = 0.18;
const double linearDamping = 0.10;
const double gravity = 9.81;

// ── Exact regulation profile stations (height above base, diameter) ─────

const List<double> _stationHeightsIn = <double>[
  0.0,
  0.75,
  2.25,
  3.375,
  4.5,
  5.875,
  7.25,
  8.625,
  9.375,
  10.0,
  10.875,
  11.75,
  12.625,
  13.5,
  15.0,
];

const List<double> _stationDiametersIn = <double>[
  2.031,
  2.828,
  3.906,
  4.510,
  4.766,
  4.563,
  3.703,
  2.472,
  1.965,
  1.797,
  1.870,
  2.094,
  2.406,
  2.547,
  0.0,
];

final List<double> _stationHeights =
    _stationHeightsIn.map(inch).toList(growable: false);
final List<double> _stationRadii = _stationDiametersIn
    .map((d) => 0.5 * inch(d))
    .toList(growable: false);

// ── Vector3 ─────────────────────────────────────────────────

class Vec3 {
  double x, y, z;
  Vec3(this.x, this.y, this.z);

  Vec3 operator +(Vec3 o) => Vec3(x + o.x, y + o.y, z + o.z);
  Vec3 operator -(Vec3 o) => Vec3(x - o.x, y - o.y, z - o.z);
  Vec3 operator *(double s) => Vec3(x * s, y * s, z * s);
  Vec3 operator /(double s) => Vec3(x / s, y / s, z / s);

  double dot(Vec3 o) => x * o.x + y * o.y + z * o.z;

  Vec3 cross(Vec3 o) => Vec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get length => sqrt(x * x + y * y + z * z);

  Vec3 get normalized {
    final l = length;
    return l > 1e-10 ? this / l : Vec3(0, 0, 0);
  }

  Vec3 clone() => Vec3(x, y, z);

  @override
  String toString() =>
      'Vec3(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}

// ── Profile & inertia helpers ───────────────────────────────

double pinRadiusAtLocalHeight(double localY) {
  final y = localY.clamp(0.0, pinHeight);

  for (int i = 0; i < _stationHeights.length - 1; i++) {
    final y0 = _stationHeights[i];
    final y1 = _stationHeights[i + 1];
    if (y >= y0 && y <= y1) {
      final t = (y - y0) / max(1e-9, y1 - y0);
      return _stationRadii[i] + (_stationRadii[i + 1] - _stationRadii[i]) * t;
    }
  }

  return _stationRadii.last;
}

class InertiaTensor {
  final double ixx; // transverse about x through CoG
  final double iyy; // spin about vertical y through CoG
  final double izz; // transverse about z through CoG

  const InertiaTensor(this.ixx, this.iyy, this.izz);

  double get transverse => ixx;

  static InertiaTensor forPin() {
    final iTransverse = pinMass * pinRgSquared;
    final iSpin = _estimatePinSpinInertia();
    return InertiaTensor(iTransverse, iSpin, iTransverse);
  }
}

double _estimatePinSpinInertia() {
  // Numerical integration as stacked thin disks with uniform density.
  const int slices = 240;
  final dy = pinHeight / slices;

  double volume = 0.0;
  double polarSecondMoment = 0.0;

  for (int i = 0; i < slices; i++) {
    final y = (i + 0.5) * dy;
    final r = pinRadiusAtLocalHeight(y);
    final dV = pi * r * r * dy;
    volume += dV;
    polarSecondMoment += 0.5 * dV * r * r;
  }

  final density = pinMass / volume;
  return density * polarSecondMoment;
}

const double criticalTipAngle = 0.0; // initialized below via getter pattern not const

double get pinCriticalTipAngle => atan(pinBaseRadius / pinCogHeight);

double get pinPivotInertia =>
    InertiaTensor.forPin().transverse +
    pinMass * (pinCogHeight * pinCogHeight + pinBaseRadius * pinBaseRadius);

// ── Pin ──────────────────────────────────────────────────────

enum PinStatus { standing, falling, down }

class BowlingPin {
  final int id;
  Vec3 position; // base-center point on lane
  Vec3 velocity;
  Vec3 tiltAxis;
  double tiltAngle; // radians
  double tiltRate; // radians/sec about tiltAxis
  PinStatus status;
  final InertiaTensor inertia;

  BowlingPin({required this.id, required this.position})
      : velocity = Vec3(0, 0, 0),
        tiltAxis = Vec3(1, 0, 0),
        tiltAngle = 0.0,
        tiltRate = 0.0,
        status = PinStatus.standing,
        inertia = InertiaTensor.forPin();

  bool get isActive => status != PinStatus.down;

  Vec3 get cogPosition => Vec3(position.x, pinCogHeight, position.z);

  Vec3 get angularVelocity => tiltAxis * tiltRate;

  Vec3 closestPointOnAxis(Vec3 point) {
    final t = ((point.y) / pinHeight).clamp(0.0, 1.0);
    return Vec3(position.x, t * pinHeight, position.z);
  }

  double radiusAtHeight(double worldY) {
    return pinRadiusAtLocalHeight(worldY.clamp(0.0, pinHeight));
  }

  void addTiltImpulse(Vec3 axis, double angularImpulse) {
    final newAxis = axis.normalized;
    if (newAxis.length < 1e-9) return;

    if (tiltRate.abs() < 1e-9) {
      tiltAxis = newAxis;
    } else {
      tiltAxis = (tiltAxis * tiltRate.abs() + newAxis * angularImpulse.abs()).normalized;
    }

    tiltRate += angularImpulse / inertia.transverse;

    if (status == PinStatus.standing && tiltRate.abs() > 0.05) {
      status = PinStatus.falling;
    }
  }
}

// ── Ball ─────────────────────────────────────────────────────

class BowlingBall {
  Vec3 position;
  Vec3 velocity;
  Vec3 angularVelocity;
  final double mass;
  final double radius;

  BowlingBall({
    required this.position,
    required this.velocity,
    required this.angularVelocity,
    required this.mass,
    required this.radius,
  });
}

// ── Pin Deck ─────────────────────────────────────────────────

List<BowlingPin> buildPinDeck() {
  const s = pinSpacing;
  final rowOffset = s * sqrt(3) / 2.0;

  final layout = <List<double>>[
    [1, 0.0, 0.0],
    [2, -s / 2.0, rowOffset],
    [3, s / 2.0, rowOffset],
    [4, -s, 2.0 * rowOffset],
    [5, 0.0, 2.0 * rowOffset],
    [6, s, 2.0 * rowOffset],
    [7, -1.5 * s, 3.0 * rowOffset],
    [8, -0.5 * s, 3.0 * rowOffset],
    [9, 0.5 * s, 3.0 * rowOffset],
    [10, 1.5 * s, 3.0 * rowOffset],
  ];

  return layout
      .map(
        (e) => BowlingPin(
          id: e[0].toInt(),
          position: Vec3(e[1], 0.0, e[2]),
        ),
      )
      .toList();
}

// ── Collision Detection ──────────────────────────────────────

({Vec3 contactPoint, Vec3 normal, double hitHeight})? checkBallPinContact({
  required BowlingBall ball,
  required BowlingPin pin,
}) {
  if (!pin.isActive) return null;
  if (pin.tiltAngle > radians(75.0)) return null;

  final axisPoint = pin.closestPointOnAxis(ball.position);
  final dx = ball.position.x - axisPoint.x;
  final dz = ball.position.z - axisPoint.z;
  final horizontalDist = sqrt(dx * dx + dz * dz);
  if (horizontalDist < 1e-6) return null;

  final hitHeight = axisPoint.y;
  final pinR = pin.radiusAtHeight(hitHeight);
  final combinedR = ball.radius + pinR;
  if (horizontalDist >= combinedR) return null;

  final normal = Vec3(dx / horizontalDist, 0.0, dz / horizontalDist);
  final contactPoint = Vec3(
    axisPoint.x + normal.x * pinR,
    hitHeight,
    axisPoint.z + normal.z * pinR,
  );

  return (contactPoint: contactPoint, normal: normal, hitHeight: hitHeight);
}

bool pinPinCollision(BowlingPin a, BowlingPin b) {
  if (!a.isActive || !b.isActive) return false;

  final dx = a.position.x - b.position.x;
  final dz = a.position.z - b.position.z;
  final dist = sqrt(dx * dx + dz * dz);
  final contactDistance = 2.0 * pinRadiusAtLocalHeight(pinCogHeight);
  return dist < contactDistance;
}

// ── Impulse Resolution ───────────────────────────────────────

double radians(double degrees) => degrees * pi / 180.0;
double degrees(double radiansValue) => radiansValue * 180.0 / pi;

void resolveBallPinCollision({
  required BowlingBall ball,
  required BowlingPin pin,
  required Vec3 contactPoint,
  required Vec3 collisionNormal,
  required double hitHeight,
}) {
  final ballSurfaceVel =
      ball.velocity + ball.angularVelocity.cross(collisionNormal * -ball.radius);
  final relVel = ballSurfaceVel - pin.velocity;
  final velAlongNormal = relVel.dot(collisionNormal);

  if (velAlongNormal >= 0.0) return;

  final rPin = contactPoint - pin.cogPosition;
  final tipAxis = Vec3(-collisionNormal.z, 0.0, collisionNormal.x).normalized;
  final rCrossN = rPin.cross(collisionNormal);

  final rotationalTerm = pow(rCrossN.dot(tipAxis), 2) / pin.inertia.transverse;
  final effectiveMass = 1.0 / ball.mass + 1.0 / pinMass + rotationalTerm;

  final j = -(1.0 + restitutionBallPin) * velAlongNormal / max(1e-9, effectiveMass);
  final impulse = collisionNormal * j;

  ball.velocity = ball.velocity + impulse / ball.mass;
  pin.velocity = pin.velocity - impulse / pinMass;

  // Tangential friction exchange to give more realistic deflection and tilt.
  final tangent = (relVel - collisionNormal * velAlongNormal).normalized;
  if (tangent.length > 1e-9) {
    final jt = j * rollingBallPinFriction;
    final frictionImpulse = tangent * -jt;
    ball.velocity = ball.velocity + frictionImpulse / ball.mass;
    pin.velocity = pin.velocity - frictionImpulse / pinMass;
  }

  final angularImpulse = rPin.cross(impulse * -1.0).dot(tipAxis);
  pin.addTiltImpulse(tipAxis, angularImpulse);

  // A lower hit tends to slide more, while a belly hit tips more efficiently.
  final normalizedHeight = (hitHeight / pinHeight).clamp(0.0, 1.0);
  final tipBias = 0.55 + 0.45 * sin(normalizedHeight * pi);
  pin.tiltRate *= tipBias;
}

void applyPinPinImpulse(BowlingPin a, BowlingPin b) {
  final normal = (b.cogPosition - a.cogPosition).normalized;
  if (normal.length < 1e-9) return;

  final relVel = a.velocity - b.velocity;
  final velAlongNormal = relVel.dot(normal);
  if (velAlongNormal <= 0.0) return;

  final j = -(1.0 + restitutionPinPin) * velAlongNormal / (2.0 / pinMass);
  final impulse = normal * j;

  a.velocity = a.velocity - impulse / pinMass;
  b.velocity = b.velocity + impulse / pinMass;

  final tipAxis = Vec3(-normal.z, 0.0, normal.x).normalized;
  a.addTiltImpulse(tipAxis, 0.5 * j * pinCogHeight);
  b.addTiltImpulse(tipAxis, 0.8 * j * pinCogHeight);

  if (b.status == PinStatus.standing) {
    b.status = PinStatus.falling;
  }
}

// ── Simulation Step ──────────────────────────────────────────

void _advancePin(BowlingPin pin, double dt) {
  if (pin.status == PinStatus.down) return;

  // Base-edge pivot model. The gravitational torque changes sign naturally
  // at the critical tip angle where the CoG passes beyond the support base.
  final lever = pinCogHeight * sin(pin.tiltAngle) - pinBaseRadius * cos(pin.tiltAngle);
  final angularAccel = (pinMass * gravity * lever) / pinPivotInertia;

  pin.tiltRate += angularAccel * dt;
  pin.tiltRate *= (1.0 - angularDamping * dt).clamp(0.0, 1.0);
  pin.tiltAngle += pin.tiltRate * dt;

  if (pin.tiltAngle < 0.0) {
    pin.tiltAngle = 0.0;
    pin.tiltRate = 0.0;
    if (pin.velocity.length < 0.02) {
      pin.status = PinStatus.standing;
    }
  }

  // Sliding is limited until the pin tips past the support limit.
  final friction = pin.tiltAngle < pinCriticalTipAngle ? laneSlideFriction * 2.0 : laneSlideFriction;
  final speed = sqrt(pin.velocity.x * pin.velocity.x + pin.velocity.z * pin.velocity.z);
  if (speed > 1e-5) {
    final decel = min(friction * gravity * dt, speed);
    final dir = Vec3(pin.velocity.x, 0.0, pin.velocity.z).normalized;
    pin.velocity = Vec3(
      pin.velocity.x - dir.x * decel,
      0.0,
      pin.velocity.z - dir.z * decel,
    );
  }

  pin.velocity = pin.velocity * (1.0 - linearDamping * dt).clamp(0.0, 1.0);
  pin.position = pin.position + Vec3(pin.velocity.x * dt, 0.0, pin.velocity.z * dt);

  if (pin.tiltAngle >= radians(90.0)) {
    pin.tiltAngle = radians(90.0);
    pin.tiltRate = 0.0;
    pin.status = PinStatus.down;
  } else if (pin.tiltRate.abs() > 0.02 || pin.tiltAngle > radians(1.0)) {
    pin.status = PinStatus.falling;
  } else {
    pin.status = PinStatus.standing;
  }
}

void simulationStep({
  required List<BowlingPin> pins,
  required BowlingBall ball,
  required double dt,
}) {
  for (final pin in pins) {
    final contact = checkBallPinContact(ball: ball, pin: pin);
    if (contact != null) {
      resolveBallPinCollision(
        ball: ball,
        pin: pin,
        contactPoint: contact.contactPoint,
        collisionNormal: contact.normal,
        hitHeight: contact.hitHeight,
      );
    }
  }

  ball.position = ball.position + ball.velocity * dt;

  for (final pin in pins) {
    _advancePin(pin, dt);
  }

  for (int i = 0; i < pins.length; i++) {
    for (int j = i + 1; j < pins.length; j++) {
      if (pins[i].status != PinStatus.down && pinPinCollision(pins[i], pins[j])) {
        applyPinPinImpulse(pins[i], pins[j]);
      }
    }
  }
}

// ── Score Helpers ────────────────────────────────────────────

int countKnockedPins(List<BowlingPin> pins) =>
    pins.where((p) => p.status == PinStatus.down).length;

bool isStrike(List<BowlingPin> pins) => countKnockedPins(pins) == 10;

// ── Test ─────────────────────────────────────────────────────

void main() {
  final pins = buildPinDeck();

  final ball = BowlingBall(
    position: Vec3(0.05, 0.109, -0.20),
    velocity: Vec3(-1.2, 0.0, -8.5),
    angularVelocity: Vec3(0.0, 15.0, -5.0),
    mass: 6.80,
    radius: 0.1092,
  );

  print('=== Bowling Pin Physics v3 ===');
  print('Pin mass target     : ${pinMass.toStringAsFixed(4)} kg');
  print('Pin CoG height      : ${pinCogHeight.toStringAsFixed(4)} m');
  print('Pin critical tip    : ${degrees(pinCriticalTipAngle).toStringAsFixed(2)}°');
  print('Pin transverse I    : ${InertiaTensor.forPin().transverse.toStringAsExponential(4)}');
  print('Pin spin I          : ${InertiaTensor.forPin().iyy.toStringAsExponential(4)}');
  print('');

  const dt = 1.0 / 240.0;
  //const steps = (3.0 / dt).toInt();
  final steps = (3.0 / dt).toInt();

  for (int i = 0; i < steps; i++) {
    simulationStep(pins: pins, ball: ball, dt: dt);
  }

  print('--- Pin Results ---');
  for (final p in pins) {
    print('  Pin ${p.id.toString().padLeft(2)}: '
        '${p.status.name.padRight(8)} '
        'tilt=${degrees(p.tiltAngle).toStringAsFixed(1).padLeft(5)}° '
        'rate=${degrees(p.tiltRate).toStringAsFixed(1).padLeft(6)}°/s '
        'pos=(${p.position.x.toStringAsFixed(2)}, ${p.position.z.toStringAsFixed(2)})');
  }

  print('');
  print('Knocked : ${countKnockedPins(pins)}/10');
  print('Strike  : ${isStrike(pins)}');
  print('Ball exit velocity: ${ball.velocity}');
}
