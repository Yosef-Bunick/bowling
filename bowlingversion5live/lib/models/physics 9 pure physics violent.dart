import 'dart:math';

// ═══════════════════════════════════════════════════════════
// BOWLING PHYSICS ENGINE — EULER RIGID BODY MODEL (STAGE 2)
//
// This version moves the simulation closer to the continuous model:
// - state uses translational velocity v = [vx, vy]
// - spin uses body angular velocity ω = [ωx, ωy, ωz]
// - slip is v - (ω × r_contact)
// - friction uses μ(H) directly
// - torques use τ = [-R Fy, R Fx, 0]
// - rotation uses Euler rigid-body equations
// - roll / hook / skid are diagnostics only
//
// Public API is intentionally kept compatible with the current Flutter UI.
// ═══════════════════════════════════════════════════════════

const double G_MS2 = 9.81;
const double BALL_R_M = 0.358 * 0.3048;
const double BOARD_M = (1.0625 / 12.0) * 0.3048;
const int BOARDS = 39;
const int LANE_FT = 65;
const double OIL_RES = 0.25;
const int OIL_COLS = 260;
const double MU_OIL_MIN = 0.018;
const double MU_ROLL_RES = 0.0035;
const double RG_REF_IN = 2.53;
const double DIFF_REF_IN = 0.040;
const double ASY_THRESH = 0.013;

// Diagnostic phase thresholds.
const double ROLL_SLIP_EPS = 0.02;
const double HOOK_LATERAL_SLIP_EPS = 0.02;
const double HIGH_HERSEY_THRESH = 0.06;

// Contact / lubrication proxies.
const double CONTACT_AREA_M2 = 5.0e-4;
const double ETA_DRY_PAS = 0.004;
const double H_SAT = 0.60;
const double OIL_DEPLETION_K = 0.08;
const double OIL_DENSITY_PROXY = 1.0;

// Lane carrydown proxy retained from prior versions.
const double OIL_DEPOSIT_RATE = 0.010;
const double OIL_CARRY_BLEND = 0.25;

const Map<String, double> GRIT_DRY_MU = {
  '500': 0.30,
  '1000': 0.23,
  '2000': 0.20,
  '3000': 0.15,
  '4000': 0.11,
  'polish': 0.09,
};

class OilType {
  final String name;
  final double viscosityPaS;

  const OilType({
    required this.name,
    required this.viscosityPaS,
  });
}

const Map<String, OilType> OIL_LIBRARY = {
  'glide': OilType(name: 'Glide', viscosityPaS: 0.0389),
  'terrain': OilType(name: 'Terrain', viscosityPaS: 0.0810),
  'curve': OilType(name: 'Curve', viscosityPaS: 0.0550),
  'current': OilType(name: 'Current', viscosityPaS: 0.0520),
  'condition_red': OilType(name: 'Condition Red', viscosityPaS: 0.0490),
  'defense_s': OilType(name: 'Defense-S', viscosityPaS: 0.0473),
  'fire': OilType(name: 'Fire', viscosityPaS: 0.0451),
  'ice': OilType(name: 'Ice', viscosityPaS: 0.0409),
  'condition_blue': OilType(name: 'Condition Blue', viscosityPaS: 0.0394),
  'infinity': OilType(name: 'Infinity', viscosityPaS: 0.0365),
  'navigate': OilType(name: 'Navigate', viscosityPaS: 0.0313),
  'prodigy': OilType(name: 'Prodigy', viscosityPaS: 0.0312),
  'clear_super_100': OilType(name: 'Clear Super 100', viscosityPaS: 0.1010),
  'clear_super_50': OilType(name: 'Clear Super 50', viscosityPaS: 0.0500),
  'clear_801_hv': OilType(name: 'Clear #801 HV', viscosityPaS: 0.0199),
  'clear_811_lv': OilType(name: 'Clear #811 LV', viscosityPaS: 0.0152),
  'ceo_65': OilType(name: 'CEO 65', viscosityPaS: 0.0630),
  'ceo_42': OilType(name: 'CEO 42', viscosityPaS: 0.0420),
  'se_28': OilType(name: 'SE 28', viscosityPaS: 0.0250),
};

const OilType DEFAULT_OIL = OilType(name: 'Default', viscosityPaS: 0.0450);

class OilSample {
  final double totalOil;
  final double effectiveViscosity;

  const OilSample({
    required this.totalOil,
    required this.effectiveViscosity,
  });
}

// Stribeck-on-H^ friction model:
// μ(H^) = μ_min + A/(H^ + B) + C(1 - e^(-D·H^))
const double STRIBECK_A = 0.0035;
const double STRIBECK_B = 0.060;
const double STRIBECK_C = 0.018;
const double STRIBECK_D = 10.0;
const double H_REF = 1.5e-6;

class BallSpecs {
  double rg;
  double diff;
  double asy;
  String grit;
  double masslb;

  BallSpecs({
    this.rg = 2.54,
    this.diff = 0.040,
    this.asy = 0.0,
    this.grit = '2000',
    this.masslb = 15.0,
  });

  bool get isAsym => asy >= ASY_THRESH;
  double get massKg => masslb * 0.453592;
  double get rgM => (rg / 12.0) * 0.3048;
  double get moiKgM2 => massKg * rgM * rgM;
  double get muDry => GRIT_DRY_MU[grit] ?? 0.20;
  double get rgFactor => RG_REF_IN / rg;
  double get diffFactor => diff / DIFF_REF_IN;
  double get asymFactor =>
      isAsym ? 1.0 + 0.25 * ((asy - ASY_THRESH) / 0.017).clamp(0.0, 1.0) : 1.0;

  // Approximate principal moments from RG / diff / asym.
  // We only have scalar ball metadata, so these remain proxies rather than
  // directly measured CAD-derived inertias.
  InertiaTensor get inertiaTensor {
    final double iMean = moiKgM2;
    final double diffFrac = (diff / 0.060).clamp(0.0, 1.0) * 0.12;
    final double asyFrac = (asy / 0.030).clamp(0.0, 1.0) * 0.06;

    double ix = iMean * (1.0 - 0.5 * diffFrac - 0.5 * asyFrac);
    double iy = iMean * (1.0 + 0.5 * diffFrac + 0.5 * asyFrac);
    double iz = iMean * (1.0 - 0.5 * diffFrac + 0.5 * asyFrac);

    final double targetMean = (ix + iy + iz) / 3.0;
    if (targetMean > 1e-12) {
      final double s = iMean / targetMean;
      ix *= s;
      iy *= s;
      iz *= s;
    }

    return InertiaTensor(ix: ix, iy: iy, iz: iz);
  }
}

class InertiaTensor {
  final double ix;
  final double iy;
  final double iz;

  const InertiaTensor({
    required this.ix,
    required this.iy,
    required this.iz,
  });
}

class BowlerInputs {
  double speedMph;
  double revRPM;
  double angleDeg;
  double axisTilt;
  double axisRotation;
  double hookK0;
  double releaseBoard;
  double landingDistanceFt;
  double handedness;

  BowlerInputs({
    this.speedMph = 18.0,
    this.revRPM = 300.0,
    this.angleDeg = 0.0,
    this.axisTilt = 15.0,
    this.axisRotation = 40.0,
    this.hookK0 = 1.0,
    this.releaseBoard = 25.0,
    this.landingDistanceFt = 1.0,
    this.handedness = 1.0,
  });

  double get effectiveAngleDeg => angleDeg;
  double get effectiveLandBoard => releaseBoard;
  double get effectiveLandingDistanceFt => landingDistanceFt;
}

class LoadRow {
  final int sl, sr, loads, mics, buff;
  final double d0, d1, toil;
  final String oilType;

  const LoadRow({
    required this.sl,
    required this.sr,
    required this.loads,
    required this.mics,
    required this.buff,
    required this.d0,
    required this.d1,
    required this.toil,
    this.oilType = 'fire',
  });

  LoadRow copyWith({
    int? sl,
    int? sr,
    int? loads,
    int? mics,
    int? buff,
    double? d0,
    double? d1,
    double? toil,
    String? oilType,
  }) =>
      LoadRow(
        sl: sl ?? this.sl,
        sr: sr ?? this.sr,
        loads: loads ?? this.loads,
        mics: mics ?? this.mics,
        buff: buff ?? this.buff,
        d0: d0 ?? this.d0,
        d1: d1 ?? this.d1,
        toil: toil ?? this.toil,
        oilType: oilType ?? this.oilType,
      );

  OilType get oil => OIL_LIBRARY[oilType] ?? DEFAULT_OIL;
}

class PatternData {
  String name;
  double distance;
  List<LoadRow> fwdRows;
  List<LoadRow> revRows;
  String fwdOilType;
  String revOilType;

  PatternData({
    required this.name,
    required this.distance,
    required this.fwdRows,
    required this.revRows,
    this.fwdOilType = 'fire',
    this.revOilType = 'fire',
  });

  OilType get fwdOil => OIL_LIBRARY[fwdOilType] ?? DEFAULT_OIL;
  OilType get revOil => OIL_LIBRARY[revOilType] ?? DEFAULT_OIL;

  static PatternData masters2026() => PatternData(
        name: '2026 USBC Masters',
        distance: 41,
        fwdOilType: 'glide',
        revOilType: 'ice',
        fwdRows: [
          LoadRow(sl: 2, sr: 2, loads: 5, mics: 50, buff: 500, d0: 0, d1: 8, toil: 9250, oilType: 'glide'),
          LoadRow(sl: 3, sr: 4, loads: 1, mics: 45, buff: 500, d0: 8, d1: 10, toil: 1530, oilType: 'ice'),
          LoadRow(sl: 5, sr: 5, loads: 2, mics: 45, buff: 500, d0: 10, d1: 14, toil: 2790, oilType: 'ice'),
          LoadRow(sl: 6, sr: 6, loads: 3, mics: 50, buff: 500, d0: 14, d1: 20, toil: 4350, oilType: 'glide'),
          LoadRow(sl: 2, sr: 2, loads: 1, mics: 50, buff: 500, d0: 20, d1: 22, toil: 1850, oilType: 'glide'),
          LoadRow(sl: 7, sr: 7, loads: 2, mics: 50, buff: 500, d0: 22, d1: 27, toil: 2700, oilType: 'ice'),
          LoadRow(sl: 2, sr: 2, loads: 0, mics: 50, buff: 500, d0: 27, d1: 32, toil: 0, oilType: 'ice'),
          LoadRow(sl: 2, sr: 2, loads: 0, mics: 50, buff: 350, d0: 32, d1: 38, toil: 0, oilType: 'ice'),
          LoadRow(sl: 2, sr: 2, loads: 0, mics: 50, buff: 150, d0: 38, d1: 41, toil: 0, oilType: 'ice'),
        ],
        revRows: [
          LoadRow(sl: 2, sr: 2, loads: 0, mics: 50, buff: 500, d0: 39, d1: 28, toil: 0, oilType: 'ice'),
          LoadRow(sl: 11, sr: 11, loads: 2, mics: 50, buff: 500, d0: 28, d1: 23, toil: 1900, oilType: 'ice'),
          LoadRow(sl: 9, sr: 9, loads: 2, mics: 50, buff: 500, d0: 23, d1: 18, toil: 2300, oilType: 'ice'),
          LoadRow(sl: 7, sr: 7, loads: 2, mics: 50, buff: 500, d0: 18, d1: 14, toil: 2700, oilType: 'ice'),
          LoadRow(sl: 6, sr: 6, loads: 1, mics: 50, buff: 500, d0: 14, d1: 12, toil: 1450, oilType: 'ice'),
          LoadRow(sl: 4, sr: 5, loads: 1, mics: 45, buff: 500, d0: 12, d1: 10, toil: 1440, oilType: 'ice'),
          LoadRow(sl: 2, sr: 2, loads: 2, mics: 45, buff: 500, d0: 10, d1: 6, toil: 3330, oilType: 'glide'),
          LoadRow(sl: 2, sr: 2, loads: 0, mics: 50, buff: 500, d0: 6, d1: 0, toil: 0, oilType: 'ice'),
        ],
      );
}

class PathPoint {
  final double ft, board, vx, omega, theta, oil, mu, AR;
  final double slipRatio, tractionX;
  final String phase;
  final bool atBreakpoint;

  const PathPoint({
    required this.ft,
    required this.board,
    required this.vx,
    required this.omega,
    required this.theta,
    required this.oil,
    required this.mu,
    required this.AR,
    required this.slipRatio,
    required this.tractionX,
    required this.phase,
    required this.atBreakpoint,
  });
}

class SimResult {
  final List<PathPoint> path;
  final double pinSpeed, pinRPM, pinBoard, pinAR;

  const SimResult({
    required this.path,
    required this.pinSpeed,
    required this.pinRPM,
    required this.pinBoard,
    required this.pinAR,
  });
}

const double HEAD_PIN_FT = 60.0;

PathPoint? pointNearestFt(List<PathPoint> path, double targetFt) {
  if (path.isEmpty) return null;
  PathPoint best = path.first;
  double bestErr = (best.ft - targetFt).abs();
  for (final p in path.skip(1)) {
    final double err = (p.ft - targetFt).abs();
    if (err < bestErr) {
      best = p;
      bestErr = err;
    }
  }
  return best;
}

class OilMatrix {
  final List<List<double>> forwardOil;
  final List<List<double>> reverseOil;
  final double fwdViscosity;
  final double revViscosity;

  OilMatrix({
    required this.forwardOil,
    required this.reverseOil,
    required this.fwdViscosity,
    required this.revViscosity,
  });

  OilSample sampleAt(double board, double ft) {
    final double fwd = _interpolate(forwardOil, board, ft);
    final double rev = _interpolate(reverseOil, board, ft);
    final double total = fwd + rev;
    final double eta = total > 1e-9
        ? (fwd * fwdViscosity + rev * revViscosity) / total
        : 0.0;

    return OilSample(
      totalOil: total.clamp(0.0, 1.0),
      effectiveViscosity: eta,
    );
  }

  double _interpolate(List<List<double>> oil, double board, double ft) {
    if (oil.isEmpty) return 0.0;
    final int cols = oil[0].length;
    final double res = cols > LANE_FT ? OIL_RES : 1.0;
    final double bIdx = (board - 1.0).clamp(0.0, (BOARDS - 1).toDouble());
    final double fIdx = (ft / res).clamp(0.0, (cols - 1).toDouble());
    final int b0 = bIdx.floor().clamp(0, BOARDS - 1);
    final int b1 = (b0 + 1).clamp(0, BOARDS - 1);
    final int f0 = fIdx.floor().clamp(0, cols - 1);
    final int f1 = (f0 + 1).clamp(0, cols - 1);
    final double tb = bIdx - b0;
    final double tf = fIdx - f0;
    final double v00 = oil[b0][f0];
    final double v10 = b1 < oil.length ? oil[b1][f0] : v00;
    final double v01 = oil[b0][f1];
    final double v11 = b1 < oil.length ? oil[b1][f1] : v01;
    return ((v00 * (1 - tb) + v10 * tb) * (1 - tf) +
            (v01 * (1 - tb) + v11 * tb) * tf)
        .clamp(0.0, 1.0);
  }

  void pickupAt(double board, double ft, double amount) {
    if (amount <= 0) return;
    final int b = (board - 1.0).round().clamp(0, BOARDS - 1);
    final int f = (ft / OIL_RES).round().clamp(0, OIL_COLS - 1);
    final double fwd = forwardOil[b][f];
    final double rev = reverseOil[b][f];
    final double total = fwd + rev;
    if (total > 1e-9) {
      forwardOil[b][f] = (fwd - amount * fwd / total).clamp(0.0, 1.0);
      reverseOil[b][f] = (rev - amount * rev / total).clamp(0.0, 1.0);
    }
  }

  void depositAt(double board, double ft, double amount) {
    if (amount <= 0) return;
    final int b = (board - 1.0).round().clamp(0, BOARDS - 1);
    final int f = (ft / OIL_RES).round().clamp(0, OIL_COLS - 1);
    forwardOil[b][f] = (forwardOil[b][f] + amount).clamp(0.0, 1.0);
  }

  List<List<double>> get combined => List.generate(
        BOARDS,
        (b) => List.generate(
          OIL_COLS,
          (c) => (forwardOil[b][c] + reverseOil[b][c]).clamp(0.0, 1.0),
        ),
      );
}

OilMatrix buildOilMatrix(
  List<LoadRow> fwdRows,
  List<LoadRow> revRows,
  OilType fwdOilType,
  OilType revOilType, {
  double alphaF = 0.25,
  double rF = 0.91,
  double alphaR = 0.32,
  double rR = 0.94,
  double k = 0.85,
  double beta = 0.35,
  double gamma = 0.12,
  double eta = 0.45,
  double lambda = 0.75,
}) {
  final fwdRaw = List.generate(BOARDS, (_) => List<double>.filled(OIL_COLS, 0.0));
  final revRaw = List.generate(BOARDS, (_) => List<double>.filled(OIL_COLS, 0.0));

  double applyForwardRows(List<LoadRow> rows, List<List<double>> target) {
    double prevSaturation = 0.0;
    for (final row in rows) {
      if (row.toil == 0 || row.loads == 0) continue;
      final int bStart = row.sl;
      final int bEnd = BOARDS - 1 - row.sr;
      if (bStart > bEnd) continue;

      final double x0 = min(row.d0, row.d1);
      final double x1 = max(row.d0, row.d1);
      final double buffFt = row.buff / 12.0;
      final int numBoards = bEnd - bStart + 1;
      final double loadZoneLen = x1 - x0;
      if (numBoards <= 0 || loadZoneLen <= 0) continue;

      final double saturation = beta * prevSaturation;
      final double baseAmount = row.toil / (numBoards * loadZoneLen);
      final double endSaturation = eta * saturation + lambda * baseAmount;

      for (int b = bStart; b <= bEnd; b++) {
        for (int col = 0; col < OIL_COLS; col++) {
          final double x = col * OIL_RES;
          double contribution = 0.0;
          if (x < x0) {
            contribution = 0.0;
          } else if (x <= x1) {
            contribution = baseAmount + gamma * saturation;
          } else {
            final double d = x - x1;
            if (d <= buffFt) {
              contribution = endSaturation * alphaF * pow(rF, d);
            }
          }
          target[b][col] += contribution;
        }
      }
      prevSaturation = endSaturation;
    }
    return prevSaturation;
  }

  void applyReverseRows(
    List<LoadRow> rows,
    List<List<double>> target,
    double forwardEndSaturation,
  ) {
    double prevSaturation = k * forwardEndSaturation;
    for (final row in rows) {
      if (row.toil == 0 || row.loads == 0) continue;
      final int bStart = row.sl;
      final int bEnd = BOARDS - 1 - row.sr;
      if (bStart > bEnd) continue;

      final double xDrop = max(row.d0, row.d1);
      final double xStop = min(row.d0, row.d1);
      final double buffFt = row.buff / 12.0;
      final int numBoards = bEnd - bStart + 1;
      final double loadZoneLen = xDrop - xStop;
      if (numBoards <= 0 || loadZoneLen <= 0) continue;

      final double saturation = beta * prevSaturation;
      final double baseAmount = row.toil / (numBoards * loadZoneLen);
      final double endSaturation = eta * saturation + lambda * baseAmount;

      for (int b = bStart; b <= bEnd; b++) {
        for (int col = 0; col < OIL_COLS; col++) {
          final double x = col * OIL_RES;
          double contribution = 0.0;
          if (x > xDrop) {
            contribution = 0.0;
          } else if (x >= xStop) {
            contribution = baseAmount + gamma * saturation;
          } else {
            final double d = xStop - x;
            if (d <= buffFt && d >= 0) {
              contribution = endSaturation * alphaR * pow(rR, d);
            }
          }
          target[b][col] += contribution;
        }
      }
      prevSaturation = endSaturation;
    }
  }

  final double forwardEndSat = applyForwardRows(fwdRows, fwdRaw);
  applyReverseRows(revRows, revRaw, forwardEndSat);

  double maxV = 0.0;
  for (int b = 0; b < BOARDS; b++) {
    for (int col = 0; col < OIL_COLS; col++) {
      final double total = fwdRaw[b][col] + revRaw[b][col];
      if (total > maxV) maxV = total;
    }
  }

  if (maxV > 0) {
    for (int b = 0; b < BOARDS; b++) {
      for (int col = 0; col < OIL_COLS; col++) {
        fwdRaw[b][col] = (fwdRaw[b][col] / maxV).clamp(0.0, 1.0);
        revRaw[b][col] = (revRaw[b][col] / maxV).clamp(0.0, 1.0);
      }
    }
  }

  return OilMatrix(
    forwardOil: fwdRaw,
    reverseOil: revRaw,
    fwdViscosity: fwdOilType.viscosityPaS,
    revViscosity: revOilType.viscosityPaS,
  );
}

double stribeckMuHat(double hHat, double muMin) {
  return muMin +
      STRIBECK_A / (hHat + STRIBECK_B) +
      STRIBECK_C * (1.0 - exp(-STRIBECK_D * hHat));
}

double rollReadinessFromSlip(double slipRatio) {
  return ((0.20 - slipRatio) / 0.20).clamp(0.0, 1.0);
}

double _clampAbs(double v, double maxAbs) {
  if (v > maxAbs) return maxAbs;
  if (v < -maxAbs) return -maxAbs;
  return v;
}

SimResult runSimulationV2(BowlerInputs inp, PatternData pat, BallSpecs ball, OilMatrix oil) {
  final double massKg = ball.massKg;
  final InertiaTensor inertia = ball.inertiaTensor;

  final double angleRad = inp.effectiveAngleDeg * pi / 180.0;
  final double speed = inp.speedMph * 0.44704;
  double vx = speed * cos(angleRad);
  double vy = speed * sin(angleRad);

  final double omega0 = inp.revRPM * 2.0 * pi / 60.0;
  final double psi = inp.axisRotation * pi / 180.0 * inp.handedness;
  final double phi = inp.axisTilt * pi / 180.0;

  // ω0 = ω [sinψ cosφ, cosψ cosφ, sinφ]^T
  double omegaX = omega0 * sin(psi) * cos(phi);
  double omegaY = omega0 * cos(psi) * cos(phi);
  double omegaZ = omega0 * sin(phi);

  double ft = inp.effectiveLandingDistanceFt.clamp(0.5, 15.0);
  double board = inp.effectiveLandBoard.clamp(1.0, 39.0);
  double theta = -pi / 2.0;

  const double stepFt = 0.25;
  final double stepM = stepFt * 0.3048;

  bool inGutter = false;
  double ballOil = 0.0;
  double prevOil = oil.sampleAt(board, ft).totalOil;
  final List<PathPoint> path = [];

  while (ft < LANE_FT.toDouble() && vx > 0.3) {
    final OilSample sample = oil.sampleAt(board, ft);
    final double hLocal = sample.totalOil;
    final double etaOil = sample.effectiveViscosity > 0.0 ? sample.effectiveViscosity : DEFAULT_OIL.viscosityPaS;
    final double etaLocal = ETA_DRY_PAS + (etaOil - ETA_DRY_PAS) * min(hLocal / H_SAT, 1.0);

    final double dt = stepM / vx.clamp(0.3, 30.0);
    final double fn = massKg * G_MS2;
    final double pressure = fn / CONTACT_AREA_M2;

    // Contact kinematics: v_slip = [vx - R ωy, vy + R ωx]
    final double slipX = vx - BALL_R_M * omegaY;
    final double slipY = vy + BALL_R_M * omegaX;
    final double slipMag = sqrt(slipX * slipX + slipY * slipY);
    final double slipRatio = slipMag / vx.clamp(0.3, 30.0);
    final double ux = slipMag > 1e-9 ? slipX / slipMag : 0.0;
    final double uy = slipMag > 1e-9 ? slipY / slipMag : 0.0;

    final double film = hLocal.clamp(0.0, 1.0);
    final double hersey = pressure > 1e-9
        ? film * (etaLocal * slipMag) / pressure
        : 0.0;
    final double hHat = hersey / H_REF;

    final double muCap = ball.muDry;
    final double muEff = stribeckMuHat(hHat, MU_OIL_MIN).clamp(MU_ROLL_RES, muCap);

    double fx = 0.0;
    double fy = 0.0;

    if (inGutter) {
      final double speedMag = sqrt(vx * vx + vy * vy);
      final double decel = MU_ROLL_RES * fn / massKg;
      final double nextSpeed = max(0.3, speedMag - decel * dt);
      if (speedMag > 1e-9) {
        final double s = nextSpeed / speedMag;
        vx *= s;
        vy *= s;
      }
      omegaX *= (1.0 - 0.10 * dt * 60.0).clamp(0.0, 1.0);
      omegaY *= (1.0 - 0.08 * dt * 60.0).clamp(0.0, 1.0);
      omegaZ *= (1.0 - 0.05 * dt * 60.0).clamp(0.0, 1.0);
    } else if (slipMag > 1e-9) {
      fx = -muEff * fn * ux;
      fy = -muEff * fn * uy;

      // Translation ODEs.
      vx = max(0.3, vx + (fx / massKg) * dt);
      vy += (fy / massKg) * dt;

      // Euler rigid-body ODEs.
      final double tauX = -BALL_R_M * fy;
      final double tauY = BALL_R_M * fx;
      const double tauZ = 0.0;

      final double domegaX = (tauX - (inertia.iz - inertia.iy) * omegaY * omegaZ) / inertia.ix;
      final double domegaY = (tauY - (inertia.ix - inertia.iz) * omegaZ * omegaX) / inertia.iy;
      final double domegaZ = (tauZ - (inertia.iy - inertia.ix) * omegaX * omegaY) / inertia.iz;

      omegaX += domegaX * dt;
      omegaY += domegaY * dt;
      omegaZ += domegaZ * dt;
    } else {
      final double speedMag = sqrt(vx * vx + vy * vy);
      final double decel = MU_ROLL_RES * fn / massKg;
      final double nextSpeed = max(0.3, speedMag - decel * dt);
      if (speedMag > 1e-9) {
        final double s = nextSpeed / speedMag;
        vx *= s;
        vy *= s;
      }
      omegaX *= (1.0 - 0.02 * dt * 60.0).clamp(0.0, 1.0);
      omegaY *= (1.0 - 0.02 * dt * 60.0).clamp(0.0, 1.0);
      omegaZ *= (1.0 - 0.02 * dt * 60.0).clamp(0.0, 1.0);
    }

    omegaX = _clampAbs(omegaX, 200.0);
    omegaY = _clampAbs(omegaY, 200.0);
    omegaZ = _clampAbs(omegaZ, 200.0);

    // Oil depletion proxy from ∂h/∂t = -(k/ρ) μ mg U / A_c at the contact point.
    final double depletionRate = OIL_DEPLETION_K * muEff * fn * slipMag / (OIL_DENSITY_PROXY * CONTACT_AREA_M2);
    final double pickup = depletionRate * dt * 1.0e-4;
    final double deposit = OIL_DEPOSIT_RATE * ballOil * (0.25 + 0.75 * OIL_CARRY_BLEND) * dt;
    oil.pickupAt(board, ft, pickup);
    oil.depositAt(board, ft + 1.0 + 2.0 * vx / 8.5, deposit);
    ballOil = (ballOil + pickup - deposit).clamp(0.0, 1.0);

    final double omegaTot = sqrt(omegaX * omegaX + omegaY * omegaY + omegaZ * omegaZ);
    final double ar = atan2(omegaX.abs(), omegaY.abs()) * 180.0 / pi;

    final double slipXPost = vx - BALL_R_M * omegaY;
    final double slipYPost = vy + BALL_R_M * omegaX;
    final double slipMagPost = sqrt(slipXPost * slipXPost + slipYPost * slipYPost);
    final double slipRatioPost = vx.abs() > 1e-9 ? slipMagPost / vx.abs() : 0.0;
    final double tractionXDiagnostic = rollReadinessFromSlip(slipRatioPost);

    final bool isRoll = slipMagPost < ROLL_SLIP_EPS;
    final bool isHook = !isRoll && slipYPost.abs() > HOOK_LATERAL_SLIP_EPS && omegaZ.abs() > 1e-3;
    final bool isSkid = !isRoll && !isHook && hHat > HIGH_HERSEY_THRESH;
    final String phase = isRoll ? 'roll' : (isHook ? 'hook' : (isSkid ? 'skid' : 'hook'));

    theta += omegaTot * dt;
    ft += stepFt;
    board += (vy * dt) / BOARD_M;

    if (!inGutter && (board < 0.5 || board > 39.5)) inGutter = true;
    board = board.clamp(-3.0, 43.0);

    final double oilDrop = prevOil - hLocal;
    final bool atBreakpoint = !inGutter && oilDrop > 0.05 && hLocal < 0.18 && slipYPost.abs() > 0.05 && ft > 25.0;
    prevOil = hLocal;

    path.add(PathPoint(
      ft: ft,
      board: board,
      vx: vx * 2.23694,
      omega: omegaTot,
      theta: theta,
      oil: hLocal,
      mu: muEff,
      AR: ar,
      slipRatio: slipRatioPost,
      tractionX: tractionXDiagnostic,
      phase: phase,
      atBreakpoint: atBreakpoint,
    ));
  }

  final PathPoint? pinPoint = pointNearestFt(path, HEAD_PIN_FT);
  return SimResult(
    path: path,
    pinSpeed: pinPoint?.vx ?? 0.0,
    pinRPM: pinPoint != null ? pinPoint.omega * 60.0 / (2.0 * pi) : 0.0,
    pinBoard: pinPoint?.board ?? 0.0,
    pinAR: pinPoint?.AR ?? 0.0,
  );
}
