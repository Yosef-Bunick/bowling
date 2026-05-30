import 'dart:math';

// ═══════════════════════════════════════════════════════════
// BOWLING PHYSICS ENGINE v22 — UNIFIED CONTACT MODEL
//
// Key principles:
//   1. ONE set of equations for all phases (skid, hook, roll).
//      No if/else solver branches based on phase.
//   2. Full Euler rigid-body spin dynamics with gyroscopic
//      cross-coupling: dω = I⁻¹(τ − ω × Iω)
//   3. Contact force from slip vector, continuous everywhere,
//      regularized near zero slip with ε-softening.
//   4. Phase labels are DIAGNOSTIC ONLY — they describe
//      the current regime, they never switch the solver.
//   5. Roll emerges naturally when slip → 0 and static
//      friction can sustain the rolling constraint.
//   6. Axis migration emerges from anisotropic inertia +
//      Euler coupling, not from manual component damping.
//
// Retained from P18/P19:
//   - RK4 integration (dt = 1/240s)
//   - Full vector spin state (ωx, ωy, ωz) + InertiaTensor
//   - Traction X/Y as integrated state variables
//   - Stribeck-on-Ĥ friction with grit scaling (from P11)
//   - Dynamic contact area (from P11)
//   - Dual-layer oil matrix with viscosity blending
//   - Oil pickup/deposit heuristic
//
// Removed:
//   - Phase-switched solver (rolling/else branches)
//   - Heading lock (was a control rule, not physics)
//   - Manual ω component damping (ωH *= 0.80, etc.)
//   - ω = v/R snap-to-roll overwrite
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
const double CONTACT_AREA_M2 = 5.0e-4;
const double OIL_AREA_GAIN = 1.25;
const double HEAD_PIN_FT = 60.0;

// Slip regularization epsilon — prevents division by zero
// near pure roll without snapping or branching.
const double SLIP_EPS = 1e-4;

// Rolling resistance torque coefficient.
// Small resistive torque that exists even in pure roll,
// from coverstock deformation / hysteresis.
const double ROLL_RESISTANCE_TORQUE_K = 0.00025;

// Static/kinetic contact blend near roll.
// Below STATIC_SLIP_MPS the solver behaves mostly like a no-slip
// static contact constraint. Above KINETIC_SLIP_MPS it behaves like
// kinetic friction along the slip direction.
const double STATIC_SLIP_MPS = 0.08;
const double KINETIC_SLIP_MPS = 0.24;
const double ROLL_SERVO_GAIN_X = 125.0;
const double ROLL_SERVO_GAIN_Y = 65.0;

const Map<String, double> GRIT_DRY_MU = {
  '500': 0.30,
  '1000': 0.23,
  '2000': 0.20,
  '3000': 0.15,
  '4000': 0.11,
  'polish': 0.09,
};

// Surface grit scaling for Stribeck friction (from P11)
class SurfaceFrictionTuning {
  final double aScale;
  final double hRefScale;
  const SurfaceFrictionTuning({required this.aScale, required this.hRefScale});
}

SurfaceFrictionTuning surfaceFrictionTuningForGrit(String grit) {
  switch (grit) {
    case '500':
      return const SurfaceFrictionTuning(aScale: 1.20, hRefScale: 0.80);
    case '1000':
      return const SurfaceFrictionTuning(aScale: 1.10, hRefScale: 0.90);
    case '2000':
      return const SurfaceFrictionTuning(aScale: 1.00, hRefScale: 1.00);
    case '3000':
      return const SurfaceFrictionTuning(aScale: 0.92, hRefScale: 1.10);
    case '4000':
      return const SurfaceFrictionTuning(aScale: 0.86, hRefScale: 1.20);
    case 'polish':
      return const SurfaceFrictionTuning(aScale: 0.78, hRefScale: 1.35);
    default:
      return const SurfaceFrictionTuning(aScale: 1.00, hRefScale: 1.00);
  }
}

// Contact / friction tuning
const double MU_K_DRY_SCALE = 0.86;
const double MU_S_OIL_MIN = 0.020;
const double X_SKID_SLIP = 0.95;
const double X_ROLL_SLIP = 0.06;
const double Y_SKID_SLIP = 0.70;
const double Y_ROLL_SLIP = 0.04;
const double STRIBECK_A = 0.005;
const double STRIBECK_B = 0.045;
const double STRIBECK_C = 0.025;
const double STRIBECK_D = 12.0;
const double H_REF = 1.5e-6;
const double OIL_X_DROP = 0.10;
const double OIL_Y_DROP = 0.24;
const double OIL_X_EXP = 1.15;
const double OIL_Y_EXP = 1.35;
const double TX_RISE = 11.0;
const double TX_FALL = 5.0;
const double TY_RISE = 4.0;
const double TY_FALL = 8.5;
const double MIN_LATERAL_TRACTION = 0.04;

// Phase classification thresholds (diagnostic only — never drive solver)
const double PHASE_SLIP_ROLL_THRESH = 0.03;
const double PHASE_HOOK_LATERAL_THRESH = 0.15;
const double BREAKPOINT_TY_THRESH = 0.20;
const double PHASE_HOOK_TY_THRESH = 0.18;

const double OIL_PICKUP_RATE = 0.02;
const double OIL_DEPOSIT_RATE = 0.012;
const double OIL_CARRY_BLEND = 0.25;
const double FLARE_DIFF_GAIN = 0.90;
const double FLARE_ASY_GAIN = 0.45;

// Gutter friction — separate from lane physics.
// The gutter is a different surface so we use a simple
// deceleration model rather than the lane contact equations.
const double GUTTER_MU = 0.06;
const double GUTTER_LATERAL_DAMP = 12.0;
const double GUTTER_SPIN_DAMP = 3.0;

// ═══════════════════════════════════════════════════════════
// OIL TYPE LIBRARY
// ═══════════════════════════════════════════════════════════

class OilType {
  final String name;
  final double viscosityPaS;
  final double carrydownMobility;

  const OilType({
    required this.name,
    required this.viscosityPaS,
    this.carrydownMobility = 1.0,
  });
}

const Map<String, OilType> OIL_LIBRARY = {
  'glide': OilType(name: 'Glide', viscosityPaS: 0.0389, carrydownMobility: 1.06),
  'terrain': OilType(name: 'Terrain', viscosityPaS: 0.0810, carrydownMobility: 0.70),
  'curve': OilType(name: 'Curve', viscosityPaS: 0.0550, carrydownMobility: 0.85),
  'current': OilType(name: 'Current', viscosityPaS: 0.0520, carrydownMobility: 0.90),
  'condition_red': OilType(name: 'Condition Red', viscosityPaS: 0.0490, carrydownMobility: 0.88),
  'defense_s': OilType(name: 'Defense-S', viscosityPaS: 0.0473, carrydownMobility: 0.82),
  'fire': OilType(name: 'Fire', viscosityPaS: 0.0451, carrydownMobility: 1.00),
  'ice': OilType(name: 'Ice', viscosityPaS: 0.0409, carrydownMobility: 1.00),
  'condition_blue': OilType(name: 'Condition Blue', viscosityPaS: 0.0394, carrydownMobility: 1.05),
  'infinity': OilType(name: 'Infinity', viscosityPaS: 0.0365, carrydownMobility: 1.10),
  'navigate': OilType(name: 'Navigate', viscosityPaS: 0.0313, carrydownMobility: 1.15),
  'prodigy': OilType(name: 'Prodigy', viscosityPaS: 0.0312, carrydownMobility: 1.15),
  'clear_super_100': OilType(name: 'Clear Super 100', viscosityPaS: 0.1010, carrydownMobility: 0.50),
  'clear_super_50': OilType(name: 'Clear Super 50', viscosityPaS: 0.0500, carrydownMobility: 0.75),
  'clear_801_hv': OilType(name: 'Clear #801 HV', viscosityPaS: 0.0199, carrydownMobility: 1.30),
  'clear_811_lv': OilType(name: 'Clear #811 LV', viscosityPaS: 0.0152, carrydownMobility: 1.40),
  'ceo_65': OilType(name: 'CEO 65', viscosityPaS: 0.0630, carrydownMobility: 0.80),
  'ceo_42': OilType(name: 'CEO 42', viscosityPaS: 0.0420, carrydownMobility: 1.00),
  'se_28': OilType(name: 'SE 28', viscosityPaS: 0.0250, carrydownMobility: 1.20),
};

const OilType DEFAULT_OIL = OilType(name: 'Default', viscosityPaS: 0.0450);

class OilSample {
  final double totalOil;
  final double forwardOil;
  final double reverseOil;
  final double effectiveViscosity;

  const OilSample({
    required this.totalOil,
    required this.forwardOil,
    required this.reverseOil,
    required this.effectiveViscosity,
  });
}

// ═══════════════════════════════════════════════════════════
// BALL SPECS + INERTIA TENSOR
// ═══════════════════════════════════════════════════════════

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

class BallSpecs {
  double rg;
  double diff;
  double asy;
  String grit;
  double masslb;

  BallSpecs({
    this.rg = 2.53,
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

  /// Principal-axis inertia tensor estimated from RG/diff/asy.
  ///
  /// I_mean = m × k²
  /// Differential spreads I_low vs I_high.
  /// Asymmetry offsets the third axis from midpoint.
  /// Renormalized so mean stays at I_mean.
  InertiaTensor get inertiaTensor {
    final double iMean = moiKgM2;
    final double diffFrac = (diff / 0.060).clamp(0.0, 1.0) * 0.12;
    final double asyFrac = (asy / 0.030).clamp(0.0, 1.0) * 0.06;

    double ix = iMean * (1.0 - 0.5 * diffFrac - 0.5 * asyFrac);
    double iy = iMean * (1.0 + 0.5 * diffFrac + 0.5 * asyFrac);
    double iz = iMean * (1.0 - 0.5 * diffFrac + 0.5 * asyFrac);

    final double mean = (ix + iy + iz) / 3.0;
    if (mean > 1e-12) {
      final double s = iMean / mean;
      ix *= s;
      iy *= s;
      iz *= s;
    }

    return InertiaTensor(ix: ix, iy: iy, iz: iz);
  }
}

// ═══════════════════════════════════════════════════════════
// INPUT / OUTPUT DATA CLASSES
// ═══════════════════════════════════════════════════════════

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
    required this.sl, required this.sr, required this.loads,
    required this.mics, required this.buff,
    required this.d0, required this.d1, required this.toil,
    this.oilType = 'fire',
  });

  LoadRow copyWith({
    int? sl, int? sr, int? loads, int? mics, int? buff,
    double? d0, double? d1, double? toil, String? oilType,
  }) => LoadRow(
    sl: sl ?? this.sl, sr: sr ?? this.sr,
    loads: loads ?? this.loads, mics: mics ?? this.mics,
    buff: buff ?? this.buff, d0: d0 ?? this.d0, d1: d1 ?? this.d1,
    toil: toil ?? this.toil, oilType: oilType ?? this.oilType,
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
    required this.name, required this.distance,
    required this.fwdRows, required this.revRows,
    this.fwdOilType = 'fire', this.revOilType = 'fire',
  });

  OilType get fwdOil => OIL_LIBRARY[fwdOilType] ?? DEFAULT_OIL;
  OilType get revOil => OIL_LIBRARY[revOilType] ?? DEFAULT_OIL;

  static PatternData masters2026() => PatternData(
    name: '2026 USBC Masters', distance: 41,
    fwdOilType: 'glide', revOilType: 'ice',
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

class SegmentResult {
  final String seg;
  final double ft0, ft1, mu, vIn, vOut, ARin, ARout, ATin, ATout, dboards, boardOut, totalRPM;
  final String phase;

  const SegmentResult({
    required this.seg, required this.ft0, required this.ft1,
    required this.mu, required this.vIn, required this.vOut,
    required this.ARin, required this.ARout,
    required this.ATin, required this.ATout,
    required this.dboards, required this.boardOut,
    required this.totalRPM, required this.phase,
  });
}

class PathPoint {
  final double ft, board, vx, omega, theta, oil, mu, AR;
  final double slipRatio, tractionX, tractionY, muX, muY;
  final double lateralForceRatio, kineticBlend, slipX, slipY;
  final String phase;
  final bool atBreakpoint;

  const PathPoint({
    required this.ft, required this.board, required this.vx,
    required this.omega, required this.theta, required this.oil,
    required this.mu, required this.AR,
    required this.slipRatio, required this.tractionX,
    required this.tractionY, required this.muX, required this.muY,
    required this.lateralForceRatio, required this.kineticBlend,
    required this.slipX, required this.slipY,
    required this.phase, required this.atBreakpoint,
  });
}

class SimResult {
  final List<PathPoint> path;
  final List<SegmentResult> segments;
  final double pinSpeed, pinRPM, pinBoard, pinAR;

  const SimResult({
    required this.path, required this.segments,
    required this.pinSpeed, required this.pinRPM,
    required this.pinBoard, required this.pinAR,
  });
}

PathPoint? pointNearestFt(List<PathPoint> path, double targetFt) {
  if (path.isEmpty) return null;
  PathPoint best = path.first;
  double bestErr = (best.ft - targetFt).abs();
  for (final p in path.skip(1)) {
    final err = (p.ft - targetFt).abs();
    if (err < bestErr) { best = p; bestErr = err; }
  }
  return best;
}

// ═══════════════════════════════════════════════════════════
// OIL MATRIX (unchanged from P18)
// ═══════════════════════════════════════════════════════════

class OilMatrix {
  final List<List<double>> forwardOil;
  final List<List<double>> reverseOil;
  final double fwdViscosity;
  final double revViscosity;

  OilMatrix({
    required this.forwardOil, required this.reverseOil,
    required this.fwdViscosity, required this.revViscosity,
  });

  OilSample sampleAt(double board, double ft) {
    final fwd = _interpolate(forwardOil, board, ft);
    final rev = _interpolate(reverseOil, board, ft);
    final total = fwd + rev;
    final eta = total > 1e-9
        ? (fwd * fwdViscosity + rev * revViscosity) / total
        : DEFAULT_OIL.viscosityPaS;
    return OilSample(
      totalOil: total.clamp(0.0, 1.0), forwardOil: fwd,
      reverseOil: rev, effectiveViscosity: eta,
    );
  }

  double _interpolate(List<List<double>> oil, double board, double ft) {
    if (oil.isEmpty) return 0.0;
    final int cols = oil[0].length;
    final double bIdx = (board - 1.0).clamp(0.0, (BOARDS - 1).toDouble());
    final double fIdx = (ft / OIL_RES).clamp(0.0, (cols - 1).toDouble());
    final int b0 = bIdx.floor().clamp(0, BOARDS - 1);
    final int b1 = (b0 + 1).clamp(0, BOARDS - 1);
    final int f0 = fIdx.floor().clamp(0, cols - 1);
    final int f1 = (f0 + 1).clamp(0, cols - 1);
    final double tb = bIdx - b0;
    final double tf = fIdx - f0;
    return ((oil[b0][f0] * (1 - tb) + oil[b1][f0] * tb) * (1 - tf) +
            (oil[b0][f1] * (1 - tb) + oil[b1][f1] * tb) * tf)
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
    BOARDS, (b) => List.generate(
      OIL_COLS, (c) => (forwardOil[b][c] + reverseOil[b][c]).clamp(0.0, 1.0),
    ),
  );
}

OilMatrix buildOilMatrix(
  List<LoadRow> fwdRows, List<LoadRow> revRows,
  OilType fwdOilType, OilType revOilType, {
  double alphaF = 0.25, double rF = 0.91,
  double alphaR = 0.32, double rR = 0.94,
  double k = 0.85, double beta = 0.35,
  double gamma = 0.12, double eta = 0.45,
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
          if (x >= x0 && x <= x1) {
            contribution = baseAmount + gamma * saturation;
          } else if (x > x1) {
            final double d = x - x1;
            if (d <= buffFt) contribution = endSaturation * alphaF * pow(rF, d);
          }
          target[b][col] += contribution;
        }
      }
      prevSaturation = endSaturation;
    }
    return prevSaturation;
  }

  void applyReverseRows(List<LoadRow> rows, List<List<double>> target, double forwardEndSaturation) {
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
          if (x <= xDrop && x >= xStop) {
            contribution = baseAmount + gamma * saturation;
          } else if (x < xStop) {
            final double d = xStop - x;
            if (d <= buffFt && d >= 0) contribution = endSaturation * alphaR * pow(rR, d);
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
    forwardOil: fwdRaw, reverseOil: revRaw,
    fwdViscosity: fwdOilType.viscosityPaS, revViscosity: revOilType.viscosityPaS,
  );
}

// ═══════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════

double clamp01(double x) => x.clamp(0.0, 1.0);


double smoothstep(double edge0, double edge1, double x) {
  if ((edge1 - edge0).abs() < 1e-12) return x <= edge0 ? 0.0 : 1.0;
  final double t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

double rollResistanceActivation(double slipMag) {
  // Almost no roll resistance in high-slip skid/hook.
  // It ramps up only as the contact approaches true rolling.
  return 1.0 - smoothstep(STATIC_SLIP_MPS, KINETIC_SLIP_MPS, slipMag);
}

double blendBySlip(double slipRatio, double skidSlip, double rollSlip) {
  return ((skidSlip - slipRatio) / (skidSlip - rollSlip)).clamp(0.0, 1.0);
}

double hookBlendBySlip(double slipRatio, double rollSlip, double skidSlip) {
  return ((slipRatio - rollSlip) / (skidSlip - rollSlip)).clamp(0.0, 1.0);
}

double stateRate(double current, double target, double riseRate, double fallRate) {
  final double rate = target >= current ? riseRate : fallRate;
  return (target - current) * rate;
}

double oilGripFactor(double oil, double drop, double expP) {
  return clamp01(1.0 - drop * pow(oil, expP).toDouble());
}

double flareExposure(BallSpecs ball, double omegaTot, double axisTiltDeg) {
  final double diffNorm = (ball.diff / 0.060).clamp(0.0, 1.0);
  final double asyNorm = (ball.asy / 0.030).clamp(0.0, 1.0);
  final double rpmNorm = ((omegaTot * 60.0 / (2.0 * pi)) / 500.0).clamp(0.0, 1.0);
  final double tiltSupp = 1.0 - (axisTiltDeg / 90.0).clamp(0.0, 1.0) * 0.35;
  return clamp01(diffNorm * FLARE_DIFF_GAIN + asyNorm * FLARE_ASY_GAIN + rpmNorm * 0.20) * tiltSupp;
}

// ═══════════════════════════════════════════════════════════
// PHASE CLASSIFICATION — DIAGNOSTIC ONLY
//
// These functions NEVER drive the solver.
// They describe what regime the contact solution is in.
// ═══════════════════════════════════════════════════════════

enum BallPhase { skid, hook, roll }

/// Classify phase from static-roll feasibility and lateral force ratio.
/// This is a READOUT, not a control signal.
BallPhase classifyPhaseUnified({
  required bool canStaticRoll,
  required double lateralForceRatio,
  required double tractionY,
}) {
  if (canStaticRoll) return BallPhase.roll;

  final bool hookByForce = lateralForceRatio > PHASE_HOOK_LATERAL_THRESH;
  final bool hookByGrip = tractionY > PHASE_HOOK_TY_THRESH;

  if (hookByForce && hookByGrip) return BallPhase.hook;
  return BallPhase.skid;
}

String phaseLabel(BallPhase phase) {
  switch (phase) {
    case BallPhase.roll: return 'roll';
    case BallPhase.hook: return 'hook';
    case BallPhase.skid: return 'skid';
  }
}

// ═══════════════════════════════════════════════════════════
// RK4 STATE + DERIVATIVE
// ═══════════════════════════════════════════════════════════

class _BallState {
  final double ft, board, vx, vy;
  final double omegaX, omegaY, omegaZ;
  final double theta, tractionX, tractionY, ballOil;

  const _BallState({
    required this.ft, required this.board,
    required this.vx, required this.vy,
    required this.omegaX, required this.omegaY, required this.omegaZ,
    required this.theta,
    required this.tractionX, required this.tractionY, required this.ballOil,
  });

  _BallState addScaled(_BallDeriv k, double h) => _BallState(
    ft: ft + k.dFt * h, board: board + k.dBoard * h,
    vx: vx + k.dVx * h, vy: vy + k.dVy * h,
    omegaX: omegaX + k.dOmegaX * h,
    omegaY: omegaY + k.dOmegaY * h,
    omegaZ: omegaZ + k.dOmegaZ * h,
    theta: theta + k.dTheta * h,
    tractionX: tractionX + k.dTractionX * h,
    tractionY: tractionY + k.dTractionY * h,
    ballOil: ballOil + k.dBallOil * h,
  );

  _BallState clampState() => _BallState(
    ft: ft, board: board.clamp(-3.0, 43.0),
    vx: vx < 0.3 ? 0.3 : vx, vy: vy,
    omegaX: omegaX, omegaY: omegaY, omegaZ: omegaZ,
    theta: theta,
    tractionX: tractionX.clamp(0.0, 1.0),
    tractionY: tractionY.clamp(0.0, 1.0),
    ballOil: ballOil.clamp(0.0, 1.0),
  );
}

class _BallDeriv {
  final double dFt, dBoard, dVx, dVy;
  final double dOmegaX, dOmegaY, dOmegaZ;
  final double dTheta, dTractionX, dTractionY, dBallOil;

  const _BallDeriv({
    required this.dFt, required this.dBoard,
    required this.dVx, required this.dVy,
    required this.dOmegaX, required this.dOmegaY, required this.dOmegaZ,
    required this.dTheta,
    required this.dTractionX, required this.dTractionY, required this.dBallOil,
  });
}

typedef _OilSampleFn = OilSample Function(double board, double ft);
typedef _OilUpdateFn = void Function(double board, double ft, double amount);

// ═══════════════════════════════════════════════════════════
// UNIFIED DERIVATIVE — SINGLE CODE PATH
//
// The core physics equation set:
//
//   Slip:     u = [vx − R·ωy,  vy + h·R·ωx]
//   Force:    F = −μ(H) · N · û      (ε-regularized)
//   Transl:   m·dv/dt = F − F_roll_resistance
//   Rot:      I·dω/dt = τ − ω×(I·ω)   (full Euler coupling)
//   Torque:   τ = r_contact × F = [−R·Fy, R·Fx, τ_z_resist]
//
// NO branching on phase. Gutter uses separate simple model
// because it is a physically different surface.
// ═══════════════════════════════════════════════════════════

_BallDeriv _evalDeriv({
  required _BallState s,
  required BallSpecs ball,
  required double massKg,
  required InertiaTensor inertia,
  required _OilSampleFn sampleOil,
  required bool inGutter,
  required double handedness,
}) {
  final double fn = massKg * G_MS2;

  // ─── Gutter: different surface, simple deceleration ───
  if (inGutter) {
    final double speedMag = sqrt(s.vx * s.vx + s.vy * s.vy);
    final double decel = GUTTER_MU * fn / massKg;
    double dVx = 0.0;
    double dVy = 0.0;
    if (speedMag > 0.31) {
      dVx = -decel * s.vx / max(speedMag, 1e-6);
      dVy = -decel * s.vy / max(speedMag, 1e-6) - GUTTER_LATERAL_DAMP * s.vy;
    }
    final double omegaTot = sqrt(s.omegaX * s.omegaX + s.omegaY * s.omegaY + s.omegaZ * s.omegaZ);
    return _BallDeriv(
      dFt: s.vx / 0.3048, dBoard: s.vy / BOARD_M,
      dVx: dVx, dVy: dVy,
      dOmegaX: -GUTTER_SPIN_DAMP * s.omegaX,
      dOmegaY: -GUTTER_SPIN_DAMP * s.omegaY,
      dOmegaZ: -GUTTER_SPIN_DAMP * s.omegaZ,
      dTheta: omegaTot,
      dTractionX: stateRate(s.tractionX, 0.0, TX_RISE, TX_FALL),
      dTractionY: stateRate(s.tractionY, 0.0, TY_RISE, TY_FALL),
      dBallOil: 0.0,
    );
  }

  // ─── Lane contact: unified model, no phase branching ───

  // 1. Sample oil
  final OilSample sample = sampleOil(s.board, s.ft);
  final double oilLocal = sample.totalOil;
  final double etaEff = sample.effectiveViscosity;

  // 2. Contact-point slip vector
  //    u = [vx − R·ωy,  vy + h·R·ωx]
  final double slipX = s.vx - BALL_R_M * s.omegaY;
  final double slipY = s.vy + handedness * BALL_R_M * s.omegaX;
  final double slipMag = sqrt(slipX * slipX + slipY * slipY);
  final double speedMag = sqrt(s.vx * s.vx + s.vy * s.vy);
  final double slipRatio = slipMag / max(speedMag, 0.3);

  // ε-regularized unit slip direction.
  // At zero slip this goes to zero smoothly — no division by zero,
  // no snap, no branch. Force naturally vanishes at pure roll.
  final double slipDenom = slipMag + SLIP_EPS;
  final double ux = slipX / slipDenom;
  final double uy = slipY / slipDenom;

  // 3. Spin-derived state
  final double omegaTot = sqrt(s.omegaX * s.omegaX + s.omegaY * s.omegaY + s.omegaZ * s.omegaZ);
  final double axisTiltDeg = omegaTot > 1e-6
      ? asin((s.omegaZ.abs() / omegaTot).clamp(0.0, 1.0)) * 180.0 / pi
      : 0.0;
  final double flare = flareExposure(ball, omegaTot, axisTiltDeg);

  // 4. Friction coefficient from Stribeck model
  final double muMax = ball.muDry * (1.0 + 0.12 * flare);
  final double film = oilLocal.clamp(0.0, 1.0);

  // Dynamic contact area (from P11)
  final double contactAreaEff = CONTACT_AREA_M2 * (1.0 + OIL_AREA_GAIN * film);
  final double pEff = fn / contactAreaEff;

  // Grit-scaled Stribeck parameters
  final SurfaceFrictionTuning surface = surfaceFrictionTuningForGrit(ball.grit);
  final double hersey = pEff > 1e-6 ? film * (etaEff * slipMag) / pEff : 0.0;
  final double hHat = hersey / (H_REF * surface.hRefScale);
  final double muBaseRaw = MU_OIL_MIN +
      (STRIBECK_A * surface.aScale) / (hHat + STRIBECK_B) +
      STRIBECK_C * (1.0 - exp(-STRIBECK_D * hHat));
  final double muBase = muBaseRaw.clamp(MU_ROLL_RES, muMax);

  // Static/kinetic split
  final double muK = max(MU_ROLL_RES, min(muBase * MU_K_DRY_SCALE, ball.muDry));
  final double muS = max(muK, min(ball.muDry, max(MU_S_OIL_MIN, muBase)));

  // 5. Traction state targets (oil + slip readiness)
  final double slipBlendX = smoothstep(
    0.0,
    1.0,
    blendBySlip(slipRatio, X_SKID_SLIP, X_ROLL_SLIP),
  );
  final double slipBlendY = smoothstep(
    0.0,
    1.0,
    hookBlendBySlip(slipRatio, Y_ROLL_SLIP, Y_SKID_SLIP),
  );
  final double oilGripX = oilGripFactor(oilLocal, OIL_X_DROP, OIL_X_EXP);
  final double oilGripY = oilGripFactor(oilLocal, OIL_Y_DROP, OIL_Y_EXP);

  final double tractionTargetX = slipBlendX * oilGripX;
  final double tractionTargetY = slipBlendY * oilGripY;
  final double tractionX = s.tractionX.clamp(0.0, 1.0);
  final double tractionY = s.tractionY.clamp(0.0, 1.0);

  // Effective directional friction (blend kinetic → static as traction builds)
  final double muX = muK * (1.0 - tractionX) + muS * tractionX;
  final double muY = muK * (1.0 - tractionY) + muS * tractionY;
  final double lateralScale = MIN_LATERAL_TRACTION + (1.0 - MIN_LATERAL_TRACTION) * tractionY;

  // 6. Contact force — unified static/kinetic blend
  //    High slip  -> kinetic friction along slip direction.
  //    Tiny slip  -> static-like servo enforces the no-slip constraint
  //                  without any hard ω = v/R snap.
  //    X is allowed to transfer speed into roll more strongly than Y so
  //    rev rate can rise as ball speed falls before convergence to roll.
  final double kineticBlend = smoothstep(STATIC_SLIP_MPS, KINETIC_SLIP_MPS, slipMag);

  final double fxK = -muX * fn * ux;
  final double fyK = -muY * fn * uy * lateralScale;

  double fxS = -ROLL_SERVO_GAIN_X * slipX;
  double fyS = -ROLL_SERVO_GAIN_Y * slipY;

  final double fxStaticCap = muX * fn;
  final double fyStaticCap = muY * fn * lateralScale;

  fxS = fxS.clamp(-fxStaticCap, fxStaticCap);
  fyS = fyS.clamp(-fyStaticCap, fyStaticCap);

  final double nx = fxStaticCap > 1e-9 ? fxS / fxStaticCap : 0.0;
  final double ny = fyStaticCap > 1e-9 ? fyS / fyStaticCap : 0.0;
  final double ell = sqrt(nx * nx + ny * ny);
  if (ell > 1.0) {
    fxS /= ell;
    fyS /= ell;
  }

  final double fx = fxS * (1.0 - kineticBlend) + fxK * kineticBlend;
  final double fy = fyS * (1.0 - kineticBlend) + fyK * kineticBlend;

  // Rolling resistance: always present, small, acts along velocity
  final double rollResistDecel = MU_ROLL_RES * fn / massKg;
  final double vrx = speedMag > 1e-6 ? s.vx / speedMag : 1.0;
  final double vry = speedMag > 1e-6 ? s.vy / speedMag : 0.0;

  // Translation: m·dv/dt = F_contact − F_roll_resistance
  final double dVx = fx / massKg - rollResistDecel * vrx;
  final double dVy = fy / massKg - rollResistDecel * vry;

  // 7. Contact torque
  //    τ = r_contact × F
  //    r_contact = [0, 0, −R]
  //    τ = [−R·Fy,  R·Fx,  τ_z_resist]
  final double tauX = -BALL_R_M * fy;
  final double tauY = BALL_R_M * fx;
  final double rrBlend = rollResistanceActivation(slipMag);
  final double tauZ = -ROLL_RESISTANCE_TORQUE_K * rrBlend * fn * BALL_R_M *
      (omegaTot > 1e-6 ? s.omegaZ / omegaTot : 0.0);

  // 8. Full Euler rigid-body spin dynamics
  //    I·dω/dt + ω × (I·ω) = τ
  //
  //    dωx = (τx − (Iz − Iy)·ωy·ωz) / Ix
  //    dωy = (τy − (Ix − Iz)·ωz·ωx) / Iy
  //    dωz = (τz − (Iy − Ix)·ωx·ωy) / Iz
  final double dOmegaX = (tauX - (inertia.iz - inertia.iy) * s.omegaY * s.omegaZ) / inertia.ix;
  final double dOmegaY = (tauY - (inertia.ix - inertia.iz) * s.omegaZ * s.omegaX) / inertia.iy;
  final double dOmegaZ = (tauZ - (inertia.iy - inertia.ix) * s.omegaX * s.omegaY) / inertia.iz;

  // 9. Oil transfer
  final double pickup = OIL_PICKUP_RATE * oilLocal * (0.35 + 0.65 * flare) *
      (0.30 + 0.70 * tractionX);
  final double deposit = OIL_DEPOSIT_RATE * s.ballOil.clamp(0.0, 1.0) *
      (0.25 + 0.75 * OIL_CARRY_BLEND);

  return _BallDeriv(
    dFt: s.vx / 0.3048, dBoard: s.vy / BOARD_M,
    dVx: dVx, dVy: dVy,
    dOmegaX: dOmegaX, dOmegaY: dOmegaY, dOmegaZ: dOmegaZ,
    dTheta: omegaTot,
    dTractionX: stateRate(s.tractionX, tractionTargetX, TX_RISE, TX_FALL),
    dTractionY: stateRate(s.tractionY, tractionTargetY, TY_RISE, TY_FALL),
    dBallOil: pickup - deposit,
  );
}

// ═══════════════════════════════════════════════════════════
// RK4 INTEGRATOR
// ═══════════════════════════════════════════════════════════

_BallState _rk4Step({
  required _BallState s,
  required double dt,
  required BallSpecs ball,
  required double massKg,
  required InertiaTensor inertia,
  required _OilSampleFn sampleOil,
  required bool inGutter,
  required double handedness,
}) {
  _BallDeriv eval(_BallState st) => _evalDeriv(
    s: st, ball: ball, massKg: massKg, inertia: inertia,
    sampleOil: sampleOil, inGutter: inGutter, handedness: handedness,
  );

  final k1 = eval(s);
  final k2 = eval(s.addScaled(k1, dt * 0.5));
  final k3 = eval(s.addScaled(k2, dt * 0.5));
  final k4 = eval(s.addScaled(k3, dt));

  double rk(double f1, double f2, double f3, double f4) =>
      dt * (f1 + 2.0 * f2 + 2.0 * f3 + f4) / 6.0;

  return _BallState(
    ft: s.ft + rk(k1.dFt, k2.dFt, k3.dFt, k4.dFt),
    board: s.board + rk(k1.dBoard, k2.dBoard, k3.dBoard, k4.dBoard),
    vx: s.vx + rk(k1.dVx, k2.dVx, k3.dVx, k4.dVx),
    vy: s.vy + rk(k1.dVy, k2.dVy, k3.dVy, k4.dVy),
    omegaX: s.omegaX + rk(k1.dOmegaX, k2.dOmegaX, k3.dOmegaX, k4.dOmegaX),
    omegaY: s.omegaY + rk(k1.dOmegaY, k2.dOmegaY, k3.dOmegaY, k4.dOmegaY),
    omegaZ: s.omegaZ + rk(k1.dOmegaZ, k2.dOmegaZ, k3.dOmegaZ, k4.dOmegaZ),
    theta: s.theta + rk(k1.dTheta, k2.dTheta, k3.dTheta, k4.dTheta),
    tractionX: s.tractionX + rk(k1.dTractionX, k2.dTractionX, k3.dTractionX, k4.dTractionX),
    tractionY: s.tractionY + rk(k1.dTractionY, k2.dTractionY, k3.dTractionY, k4.dTractionY),
    ballOil: s.ballOil + rk(k1.dBallOil, k2.dBallOil, k3.dBallOil, k4.dBallOil),
  ).clampState();
}

// ═══════════════════════════════════════════════════════════
// SIMULATION CORE
//
// No heading lock, no wasRolling, no phase-switching.
// The loop just: integrate → record diagnostics.
// ═══════════════════════════════════════════════════════════

SimResult _runSimulationCore({
  required BowlerInputs inp,
  required BallSpecs ball,
  required _OilSampleFn sampleOil,
  required _OilUpdateFn pickupOil,
  required _OilUpdateFn depositOil,
}) {
  final double massKg = ball.massKg;
  final InertiaTensor inertia = ball.inertiaTensor;
  final double angleRad = inp.effectiveAngleDeg * pi / 180.0;
  final double speed = inp.speedMph * 0.44704;
  final double omega0 = inp.revRPM * 2.0 * pi / 60.0;
  final double psi = inp.axisRotation * pi / 180.0 * inp.handedness;
  final double phi = inp.axisTilt * pi / 180.0;
  final double fn = massKg * G_MS2;

  _BallState state = _BallState(
    ft: inp.effectiveLandingDistanceFt.clamp(0.5, 15.0),
    board: inp.effectiveLandBoard.clamp(1.0, 39.0),
    vx: speed * cos(angleRad),
    vy: speed * sin(angleRad),
    omegaX: omega0 * sin(psi) * cos(phi),
    omegaY: max(0.0, omega0 * cos(psi) * cos(phi)),
    omegaZ: omega0 * sin(phi),
    theta: -pi / 2.0,
    tractionX: 0.0, tractionY: 0.0, ballOil: 0.0,
  );

  const double dt = 1.0 / 240.0;
  final List<PathPoint> path = [];
  final List<SegmentResult> segs = [];
  double segStart = state.ft;
  double segVin = state.vx * 2.23694;
  double segBoard = state.board;
  double segARin = inp.axisRotation;
  double segATin = inp.axisTilt;
  double muAccum = 0.0;
  int muCount = 0;
  String segPhase = 'skid';
  double prevOil = sampleOil(state.board, state.ft).totalOil;
  bool inGutter = false;

  while (state.ft < LANE_FT.toDouble() && state.vx > 0.3) {
    // ─── Integrate ───
    state = _rk4Step(
      s: state, dt: dt, ball: ball, massKg: massKg,
      inertia: inertia, sampleOil: sampleOil,
      inGutter: inGutter, handedness: inp.handedness,
    );

    if (!inGutter && (state.board < 0.5 || state.board > 39.5)) {
      inGutter = true;
    }
    state = state.clampState();

    // ─── Post-step diagnostics ───
    final OilSample postSample = sampleOil(state.board, state.ft);
    final double oilLocal = postSample.totalOil;

    final double slipX = state.vx - BALL_R_M * state.omegaY;
    final double slipY = state.vy + inp.handedness * BALL_R_M * state.omegaX;
    final double slipMag = sqrt(slipX * slipX + slipY * slipY);
    final double speedMag = sqrt(state.vx * state.vx + state.vy * state.vy);
    final double slipRatio = slipMag / max(speedMag, 0.3);

    // Compute post-step friction for diagnostics
    final double film = oilLocal.clamp(0.0, 1.0);
    final double contactAreaEff = CONTACT_AREA_M2 * (1.0 + OIL_AREA_GAIN * film);
    final double pEff = fn / contactAreaEff;
    final SurfaceFrictionTuning surface = surfaceFrictionTuningForGrit(ball.grit);
    final double hersey = pEff > 1e-6
        ? film * (postSample.effectiveViscosity * slipMag) / pEff : 0.0;
    final double hHat = hersey / (H_REF * surface.hRefScale);
    final double omegaTot = sqrt(state.omegaX * state.omegaX +
        state.omegaY * state.omegaY + state.omegaZ * state.omegaZ);
    final double axisTiltDeg = omegaTot > 1e-6
        ? asin((state.omegaZ.abs() / omegaTot).clamp(0.0, 1.0)) * 180.0 / pi : 0.0;
    final double flare = flareExposure(ball, omegaTot, axisTiltDeg);
    final double muMax = ball.muDry * (1.0 + 0.12 * flare);
    final double muBaseRaw = MU_OIL_MIN +
        (STRIBECK_A * surface.aScale) / (hHat + STRIBECK_B) +
        STRIBECK_C * (1.0 - exp(-STRIBECK_D * hHat));
    final double muBase = muBaseRaw.clamp(MU_ROLL_RES, muMax);
    final double muK = max(MU_ROLL_RES, min(muBase * MU_K_DRY_SCALE, ball.muDry));
    final double muS = max(muK, min(ball.muDry, max(MU_S_OIL_MIN, muBase)));
    final double muX = muK * (1.0 - state.tractionX) + muS * state.tractionX;
    final double muY = muK * (1.0 - state.tractionY) + muS * state.tractionY;

    // Lateral force ratio for phase classification
    final double slipDenom = slipMag + SLIP_EPS;
    final double lateralScale = MIN_LATERAL_TRACTION +
        (1.0 - MIN_LATERAL_TRACTION) * state.tractionY;
    final double fxMag = muX * fn * (slipX / slipDenom).abs();
    final double fyMag = muY * fn * (slipY / slipDenom).abs() * lateralScale;
    final double lateralForceRatio = (fxMag + fyMag) > 1e-9
        ? fyMag / (fxMag + fyMag) : 0.0;
    final double kineticBlend =
        smoothstep(STATIC_SLIP_MPS, KINETIC_SLIP_MPS, slipMag);

    final double requestedStaticMag = sqrt(
        pow(ROLL_SERVO_GAIN_X * slipX, 2) +
        pow(ROLL_SERVO_GAIN_Y * slipY, 2),
    );
    final bool canStaticRoll =
        slipMag < STATIC_SLIP_MPS && requestedStaticMag <= muS * fn;

    // Phase — diagnostic only
    final BallPhase phaseState = classifyPhaseUnified(
      canStaticRoll: canStaticRoll,
      lateralForceRatio: lateralForceRatio,
      tractionY: state.tractionY,
    );
    final String phase = phaseLabel(phaseState);

    // Oil transfer
    final double pickup = OIL_PICKUP_RATE * oilLocal * (0.35 + 0.65 * flare) *
        (0.30 + 0.70 * state.tractionX.clamp(0.0, 1.0));
    final double deposit = OIL_DEPOSIT_RATE * state.ballOil.clamp(0.0, 1.0) *
        (0.25 + 0.75 * OIL_CARRY_BLEND);
    pickupOil(state.board, state.ft, pickup * dt * 8.0);
    depositOil(state.board, state.ft + 1.0 + 2.0 * state.vx / 8.5, deposit * dt * 8.0);

    // Axis rotation / tilt diagnostics
    double ar = 0.0, at = 0.0;
    if (omegaTot > 1e-6) {
      ar = atan2(state.omegaX.abs(), state.omegaY.abs()) * 180.0 / pi;
      at = asin((state.omegaZ.abs() / omegaTot).clamp(0.0, 1.0)) * 180.0 / pi;
    }

    // Breakpoint detection
    final double oilDrop = prevOil - oilLocal;
    final bool atBP = !inGutter && oilDrop > 0.05 && oilLocal < 0.18 &&
        state.tractionY > BREAKPOINT_TY_THRESH && state.ft > 25.0;

    prevOil = oilLocal;
    muAccum += 0.5 * (muX + muY);
    muCount++;
    segPhase = phase;

    path.add(PathPoint(
      ft: state.ft, board: state.board,
      vx: state.vx * 2.23694, omega: omegaTot,
      theta: state.theta, oil: oilLocal,
      mu: 0.5 * (muX + muY), AR: ar,
      slipRatio: slipRatio,
      tractionX: state.tractionX, tractionY: state.tractionY,
      muX: muX, muY: muY,
      lateralForceRatio: lateralForceRatio,
      kineticBlend: kineticBlend,
      slipX: slipX, slipY: slipY,
      phase: phase, atBreakpoint: atBP,
    ));

    if (state.ft - segStart >= 2.0 || state.ft >= 59.5) {
      segs.add(SegmentResult(
        seg: '${segStart.toStringAsFixed(0)}–${state.ft.toStringAsFixed(0)}',
        ft0: segStart, ft1: state.ft,
        mu: muCount > 0 ? muAccum / muCount : 0.5 * (muX + muY),
        vIn: segVin, vOut: state.vx * 2.23694,
        ARin: segARin, ARout: ar, ATin: segATin, ATout: at,
        dboards: segBoard - state.board, boardOut: state.board,
        totalRPM: omegaTot * 60.0 / (2.0 * pi), phase: segPhase,
      ));
      segStart = state.ft;
      segVin = state.vx * 2.23694;
      segBoard = state.board;
      segARin = ar; segATin = at;
      muAccum = 0.0; muCount = 0;
    }
  }

  final PathPoint? pinPoint = pointNearestFt(path, HEAD_PIN_FT);
  return SimResult(
    path: path, segments: segs,
    pinSpeed: pinPoint?.vx ?? 0.0,
    pinRPM: pinPoint != null ? pinPoint.omega * 60.0 / (2.0 * pi) : 0.0,
    pinBoard: pinPoint?.board ?? 0.0,
    pinAR: pinPoint?.AR ?? 0.0,
  );
}

// ═══════════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════════

SimResult runSimulationV2(BowlerInputs inp, PatternData pat, BallSpecs ball, OilMatrix oil) {
  return _runSimulationCore(
    inp: inp, ball: ball,
    sampleOil: (double board, double ft) => oil.sampleAt(board, ft),
    pickupOil: (double board, double ft, double amount) => oil.pickupAt(board, ft, amount),
    depositOil: (double board, double ft, double amount) => oil.depositAt(board, ft, amount),
  );
}