import 'dart:math';

// ═══════════════════════════════════════════════════════════
// BOWLING PHYSICS ENGINE — HYBRID KEEPER MATRIX
//
// Extracted keepers:
// - Spin state         : p11/p16 -> full vector (omegaX, omegaY, omegaZ)
// - Slip math          : p8/p17  -> stable contact slip with explicit handedness
// - Friction / shaping : p8/p17  -> tractionX / tractionY, muX / muY, lateralScale
// - Roll transition    : p8/p17  -> stable rolling handoff (WITH PERMANENT LATCH)
// - Phase classification: p7/p8/p17 -> skid / hook / roll by traction build + roll state
// - Spin update        : p7/p8/p17 for now -> simple contact torques, NO Euler coupling
// - Integration        : p17 -> RK4
//
// Notes:
// - This file intentionally keeps the p11-style full-vector state, but uses the
//   traction-family contact logic and phase labeling because those gave the best
//   visuals and the cleanest skid / hook / roll separation.
// - Euler cross-coupling terms are intentionally disabled for now.
// ═══════════════════════════════════════════════════════════

const double G_MS2 = 9.81;
const double BALL_R_M = 0.108; // length conveted to meters 0.358 * 0.3048;
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
const double HEAD_PIN_FT = 60.0;

const Map<String, double> GRIT_DRY_MU = {
  '500': 0.30,
  '1000': 0.23,
  '2000': 0.20,
  '3000': 0.15,
  '4000': 0.11,
  'polish': 0.09,
};

// Contact / friction tuning
const double MU_K_DRY_SCALE = 0.2;
const double MU_S_OIL_MIN = 0.024;
const double X_SKID_SLIP = 0.95;
const double X_ROLL_SLIP = 0.06;
const double Y_SKID_SLIP = 0.85;
const double Y_ROLL_SLIP = 0.04;
const double STRIBECK_A = 0.012;
const double STRIBECK_B = 0.010;
const double STRIBECK_C = 0.015;
const double STRIBECK_D = 15.0;
const double H_REF = 2.2e-6;
const double OIL_X_DROP = 0.10;
const double OIL_Y_DROP = 0.4;//increase for delayed hook
const double OIL_X_EXP = 1.15;
const double OIL_Y_EXP = 1.30;
const double TX_RISE = 1.0;
const double TX_FALL = 5.0;
const double TY_RISE = .7;//deecrease to delay hook
const double TY_FALL = 5.0;
const double ROLL_SLIP_THRESH = 0.06;//if the mph and rpm are 
const double ROLL_TX_THRESH = 0.013;
const double MIN_LATERAL_TRACTION = 0.03;
const double HOOK_PHASE_TY_THRESH = 0.25;
const double BREAKPOINT_TY_THRESH = 0.28;
const double OIL_PICKUP_RATE = 0.02;
const double OIL_DEPOSIT_RATE = 0.012;
const double OIL_CARRY_BLEND = 0.25;
const double FLARE_DIFF_GAIN = 0.90;
const double FLARE_ASY_GAIN = 0.45;

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
  }) => LoadRow(
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

class SegmentResult {
  final String seg;
  final double ft0, ft1, mu, vIn, vOut, ARin, ARout, ATin, ATout, dboards, boardOut, totalRPM;
  final String phase;

  const SegmentResult({
    required this.seg,
    required this.ft0,
    required this.ft1,
    required this.mu,
    required this.vIn,
    required this.vOut,
    required this.ARin,
    required this.ARout,
    required this.ATin,
    required this.ATout,
    required this.dboards,
    required this.boardOut,
    required this.totalRPM,
    required this.phase,
  });
}

class PathPoint {
  final double ft, board, vx, omega, theta, oil, mu, AR;
  final double slipRatio, tractionX, tractionY, muX, muY;
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
    required this.tractionY,
    required this.muX,
    required this.muY,
    required this.phase,
    required this.atBreakpoint,
  });
}

class SimResult {
  final List<PathPoint> path;
  final List<SegmentResult> segments;
  final double pinSpeed, pinRPM, pinBoard, pinAR;

  const SimResult({
    required this.path,
    required this.segments,
    required this.pinSpeed,
    required this.pinRPM,
    required this.pinBoard,
    required this.pinAR,
  });
}

PathPoint? pointNearestFt(List<PathPoint> path, double targetFt) {
  if (path.isEmpty) return null;
  PathPoint best = path.first;
  double bestErr = (best.ft - targetFt).abs();
  for (final p in path.skip(1)) {
    final err = (p.ft - targetFt).abs();
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
    final fwd = _interpolate(forwardOil, board, ft);
    final rev = _interpolate(reverseOil, board, ft);
    final total = fwd + rev;
    final eta = total > 1e-9
        ? (fwd * fwdViscosity + rev * revViscosity) / total
        : DEFAULT_OIL.viscosityPaS;

    return OilSample(
      totalOil: total.clamp(0.0, 1.0),
      forwardOil: fwd,
      reverseOil: rev,
      effectiveViscosity: eta,
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
    final double v00 = oil[b0][f0];
    final double v10 = oil[b1][f0];
    final double v01 = oil[b0][f1];
    final double v11 = oil[b1][f1];
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

  List<List<double>> get combined {
    return List.generate(
      BOARDS,
      (b) => List.generate(
        OIL_COLS,
        (c) => (forwardOil[b][c] + reverseOil[b][c]).clamp(0.0, 1.0),
      ),
    );
  }
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

double clamp01(double x) => x.clamp(0.0, 1.0);

double stribeckMuHat(double hHat, double muMin) {
  return muMin +
      STRIBECK_A / (hHat + STRIBECK_B) +
      STRIBECK_C * (1.0 - exp(-STRIBECK_D * hHat));
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

class _BallState {
  final double ft;
  final double board;
  final double vx;
  final double vy;
  final double omegaX;
  final double omegaY;
  final double omegaZ;
  final double theta;
  final double tractionX;
  final double tractionY;
  final double ballOil;
  final bool isRollLocked; // <-- NEW LATCH

  const _BallState({
    required this.ft,
    required this.board,
    required this.vx,
    required this.vy,
    required this.omegaX,
    required this.omegaY,
    required this.omegaZ,
    required this.theta,
    required this.tractionX,
    required this.tractionY,
    required this.ballOil,
    this.isRollLocked = false, // <-- DEFAULTS TO FALSE
  });

  _BallState addScaled(_BallDeriv k, double h) => _BallState(
        ft: ft + k.dFt * h,
        board: board + k.dBoard * h,
        vx: vx + k.dVx * h,
        vy: vy + k.dVy * h,
        omegaX: omegaX + k.dOmegaX * h,
        omegaY: omegaY + k.dOmegaY * h,
        omegaZ: omegaZ + k.dOmegaZ * h,
        theta: theta + k.dTheta * h,
        tractionX: tractionX + k.dTractionX * h,
        tractionY: tractionY + k.dTractionY * h,
        ballOil: ballOil + k.dBallOil * h,
        isRollLocked: isRollLocked, // <-- PASS LATCH THROUGH
      );

  _BallState clamp() => _BallState(
        ft: ft,
        board: board.clamp(-3.0, 43.0),
        vx: vx < 0.3 ? 0.3 : vx,
        vy: vy.abs() < 1e-8 ? 0.0 : vy,
        omegaX: omegaX.abs() < 1e-8 ? 0.0 : omegaX,
        omegaY: omegaY.abs() < 1e-8 ? 0.0 : omegaY,
        omegaZ: omegaZ.abs() < 1e-8 ? 0.0 : omegaZ,
        theta: theta,
        tractionX: tractionX.clamp(0.0, 1.0),
        tractionY: tractionY.clamp(0.0, 1.0),
        ballOil: ballOil.clamp(0.0, 1.0),
        isRollLocked: isRollLocked, // <-- PASS LATCH THROUGH
      );
}

class _BallDeriv {
  final double dFt;
  final double dBoard;
  final double dVx;
  final double dVy;
  final double dOmegaX;
  final double dOmegaY;
  final double dOmegaZ;
  final double dTheta;
  final double dTractionX;
  final double dTractionY;
  final double dBallOil;

  const _BallDeriv({
    required this.dFt,
    required this.dBoard,
    required this.dVx,
    required this.dVy,
    required this.dOmegaX,
    required this.dOmegaY,
    required this.dOmegaZ,
    required this.dTheta,
    required this.dTractionX,
    required this.dTractionY,
    required this.dBallOil,
  });
}

class _ContactEval {
  final double oilLocal;
  final double etaEff;
  final double omegaTot;
  final double axisTiltDeg;
  final double flare;
  final double slipRatio;
  final double ux;
  final double uy;
  final double muX;
  final double muY;
  final double tractionTargetX;
  final double tractionTargetY;
  final double lateralScale;
  final bool rolling;
  final double slipX;
  final double slipY;

  const _ContactEval({
    required this.oilLocal,
    required this.etaEff,
    required this.omegaTot,
    required this.axisTiltDeg,
    required this.flare,
    required this.slipRatio,
    required this.ux,
    required this.uy,
    required this.muX,
    required this.muY,
    required this.tractionTargetX,
    required this.tractionTargetY,
    required this.lateralScale,
    required this.rolling,
    required this.slipX,
    required this.slipY,
  });
}

typedef _OilSampleFn = OilSample Function(double board, double ft);
typedef _OilUpdateFn = void Function(double board, double ft, double amount);


enum BallPhase { skid, hook, roll }

BallPhase classifyPhase({
  required bool rolling,
  required double tractionY,
  double hookThreshold = HOOK_PHASE_TY_THRESH,
}) {
  if (rolling) return BallPhase.roll;
  if (tractionY > hookThreshold) return BallPhase.hook;
  return BallPhase.skid;
}

String phaseLabel(BallPhase phase) {
  switch (phase) {
    case BallPhase.roll:
      return 'roll';
    case BallPhase.hook:
      return 'hook';
    case BallPhase.skid:
      return 'skid';
  }
}

_ContactEval _evaluateContact({
  required _BallState s,
  required BallSpecs ball,
  required double massKg,
  required _OilSampleFn sampleOil,
  required bool inGutter,
  required double handedness,
}) {
  final double fn = massKg * G_MS2;
  final OilSample sample = sampleOil(s.board, s.ft);
  final double oilLocal = sample.totalOil;
  final double etaEff = sample.effectiveViscosity;

  // Keep p11 full vector state, but use p7/p17-style stable slip signs.
  // Forward roll lives mostly in omegaY, lateral / hook lives in omegaX.
  final double slipX = s.vx - BALL_R_M * s.omegaY;
  final double slipY = s.vy + handedness * BALL_R_M * s.omegaX;
  final double slipMag = sqrt(slipX * slipX + slipY * slipY);
  final double vxRef = s.vx.clamp(0.3, 30.0);
  final double slipRatio = slipMag / vxRef;
  final double ux = slipMag > 1e-8 ? slipX / slipMag : 0.0;
  final double uy = slipMag > 1e-8 ? slipY / slipMag : 0.0;

  final double omegaTot = sqrt(s.omegaX * s.omegaX + s.omegaY * s.omegaY + s.omegaZ * s.omegaZ);
  final double axisTiltDeg = omegaTot > 1e-6
      ? asin((s.omegaZ.abs() / omegaTot).clamp(0.0, 1.0)) * 180.0 / pi
      : 0.0;
  final double flare = flareExposure(ball, omegaTot, axisTiltDeg);

  final double muMax = ball.muDry * (1.0 + 0.12 * flare);
  final double muMin = MU_OIL_MIN;
  final double pEff = fn / CONTACT_AREA_M2;
  final double film = oilLocal.clamp(0.0, 1.0);
  final double hersey = pEff > 1e-6 ? film * (etaEff * slipMag) / pEff : 0.0;
  final double hHat = hersey / H_REF;
  final double muBaseRaw = stribeckMuHat(hHat, muMin);
  final double muBase = muBaseRaw.clamp(MU_ROLL_RES, muMax);

  final double muK = max(MU_ROLL_RES, min(muBase * MU_K_DRY_SCALE, ball.muDry));
  final double muS = max(muK, min(ball.muDry, max(MU_S_OIL_MIN, muBase)));

  final double slipBlendX = blendBySlip(slipRatio, X_SKID_SLIP, X_ROLL_SLIP);
  final double slipBlendY = hookBlendBySlip(slipRatio, Y_ROLL_SLIP, Y_SKID_SLIP);
  final double oilGripX = oilGripFactor(oilLocal, OIL_X_DROP, OIL_X_EXP);
  final double oilGripY = oilGripFactor(oilLocal, OIL_Y_DROP, OIL_Y_EXP);

  final double tractionTargetX = slipBlendX * oilGripX;
  final double tractionTargetY = slipBlendY * oilGripY;
  final double tractionX = s.tractionX.clamp(0.0, 1.0);
  final double tractionY = s.tractionY.clamp(0.0, 1.0);
  final double muX = muK * (1.0 - tractionX) + muS * tractionX;
  final double muY = muK * (1.0 - tractionY) + muS * tractionY;
  final double lateralScale = MIN_LATERAL_TRACTION + (1.0 - MIN_LATERAL_TRACTION) * tractionY;

  // <-- LATCH EVALUATED HERE -->
  final bool rolling = !inGutter && (s.isRollLocked || (slipRatio < ROLL_SLIP_THRESH && tractionX > ROLL_TX_THRESH));

  return _ContactEval(
    oilLocal: oilLocal,
    etaEff: etaEff,
    omegaTot: omegaTot,
    axisTiltDeg: axisTiltDeg,
    flare: flare,
    slipRatio: slipRatio,
    ux: ux,
    uy: uy,
    muX: muX,
    muY: muY,
    tractionTargetX: tractionTargetX,
    tractionTargetY: tractionTargetY,
    lateralScale: lateralScale,
    rolling: rolling,
    slipX: slipX,
    slipY: slipY,
  );
}

_BallDeriv _evalDeriv({
  required _BallState s,
  required BallSpecs ball,
  required double massKg,
  required InertiaTensor inertia,
  required _OilSampleFn sampleOil,
  required bool inGutter,
  required double dt,
  required double rollHeadingRad,
  required double handedness,
}) {
  final _ContactEval c = _evaluateContact(
    s: s,
    ball: ball,
    massKg: massKg,
    sampleOil: sampleOil,
    inGutter: inGutter,
    handedness: handedness,
  );
  final double fn = massKg * G_MS2;

  double dVx = 0.0;
  double dVy = 0.0;
  double dOmegaX = 0.0;
  double dOmegaY = 0.0;
  double dOmegaZ = 0.0;

  if (inGutter) {
    final double decel = MU_ROLL_RES * fn / massKg;
    dVx = -decel;
    dVy = log(0.88) / dt * s.vy;
    final double targetOmegaY = s.vx / BALL_R_M;
    dOmegaY = (targetOmegaY - s.omegaY) * 40.0;
    dOmegaX = log(0.85) / dt * s.omegaX;
    dOmegaZ = log(0.95) / dt * s.omegaZ;
  } else if (c.rolling) {
    final double speedMag = sqrt(s.vx * s.vx + s.vy * s.vy).clamp(0.3, 30.0);
    final double decel = MU_ROLL_RES * fn / massKg;
    final double dSpeed = speedMag <= 0.31 ? 0.0 : -decel;
    
    dVx = dSpeed * cos(rollHeadingRad);
    dVy = dSpeed * sin(rollHeadingRad);
    
    // Calculate the perfect "no-slip" spin on BOTH axes based on the ball's actual angle
    final double targetOmegaY = s.vx / BALL_R_M;
    final double targetOmegaX = -s.vy / (handedness * BALL_R_M);
    
    // Apply static friction "spring" to force the spin to perfectly track the velocity on both axes.
    // The feed-forward term (dSpeed...) keeps them perfectly in sync as the ball slows down.
    dOmegaY = (targetOmegaY - s.omegaY) * 30.0 + ((dSpeed * cos(rollHeadingRad)) / BALL_R_M);
    dOmegaX = (targetOmegaX - s.omegaX) * 30.0 - ((dSpeed * sin(rollHeadingRad)) / (handedness * BALL_R_M));
    
    // Tilt (Z-axis spin) doesn't cause floor slip, so we can still smoothly kill it
    dOmegaZ = log(0.95) / dt * s.omegaZ; 
    } else {
    final double fx = -c.muX * fn * c.ux;
    final double fy = -c.muY * fn * c.uy * c.lateralScale;

    dVx = fx / massKg;
    dVy = fy / massKg;

    // Keep p7/p17 simple torque update for stability: no Euler coupling yet.
    dOmegaX = (BALL_R_M * fy) / inertia.ix;
    dOmegaY = (-BALL_R_M * fx) / inertia.iy;
    dOmegaZ = -s.omegaZ * (0.018 + 0.045 * s.tractionY.clamp(0.0, 1.0)) * 60.0;
  }

  final double pickup = OIL_PICKUP_RATE * c.oilLocal * (0.35 + 0.65 * c.flare) *
      (0.30 + 0.70 * s.tractionX.clamp(0.0, 1.0));
  final double deposit = OIL_DEPOSIT_RATE * s.ballOil.clamp(0.0, 1.0) *
      (0.25 + 0.75 * OIL_CARRY_BLEND);

  return _BallDeriv(
    dFt: s.vx / 0.3048,
    dBoard: s.vy / BOARD_M,
    dVx: dVx,
    dVy: dVy,
    dOmegaX: dOmegaX,
    dOmegaY: dOmegaY,
    dOmegaZ: dOmegaZ,
    dTheta: c.omegaTot,
    dTractionX: stateRate(s.tractionX, c.tractionTargetX, TX_RISE, TX_FALL),
    dTractionY: stateRate(s.tractionY, c.tractionTargetY, TY_RISE, TY_FALL),
    dBallOil: pickup - deposit,
  );
}

_BallState _rk4Step({
  required _BallState s,
  required double dt,
  required BallSpecs ball,
  required double massKg,
  required InertiaTensor inertia,
  required _OilSampleFn sampleOil,
  required bool inGutter,
  required double rollHeadingRad,
  required double handedness,
}) {
  final _BallDeriv k1 = _evalDeriv(
    s: s,
    ball: ball,
    massKg: massKg,
    inertia: inertia,
    sampleOil: sampleOil,
    inGutter: inGutter,
    dt: dt,
    rollHeadingRad: rollHeadingRad,
    handedness: handedness,
  );
  final _BallDeriv k2 = _evalDeriv(
    s: s.addScaled(k1, dt * 0.5),
    ball: ball,
    massKg: massKg,
    inertia: inertia,
    sampleOil: sampleOil,
    inGutter: inGutter,
    dt: dt,
    rollHeadingRad: rollHeadingRad,
    handedness: handedness,
  );
  final _BallDeriv k3 = _evalDeriv(
    s: s.addScaled(k2, dt * 0.5),
    ball: ball,
    massKg: massKg,
    inertia: inertia,
    sampleOil: sampleOil,
    inGutter: inGutter,
    dt: dt,
    rollHeadingRad: rollHeadingRad,
    handedness: handedness,
  );
  final _BallDeriv k4 = _evalDeriv(
    s: s.addScaled(k3, dt),
    ball: ball,
    massKg: massKg,
    inertia: inertia,
    sampleOil: sampleOil,
    inGutter: inGutter,
    dt: dt,
    rollHeadingRad: rollHeadingRad,
    handedness: handedness,
  );

  return _BallState(
    ft: s.ft + dt * (k1.dFt + 2.0 * k2.dFt + 2.0 * k3.dFt + k4.dFt) / 6.0,
    board: s.board + dt * (k1.dBoard + 2.0 * k2.dBoard + 2.0 * k3.dBoard + k4.dBoard) / 6.0,
    vx: s.vx + dt * (k1.dVx + 2.0 * k2.dVx + 2.0 * k3.dVx + k4.dVx) / 6.0,
    vy: s.vy + dt * (k1.dVy + 2.0 * k2.dVy + 2.0 * k3.dVy + k4.dVy) / 6.0,
    omegaX: s.omegaX + dt * (k1.dOmegaX + 2.0 * k2.dOmegaX + 2.0 * k3.dOmegaX + k4.dOmegaX) / 6.0,
    omegaY: s.omegaY + dt * (k1.dOmegaY + 2.0 * k2.dOmegaY + 2.0 * k3.dOmegaY + k4.dOmegaY) / 6.0,
    omegaZ: s.omegaZ + dt * (k1.dOmegaZ + 2.0 * k2.dOmegaZ + 2.0 * k3.dOmegaZ + k4.dOmegaZ) / 6.0,
    theta: s.theta + dt * (k1.dTheta + 2.0 * k2.dTheta + 2.0 * k3.dTheta + k4.dTheta) / 6.0,
    tractionX: s.tractionX + dt * (k1.dTractionX + 2.0 * k2.dTractionX + 2.0 * k3.dTractionX + k4.dTractionX) / 6.0,
    tractionY: s.tractionY + dt * (k1.dTractionY + 2.0 * k2.dTractionY + 2.0 * k3.dTractionY + k4.dTractionY) / 6.0,
    ballOil: s.ballOil + dt * (k1.dBallOil + 2.0 * k2.dBallOil + 2.0 * k3.dBallOil + k4.dBallOil) / 6.0,
    isRollLocked: s.isRollLocked, // <-- PASS LATCH THROUGH
  ).clamp();
}

SimResult _runSimulationCore({
  required BowlerInputs inp,
  required BallSpecs ball,
  required _OilSampleFn sampleOil,
  required _OilUpdateFn pickupOil,
  required _OilUpdateFn depositOil,
  double hookThreshold = HOOK_PHASE_TY_THRESH,
}) {
  final double massKg = ball.massKg;
  final InertiaTensor inertia = ball.inertiaTensor;
  final double angleRad = inp.effectiveAngleDeg * pi / 180.0;
  final double speed = inp.speedMph * 0.44704;
  final double omega0 = inp.revRPM * 2.0 * pi / 60.0;
  final double psi = inp.axisRotation * pi / 180.0 * inp.handedness;
  final double phi = inp.axisTilt * pi / 180.0;

  _BallState state = _BallState(
    ft: inp.effectiveLandingDistanceFt.clamp(0.5, 15.0),
    board: inp.effectiveLandBoard.clamp(1.0, 39.0),
    vx: speed * cos(angleRad),
    vy: speed * sin(angleRad),
    omegaX: omega0 * sin(psi) * cos(phi),
    omegaY: max(0.0, omega0 * cos(psi) * cos(phi)),
    omegaZ: omega0 * sin(phi),
    theta: -pi / 2.0,
    tractionX: 0.0,
    tractionY: 0.0,
    ballOil: 0.0,
    isRollLocked: false,
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
  bool hasHooked = false;
  double rollHeadingRad = atan2(state.vy, state.vx);

  while (state.ft < LANE_FT.toDouble() && state.vx > 0.3) {
    final _ContactEval pre = _evaluateContact(
      s: state,
      ball: ball,
      massKg: massKg,
      sampleOil: sampleOil,
      inGutter: inGutter,
      handedness: inp.handedness,
    );

// Keep track of when the ball officially starts hooking
    if (state.tractionY > hookThreshold) {
      hasHooked = true;
    }

    // <-- TRIGGER THE LATCH -->
    // Lock into a roll if the slip mathematical threshold is met, 
    // OR if the lateral hook has naturally finished resolving!
    if (!inGutter && !state.isRollLocked) {
      if (pre.rolling || (hasHooked && state.tractionY < hookThreshold)) {
        rollHeadingRad = atan2(state.vy, state.vx);
        state = _BallState(
          ft: state.ft, board: state.board, vx: state.vx, vy: state.vy,
          omegaX: state.omegaX, omegaY: state.omegaY, omegaZ: state.omegaZ,
          theta: state.theta, tractionX: state.tractionX, tractionY: state.tractionY,
          ballOil: state.ballOil, 
          isRollLocked: true, // <-- Latch fires!
        );
      }
    }

    state = _rk4Step(
      s: state,
      dt: dt,
      ball: ball,
      massKg: massKg,
      inertia: inertia,
      sampleOil: sampleOil,
      inGutter: inGutter,
      rollHeadingRad: rollHeadingRad,
      handedness: inp.handedness,
    );

    if (!inGutter && (state.board < 0.5 || state.board > 39.5)) {
      inGutter = true;
    }
    state = state.clamp();

    final _ContactEval post = _evaluateContact(
      s: state,
      ball: ball,
      massKg: massKg,
      sampleOil: sampleOil,
      inGutter: inGutter,
      handedness: inp.handedness,
    );

    final double pickup = OIL_PICKUP_RATE * post.oilLocal * (0.35 + 0.65 * post.flare) *
        (0.30 + 0.70 * state.tractionX.clamp(0.0, 1.0));
    final double deposit = OIL_DEPOSIT_RATE * state.ballOil.clamp(0.0, 1.0) *
        (0.25 + 0.75 * OIL_CARRY_BLEND);
    pickupOil(state.board, state.ft, pickup * dt * 8.0);
    depositOil(state.board, state.ft + 1.0 + 2.0 * state.vx / 8.5, deposit * dt * 8.0);

    double ar = 0.0;
    double at = 0.0;
    if (post.omegaTot > 1e-6) {
      ar = atan2(state.omegaX.abs(), state.omegaY.abs()) * 180.0 / pi;
      at = asin((state.omegaZ.abs() / post.omegaTot).clamp(0.0, 1.0)) * 180.0 / pi;
    }

    final BallPhase phaseState = classifyPhase(
      rolling: post.rolling,
      tractionY: state.tractionY,
      hookThreshold: hookThreshold,
    );
    final String phase = phaseLabel(phaseState);

    final double oilDrop = prevOil - post.oilLocal;
    final bool atBP = !inGutter &&
        oilDrop > 0.05 &&
        post.oilLocal < 0.18 &&
        state.tractionY > BREAKPOINT_TY_THRESH &&
        state.ft > 25.0;

    prevOil = post.oilLocal;
    muAccum += 0.5 * (post.muX + post.muY);
    muCount++;
    segPhase = phase;

    path.add(PathPoint(
      ft: state.ft,
      board: state.board,
      vx: state.vx * 2.23694,
      omega: post.omegaTot,
      theta: state.theta,
      oil: post.oilLocal,
      mu: 0.5 * (post.muX + post.muY),
      AR: ar,
      slipRatio: post.slipRatio,
      tractionX: state.tractionX,
      tractionY: state.tractionY,
      muX: post.muX,
      muY: post.muY,
      phase: phase,
      atBreakpoint: atBP,
    ));

    if (state.ft - segStart >= 2.0 || state.ft >= 59.5) {
      segs.add(SegmentResult(
        seg: '${segStart.toStringAsFixed(0)}–${state.ft.toStringAsFixed(0)}',
        ft0: segStart,
        ft1: state.ft,
        mu: muCount > 0 ? muAccum / muCount : 0.5 * (post.muX + post.muY),
        vIn: segVin,
        vOut: state.vx * 2.23694,
        ARin: segARin,
        ARout: ar,
        ATin: segATin,
        ATout: at,
        dboards: segBoard - state.board,
        boardOut: state.board,
        totalRPM: post.omegaTot * 60.0 / (2.0 * pi),
        phase: segPhase,
      ));
      segStart = state.ft;
      segVin = state.vx * 2.23694;
      segBoard = state.board;
      segARin = ar;
      segATin = at;
      muAccum = 0.0;
      muCount = 0;
    }
  }

  final PathPoint? pinPoint = pointNearestFt(path, HEAD_PIN_FT);
  return SimResult(
    path: path,
    segments: segs,
    pinSpeed: pinPoint?.vx ?? 0.0,
    pinRPM: pinPoint != null ? pinPoint.omega * 60.0 / (2.0 * pi) : 0.0,
    pinBoard: pinPoint?.board ?? 0.0,
    pinAR: pinPoint?.AR ?? 0.0,
  );
}

SimResult runSimulationV2(BowlerInputs inp, PatternData pat, BallSpecs ball, OilMatrix oil) {
  return _runSimulationCore(
    inp: inp,
    ball: ball,
    sampleOil: (double board, double ft) => oil.sampleAt(board, ft),
    pickupOil: (double board, double ft, double amount) => oil.pickupAt(board, ft, amount),
    depositOil: (double board, double ft, double amount) => oil.depositAt(board, ft, amount),
    hookThreshold: 0.18,
  );
}
