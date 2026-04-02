//original version no buff rename to physics.dart to use. 
import 'dart:math';

// ═══════════════════════════════════════════════════════════
// BOWLING PHYSICS ENGINE v18
// Stateful contact model:
// - separate forward and lateral traction states
// - kinetic/static friction blend from oil + slip
// - traction memory (builds and falls over time)
// - partial re-skid when the ball re-enters oil
// - simple oil pickup / deposit for lane transition
// Keeps the same public API as v14.
// ═══════════════════════════════════════════════════════════

const double G_MS2    = 9.81;
const double BALL_R_M = 0.358 * 0.3048;
const double BOARD_M  = (1.0625 / 12.0) * 0.3048;
const int    BOARDS   = 39;
const int    LANE_FT  = 60;
const double MU_OIL_MIN  = 0.020;
const double MU_ROLL_RES = 0.005;
const double RG_REF_IN   = 2.53;
const double DIFF_REF_IN = 0.040;
const double ASY_THRESH  = 0.013;

const Map<String, double> GRIT_DRY_MU = {
  '500':0.30,'1000':0.23,'2000':0.20,'3000':0.15,'4000':0.11,'polish':0.09,
};

// Contact model constants
const double MU_K_DRY_SCALE = 0.86;
const double MU_S_OIL_MIN   = 0.024;

// Slip thresholds for traction readiness.
// Forward traction can build earlier than lateral traction.
const double X_SKID_SLIP = 0.95;
const double X_ROLL_SLIP = 0.08;
const double Y_SKID_SLIP = 0.70;
const double Y_ROLL_SLIP = 0.04;

// Oil suppresses usable traction. Lateral traction is more sensitive.
const double OIL_X_DROP = 0.35;
const double OIL_Y_DROP = 0.62;
const double OIL_X_EXP  = 1.10;
const double OIL_Y_EXP  = 1.25;

// Traction-state response rates (per second)
const double TX_RISE = 5.0;
const double TX_FALL = 7.0;
const double TY_RISE = 2.4;
const double TY_FALL = 6.5;

// Never let lateral bite fully vanish; weak early traction still exists.
const double MIN_LATERAL_TRACTION = 0.05;

// Oil transfer / carrydown approximation
const double OIL_PICKUP_RATE  = 0.030;
const double OIL_DEPOSIT_RATE = 0.018;
const double OIL_CARRY_BLEND  = 0.35;

// Fresh-cover / flare exposure approximation
const double FLARE_DIFF_GAIN = 0.90;
const double FLARE_ASY_GAIN  = 0.45;

class BallSpecs {
  double rg; double diff; double asy; String grit; double masslb;
  BallSpecs({this.rg=2.54,this.diff=0.040,this.asy=0.0,this.grit='2000',this.masslb=15.0});
  bool   get isAsym   => asy >= ASY_THRESH;
  double get massKg   => masslb * 0.453592;
  double get rgM      => (rg / 12.0) * 0.3048;
  double get moiKgM2  => massKg * rgM * rgM;
  double get muDry    => GRIT_DRY_MU[grit] ?? 0.20;
  double get rgFactor   => RG_REF_IN / rg;
  double get diffFactor => diff / DIFF_REF_IN;
  double get asymFactor => isAsym ? 1.0 + 0.25*((asy-ASY_THRESH)/0.017).clamp(0.0,1.0) : 1.0;
  double hookBias(double k0) => (k0 * rgFactor * diffFactor * asymFactor).clamp(0.85, 1.25);
}

class BowlerInputs {
  double speedMph;
  double revRPM;
  double angleDeg;
  double axisTilt;
  double axisRotation;
  double hookK0;
  double releaseBoard;       // touchdown board
  double landingDistanceFt;  // touchdown distance
  bool useReleaseMode;
  double handedness;
  BowlerInputs({
    this.speedMph = 19.0,
    this.revRPM = 300.0,
    this.angleDeg = 0.0,
    this.axisTilt = 15.0,
    this.axisRotation = 45.0,
    this.hookK0 = 1.0,
    this.releaseBoard = 17.0,
    this.landingDistanceFt = 7.0,
    this.useReleaseMode = true,
    this.handedness = 1.0,
  });
  double get effectiveAngleDeg => angleDeg;
  double get effectiveLandBoard => releaseBoard;
  double get effectiveLandingDistanceFt => landingDistanceFt;
}

class LoadRow {
  final int sl, sr, loads, mics, buff;
  final double d0, d1, toil;
  const LoadRow({required this.sl,required this.sr,required this.loads,
    required this.mics,required this.buff,required this.d0,required this.d1,required this.toil});
  LoadRow copyWith({int? sl,int? sr,int? loads,int? mics,int? buff,double? d0,double? d1,double? toil}) =>
    LoadRow(sl:sl??this.sl,sr:sr??this.sr,loads:loads??this.loads,mics:mics??this.mics,
      buff:buff??this.buff,d0:d0??this.d0,d1:d1??this.d1,toil:toil??this.toil);
}

class PatternData {
  String name; double distance; List<LoadRow> fwdRows; List<LoadRow> revRows;
  PatternData({required this.name,required this.distance,required this.fwdRows,required this.revRows});
  static PatternData masters2026() => PatternData(name:'2026 USBC Masters',distance:41,
    fwdRows:[
      LoadRow(sl:2,sr:2,loads:5,mics:50,buff:500,d0:0,d1:8,toil:9250),
      LoadRow(sl:3,sr:4,loads:1,mics:45,buff:500,d0:8,d1:10,toil:1530),
      LoadRow(sl:5,sr:5,loads:2,mics:45,buff:500,d0:10,d1:14,toil:2790),
      LoadRow(sl:6,sr:6,loads:3,mics:50,buff:500,d0:14,d1:20,toil:4350),
      LoadRow(sl:2,sr:2,loads:1,mics:50,buff:500,d0:20,d1:22,toil:1850),
      LoadRow(sl:7,sr:7,loads:2,mics:50,buff:500,d0:22,d1:27,toil:2700),
      LoadRow(sl:2,sr:2,loads:0,mics:50,buff:500,d0:27,d1:32,toil:0),
      LoadRow(sl:2,sr:2,loads:0,mics:50,buff:350,d0:32,d1:38,toil:0),
      LoadRow(sl:2,sr:2,loads:0,mics:50,buff:150,d0:38,d1:41,toil:0),
    ],
    revRows:[
      LoadRow(sl:2,sr:2,loads:0,mics:50,buff:500,d0:39,d1:28,toil:0),
      LoadRow(sl:11,sr:11,loads:2,mics:50,buff:500,d0:28,d1:23,toil:1900),
      LoadRow(sl:9,sr:9,loads:2,mics:50,buff:500,d0:23,d1:18,toil:2300),
      LoadRow(sl:7,sr:7,loads:2,mics:50,buff:500,d0:18,d1:14,toil:2700),
      LoadRow(sl:6,sr:6,loads:1,mics:50,buff:500,d0:14,d1:12,toil:1450),
      LoadRow(sl:4,sr:5,loads:1,mics:45,buff:500,d0:12,d1:10,toil:1440),
      LoadRow(sl:2,sr:2,loads:2,mics:45,buff:500,d0:10,d1:6,toil:3330),
      LoadRow(sl:2,sr:2,loads:0,mics:50,buff:500,d0:6,d1:0,toil:0),
    ]);
}

class SegmentResult {
  final String seg;
  final double ft0,ft1,mu,vIn,vOut,ARin,ARout,ATin,ATout,dboards,boardOut,totalRPM;
  final String phase;
  const SegmentResult({required this.seg,required this.ft0,required this.ft1,
    required this.mu,required this.vIn,required this.vOut,
    required this.ARin,required this.ARout,required this.ATin,required this.ATout,
    required this.dboards,required this.boardOut,required this.totalRPM,required this.phase});
}

class PathPoint {
  final double ft,board,vx,omega,theta,oil,mu,AR;
  final String phase; final bool atBreakpoint;
  const PathPoint({required this.ft,required this.board,required this.vx,
    required this.omega,required this.theta,required this.oil,required this.mu,
    required this.AR,required this.phase,required this.atBreakpoint});
}

class SimResult {
  final List<PathPoint> path; final List<SegmentResult> segments;
  final double pinSpeed,pinRPM,pinBoard,pinAR;
  const SimResult({required this.path,required this.segments,
    required this.pinSpeed,required this.pinRPM,required this.pinBoard,required this.pinAR});
}

List<List<double>> buildOilMatrix(List<LoadRow> fwdRows, List<LoadRow> revRows) {
  final raw = List.generate(BOARDS, (_) => List<double>.filled(LANE_FT, 0.0));
  void applyRows(List<LoadRow> rows) {
    for (final row in rows) {
      if (row.toil==0||row.loads==0) continue;
      final start=row.sl+1; final end=39-row.sr;
      final ft0=min(row.d0,row.d1).toInt(); final ft1=max(row.d0,row.d1).toInt();
      final nb=end-start+1; final ftS=ft1-ft0;
      if (nb<=0||ftS<=0) continue;
      final opbf=row.toil/(nb*ftS);
      for (int b=start-1;b<=end-1&&b<BOARDS;b++) {
        for (int f=ft0;f<ft1&&f<LANE_FT;f++) {
          raw[b][f]+=opbf;
        }
      }
    }
  }
  applyRows(fwdRows); applyRows(revRows);
  double maxV=0;
  for (int b=0;b<BOARDS;b++) {
    for (int f=0;f<LANE_FT;f++) {
      if(raw[b][f]>maxV) maxV=raw[b][f];
    }
  }
  if (maxV==0) return raw;
  return List.generate(BOARDS,(b)=>List.generate(LANE_FT,(f)=>(raw[b][f]/maxV).clamp(0.0,1.0)));
}

double oilAt2D(List<List<double>> oil, double board, double ft) {
  if (oil.isEmpty) return 0.0;
  final double bIdx=(board-1.0).clamp(0.0,(BOARDS-1).toDouble());
  final double fIdx=ft.clamp(0.0,(LANE_FT-1).toDouble());
  final int b0=bIdx.floor().clamp(0,BOARDS-1); final int b1=(b0+1).clamp(0,BOARDS-1);
  final int f0=fIdx.floor().clamp(0,LANE_FT-1); final int f1=(f0+1).clamp(0,LANE_FT-1);
  final double tb=bIdx-b0; final double tf=fIdx-f0;
  final double v00=oil[b0][f0]; final double v10=b1<oil.length?oil[b1][f0]:v00;
  final double v01=oil[b0][f1]; final double v11=b1<oil.length?oil[b1][f1]:v01;
  return ((v00*(1-tb)+v10*tb)*(1-tf)+(v01*(1-tb)+v11*tb)*tf).clamp(0.0,1.0);
}

double clamp01(double x) => x.clamp(0.0, 1.0);

double blendBySlip(double slipRatio, double skidSlip, double rollSlip) {
  return ((skidSlip - slipRatio) / (skidSlip - rollSlip)).clamp(0.0, 1.0);
}

double evolveState(double current, double target, double riseRate, double fallRate, double dt) {
  final double rate = target >= current ? riseRate : fallRate;
  final double k = 1.0 - exp(-rate * dt);
  return current + (target - current) * k;
}

double oilGripFactor(double oil, double drop, double expP) {
  return clamp01(1.0 - drop * pow(oil, expP).toDouble());
}

double flareExposure(BallSpecs ball, double omegaTot, double axisTiltDeg) {
  final double diffNorm = (ball.diff / 0.060).clamp(0.0, 1.0);
  final double asyNorm  = (ball.asy / 0.030).clamp(0.0, 1.0);
  final double rpmNorm  = ((omegaTot * 60.0 / (2.0 * pi)) / 500.0).clamp(0.0, 1.0);
  final double tiltSupp = 1.0 - (axisTiltDeg / 90.0).clamp(0.0, 1.0) * 0.35;
  return clamp01(
    diffNorm * FLARE_DIFF_GAIN +
    asyNorm  * FLARE_ASY_GAIN +
    rpmNorm  * 0.20,
  ) * tiltSupp;
}

void depositOilAt2D(List<List<double>> oil, double board, double ft, double amount) {
  if (oil.isEmpty || amount <= 0) return;
  final int b = (board - 1.0).round().clamp(0, BOARDS - 1);
  final int f = ft.round().clamp(0, LANE_FT - 1);
  oil[b][f] = (oil[b][f] + amount).clamp(0.0, 1.0);
}

void pickupOilAt2D(List<List<double>> oil, double board, double ft, double amount) {
  if (oil.isEmpty || amount <= 0) return;
  final int b = (board - 1.0).round().clamp(0, BOARDS - 1);
  final int f = ft.round().clamp(0, LANE_FT - 1);
  oil[b][f] = (oil[b][f] - amount).clamp(0.0, 1.0);
}

SimResult runSimulation(BowlerInputs inp, PatternData pat, BallSpecs ball, List<List<double>> oilMatrix) {
  final double massKg = ball.massKg;
  final double I      = ball.moiKgM2;
  final double bias   = ball.hookBias(inp.hookK0);
  final double If     = I;
  final double Ih     = I / bias;
  final double h      = inp.handedness;

  final double angleRad  = inp.effectiveAngleDeg * pi / 180.0;
  final double speed     = inp.speedMph * 0.44704;

  double vx = speed * cos(angleRad);
  double vy = 0.20 * speed * sin(angleRad);

  final double omega0 = inp.revRPM * 2.0 * pi / 60.0;
  final double ar0    = inp.axisRotation * pi / 180.0;
  final double at0    = inp.axisTilt * pi / 180.0;

  double omegaF = omega0 * cos(ar0) * cos(at0);
  double omegaH = omega0 * sin(ar0) * cos(at0);
  double omegaT = omega0 * sin(at0);

  double ft = inp.effectiveLandingDistanceFt.clamp(0.5, 15.0);
  double board = inp.effectiveLandBoard.clamp(1.0, 39.0);
  double theta = -pi/2.0;
  const double STEP_FT = 0.25;
  final double STEP_M = STEP_FT * 0.3048;

  // Stateful contact variables
  double tractionX = 0.0;
  double tractionY = 0.0;
  double ballOil   = 0.0;

  final List<PathPoint> path = [];
  final List<SegmentResult> segs = [];
  double segStart = ft, segVin = vx * 2.23694, segBoard = board;
  double segARin = inp.axisRotation, segATin = inp.axisTilt;
  double muAccum = 0.0;
  int muCount = 0;
  String segPhase = 'skid';
  double prevOil = oilAt2D(oilMatrix, board, ft);
  bool inGutter = false;

  while (ft < 60.0 && vx > 0.3) {
    final double oilLocal = oilAt2D(oilMatrix, board, ft);
    final double dt = STEP_M / vx.clamp(0.3, 30.0);
    final double Fn = massKg * G_MS2;

    // Contact slip
    final double slipX = vx - BALL_R_M * omegaF;
    final double slipY = vy + h * BALL_R_M * omegaH;
    final double slipMag = sqrt(slipX * slipX + slipY * slipY);
    final double slipRatio = slipMag / vx.clamp(0.3, 30.0);
    final double ux = slipMag > 1e-8 ? slipX / slipMag : 0.0;
    final double uy = slipMag > 1e-8 ? slipY / slipMag : 0.0;

    // Spin-derived state
    final double omegaTot = sqrt(omegaF * omegaF + omegaH * omegaH + omegaT * omegaT);
    final double axisTiltDeg = omegaTot > 1e-6
        ? asin((omegaT.abs() / omegaTot).clamp(0.0, 1.0)) * 180.0 / pi
        : 0.0;
    final double flare = flareExposure(ball, omegaTot, axisTiltDeg);

    // Oil sets friction ceilings; flare slightly enhances fresh-cover engagement.
    final double muKDryEff = ball.muDry * MU_K_DRY_SCALE * (1.0 + 0.18 * flare);
    final double muSDryEff = ball.muDry * (1.0 + 0.12 * flare);

    final double muK = MU_OIL_MIN + (muKDryEff - MU_OIL_MIN) * pow(1.0 - oilLocal, 1.10).toDouble();
    final double muS = MU_S_OIL_MIN + (muSDryEff - MU_S_OIL_MIN) * pow(1.0 - oilLocal, 1.25).toDouble();

    // Slip controls whether the ball is ready to grip; oil controls how much grip survives.
    final double slipBlendX = blendBySlip(slipRatio, X_SKID_SLIP, X_ROLL_SLIP);
    final double slipBlendY = blendBySlip(slipRatio, Y_SKID_SLIP, Y_ROLL_SLIP);
    final double oilGripX = oilGripFactor(oilLocal, OIL_X_DROP, OIL_X_EXP);
    final double oilGripY = oilGripFactor(oilLocal, OIL_Y_DROP, OIL_Y_EXP);

    final double tractionTargetX = slipBlendX * oilGripX;
    final double tractionTargetY = slipBlendY * oilGripY;

    tractionX = evolveState(tractionX, tractionTargetX, TX_RISE, TX_FALL, dt);
    tractionY = evolveState(tractionY, tractionTargetY, TY_RISE, TY_FALL, dt);

    final double muX = muK * (1.0 - tractionX) + muS * tractionX;
    final double muY = muK * (1.0 - tractionY) + muS * tractionY;
    final double lateralScale = MIN_LATERAL_TRACTION + (1.0 - MIN_LATERAL_TRACTION) * tractionY;

    double Fx = 0.0;
    double Fy = 0.0;

    if (inGutter) {
      final double decel = MU_ROLL_RES * Fn / massKg;
      vx = max(0.3, vx - decel * dt);
      vy *= 0.88;
      omegaF = vx / BALL_R_M;
      omegaH *= 0.85;
      omegaT *= 0.95;
    } else if (slipMag > 1e-8) {
      Fx = -muX * Fn * ux;
      Fy = -muY * Fn * uy * lateralScale;

      vx = max(0.3, vx + (Fx / massKg) * dt);
      vy = vy + (Fy / massKg) * dt;

      final double alphaF = -(BALL_R_M * Fx) / If;
      final double alphaH =  (h * BALL_R_M * Fy) / Ih;
      omegaF += alphaF * dt;
      omegaH += alphaH * dt;

      omegaF = omegaF.clamp(0.0, vx / BALL_R_M + 8.0);
      omegaH = omegaH.clamp(-omega0, omega0);

      // Tilt decays more when side traction develops.
      omegaT *= (1.0 - (0.018 + 0.045 * tractionY) * dt * 60.0).clamp(0.0, 1.0);
    } else {
      omegaF = vx / BALL_R_M;
      omegaH *= (1.0 - 0.04 * dt * 60.0).clamp(0.0, 1.0);
      omegaT *= (1.0 - 0.02 * dt * 60.0).clamp(0.0, 1.0);
      vx = max(0.3, vx - (MU_ROLL_RES * Fn / massKg) * dt);
      vy *= 0.985;
    }

    // Very simple lane transition model.
    final double pickup = OIL_PICKUP_RATE * oilLocal * (0.35 + 0.65 * flare) * (0.30 + 0.70 * tractionX);
    final double deposit = OIL_DEPOSIT_RATE * ballOil * (0.25 + 0.75 * OIL_CARRY_BLEND);
    pickupOilAt2D(oilMatrix, board, ft, pickup * dt * 8.0);
    depositOilAt2D(oilMatrix, board, ft + 1.0 + 2.0 * vx / 8.5, deposit * dt * 8.0);
    ballOil = (ballOil + pickup * dt - deposit * dt).clamp(0.0, 1.0);

    final double newOmegaTot = sqrt(omegaF * omegaF + omegaH * omegaH + omegaT * omegaT);
    double AR = 0.0, AT = 0.0;
    if (newOmegaTot > 1e-6) {
      AR = atan2(omegaH.abs(), omegaF.abs()) * 180.0 / pi;
      AT = asin((omegaT.abs() / newOmegaTot).clamp(0.0, 1.0)) * 180.0 / pi;
    }

    final bool rolling = slipRatio < 0.015 && tractionX > 0.85;
    final String phase = rolling ? 'roll' : (tractionY > 0.18 ? 'hook' : 'skid');

    theta += newOmegaTot * dt;
    ft += STEP_FT;
    board += (vy * dt) / BOARD_M;

    if (!inGutter && (board < 0.5 || board > 39.5)) inGutter = true;
    board = board.clamp(-3.0, 43.0);

    final double oilDrop = prevOil - oilLocal;
    final bool atBP = !inGutter &&
        oilDrop > 0.05 &&
        oilLocal < 0.18 &&
        tractionY > 0.20 &&
        ft > 25.0;

    prevOil = oilLocal;
    muAccum += 0.5 * (muX + muY);
    muCount++;
    segPhase = phase;

    path.add(PathPoint(
      ft: ft,
      board: board,
      vx: vx * 2.23694,
      omega: newOmegaTot,
      theta: theta,
      oil: oilLocal,
      mu: 0.5 * (muX + muY),
      AR: AR,
      phase: phase,
      atBreakpoint: atBP,
    ));

    if (ft - segStart >= 2.0 || ft >= 59.5) {
      segs.add(SegmentResult(
        seg: '${segStart.toStringAsFixed(0)}–${ft.toStringAsFixed(0)}',
        ft0: segStart,
        ft1: ft,
        mu: muCount > 0 ? muAccum / muCount : 0.5 * (muX + muY),
        vIn: segVin,
        vOut: vx * 2.23694,
        ARin: segARin,
        ARout: AR,
        ATin: segATin,
        ATout: AT,
        dboards: segBoard - board,
        boardOut: board,
        totalRPM: newOmegaTot * 60.0 / (2.0 * pi),
        phase: segPhase,
      ));
      segStart = ft;
      segVin = vx * 2.23694;
      segBoard = board;
      segARin = AR;
      segATin = AT;
      muAccum = 0.0;
      muCount = 0;
    }
  }

  final PathPoint? last = path.isNotEmpty ? path.last : null;
  return SimResult(
    path: path,
    segments: segs,
    pinSpeed: last?.vx ?? 0.0,
    pinRPM: last != null ? last.omega * 60.0 / (2.0 * pi) : 0.0,
    pinBoard: last?.board ?? 0.0,
    pinAR: last?.AR ?? 0.0,
  );
}
