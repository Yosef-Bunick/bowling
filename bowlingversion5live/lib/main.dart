import 'dart:async';
import 'dart:math';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'models/physics.dart';
import 'painters/lane_painter.dart';
import 'package:flutter/gestures.dart';
import 'pinsPhysics/bowling_pin_physics_v3.dart' as pinphys;

void main() => runApp(const BowlingSimApp());

class BowlingSimApp extends StatelessWidget {
  const BowlingSimApp({super.key});
  @override
  Widget build(BuildContext context) => MaterialApp(
    title: 'Bowling Simulator',
    debugShowCheckedModeBanner: false,
    theme: ThemeData.dark().copyWith(
      useMaterial3: true,
      scaffoldBackgroundColor: const Color(0xFF0A0A12),
      colorScheme: ColorScheme.fromSeed(
        seedColor: const Color(0xFF6366F1),
        brightness: Brightness.dark,
        surface: const Color(0xFF1A1B26),
        surfaceContainer: const Color(0xFF242736),
      ),
      appBarTheme: const AppBarTheme(
        backgroundColor: Color(0xFF0A0A12),
        surfaceTintColor: Colors.transparent,
        elevation: 0,
      ),
      cardTheme: CardThemeData(
        elevation: 0,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        color: const Color(0xFF1A1B26),
        surfaceTintColor: Colors.transparent,
      ),
      sliderTheme: SliderThemeData(
        activeTrackColor: const Color(0xFF6366F1),
        inactiveTrackColor: const Color(0xFF2A2D3A),
        thumbColor: const Color(0xFF6366F1),
        overlayColor: const Color(0xFF6366F1).withOpacity(0.2),
        trackHeight: 4,
        thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 10),
      ),
      inputDecorationTheme: InputDecorationTheme(
        filled: true,
        fillColor: const Color(0xFF242736),
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: BorderSide.none,
        ),
        contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
      ),
    ),
    home: const MainScreen(),
  );
}

// ─── Main Screen ────────────────────────────────────────────
class MainScreen extends StatefulWidget {
  const MainScreen({super.key});
  @override State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> with TickerProviderStateMixin {
  late TabController _tabs;
  BowlerInputs bowler  = BowlerInputs();
  BallSpecs    ball    = BallSpecs();
  PatternData  pattern = PatternData.masters2026();
  SimResult?   simResult;
  List<List<double>> oilMatrix = [];
  int animIdx = 0;
  bool playing = false;
  Timer? animTimer;
  Timer? _debounce;

  OilMatrix? simOilMatrix;

  @override
  void initState() {
    super.initState();
    _tabs = TabController(length: 5, vsync: this);
    _rebuild();
  }

  @override
  void dispose() {
    _debounce?.cancel(); animTimer?.cancel(); _tabs.dispose(); super.dispose();
  }

  void _rebuild() {
    // Full reset: fresh oil matrix using V2 dual-layer model
    animTimer?.cancel();
    final freshOil = buildOilMatrix(
      pattern.fwdRows, pattern.revRows, pattern.fwdOil, pattern.revOil);
    simOilMatrix = _copyOilMatrix(freshOil);
    setState(() {
      oilMatrix = freshOil.combined;
      simResult = runSimulationV2(bowler, pattern, ball, simOilMatrix!);
      animIdx = 0; playing = false;
    });
  }

  void _recalc() {
    // Re-run on existing oil — breakdown persists
    animTimer?.cancel();
    if (simOilMatrix == null) { _rebuild(); return; }
    setState(() {
      simResult = runSimulationV2(bowler, pattern, ball, simOilMatrix!);
      oilMatrix = simOilMatrix!.combined;  // refresh display
      animIdx = 0; playing = true;
    });
    _startAnim();
  }

  OilMatrix _copyOilMatrix(OilMatrix src) {
    return OilMatrix(
      forwardOil: List.generate(src.forwardOil.length, (b) => List<double>.from(src.forwardOil[b])),
      reverseOil: List.generate(src.reverseOil.length, (b) => List<double>.from(src.reverseOil[b])),
      fwdViscosity: src.fwdViscosity,
      revViscosity: src.revViscosity,
    );
  }

  void _startAnim() {
    animTimer = Timer.periodic(const Duration(milliseconds: 16), (t) {
      if (!playing || simResult == null) { t.cancel(); return; }
      setState(() {
        animIdx += 2;
        if (animIdx >= simResult!.path.length) {
          animIdx = simResult!.path.length - 1;
          playing = false;
          t.cancel();
        }
      });
    });
  }

  void _scheduleRecalc() {
    _debounce?.cancel();
    _debounce = Timer(const Duration(milliseconds: 11), () {
      if (mounted) _recalc();
    });
  }

  void _scheduleRebuild() {
    _debounce?.cancel();
    _debounce = Timer(const Duration(milliseconds: 1), () {
      if (mounted) _rebuild();
    });
  }

  void _togglePlay() {
    setState(() => playing = !playing);
    if (playing) {
      animTimer = Timer.periodic(const Duration(milliseconds: 16), (t) {
        if (!playing || simResult == null) { t.cancel(); return; }
        setState(() {
          animIdx += 2;
          if (animIdx >= simResult!.path.length) {
            animIdx = simResult!.path.length - 1;
            playing = false; t.cancel();
          }
        });
      });
    } else { animTimer?.cancel(); }
  }

  void _reset() {
    animTimer?.cancel();
    setState(() { animIdx = 0; playing = false; });
  }

  @override
  Widget build(BuildContext context) => Scaffold(
    appBar: AppBar(
      title: const Row(children: [
        Icon(Icons.sports_score, color: Color(0xFF6366F1), size: 22),
        SizedBox(width: 10),
        Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Text('BOWLING SIMULATOR', style: TextStyle(fontSize: 13,
            fontWeight: FontWeight.w700, color: Color(0xFFA0A6D0), letterSpacing: 1.5)),
          Text('Physics Engine v14', style: TextStyle(fontSize: 10,
            color: Color(0xFF8A91B8))),
        ]),
      ]),
      bottom: PreferredSize(
        preferredSize: const Size.fromHeight(46),
        child: Container(
          decoration: const BoxDecoration(border: Border(bottom: BorderSide(color: Color(0xFF242736)))),
          child: TabBar(
            controller: _tabs, isScrollable: true,
            labelColor: Colors.white,
            unselectedLabelColor: const Color(0xFF8A91B8),
            indicatorColor: const Color(0xFF6366F1),
            indicatorWeight: 3,
            labelStyle: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600, letterSpacing: 0.5),
            tabs: const [
              Tab(text: 'BOWLER'), Tab(text: 'BALL'), Tab(text: 'PATTERN'),
              Tab(text: 'LANE'), Tab(text: 'OIL MAP'),
            ],
          ),
        ),
      ),
    ),
    body: TabBarView(
      controller: _tabs,
      physics: const NeverScrollableScrollPhysics(),
      children: [
      _BowlerTab(bowler: bowler, onChanged: (b) { bowler = b; _scheduleRecalc(); }),
      _BallTab(ball: ball, onChanged: (b) { ball = b; _scheduleRecalc(); }),
      _PatternTab(pattern: pattern, onChanged: (p) { pattern = p; _scheduleRebuild(); }),
      _LaneTab(simResult: simResult, oilMatrix: oilMatrix,
        animIdx: animIdx, playing: playing,
        onPlay: _togglePlay, onReset: _rebuild, onRecalc: _recalc,
        pattern: pattern),
      _OilMapTab(oilMatrix: oilMatrix, pattern: pattern),
    ]),
  );
}

// ─── Shared widgets ─────────────────────────────────────────
class _Section extends StatelessWidget {
  final String title;
  const _Section(this.title);
  @override
  Widget build(BuildContext context) => Padding(
    padding: const EdgeInsets.only(top: 20, bottom: 10),
    child: Text(title.toUpperCase(), style: const TextStyle(
      fontSize: 11, fontWeight: FontWeight.w600,
      color: Color(0xFF6366F1), letterSpacing: 1.2)),
  );
}

class _Slider extends StatelessWidget {
  final String label, sublabel;
  final double value, min, max, step;
  final ValueChanged<double> onChanged;
  final String unit;
  const _Slider(this.label, this.sublabel, this.value, this.min, this.max, this.step,
    this.onChanged, {this.unit = ''});
  @override
  Widget build(BuildContext context) {
    final int divs = ((max - min) / step).round();
    return Padding(padding: const EdgeInsets.only(bottom: 12),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Row(children: [
          Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text(label, style: const TextStyle(fontSize: 13, color: Colors.white, fontWeight: FontWeight.w500)),
            if (sublabel.isNotEmpty)
              Text(sublabel, style: const TextStyle(fontSize: 10, color: Color(0xFF8A91B8))),
          ])),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
            decoration: BoxDecoration(
              color: const Color(0xFF242736),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Text('${value.toStringAsFixed(step < 1 ? (step < 0.01 ? 3 : 2) : 0)}$unit',
              style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: Color(0xFF6366F1))),
          ),
        ]),
        Slider(
          value: value.clamp(min, max), min: min, max: max,
          divisions: divs > 0 ? divs : null,
          onChanged: onChanged,
        ),
      ]));
  }
}

class _Toggle extends StatelessWidget {
  final String label, sublabel;
  final bool value;
  final ValueChanged<bool> onChanged;
  const _Toggle(this.label, this.sublabel, this.value, this.onChanged);
  @override
  Widget build(BuildContext context) => Container(
    margin: const EdgeInsets.only(bottom: 12),
    padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
    decoration: BoxDecoration(
      color: const Color(0xFF1A1B26),
      borderRadius: BorderRadius.circular(12),
      border: Border.all(color: const Color(0xFF2A2D3A)),
    ),
    child: Row(children: [
      Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Text(label, style: const TextStyle(fontSize: 13, color: Colors.white, fontWeight: FontWeight.w500)),
        if (sublabel.isNotEmpty)
          Text(sublabel, style: const TextStyle(fontSize: 10, color: Color(0xFF8A91B8))),
      ])),
      Switch(value: value, onChanged: onChanged, activeColor: const Color(0xFF6366F1)),
    ]),
  );
}

class _InfoCard extends StatelessWidget {
  final String label, value; final Color? valueColor;
  const _InfoCard(this.label, this.value, {this.valueColor});
  @override
  Widget build(BuildContext context) => Container(
    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
    decoration: BoxDecoration(
      color: const Color(0xFF1A1B26),
      borderRadius: BorderRadius.circular(10),
      border: Border.all(color: const Color(0xFF2A2D3A)),
    ),
    child: Column(children: [
      Text(value, style: TextStyle(fontSize: 16, fontWeight: FontWeight.w700,
        color: valueColor ?? Colors.white)),
      const SizedBox(height: 2),
      Text(label, style: const TextStyle(fontSize: 10, color: Color(0xFF8A91B8))),
    ]),
  );
}

// ─── Bowler Tab ──────────────────────────────────────────────
class _BowlerTab extends StatefulWidget {
  final BowlerInputs bowler; final ValueChanged<BowlerInputs> onChanged;
  const _BowlerTab({required this.bowler, required this.onChanged});
  @override State<_BowlerTab> createState() => _BowlerTabState();
}
class _BowlerTabState extends State<_BowlerTab> {
  late BowlerInputs b;
  @override void initState() { super.initState(); b = widget.bowler; }
  void _emit() => widget.onChanged(b);

  @override
  Widget build(BuildContext context) => SingleChildScrollView(
    padding: const EdgeInsets.all(20),
    child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      const _Section('Release Geometry'),
      _Slider('Release Board', 'Board where the ball first touches the lane',
        b.releaseBoard, 1, 39, 1, (v) { setState(() => b.releaseBoard = v); _emit(); }),
      _Slider('Landing Distance', 'Feet downlane where the ball first touches',
        b.landingDistanceFt, 0.5, 15.0, 0.25, (v) { setState(() => b.landingDistanceFt = v); _emit(); }, unit: ' ft'),
      _Slider('Launch Angle', 'Initial direction after touchdown',
        b.angleDeg, -10, 10, .25, (v) { setState(() => b.angleDeg = v); _emit(); }, unit: '°'),

      const _Section('Delivery'),
      _Slider('Ball Speed', 'mph at release',
        b.speedMph, 10, 36, 1, (v) { setState(() => b.speedMph = v); _emit(); }, unit: ' mph'),
      _Slider('Rev Rate', 'RPM at release',
        b.revRPM, 50, 500, 10, (v) { setState(() => b.revRPM = v); _emit(); }, unit: ' rpm'),

      const _Section('Axis'),
      _Slider('Axis Rotation', '0°=end-over-end  90°=full side roll',
        b.axisRotation, -90, 90, 1, (v) { setState(() => b.axisRotation = v); _emit(); }, unit: '°'),
      _Slider('Axis Tilt', '0°=flat  90°=spinning top',
        b.axisTilt, 0, 90, 1, (v) { setState(() => b.axisTilt = v); _emit(); }, unit: '°'),

      const _Section('Handedness'),
      Row(children: [
        Expanded(child: _HandBtn('Right', b.handedness == 1.0,
          () { setState(() => b.handedness = 1.0); _emit(); })),
        const SizedBox(width: 12),
        Expanded(child: _HandBtn('Left', b.handedness == -1.0,
          () { setState(() => b.handedness = -1.0); _emit(); })),
      ]),

      const _Section('Calibration'),
      _Slider('Hook K0', 'Base calibration  1.0=normal',
        b.hookK0, 0.2, 3.0, 0.1, (v) { setState(() => b.hookK0 = v); _emit(); }),
    ]),
  );
}

class _HandBtn extends StatelessWidget {
  final String label; final bool active; final VoidCallback onTap;
  const _HandBtn(this.label, this.active, this.onTap);
  @override
  Widget build(BuildContext context) => GestureDetector(
    onTap: onTap,
    child: Container(
      padding: const EdgeInsets.symmetric(vertical: 12),
      decoration: BoxDecoration(
        color: active ? const Color(0xFF6366F1) : const Color(0xFF1A1B26),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: active ? const Color(0xFF6366F1) : const Color(0xFF2A2D3A)),
      ),
      child: Center(child: Text(label, style: TextStyle(
        fontSize: 13, fontWeight: FontWeight.w600,
        color: active ? Colors.white : const Color(0xFF8A91B8)))),
    ),
  );
}

// ─── Ball Tab ────────────────────────────────────────────────
class _BallTab extends StatefulWidget {
  final BallSpecs ball; final ValueChanged<BallSpecs> onChanged;
  const _BallTab({required this.ball, required this.onChanged});
  @override State<_BallTab> createState() => _BallTabState();
}
class _BallTabState extends State<_BallTab> {
  late BallSpecs b;
  @override void initState() { super.initState(); b = widget.ball; }
  void _emit() => widget.onChanged(b);

  @override
  Widget build(BuildContext context) => SingleChildScrollView(
    padding: const EdgeInsets.all(20),
    child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      const _Section('Core Specs'),
      _Slider('RG', '2.46=early read  2.80=more length',
        b.rg, 2.46, 2.80, 0.01, (v) { setState(() => b.rg = v); _emit(); }, unit: '"'),
      _Slider('Differential', 'Controls flare potential',
        b.diff, 0.0, 0.060, 0.001, (v) { setState(() => b.diff = v); _emit(); }, unit: '"'),
      _Slider('Asymmetry', '0=symmetric  ≥0.013=asymmetric',
        b.asy, 0.0, 0.030, 0.001, (v) { setState(() => b.asy = v); _emit(); }, unit: '"'),

      const _Section('Cover'),
      _Slider('Ball Weight', 'lbs',
        b.masslb, 10, 16, 0.5, (v) { setState(() => b.masslb = v); _emit(); }, unit: ' lb'),
      Container(
        margin: const EdgeInsets.only(bottom: 12),
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
        decoration: BoxDecoration(color: const Color(0xFF1A1B26),
          borderRadius: BorderRadius.circular(12), border: Border.all(color: const Color(0xFF2A2D3A))),
        child: DropdownButtonHideUnderline(child: DropdownButton<String>(
          value: b.grit,
          isExpanded: true,
          dropdownColor: const Color(0xFF242736),
          style: const TextStyle(fontSize: 13, color: Colors.white),
          items: const [
            DropdownMenuItem(value:'500',  child: Text('500 — very rough (sanded solid)')),
            DropdownMenuItem(value:'1000', child: Text('1000 — rough')),
            DropdownMenuItem(value:'2000', child: Text('2000 — medium')),
            DropdownMenuItem(value:'3000', child: Text('3000 — smooth')),
            DropdownMenuItem(value:'4000', child: Text('4000 — very smooth')),
            DropdownMenuItem(value:'polish', child: Text('Polish (pearl / shiny)')),
          ],
          onChanged: (v) { if (v!=null) { setState(()=>b.grit=v); _emit(); }},
        )),
      ),

      const _Section('Computed Factors'),
      Row(children: [
        Expanded(child: _InfoCard('RG Factor', b.rgFactor.toStringAsFixed(3),
          valueColor: b.rgFactor > 1.0 ? Colors.greenAccent : const Color(0xFFA0A6D0))),
        const SizedBox(width: 8),
        Expanded(child: _InfoCard('Diff Factor', b.diffFactor.toStringAsFixed(3),
          valueColor: b.diffFactor > 1.0 ? Colors.orangeAccent : const Color(0xFFA0A6D0))),
        const SizedBox(width: 8),
        Expanded(child: _InfoCard('Asym Factor', b.asymFactor.toStringAsFixed(3),
          valueColor: b.isAsym ? Colors.purpleAccent : const Color(0xFF8A91B8))),
      ]),
      const SizedBox(height: 8),
      Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(color: const Color(0xFF1A1B26),
          borderRadius: BorderRadius.circular(10), border: Border.all(color: const Color(0xFF2A2D3A))),
        child: Row(children: [
          Icon(b.isAsym ? Icons.compare_arrows : Icons.circle_outlined,
            size: 16, color: b.isAsym ? Colors.purpleAccent : const Color(0xFF8A91B8)),
          const SizedBox(width: 8),
          Text(b.isAsym ? 'Asymmetric (asy ≥ 0.013)' : 'Symmetric',
            style: TextStyle(fontSize: 12,
              color: b.isAsym ? Colors.purpleAccent : const Color(0xFF8A91B8))),
        ]),
      ),

      const _Section('Reference'),
      Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(color: const Color(0xFF0F1018),
          borderRadius: BorderRadius.circular(10), border: Border.all(color: const Color(0xFF2A2D3A))),
        child: const Text(
          'RG:   2.46–2.49 = early   2.50–2.56 = medium   2.57+ = long\n'
          'Diff: 0.000–0.029 = low   0.030–0.045 = med   0.046+ = high\n'
          'Asy:  0 = symmetric   ≥0.013 = asymmetric (industry heuristic)',
          style: TextStyle(fontSize: 10, color: Color(0xFF8A91B8), height: 1.7,
            fontFamily: 'monospace')),
      ),
    ]),
  );
}

// ─── Pattern Tab ─────────────────────────────────────────────
class _PatternTab extends StatefulWidget {
  final PatternData pattern; final ValueChanged<PatternData> onChanged;
  const _PatternTab({required this.pattern, required this.onChanged});
  @override State<_PatternTab> createState() => _PatternTabState();
}
class _PatternTabState extends State<_PatternTab> {
  late PatternData p;
  bool usePerRowOil = true;  // Toggle for per-row oil types
  
  @override void initState() { super.initState(); p = widget.pattern; }

  // Oil type names for dropdown
  static const List<String> _oilTypes = [
    'glide', 'terrain', 'curve', 'current', 'condition_red', 'defense_s',
    'fire', 'ice', 'condition_blue', 'infinity', 'navigate', 'prodigy',
    'clear_super_100', 'clear_super_50', 'clear_801_hv', 'clear_811_lv',
    'ceo_65', 'ceo_42', 'se_28',
  ];

  String _oilLabel(String key) {
    final oil = OIL_LIBRARY[key];
    if (oil == null) return key;
    return '${oil.name} (${(oil.viscosityPaS * 1000).toStringAsFixed(1)} cps)';
  }

  @override
  Widget build(BuildContext context) => SingleChildScrollView(
    padding: const EdgeInsets.all(20),
    child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      Row(children: [
        Expanded(child: TextField(
          controller: TextEditingController(text: p.name)..selection =
            TextSelection.collapsed(offset: p.name.length),
          style: const TextStyle(fontSize: 13),
          decoration: const InputDecoration(labelText: 'Pattern Name',
            labelStyle: TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
          onChanged: (v) => setState(() => p.name = v),
          onSubmitted: (_) => widget.onChanged(p),
        )),
        const SizedBox(width: 12),
        SizedBox(width: 90, child: TextField(
          controller: TextEditingController(text: p.distance.toStringAsFixed(0)),
          style: const TextStyle(fontSize: 13),
          keyboardType: TextInputType.number,
          decoration: const InputDecoration(labelText: 'Dist (ft)',
            labelStyle: TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
          onSubmitted: (v) {
            setState(() => p.distance = double.tryParse(v) ?? p.distance);
            widget.onChanged(p);
          },
        )),
      ]),
      
      const _Section('Oil Types'),
      
      // Per-row oil toggle
      Container(
        margin: const EdgeInsets.only(bottom: 12),
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          color: const Color(0xFF1A1B26),
          borderRadius: BorderRadius.circular(10),
          border: Border.all(color: const Color(0xFF2A2D3A)),
        ),
        child: Row(children: [
          const Icon(Icons.layers, size: 16, color: Color(0xFF6366F1)),
          const SizedBox(width: 8),
          const Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text('Per-Row Oil Types', style: TextStyle(fontSize: 12, color: Colors.white, fontWeight: FontWeight.w500)),
            Text('Enable to set different oil for each load row', style: TextStyle(fontSize: 10, color: Color(0xFF8A91B8))),
          ])),
          Switch(value: usePerRowOil, onChanged: (v) => setState(() => usePerRowOil = v),
            activeColor: const Color(0xFF6366F1)),
        ]),
      ),
      
      // Global oil dropdowns (shown when per-row is OFF)
      if (!usePerRowOil) ...[
        Row(children: [
          Expanded(child: _buildOilDropdown('Forward Oil', p.fwdOilType, (v) {
            setState(() => p.fwdOilType = v);
            widget.onChanged(p);
          })),
          const SizedBox(width: 12),
          Expanded(child: _buildOilDropdown('Reverse Oil', p.revOilType, (v) {
            setState(() => p.revOilType = v);
            widget.onChanged(p);
          })),
        ]),
        const SizedBox(height: 8),
        Container(
          padding: const EdgeInsets.all(10),
          decoration: BoxDecoration(
            color: const Color(0xFF0F1018),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: const Color(0xFF2A2D3A)),
          ),
          child: Row(children: [
            _oilChip('FWD', p.fwdOil.viscosityPaS, const Color(0xFF6366F1)),
            const SizedBox(width: 16),
            _oilChip('REV', p.revOil.viscosityPaS, const Color(0xFFE67E22)),
          ]),
        ),
      ],
      
      // Info when per-row is ON
      if (usePerRowOil)
        Container(
          padding: const EdgeInsets.all(10),
          decoration: BoxDecoration(
            color: const Color(0xFF0F1018),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: const Color(0xFF6366F1).withOpacity(0.3)),
          ),
          child: const Row(children: [
            Icon(Icons.info_outline, size: 14, color: Color(0xFF6366F1)),
            SizedBox(width: 8),
            Expanded(child: Text('Set oil type per row using the Oil column in the tables below',
              style: TextStyle(fontSize: 11, color: Color(0xFFA0A6D0)))),
          ]),
        ),
      
      const _Section('Forward Loads'),
      _buildLoadTable(p.fwdRows, (rows) { setState(() => p.fwdRows = rows); widget.onChanged(p); }, usePerRowOil),
      TextButton.icon(
        onPressed: () {
          setState(() => p.fwdRows.add(LoadRow(sl:2,sr:2,loads:1,mics:50,buff:500,d0:0,d1:5,toil:0,oilType:p.fwdOilType)));
          widget.onChanged(p);
        },
        icon: const Icon(Icons.add, size: 16), label: const Text('Add Row'),
      ),
      const _Section('Reverse Loads'),
      _buildLoadTable(p.revRows, (rows) { setState(() => p.revRows = rows); widget.onChanged(p); }, usePerRowOil),
      TextButton.icon(
        onPressed: () {
          setState(() => p.revRows.add(LoadRow(sl:2,sr:2,loads:1,mics:50,buff:500,d0:0,d1:5,toil:0,oilType:p.revOilType)));
          widget.onChanged(p);
        },
        icon: const Icon(Icons.add, size: 16), label: const Text('Add Row'),
      ),
    ]),
  );

  Widget _buildOilDropdown(String label, String value, ValueChanged<String> onChanged) =>
    Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      Text(label, style: const TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
      const SizedBox(height: 4),
      Container(
        padding: const EdgeInsets.symmetric(horizontal: 12),
        decoration: BoxDecoration(
          color: const Color(0xFF242736),
          borderRadius: BorderRadius.circular(8),
        ),
        child: DropdownButton<String>(
          value: _oilTypes.contains(value) ? value : 'fire',
          isExpanded: true,
          underline: const SizedBox(),
          dropdownColor: const Color(0xFF242736),
          style: const TextStyle(fontSize: 12, color: Colors.white),
          items: _oilTypes.map((k) => DropdownMenuItem(
            value: k,
            child: Text(_oilLabel(k), overflow: TextOverflow.ellipsis),
          )).toList(),
          onChanged: (v) { if (v != null) onChanged(v); },
        ),
      ),
    ]);

  Widget _oilChip(String label, double viscosity, Color color) => Row(children: [
    Container(width: 8, height: 8, decoration: BoxDecoration(color: color, shape: BoxShape.circle)),
    const SizedBox(width: 6),
    Text('$label: ${(viscosity * 1000).toStringAsFixed(1)} cps',
      style: const TextStyle(fontSize: 11, color: Color(0xFFA0A6D0))),
  ]);

  Widget _buildLoadTable(List<LoadRow> rows, ValueChanged<List<LoadRow>> onChanged, bool showOilColumn) =>
    SingleChildScrollView(scrollDirection: Axis.horizontal,
      child: DataTable(
        columnSpacing: 8, headingRowHeight: 28,
        dataRowMinHeight: 36, dataRowMaxHeight: 36,
        headingTextStyle: const TextStyle(fontSize: 10, color: Color(0xFF8A91B8), fontWeight: FontWeight.w600),
        dataTextStyle: const TextStyle(fontSize: 11, color: Colors.white),
        columns: [
          const DataColumn(label: Text('#')), const DataColumn(label: Text('SL')), const DataColumn(label: Text('SR')),
          const DataColumn(label: Text('Loads')), const DataColumn(label: Text('MICS')), const DataColumn(label: Text('BUFF')),
          const DataColumn(label: Text('Ft0')), const DataColumn(label: Text('Ft1')), const DataColumn(label: Text('T.OIL')),
          if (showOilColumn) const DataColumn(label: Text('Oil')),
          const DataColumn(label: Text('×')),
        ],
        rows: rows.asMap().entries.map((e) {
          final i=e.key; final r=e.value;
          return DataRow(cells: [
            DataCell(Text('${i+1}', style: const TextStyle(color: Color(0xFF8A91B8)))),
            DataCell(_mf(r.sl.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(sl:int.tryParse(v)??r.sl);onChanged(u);})),
            DataCell(_mf(r.sr.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(sr:int.tryParse(v)??r.sr);onChanged(u);})),
            DataCell(_mf(r.loads.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(loads:int.tryParse(v)??r.loads);onChanged(u);})),
            DataCell(_mf(r.mics.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(mics:int.tryParse(v)??r.mics);onChanged(u);})),
            DataCell(_mf(r.buff.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(buff:int.tryParse(v)??r.buff);onChanged(u);})),
            DataCell(_mf(r.d0.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(d0:double.tryParse(v)??r.d0);onChanged(u);})),
            DataCell(_mf(r.d1.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(d1:double.tryParse(v)??r.d1);onChanged(u);})),
            DataCell(_mf(r.toil.toString(), (v){final u=List<LoadRow>.from(rows);u[i]=r.copyWith(toil:double.tryParse(v)??r.toil);onChanged(u);})),
            if (showOilColumn) DataCell(_buildRowOilDropdown(r.oilType, (v){
              final u=List<LoadRow>.from(rows);u[i]=r.copyWith(oilType:v);onChanged(u);
            })),
            DataCell(IconButton(
              icon: const Icon(Icons.close, size: 14, color: Color(0xFF8A91B8)),
              onPressed: () { final u=List<LoadRow>.from(rows)..removeAt(i); onChanged(u); },
            )),
          ]);
        }).toList(),
      ));

  Widget _buildRowOilDropdown(String value, ValueChanged<String> onChanged) =>
    SizedBox(width: 70, child: DropdownButton<String>(
      value: _oilTypes.contains(value) ? value : 'fire',
      isExpanded: true,
      underline: const SizedBox(),
      dropdownColor: const Color(0xFF242736),
      style: const TextStyle(fontSize: 10, color: Colors.white),
      isDense: true,
      items: _oilTypes.map((k) => DropdownMenuItem(
        value: k,
        child: Text(OIL_LIBRARY[k]?.name ?? k, overflow: TextOverflow.ellipsis, style: const TextStyle(fontSize: 10)),
      )).toList(),
      onChanged: (v) { if (v != null) onChanged(v); },
    ));

  Widget _mf(String v, ValueChanged<String> cb) => SizedBox(width: 46,
    child: TextFormField(initialValue: v, keyboardType: TextInputType.number,
      style: const TextStyle(fontSize: 11),
      decoration: const InputDecoration(isDense: true,
        contentPadding: EdgeInsets.symmetric(horizontal: 6, vertical: 6),
        fillColor: Color(0xFF0F1018)),
      onChanged: cb));
}

// ─── Lane Tab ────────────────────────────────────────────────
// ─── Lane Tab ────────────────────────────────────────────────
// Lane real dims: 41.875 in wide × 62.86 ft long (foul line → pit)
const double LANE_WIDTH_IN = 41.875;
const double LANE_LENGTH_FT = 62.86;
const double LANE_ASPECT = (LANE_WIDTH_IN / 12.0) / LANE_LENGTH_FT; // ≈0.0555

class _LaneTab extends StatefulWidget {
  final SimResult? simResult;
  final List<List<double>> oilMatrix;
  final int animIdx; final bool playing;
  final VoidCallback onPlay, onReset, onRecalc;
  final PatternData pattern;
  const _LaneTab({required this.simResult, required this.oilMatrix,
    required this.animIdx, required this.playing,
    required this.onPlay, required this.onReset, required this.onRecalc,
    required this.pattern});

  @override State<_LaneTab> createState() => _LaneTabState();
}

class _LaneTabState extends State<_LaneTab> {
  bool view3D = false;       // viewMode: false=2D, true=3D
  bool realAspect = false;   // aspectMode: false=Fill, true=Real
//deafault startup for 3d real
  // 3D camera (reused from _OilMapTabState convention)
  double _yaw = 0.0;
  double _pitch = .3;
  double _zoom = 16.35;
  double _lastScale = 1.0;

  // ── Pin physics integration ──
  List<pinphys.BowlingPin>? _pinDeck;
  pinphys.BowlingBall? _pinBall;
  bool _pinSimStarted = false;
  int _pinSimSteps = 0;
  static const int _pinSimSubSteps = 8;     // sub-steps per anim tick
  static const double _pinSimDt = 1.0 / 240.0;
  static const double _pinTriggerFt = 59.0; // ball ft to trigger pin sim
  static const int _pinScatterMaxTicks = 120; // ~2s at 60fps after ball ends
  Timer? _pinScatterTimer;
  bool _ballPathDone = false;

  // Conversion constants (pin physics uses meters, centered at head pin)
  static const double _headPinFt = 60.0;
  static const double _laneWidthIn = 41.5;
  static const double _boardsPerMeter = 38.0 / (_laneWidthIn * 0.0254);
  static const double _ftPerMeter = 1.0 / 0.3048;

  List<_PinRenderState> _pinRenderStates = _buildDefaultPinStates();

  static List<_PinRenderState> _buildDefaultPinStates() {
    const double boardsPerInch = 38.0 / 41.5;
    const double spB = 12.0 * boardsPerInch;
    const double spF = 12.0 / 12.0 * 0.866;
    const double cb = 19.0;
    final rows = <List<double>>[
      [0],
      [-0.5, 0.5],
      [-1.0, 0.0, 1.0],
      [-1.5, -0.5, 0.5, 1.5],
    ];
    final List<_PinRenderState> pins = [];
    for (int r = 0; r < rows.length; r++) {
      for (final off in rows[r]) {
        pins.add(_PinRenderState(
          board: cb + off * spB,
          ft: _headPinFt + r * spF,
          tiltRad: 0,
          visible: true,
        ));
      }
    }
    return pins;
  }

  @override
  void dispose() {
    _pinScatterTimer?.cancel();
    super.dispose();
  }

  @override
  void didUpdateWidget(covariant _LaneTab oldWidget) {
    super.didUpdateWidget(oldWidget);

    // Reset pin sim when a new simulation starts (animIdx resets to 0)
    if (widget.animIdx == 0 && oldWidget.animIdx != 0) {
      _resetPinSim();
    }

    // Check if ball reached pin trigger zone
    if (!_pinSimStarted && widget.simResult != null) {
      final path = widget.simResult!.path;
      if (widget.animIdx < path.length && path[widget.animIdx].ft >= _pinTriggerFt) {
        _initPinSim(path[widget.animIdx]);
      }
    }

    // Step pin sim forward each animation tick
    if (_pinSimStarted && widget.animIdx != oldWidget.animIdx) {
      _stepPinSim();
    }

    // Detect ball path just ended → keep pin sim going
    if (_pinSimStarted &&
        !_ballPathDone &&
        widget.simResult != null &&
        widget.animIdx >= widget.simResult!.path.length - 1 &&
        !widget.playing) {
      _ballPathDone = true;
      _startPinScatterTimer();
    }
  }

  void _resetPinSim() {
    _pinScatterTimer?.cancel();
    _pinScatterTimer = null;
    _ballPathDone = false;
    _pinDeck = null;
    _pinBall = null;
    _pinSimStarted = false;
    _pinSimSteps = 0;
    _pinRenderStates = _buildDefaultPinStates();
  }

  void _startPinScatterTimer() {
    int ticks = 0;
    _pinScatterTimer = Timer.periodic(const Duration(milliseconds: 16), (t) {
      if (!mounted || _pinDeck == null || ticks >= _pinScatterMaxTicks) {
        t.cancel();
        _pinScatterTimer = null;
        return;
      }
      ticks++;
      setState(() => _stepPinSim());
    });
  }

  void _initPinSim(PathPoint ballAtPins) {
    _pinDeck = pinphys.buildPinDeck();
    // Convert ball path state → pin physics ball
    // Ball position: board → X meters from center, ft → Z meters from head pin
    final double boardCenterM = (ballAtPins.board - 19.0) / _boardsPerMeter;
    final double ftOffsetM = (ballAtPins.ft - _headPinFt) / _ftPerMeter;
    // Ball velocity: vx (mph along lane) → Z, lateral from path → X
    final double speedMs = ballAtPins.vx / 2.23694; // mph → m/s
    // Estimate lateral velocity from board change rate in path
    double lateralMs = 0.0;
    final path = widget.simResult!.path;
    final idx = widget.animIdx;
    if (idx > 2) {
      final dbBoard = path[idx].board - path[idx - 3].board;
      final dbFt = path[idx].ft - path[idx - 3].ft;
      if (dbFt.abs() > 0.01) {
        lateralMs = (dbBoard / _boardsPerMeter) / (dbFt / _ftPerMeter) * speedMs;
      }
    }
    // Omega from path point (approximate)
    final double omega = ballAtPins.omega;
    final double arRad = ballAtPins.AR * pi / 180.0;

    _pinBall = pinphys.BowlingBall(
      position: pinphys.Vec3(boardCenterM, 0.0, ftOffsetM),
      velocity: pinphys.Vec3(lateralMs, 0.0, speedMs),
      angularVelocity: pinphys.Vec3(0.0, omega * sin(arRad), omega * cos(arRad)),
      mass: widget.simResult != null ? 6.80 : 6.80, // could read from ball specs
      radius: pinphys.inch(4.3), // ~8.6" diameter ball
    );
    _pinSimStarted = true;
    _pinSimSteps = 0;
  }

  void _stepPinSim() {
    if (_pinDeck == null || _pinBall == null) return;
    // Run several sub-steps per animation frame for stability
    for (int i = 0; i < _pinSimSubSteps; i++) {
      pinphys.simulationStep(pins: _pinDeck!, ball: _pinBall!, dt: _pinSimDt);
      _pinSimSteps++;
    }
    // Convert physics state → render state
    _pinRenderStates = _pinDeck!.map((p) {
      final double board = 19.0 + p.position.x * _boardsPerMeter;
      final double ft = _headPinFt + p.position.z * _ftPerMeter;
      // Tilt direction from tiltAxis (xz plane → board/ft direction)
      final double tiltDir = atan2(p.tiltAxis.x, p.tiltAxis.z);
      return _PinRenderState(
        board: board,
        ft: ft,
        tiltRad: p.tiltAngle,
        tiltDirection: tiltDir,
        visible: p.tiltAngle < pinphys.radians(88.0),
      );
    }).toList();
  }

  @override
  Widget build(BuildContext context) {
    final r    = widget.simResult;
    final List<PathPoint> path =
        (r?.path as List?)?.cast<PathPoint>() ?? <PathPoint>[];
    final cur  = widget.animIdx < path.length ? path[widget.animIdx] : null;

    final Color phaseColor = cur == null ? Colors.grey
      : cur.phase == 'roll'  ? Colors.greenAccent
      : cur.phase == 'hook'  ? Colors.orangeAccent
      : Colors.redAccent;

    return Column(children: [
      // Pin stats bar
      if (r != null) Container(
        color: const Color(0xFF0F1018),
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
        child: Row(children: [
          Expanded(child: _InfoCard('Headpin Speed',
            '${r.pinSpeed.toStringAsFixed(1)} mph',
            valueColor: r.pinSpeed>=15&&r.pinSpeed<=17 ? Colors.greenAccent : Colors.orangeAccent)),
          const SizedBox(width: 8),
          Expanded(child: _InfoCard('Pin RPM', r.pinRPM.toStringAsFixed(0))),
          const SizedBox(width: 8),
          Expanded(child: _InfoCard('Pin AR', '${r.pinAR.toStringAsFixed(1)}°')),
          const SizedBox(width: 8),
          Expanded(child: _InfoCard('Pin Board', r.pinBoard.toStringAsFixed(1),
            valueColor: r.pinBoard>=16&&r.pinBoard<=18 ? Colors.greenAccent : Colors.white)),
          if (_pinSimStarted && _pinDeck != null) ...[
            const SizedBox(width: 8),
            Expanded(child: _InfoCard('Knocked',
              '${pinphys.countKnockedPins(_pinDeck!)}/10',
              valueColor: pinphys.isStrike(_pinDeck!) ? Colors.greenAccent : Colors.orangeAccent)),
          ],
        ]),
      ),

      // Live HUD
      Container(
        color: const Color(0xFF1A1B26),
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(children: [
          _liveChip('Phase', cur?.phase.toUpperCase() ?? '—', phaseColor),
          const SizedBox(width: 8),
          _liveChip('Speed', cur != null ? '${cur.vx.toStringAsFixed(1)} mph' : '—', Colors.white),
          const SizedBox(width: 8),
          _liveChip('RPM', cur != null ? '${(cur.omega*60/(2*pi)).toStringAsFixed(0)}' : '—', Colors.white),
          const SizedBox(width: 8),
          _liveChip('Board', cur != null ? cur.board.toStringAsFixed(1) : '—', Colors.white),
          const SizedBox(width: 8),
          _liveChip('AR', cur != null ? '${cur.AR.toStringAsFixed(1)}°' : '—', Colors.white70),
          const SizedBox(width: 8),
          _liveChip('Slip', cur != null ? cur.slipRatio.toStringAsFixed(3) : '—', Colors.white70),
          const SizedBox(width: 8),
          _liveChip('Tx', cur != null ? cur.tractionX.toStringAsFixed(2) : '—', Colors.white70),
          const SizedBox(width: 8),
          _liveChip('Mu', cur != null ? cur.mu.toStringAsFixed(3) : '—', Colors.white70),
          const SizedBox(width: 8),
          _liveChip('Oil', cur != null ? '${(cur.oil*100).toStringAsFixed(0)}%' : '—',
            const Color(0xFF6366F1)),
        ]),
      ),

      // Controls + view toggles
      Padding(padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(children: [
          ElevatedButton.icon(
            style: ElevatedButton.styleFrom(
              backgroundColor: const Color(0xFF6366F1),
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: widget.onPlay,
            icon: Icon(widget.playing ? Icons.pause : Icons.play_arrow, size: 18),
            label: Text(widget.playing ? 'Pause' : 'Play'),
          ),
          const SizedBox(width: 8),
          OutlinedButton(
            style: OutlinedButton.styleFrom(
              side: const BorderSide(color: Color(0xFF2A2D3A)),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: widget.onReset, child: const Text('Reset'),
          ),
          const SizedBox(width: 8),
          OutlinedButton(
            style: OutlinedButton.styleFrom(
              side: const BorderSide(color: Color(0xFF2A2D3A)),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: widget.onRecalc, child: const Text('Recalc'),
          ),
          const Spacer(),
          // View mode toggle
          const Text('2D', style: TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
          Switch(value: view3D,
            onChanged: (v) => setState(() => view3D = v),
            activeColor: const Color(0xFF6366F1)),
          const Text('3D', style: TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
          const SizedBox(width: 12),
          // Aspect mode toggle
          const Text('Fill', style: TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
          Switch(value: realAspect,
            onChanged: (v) => setState(() => realAspect = v),
            activeColor: const Color(0xFF6366F1)),
          const Text('Real', style: TextStyle(fontSize: 11, color: Color(0xFF8A91B8))),
        ]),
      ),

      // Phase legend row
      Padding(padding: const EdgeInsets.fromLTRB(16, 0, 16, 6),
        child: Row(children: [
          const Spacer(),
          _phaseDot(Colors.redAccent, 'Skid'),
          const SizedBox(width: 8),
          _phaseDot(Colors.orangeAccent, 'Hook'),
          const SizedBox(width: 8),
          _phaseDot(Colors.greenAccent, 'Roll'),
        ]),
      ),

      // Lane canvas
      Expanded(child: Container(
        margin: const EdgeInsets.fromLTRB(16, 0, 16, 16),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(12),
          border: Border.all(color: const Color(0xFF2A2D3A)),
          color: const Color(0xFF0F1018),
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(12),
          child: Center(
            child: realAspect
              ? Center(child: AspectRatio(aspectRatio: LANE_ASPECT, child: _buildCanvas(path.cast())))
              : SizedBox.expand(child: _buildCanvas(path.cast())),
          ),
        ),
      )),
    ]);
  }

  Widget _buildCanvas(List<PathPoint> path) {
    if (!view3D) {
      return CustomPaint(
        painter: LanePainter(
          path: path,
          oilMatrix: widget.oilMatrix,
          animIdx: widget.animIdx),
        child: const SizedBox.expand(),
      );
    }
    // 3D placeholder — Phase 2 will swap in _3DLanePainter
    return Listener(
      onPointerSignal: (event) {
        if (event is PointerScrollEvent) {
          setState(() {
            final next = _zoom * (event.scrollDelta.dy > 0 ? 0.92 : 1.08);
            _zoom = next.clamp(0.5, 26.0);
          });
        }
      },
      child: RawGestureDetector(
        behavior: HitTestBehavior.opaque,
        gestures: <Type, GestureRecognizerFactory>{
          ScaleGestureRecognizer:
              GestureRecognizerFactoryWithHandlers<ScaleGestureRecognizer>(
            () => ScaleGestureRecognizer(),
            (ScaleGestureRecognizer instance) {
              instance.onStart = (_) {
                _lastScale = 1.0;
              };
              instance.onUpdate = (d) {
                setState(() {
                  if (d.pointerCount >= 2) {
                    final sd = d.scale / _lastScale;
                    _zoom = (_zoom * sd).clamp(0.5, 26.0);
                    _lastScale = d.scale;
                  } else {
                    _yaw += d.focalPointDelta.dx * 0.01;
                    _pitch = (_pitch - d.focalPointDelta.dy * 0.01)
                        .clamp(0.15, 1.45);
                  }
                });
              };
            },
          ),
          DoubleTapGestureRecognizer:
              GestureRecognizerFactoryWithHandlers<DoubleTapGestureRecognizer>(
            () => DoubleTapGestureRecognizer(),
            (DoubleTapGestureRecognizer instance) {
              instance.onDoubleTap = () => setState(() {
                    _yaw = 0.0;
                    _pitch = 1.1;
                    _zoom = 1.35;
                  });
            },
          ),
        },
        child: CustomPaint(
          painter: _3DLanePainter(
            path: path,
            oilMatrix: widget.oilMatrix,
            oilDistance: widget.pattern.distance,
            laneLengthFt: LANE_FT.toDouble(),
            animIdx: widget.animIdx,
            oilColorFn: _OilMapTabState.oilColor,
            yaw: _yaw, pitch: _pitch, zoom: _zoom,
            pinRadiusAtLocalHeight: pinphys.pinRadiusAtLocalHeight,
            pinHeightM: pinphys.pinHeight,
            pinBaseRadiusM: pinphys.pinBaseRadius,
            pinMaxRadiusM: pinphys.pinMaxRadius,
            pinStates: _pinRenderStates,
          ),
          child: const SizedBox.expand(),
        ),
      ),
    );
  }

  Widget _liveChip(String lbl, String val, Color c) => Expanded(child: Column(
    mainAxisSize: MainAxisSize.min,
    children: [
      Text(val, style: TextStyle(fontSize: 12, fontWeight: FontWeight.w600, color: c)),
      Text(lbl, style: const TextStyle(fontSize: 9, color: Color(0xFF8A91B8))),
    ]));

  Widget _phaseDot(Color c, String lbl) => Row(mainAxisSize: MainAxisSize.min, children: [
    Container(width: 8, height: 8, decoration: BoxDecoration(color: c, shape: BoxShape.circle)),
    const SizedBox(width: 4),
    Text(lbl, style: const TextStyle(fontSize: 10, color: Color(0xFF8A91B8))),
  ]);
}

// ─── Oil Map Tab ─────────────────────────────────────────────
class _OilMapTab extends StatefulWidget {
  final List<List<double>> oilMatrix;
  final PatternData pattern;
  const _OilMapTab({required this.oilMatrix, required this.pattern});
  @override State<_OilMapTab> createState() => _OilMapTabState();
}

class _OilMapTabState extends State<_OilMapTab> {
  bool show3D = false;
  double _yaw = 0.0;
  double _pitch = 1.1;
  double _zoom = 1.35;
  double _lastScale = 1.0;

  // Shared rainbow color scale — same in both 2D and 3D
  static const List<List<dynamic>> _stops = [
    [0.00, Color(0xFF32140A)],  // dry — very dark brown
    [0.12, Color(0xFF8B4513)],  // trace
    [0.28, Color(0xFFC0392B)],  // light
    [0.42, Color(0xFFE67E22)],  // medium-light
    [0.56, Color(0xFFF1C40F)],  // medium
    [0.68, Color(0xFF2ECC71)],  // medium-heavy
    [0.80, Color(0xFF2980B9)],  // heavy
    [1.00, Color(0xFF8E44AD)],  // max — purple
  ];

  static const List<String> _stopLabels = [
    'Dry', 'Trace', 'Light', 'Med-light',
    'Medium', 'Med-heavy', 'Heavy', 'Max',
  ];

  static Color oilColor(double v) {
    for (int i = 1; i < _stops.length; i++) {
      if (v <= (_stops[i][0] as double)) {
        final t = (v - (_stops[i-1][0] as double)) /
                  ((_stops[i][0] as double) - (_stops[i-1][0] as double));
        final a = _stops[i-1][1] as Color;
        final b = _stops[i][1] as Color;
        return Color.fromRGBO(
          (a.red   + (b.red   - a.red)   * t).round(),
          (a.green + (b.green - a.green) * t).round(),
          (a.blue  + (b.blue  - a.blue)  * t).round(), 1);
      }
    }
    return const Color(0xFF8E44AD);
  }

  Widget _buildInteractive3D() {
    return Listener(
      onPointerSignal: (event) {
        if (event is PointerScrollEvent) {
          setState(() {
            final next = _zoom * (event.scrollDelta.dy > 0 ? 0.92 : 1.08);
            _zoom = next.clamp(0.7, 4.0);
          });
        }
      },
      child: GestureDetector(
        behavior: HitTestBehavior.opaque,
        onDoubleTap: () {
          setState(() {
            _yaw = 0.0;
            _pitch = 1.1;
            _zoom = 1.35;
          });
        },
        onScaleStart: (_) {
          _lastScale = 1.0;
        },
        onScaleUpdate: (details) {
          setState(() {
            if (details.pointerCount >= 2) {
              final scaleDelta = details.scale / _lastScale;
              _zoom = (_zoom * scaleDelta).clamp(0.7, 4.0);
              _lastScale = details.scale;
            } else {
              _yaw += details.focalPointDelta.dx * 0.01;
              _pitch = (_pitch - details.focalPointDelta.dy * 0.01).clamp(0.15, 1.45); //clamps how much you can rotate 3d model to the side (side,top)
            }
          });
        },
        child: CustomPaint(
          painter: _3DOilPainter(
            oilMatrix: widget.oilMatrix,
            //distance: widget.pattern.distance, //short lane in 3d
            distance: LANE_FT.toDouble(), //long lane in 3d
            colorFn: oilColor,
            yaw: _yaw,
            pitch: _pitch,
            zoom: _zoom,
          ),
          child: const SizedBox.expand(),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) => Column(children: [
    // Header + toggle
    Padding(padding: const EdgeInsets.fromLTRB(16, 12, 16, 0),
      child: Row(children: [
        Text('OIL VISUALIZATION', style: Theme.of(context).textTheme.labelSmall
          ?.copyWith(color: const Color(0xFF6366F1), letterSpacing: 1.2)),
        const Spacer(),
        const Text('2D', style: TextStyle(fontSize: 12, color: Color(0xFF8A91B8))),
        Switch(value: show3D, onChanged: (v) => setState(() => show3D = v),
          activeColor: const Color(0xFF6366F1)),
        const Text('3D', style: TextStyle(fontSize: 12, color: Color(0xFF8A91B8))),
      ]),
    ),

    // Map canvas — needs SizedBox.expand so CustomPaint gets real dimensions
    Expanded(child: Container(
      margin: const EdgeInsets.fromLTRB(16, 8, 16, 8),
      decoration: BoxDecoration(
        color: const Color(0xFF0F1018),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: const Color(0xFF2A2D3A)),
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(12),
        child: show3D
          ? _buildInteractive3D()
          : CustomPaint(
              painter: OilMapPainter(
                oilMatrix: widget.oilMatrix,
                distance: widget.pattern.distance,
                name: widget.pattern.name),
              child: const SizedBox.expand()),
      ),
    )),

    // ── Color legend — individual swatches with labels ────────
    Container(
      margin: const EdgeInsets.fromLTRB(16, 0, 16, 8),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: const Color(0xFF1A1B26),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: const Color(0xFF2A2D3A)),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        const Text('OIL LEVEL', style: TextStyle(
          fontSize: 9, fontWeight: FontWeight.w600,
          color: Color(0xFF6366F1), letterSpacing: 1.0)),
        const SizedBox(height: 8),
        Row(children: [
          _swatch(const Color(0xFF32140A), 'Dry'),
          _swatch(const Color(0xFF8B4513), 'Trace'),
          _swatch(const Color(0xFFC0392B), 'Light'),
          _swatch(const Color(0xFFE67E22), 'Med-Lt'),
          _swatch(const Color(0xFFF1C40F), 'Medium'),
          _swatch(const Color(0xFF2ECC71), 'Med-Hvy'),
          _swatch(const Color(0xFF2980B9), 'Heavy'),
          _swatch(const Color(0xFF8E44AD), 'Max'),
        ]),
      ]),
    ),

    // Pattern info
    Container(
      padding: const EdgeInsets.fromLTRB(16, 4, 16, 12),
      child: Row(mainAxisAlignment: MainAxisAlignment.spaceAround, children: [
        _patInfo('Length', '${widget.pattern.distance.toInt()} ft'),
        _patInfo('Pattern', widget.pattern.name),
        _patInfo('Fwd rows', '${widget.pattern.fwdRows.length}'),
        _patInfo('Rev rows', '${widget.pattern.revRows.length}'),
      ]),
    ),
  ]);

  Widget _patInfo(String l, String v) => Column(children: [
    Text(v, style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: Colors.white)),
    Text(l, style: const TextStyle(fontSize: 10, color: Color(0xFF8A91B8))),
  ]);

  Widget _swatch(Color color, String label) => Expanded(child:
    Column(children: [
      Container(
        height: 14,
        margin: const EdgeInsets.symmetric(horizontal: 1),
        decoration: BoxDecoration(
          color: color,
          borderRadius: BorderRadius.circular(3)),
      ),
      const SizedBox(height: 3),
      Text(label,
        textAlign: TextAlign.center,
        style: const TextStyle(fontSize: 7.5, color: Color(0xFF8A91B8)),
      ),
    ]));
}

// ─── 3D Oil Painter ───────────────────────────────────────────

class _3DOilPainter extends CustomPainter {
  final List<List<double>> oilMatrix;
  final double distance;
  final Color Function(double) colorFn;
  final double yaw;
  final double pitch;
  final double zoom;

  const _3DOilPainter({
    required this.oilMatrix,
    required this.distance,
    required this.colorFn,
    required this.yaw,
    required this.pitch,
    required this.zoom,
  });

  static const int boards = 39;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      Paint()..color = const Color(0xFF111111),
    );
    if (oilMatrix.isEmpty) return;

    final int cols = oilMatrix.first.length;
    final double res = cols > 65 ? 0.25 : 1.0;
    final int distCols = (distance / res).round().clamp(2, cols);

    final smooth = _smoothMatrix(oilMatrix, boards, distCols);

    final double cx = size.width * 0.5; // moves 3d model left and right
    final double cy = size.height * 0.5; //moves 3d model up and down

    final double laneWidth = size.width * 0.30 * zoom;
    final double laneDepth = size.height * 1.28 * zoom;
    final double heightScale = size.height * 0.5 * zoom;

    final List<_Face3D> faces = [];

    for (int col = 0; col < distCols - 1; col++) {
      for (int b = 0; b < boards - 1; b++) {
        final double h00 = _heightify(smooth[b][col]);
        final double h10 = _heightify(smooth[b + 1][col]);
        final double h11 = _heightify(smooth[b + 1][col + 1]);
        final double h01 = _heightify(smooth[b][col + 1]);

        final _P3 a = _worldPoint(
          b.toDouble(),
          col.toDouble(),
          h00,
          laneWidth,
          laneDepth,
          heightScale,
          res,
        );
        final _P3 b1 = _worldPoint(
          (b + 1).toDouble(),
          col.toDouble(),
          h10,
          laneWidth,
          laneDepth,
          heightScale,
          res,
        );
        final _P3 c = _worldPoint(
          (b + 1).toDouble(),
          (col + 1).toDouble(),
          h11,
          laneWidth,
          laneDepth,
          heightScale,
          res,
        );
        final _P3 d = _worldPoint(
          b.toDouble(),
          (col + 1).toDouble(),
          h01,
          laneWidth,
          laneDepth,
          heightScale,
          res,
        );

        final _P3 ra = _rotate(a, yaw, pitch);
        final _P3 rb = _rotate(b1, yaw, pitch);
        final _P3 rc = _rotate(c, yaw, pitch);
        final _P3 rd = _rotate(d, yaw, pitch);

        final Offset pa = _project(ra, cx, cy);
        final Offset pb = _project(rb, cx, cy);
        final Offset pc = _project(rc, cx, cy);
        final Offset pd = _project(rd, cx, cy);

        final path = Path()
          ..moveTo(pa.dx, pa.dy)
          ..lineTo(pb.dx, pb.dy)
          ..lineTo(pc.dx, pc.dy)
          ..lineTo(pd.dx, pd.dy)
          ..close();

        final avgOil = (
          smooth[b][col] +
          smooth[b + 1][col] +
          smooth[b + 1][col + 1] +
          smooth[b][col + 1]
        ) / 4.0;

        final normal = _normalFromHeights(a, b1, c);
        final shade = _lighting(normal);

        faces.add(_Face3D(
          path: path,
          depth: (ra.y + rb.y + rc.y + rd.y) / 4.0,
          color: _shadeColor(colorFn(avgOil), shade),
          edge: avgOil > 0.015,
        ));
      }
    }

    faces.sort((m, n) => m.depth.compareTo(n.depth));

    for (final face in faces) {
      canvas.drawPath(face.path, Paint()..color = face.color);
      if (face.edge) {
        canvas.drawPath(
          face.path,
          Paint()
            ..color = Colors.black.withOpacity(0.08)
            ..style = PaintingStyle.stroke
            ..strokeWidth = 0.20,
        );
      }
    }

    _drawLaneOutline(canvas, laneWidth, laneDepth, cx, cy);
  }

  List<List<double>> _smoothMatrix(List<List<double>> src, int bCount, int cCount) {
    final tmp = List.generate(
      bCount,
      (b) => List<double>.generate(cCount, (c) => src[b][c]),
    );

    for (int pass = 0; pass < 2; pass++) {
      final out = List.generate(
        bCount,
        (_) => List<double>.filled(cCount, 0.0),
      );

      for (int b = 0; b < bCount; b++) {
        for (int c = 0; c < cCount; c++) {
          double sum = 0.0;
          double wsum = 0.0;

          for (int db = -1; db <= 1; db++) {
            for (int dc = -2; dc <= 2; dc++) {
              final int bb = (b + db).clamp(0, bCount - 1);
              final int cc = (c + dc).clamp(0, cCount - 1);

              final double wBoard = db == 0 ? 1.0 : 0.55;
              final int adc = dc.abs();
              final double wLane = adc == 0 ? 1.35 : (adc == 1 ? 1.0 : 0.55);

              final double w = wBoard * wLane;
              sum += tmp[bb][cc] * w;
              wsum += w;
            }
          }

          out[b][c] = sum / wsum;
        }
      }

      for (int b = 0; b < bCount; b++) {
        for (int c = 0; c < cCount; c++) {
          tmp[b][c] = out[b][c];
        }
      }
    }

    return tmp;
  }

  double _heightify(double v) {
    final double x = v.clamp(0.0, 1.0);
    return pow(x, 0.82).toDouble() * 0.48;
  }

  _P3 _worldPoint(
    double boardIdx,
    double colIdx,
    double v,
    double laneWidth,
    double laneDepth,
    double heightScale,
    double res,
  ) {
    final double x = ((boardIdx / (boards - 1)) - 0.5) * laneWidth;
    final double y = (((colIdx * res) / distance) - 0.5) * laneDepth;
    final double z = v * heightScale;
    return _P3(x, y, z);
  }

  _P3 _rotate(_P3 p, double yaw, double pitch) {
    final double cosy = cos(yaw);
    final double siny = sin(yaw);
    final double x1 = p.x * cosy - p.y * siny;
    final double y1 = p.x * siny + p.y * cosy;

    final double cosp = cos(pitch);
    final double sinp = sin(pitch);
    final double y2 = y1 * cosp;
    final double z2 = p.z + y1 * sinp;

    return _P3(x1, y2, z2);
  }

  Offset _project(_P3 p, double cx, double cy) {
    const double cam = 1100.0;
    final double depth = (cam + p.y + 500.0).clamp(250.0, 5000.0);
    final double s = cam / depth;
    return Offset(
      cx + p.x * s,
      cy + p.y * 0.16 * s - p.z * s,
    );
  }

  _P3 _normalFromHeights(_P3 a, _P3 b, _P3 c) {
    final double ux = b.x - a.x;
    final double uy = b.y - a.y;
    final double uz = b.z - a.z;

    final double vx = c.x - a.x;
    final double vy = c.y - a.y;
    final double vz = c.z - a.z;

    final double nx = uy * vz - uz * vy;
    final double ny = uz * vx - ux * vz;
    final double nz = ux * vy - uy * vx;

    final double len = sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-9) return const _P3(0, 0, 1);
    return _P3(nx / len, ny / len, nz / len);
  }

  double _lighting(_P3 n) {
    const double lx = -0.35;
    const double ly = -0.25;
    const double lz = 0.90;
    final double dot = (n.x * lx + n.y * ly + n.z * lz).clamp(-1.0, 1.0);
    return 0.72 + 0.28 * dot;
  }

  Color _shadeColor(Color c, double shade) {
    return Color.fromARGB(
      255,
      (c.red * shade).clamp(0, 255).round(),
      (c.green * shade).clamp(0, 255).round(),
      (c.blue * shade).clamp(0, 255).round(),
    );
  }

  void _drawLaneOutline(Canvas canvas, double laneWidth, double laneDepth, double cx, double cy) {
    final p0 = _project(_rotate(_P3(-laneWidth / 2, -laneDepth / 2, 0), yaw, pitch), cx, cy);
    final p1 = _project(_rotate(_P3(laneWidth / 2, -laneDepth / 2, 0), yaw, pitch), cx, cy);
    final p2 = _project(_rotate(_P3(laneWidth / 2, laneDepth / 2, 0), yaw, pitch), cx, cy);
    final p3 = _project(_rotate(_P3(-laneWidth / 2, laneDepth / 2, 0), yaw, pitch), cx, cy);

    final path = Path()
      ..moveTo(p0.dx, p0.dy)
      ..lineTo(p1.dx, p1.dy)
      ..lineTo(p2.dx, p2.dy)
      ..lineTo(p3.dx, p3.dy)
      ..close();

    canvas.drawPath(
      path,
      Paint()
        ..color = Colors.white.withOpacity(0.10)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );
  }

  @override
  bool shouldRepaint(_3DOilPainter old) =>
      old.oilMatrix != oilMatrix ||
      old.distance != distance ||
      old.yaw != yaw ||
      old.pitch != pitch ||
      old.zoom != zoom;
}

class _P3 {
  final double x;
  final double y;
  final double z;
  const _P3(this.x, this.y, this.z);
}

class _Face3D {
  final Path path;
  final double depth;
  final Color color;
  final bool edge;

  const _Face3D({
    required this.path,
    required this.depth,
    required this.color,
    required this.edge,
  });
}


// ============================================================
// _3DLanePainter  (Phase 2, item 3)
// Draws: lane surface, gutters, arrows, dots, pins, ball, oil overlay.
// Static geometry (lane, gutters, arrows, dots, oil) is cached into a
// dart:ui Picture and only rebuilt when camera/oil/size changes.
// Only ball path, ball, and pins are redrawn each frame.
// ============================================================
class _3DLanePainter extends CustomPainter {
  final List<PathPoint> path;
  final List<List<double>> oilMatrix;
  final double oilDistance;
  final double laneLengthFt;
  final int animIdx;
  final Color Function(double) oilColorFn;
  final double yaw;
  final double pitch;
  final double zoom;

  final double Function(double localYMeters) pinRadiusAtLocalHeight;
  final double pinHeightM;
  final double pinBaseRadiusM;
  final double pinMaxRadiusM;

  final List<_PinRenderState> pinStates;

  static const int BOARDS = 39;
  static const double LANE_WIDTH_IN = 41.5;
  static const double GUTTER_W_IN = 2.25;
  static const double HEAD_PIN_FT = 60.0;
  static const double PIN_SPACING_IN = 12.0;

  // ── Static geometry cache ──
  static ui.Picture? _cachedPicture;
  static double _cacheYaw = double.nan;
  static double _cachePitch = double.nan;
  static double _cacheZoom = double.nan;
  static Size _cacheSize = Size.zero;
  static List<List<double>>? _cacheOilMatrix;
  static double _cacheOilDistance = double.nan;

  const _3DLanePainter({
    required this.path,
    required this.oilMatrix,
    required this.oilDistance,
    required this.laneLengthFt,
    required this.animIdx,
    required this.oilColorFn,
    required this.yaw,
    required this.pitch,
    required this.zoom,
    required this.pinRadiusAtLocalHeight,
    required this.pinHeightM,
    required this.pinBaseRadiusM,
    required this.pinMaxRadiusM,
    required this.pinStates,
  });

  bool _staticCacheValid(Size size) {
    return _cachedPicture != null &&
        _cacheYaw == yaw &&
        _cachePitch == pitch &&
        _cacheZoom == zoom &&
        _cacheSize == size &&
        identical(_cacheOilMatrix, oilMatrix) &&
        _cacheOilDistance == oilDistance;
  }

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height),
        Paint()..color = const Color(0xFF0E0E10));

    final double cx = size.width * 0.5;
    final double cy = size.height * 0.5;
    final double baseSide = size.shortestSide;
    final double laneW = baseSide * 0.55 * zoom;
    final double laneD = baseSide * 4.8 * zoom;
    final double heightScale = baseSide * 0.6 * zoom;

    final double pxPerInchX = laneW / LANE_WIDTH_IN;

    _P3 world(double board, double ft, double z) {
      final double x = (board / (BOARDS - 1) - 0.5) * laneW;
      final double y = (ft / laneLengthFt - 0.5) * laneD;
      return _P3(x, y, z);
    }
    Offset proj(_P3 p) => _project(_rotate(p, yaw, pitch), cx, cy);

    // ── Draw cached static geometry (lane, gutters, arrows, dots, oil) ──
    if (!_staticCacheValid(size)) {
      final recorder = ui.PictureRecorder();
      final cCanvas = Canvas(recorder);

      cCanvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height),
          Paint()..color = const Color(0xFF0E0E10));

      final List<_Face3D> staticFaces = [];

      // ---- 1. LANE SURFACE + GUTTERS ----
      const Color laneColor = Color(0xFFE9C98A);
      const Color gutterColor = Color(0xFF2B2B2F);
      final double gutterBoards = (GUTTER_W_IN / LANE_WIDTH_IN) * (BOARDS - 1);

      const int ftSteps = 32;
      for (int i = 0; i < ftSteps; i++) {
        final double f0 = (i / ftSteps) * laneLengthFt;
        final double f1 = ((i + 1) / ftSteps) * laneLengthFt;

        staticFaces.add(_quadFace(
            world(0, f0, 0),
            world(BOARDS - 1.0, f0, 0),
            world(BOARDS - 1.0, f1, 0),
            world(0, f1, 0),
            laneColor, proj));

        staticFaces.add(_quadFace(
            world(-gutterBoards, f0, -0.015 * heightScale),
            world(0, f0, 0),
            world(0, f1, 0),
            world(-gutterBoards, f1, -0.015 * heightScale),
            gutterColor, proj));

        staticFaces.add(_quadFace(
            world(BOARDS - 1.0, f0, 0),
            world(BOARDS - 1.0 + gutterBoards, f0, -0.015 * heightScale),
            world(BOARDS - 1.0 + gutterBoards, f1, -0.015 * heightScale),
            world(BOARDS - 1.0, f1, 0),
            gutterColor, proj));
      }

      // ---- 2. OIL OVERLAY ----
      if (oilMatrix.isNotEmpty && oilDistance > 0) {
        final int cols = oilMatrix.first.length;
        final double res = cols > 65 ? 0.25 : 1.0;
        final int distCols = (oilDistance / res).round().clamp(2, cols);
        final double z = 0.0025 * heightScale;
        for (int col = 0; col < distCols; col++) {
          final double f0 = col * res;
          final double f1 = f0 + res;
          for (int b = 0; b < BOARDS; b++) {
            if (col >= oilMatrix[b].length) continue;
            final double v = oilMatrix[b][col];
            if (v < 0.02) continue;
            staticFaces.add(_quadFace(
                world(b.toDouble(), f0, z),
                world(b + 1.0, f0, z),
                world(b + 1.0, f1, z),
                world(b.toDouble(), f1, z),
                oilColorFn(v).withOpacity(0.55), proj));
          }
        }
      }

      // ---- 3. ARROWS ----
      for (final int b in const [4, 9, 14, 19, 24, 29, 34]) {
        final tip = proj(world(b.toDouble(), 15.0, 0.001 * heightScale));
        final bl  = proj(world(b - 0.9, 12.5, 0.001 * heightScale));
        final br  = proj(world(b + 0.9, 12.5, 0.001 * heightScale));
        final ap = Path()..moveTo(tip.dx, tip.dy)..lineTo(bl.dx, bl.dy)..lineTo(br.dx, br.dy)..close();
        staticFaces.add(_Face3D(
            path: ap,
            depth: _rotate(world(b.toDouble(), 13.75, 0), yaw, pitch).y,
            color: Colors.black.withOpacity(0.75),
            edge: false));
      }

      // ---- 4. DOTS ----
      for (int b = 4; b < BOARDS; b += 5) {
        final p = proj(world(b.toDouble(), 0.0, 0.001 * heightScale));
        final dotPath = Path()..addOval(Rect.fromCircle(center: p, radius: 2.2));
        staticFaces.add(_Face3D(
            path: dotPath,
            depth: _rotate(world(b.toDouble(), 0.0, 0), yaw, pitch).y,
            color: Colors.black.withOpacity(0.6),
            edge: false));
      }

      // Depth sort & draw static faces
      staticFaces.sort((m, n) => m.depth.compareTo(n.depth));
      for (final f in staticFaces) {
        cCanvas.drawPath(f.path, Paint()..color = f.color);
      }

      _cachedPicture = recorder.endRecording();
      _cacheYaw = yaw;
      _cachePitch = pitch;
      _cacheZoom = zoom;
      _cacheSize = size;
      _cacheOilMatrix = oilMatrix;
      _cacheOilDistance = oilDistance;
    }

    // Blit cached static geometry
    canvas.drawPicture(_cachedPicture!);

    // ── Dynamic: Pins (depth-sorted among themselves) ──
    final List<_Face3D> dynFaces = [];

    final double boardsPerMeter = (1.0 / LANE_WIDTH_IN) * 39.37 * (BOARDS - 1);
    final double ftPerMeter = 39.37 / 12.0;
    final double zPerMeterNorm = (pxPerInchX * 39.37) / heightScale;

    for (final ps in pinStates) {
      if (!ps.visible) continue;
      _drawPin3D(dynFaces, ps, world, proj,
          boardsPerMeter: boardsPerMeter,
          ftPerMeter: ftPerMeter,
          zPerMeterNorm: zPerMeterNorm,
          heightScale: heightScale);
    }

    dynFaces.sort((m, n) => m.depth.compareTo(n.depth));
    for (final f in dynFaces) {
      canvas.drawPath(f.path, Paint()..color = f.color);
    }

    // ── Dynamic: Ball path + ball (always on top) ──
    if (path.isNotEmpty) {
      final Paint pathPaint = Paint()
        ..color = Colors.redAccent.withOpacity(0.9)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0;
      final int upto = animIdx.clamp(0, path.length - 1);
      for (int i = 0; i < upto; i++) {
        final a = proj(world(path[i].board, path[i].ft, 0.01 * heightScale));
        final b = proj(world(path[i + 1].board, path[i + 1].ft, 0.01 * heightScale));
        canvas.drawLine(a, b, pathPaint);
      }
      final cur = path[upto];
      final ballCenter = proj(world(cur.board, cur.ft, 0.04 * heightScale));
      final double ballR = (baseSide * 0.018 * zoom).clamp(4.0, 18.0);
      canvas.drawCircle(ballCenter, ballR + 1.5,
          Paint()..color = Colors.black.withOpacity(0.45));
      canvas.drawCircle(ballCenter, ballR, Paint()..color = Colors.red.shade800);
      canvas.drawCircle(ballCenter, ballR,
          Paint()..color = Colors.black..style = PaintingStyle.stroke..strokeWidth = 1);
    }
  }

  _Face3D _quadFace(_P3 a, _P3 b, _P3 c, _P3 d, Color color,
      Offset Function(_P3) proj) {
    final pa = proj(a), pb = proj(b), pc = proj(c), pd = proj(d);
    final path = Path()
      ..moveTo(pa.dx, pa.dy)
      ..lineTo(pb.dx, pb.dy)
      ..lineTo(pc.dx, pc.dy)
      ..lineTo(pd.dx, pd.dy)
      ..close();
    final ra = _rotate(a, yaw, pitch);
    final rb = _rotate(b, yaw, pitch);
    final rc = _rotate(c, yaw, pitch);
    final rd = _rotate(d, yaw, pitch);
    return _Face3D(
      path: path,
      depth: (ra.y + rb.y + rc.y + rd.y) / 4.0,
      color: color,
      edge: false,
    );
  }

  void _drawPin3D(
    List<_Face3D> faces,
    _PinRenderState ps,
    _P3 Function(double, double, double) world,
    Offset Function(_P3) proj, {
    required double boardsPerMeter,
    required double ftPerMeter,
    required double zPerMeterNorm,
    required double heightScale,
  }) {
    const int rings = 14;
    const int ribs = 10;
    final double hM = pinHeightM;

    // Tilt direction decomposed into board and ft components
    final double tiltCos = cos(ps.tiltRad);
    final double tiltSin = sin(ps.tiltRad);
    // tiltDirection: angle in XZ plane — 0 = +Z(ft), pi/2 = +X(board)
    final double dirBoard = sin(ps.tiltDirection); // how much tilt goes into board axis
    final double dirFt = cos(ps.tiltDirection);    // how much tilt goes into ft axis

    final List<List<_P3>> ringPts = [];
    for (int r = 0; r <= rings; r++) {
      final double t = r / rings;
      final double localY = t * hM;
      final double radM = pinRadiusAtLocalHeight(localY);
      final List<_P3> ring = [];

      // Vertical rise after tilt
      final double verticalM = localY * tiltCos;
      // Horizontal displacement from tilt
      final double fallM = localY * tiltSin;

      for (int k = 0; k < ribs; k++) {
        final double ang = (k / ribs) * 2 * pi;
        final double dxM = radM * cos(ang);
        final double dzM = radM * sin(ang);

        final double boardOff = (dxM + fallM * dirBoard) * boardsPerMeter;
        final double ftOff = (dzM + fallM * dirFt) * ftPerMeter;
        final double zW = verticalM * zPerMeterNorm * heightScale;

        ring.add(world(ps.board + boardOff, ps.ft + ftOff, zW));
      }
      ringPts.add(ring);
    }

    final Color pinBody = Colors.white;
    final Color pinStripe = Colors.red.shade700;
    for (int r = 0; r < rings; r++) {
      final double t = r / rings;
      final bool stripe = (t > 0.63 && t < 0.73);
      final Color c = stripe ? pinStripe : pinBody;
      for (int k = 0; k < ribs; k++) {
        final int k2 = (k + 1) % ribs;
        faces.add(_quadFace(
            ringPts[r][k],
            ringPts[r][k2],
            ringPts[r + 1][k2],
            ringPts[r + 1][k],
            c, proj));
      }
    }
  }

  _P3 _rotate(_P3 p, double yaw, double pitch) {
    final double cy = cos(yaw), sy = sin(yaw);
    final double x1 = p.x * cy - p.y * sy;
    final double y1 = p.x * sy + p.y * cy;
    final double cp = cos(pitch), sp = sin(pitch);
    return _P3(x1, y1 * cp, p.z + y1 * sp);
  }

  Offset _project(_P3 p, double cx, double cy) {
    const double cam = 1100.0;
    final double depth = (cam + p.y + 500.0).clamp(250.0, 5000.0);
    final double s = cam / depth;
    return Offset(cx + p.x * s, cy + p.y * 0.16 * s - p.z * s);
  }

  @override
  bool shouldRepaint(_3DLanePainter old) =>
      old.yaw != yaw ||
      old.pitch != pitch ||
      old.zoom != zoom ||
      old.animIdx != animIdx ||
      old.path != path ||
      old.oilMatrix != oilMatrix ||
      old.pinStates != pinStates;
}

class _PinRenderState {
  final double board;
  final double ft;
  final double tiltRad;
  final double tiltDirection; // radians: 0 = +ft, pi/2 = +board, etc.
  final bool visible;
  const _PinRenderState({
    required this.board,
    required this.ft,
    required this.tiltRad,
    this.tiltDirection = 0.0,
    required this.visible,
  });
}