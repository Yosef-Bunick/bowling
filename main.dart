import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'models/physics.dart';
import 'painters/lane_painter.dart';

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
    animTimer = Timer.periodic(const Duration(milliseconds: 25), (t) {
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
    _debounce = Timer(const Duration(milliseconds: 180), () {
      if (mounted) _recalc();
    });
  }

  void _scheduleRebuild() {
    _debounce?.cancel();
    _debounce = Timer(const Duration(milliseconds: 180), () {
      if (mounted) _rebuild();
    });
  }

  void _togglePlay() {
    setState(() => playing = !playing);
    if (playing) {
      animTimer = Timer.periodic(const Duration(milliseconds: 25), (t) {
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
    body: TabBarView(controller: _tabs, children: [
      _BowlerTab(bowler: bowler, onChanged: (b) { bowler = b; _scheduleRecalc(); }),
      _BallTab(ball: ball, onChanged: (b) { ball = b; _scheduleRecalc(); }),
      _PatternTab(pattern: pattern, onChanged: (p) { pattern = p; _scheduleRebuild(); }),
      _LaneTab(simResult: simResult, oilMatrix: oilMatrix,
        animIdx: animIdx, playing: playing,
        onPlay: _togglePlay, onReset: _rebuild, onRecalc: _recalc),
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
        b.angleDeg, -78, 78, 1, (v) { setState(() => b.angleDeg = v); _emit(); }, unit: '°'),

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
class _LaneTab extends StatelessWidget {
  final SimResult? simResult;
  final List<List<double>> oilMatrix;
  final int animIdx; final bool playing;
  final VoidCallback onPlay, onReset, onRecalc;
  const _LaneTab({required this.simResult, required this.oilMatrix,
    required this.animIdx, required this.playing,
    required this.onPlay, required this.onReset, required this.onRecalc});

  @override
  Widget build(BuildContext context) {
    final r    = simResult;
    final path = r?.path ?? [];
    final cur  = animIdx < path.length ? path[animIdx] : null;

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
          Expanded(child: _InfoCard('Pin RPM',
            r.pinRPM.toStringAsFixed(0))),
          const SizedBox(width: 8),
          Expanded(child: _InfoCard('Pin AR',
            '${r.pinAR.toStringAsFixed(1)}°')),
          const SizedBox(width: 8),
          Expanded(child: _InfoCard('Pin Board',
            r.pinBoard.toStringAsFixed(1),
            valueColor: r.pinBoard>=16&&r.pinBoard<=18 ? Colors.greenAccent : Colors.white)),
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

      // Controls
      Padding(padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(children: [
          ElevatedButton.icon(
            style: ElevatedButton.styleFrom(
              backgroundColor: const Color(0xFF6366F1),
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: onPlay,
            icon: Icon(playing ? Icons.pause : Icons.play_arrow, size: 18),
            label: Text(playing ? 'Pause' : 'Play'),
          ),
          const SizedBox(width: 8),
          OutlinedButton(
            style: OutlinedButton.styleFrom(
              side: const BorderSide(color: Color(0xFF2A2D3A)),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: onReset, child: const Text('Reset'),
          ),
          const SizedBox(width: 8),
          OutlinedButton(
            style: OutlinedButton.styleFrom(
              side: const BorderSide(color: Color(0xFF2A2D3A)),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
            onPressed: onRecalc, child: const Text('Recalc'),
          ),
          const Spacer(),
          // Phase legend
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
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(12),
          child: CustomPaint(
            painter: LanePainter(path: path, oilMatrix: oilMatrix, animIdx: animIdx),
            child: Container(),
          ),
        ),
      )),
    ]);
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
          ? CustomPaint(
              painter: _3DOilPainter(
                oilMatrix: widget.oilMatrix,
                distance: widget.pattern.distance,
                colorFn: oilColor),
              child: const SizedBox.expand())
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
  const _3DOilPainter({
    required this.oilMatrix,
    required this.distance,
    required this.colorFn,
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(Rect.fromLTWH(0,0,size.width,size.height),
      Paint()..color = const Color(0xFF111111));
    if (oilMatrix.isEmpty) return;

    const int B = 39;
    final int cols = oilMatrix[0].length;
    final double res = cols > 65 ? 0.25 : 1.0;  // detect resolution
    final int distCols = (distance / res).round();  // columns to render
    
    final iX = size.width * 0.5, iY = size.height * 0.88;
    final sX = size.width / B * 0.52;
    final sY = size.height * 0.42, sZ = size.height * 0.42;

    Offset proj(int b, int col, double v) {
      final ftPos = col * res;  // convert column to feet for projection
      final bx = (b - B / 2) * sX;
      final fy = (distance - ftPos) * sY / distance;
      final oz = v * sZ;
      return Offset(iX + bx * 0.98 - fy * 0.42, iY - fy * 0.52 - oz + bx * 0.07);
    }

    for (int col = 0; col < distCols - 1 && col < cols - 1; col++) {
      for (int b = 0; b < B - 1 && b < oilMatrix.length - 1; b++) {
        final v = oilMatrix[b][col];
        final p = Path()
          ..moveTo(proj(b,   col,   v).dx,                        proj(b,   col,   v).dy)
          ..lineTo(proj(b+1, col,   oilMatrix[b+1][col]).dx,      proj(b+1, col,   oilMatrix[b+1][col]).dy)
          ..lineTo(proj(b+1, col+1, oilMatrix[b+1][col+1]).dx,    proj(b+1, col+1, oilMatrix[b+1][col+1]).dy)
          ..lineTo(proj(b,   col+1, oilMatrix[b][col+1]).dx,      proj(b,   col+1, oilMatrix[b][col+1]).dy)
          ..close();
        canvas.drawPath(p, Paint()..color = colorFn(v));
        canvas.drawPath(p, Paint()
          ..color = Colors.black.withOpacity(0.10)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 0.25);
      }
    }
  }

  @override
  bool shouldRepaint(_3DOilPainter o) =>
    o.oilMatrix != oilMatrix || o.distance != distance;
}
