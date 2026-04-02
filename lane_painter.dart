import 'dart:math';
import 'package:flutter/material.dart';
import '../models/physics.dart';

class LanePainter extends CustomPainter {
  final List<PathPoint> path;
  final List<List<double>> oilMatrix;
  final int animIdx;

  LanePainter({required this.path, required this.oilMatrix, required this.animIdx});

  static const int BOARDS = 39;
  static const int LANE_FT = 60;

  @override
  void paint(Canvas canvas, Size size) {
    final bw = size.width / BOARDS;
    final fh = size.height / LANE_FT;

    // Gutters (left and right edges, dark)
    canvas.drawRect(Rect.fromLTWH(-bw * 1.5, 0, bw * 1.5, size.height),
      Paint()..color = const Color(0xFF333333));
    canvas.drawRect(Rect.fromLTWH(size.width, 0, bw * 1.5, size.height),
      Paint()..color = const Color(0xFF333333));

    // Draw wood planks
    for (int b = 0; b < BOARDS; b++) {
      final paint = Paint()
        ..color = b % 5 == 0 ? const Color(0xFFC8A85A) : const Color(0xFFD4B46A);
      canvas.drawRect(Rect.fromLTWH(b * bw, 0, bw, size.height), paint);
      canvas.drawRect(
        Rect.fromLTWH(b * bw, 0, bw, size.height),
        Paint()..color = Colors.black.withOpacity(0.06)..style = PaintingStyle.stroke..strokeWidth = 0.5,
      );
    }

    // Oil overlay
    if (oilMatrix.isNotEmpty) {
      for (int f = 0; f < LANE_FT; f++) {
        for (int b = 0; b < BOARDS && b < oilMatrix.length; b++) {
          final v = oilMatrix[b][f];
          if (v > 0.02) {
            canvas.drawRect(
              Rect.fromLTWH(b * bw, (LANE_FT - f - 1) * fh, bw, fh),
              Paint()..color = Color.fromRGBO(20, 60, 200, v * 0.42),
            );
          }
        }
      }
    }

    // Distance markers
    final textPaint = Paint()..color = Colors.black.withOpacity(0.28);
    for (int f = 10; f < 60; f += 10) {
      canvas.drawLine(
        Offset(0, (LANE_FT - f) * fh),
        Offset(size.width, (LANE_FT - f) * fh),
        Paint()..color = Colors.black.withOpacity(0.18)..strokeWidth = 1,
      );
      _drawText(canvas, '${f}ft', Offset(2, (LANE_FT - f) * fh - 10), 9, Colors.black.withOpacity(0.3));
    }

    // Board numbers
    for (int b = 4; b < BOARDS; b += 5) {
      _drawText(canvas, '${b + 1}', Offset(b * bw + 1, size.height - 12), 8, Colors.black.withOpacity(0.25));
    }

    if (path.isEmpty) return;

    // Trail
    final end = (animIdx).clamp(0, path.length - 1);
    for (int i = 1; i <= end; i++) {
      final p = path[i];
      final pp = path[i - 1];
      Color col;
      if (p.phase == 'roll') col = Colors.green.withOpacity(0.75);
      else if (p.phase == 'hook') col = Colors.orange.withOpacity(0.85);
      else col = Colors.red.withOpacity(0.65);

      canvas.drawLine(
        Offset((pp.board - 0.5) * bw, (LANE_FT - pp.ft) * fh),
        Offset((p.board - 0.5) * bw, (LANE_FT - p.ft) * fh),
        Paint()..color = col..strokeWidth = p.atBreakpoint ? 4 : 2..strokeCap = StrokeCap.round,
      );
    }

    // Ball
    if (animIdx < path.length) {
      final p = path[animIdx];
      final bx = (p.board - 0.5) * bw;
      final by = (LANE_FT - p.ft) * fh;
      final br = max(7.0, bw * 0.85);

      // Breakpoint ring
      if (p.atBreakpoint) {
        canvas.drawCircle(Offset(bx, by), br + 4,
          Paint()..color = Colors.orange.withOpacity(0.85)..style = PaintingStyle.stroke..strokeWidth = 3);
      }

      // Shadow
      canvas.drawOval(
        Rect.fromCenter(center: Offset(bx + 2, by + 3), width: br * 2, height: br * 0.8),
        Paint()..color = Colors.black.withOpacity(0.15),
      );

      // Ball color by phase
      Color ballColor;
      if (p.atBreakpoint) ballColor = Colors.red;
      else if (p.phase == 'roll') ballColor = const Color(0xFF27AE60);
      else if (p.phase == 'hook') ballColor = const Color(0xFFE67E22);
      else ballColor = const Color(0xFFC0392B);

      canvas.drawCircle(Offset(bx, by), br, Paint()..color = ballColor);
      canvas.drawCircle(Offset(bx, by), br,
        Paint()..color = Colors.black.withOpacity(0.2)..style = PaintingStyle.stroke..strokeWidth = 1);

      // Rotation line
      canvas.drawLine(
        Offset(bx, by),
        Offset(bx + br * cos(p.theta), by + br * sin(p.theta)),
        Paint()..color = Colors.white.withOpacity(0.85)..strokeWidth = 1.5,
      );

      // Finger holes
      for (final offset in [1.0, 2.3]) {
        final hx = bx + br * 0.4 * cos(p.theta + offset);
        final hy = by + br * 0.4 * sin(p.theta + offset);
        canvas.drawCircle(Offset(hx, hy), br * (offset < 2 ? 0.17 : 0.11),
          Paint()..color = Colors.black.withOpacity(0.55));
      }
    }
  }

  void _drawText(Canvas canvas, String text, Offset offset, double size, Color color) {
    final tp = TextPainter(
      text: TextSpan(text: text, style: TextStyle(fontSize: size, color: color)),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, offset);
  }

  @override
  bool shouldRepaint(LanePainter old) =>
    old.animIdx != animIdx || old.path != path || old.oilMatrix != oilMatrix;
}

class OilMapPainter extends CustomPainter {
  final List<List<double>> oilMatrix;
  final double distance;
  final String name;

  OilMapPainter({required this.oilMatrix, required this.distance, required this.name});

  static const int BOARDS = 39;

  Color _oilColor(double v) {
    const stops = [
      [0.0, Color(0xFF32140A)],
      [0.12, Color(0xFF8B4513)],
      [0.28, Color(0xFFC0392B)],
      [0.42, Color(0xFFE67E22)],
      [0.56, Color(0xFFF1C40F)],
      [0.68, Color(0xFF2ECC71)],
      [0.80, Color(0xFF2980B9)],
      [1.00, Color(0xFF8E44AD)],
    ];
    for (int i = 1; i < stops.length; i++) {
      if (v <= (stops[i][0] as double)) {
        final t = (v - (stops[i-1][0] as double)) /
                  ((stops[i][0] as double) - (stops[i-1][0] as double));
        final a = stops[i-1][1] as Color;
        final b = stops[i][1] as Color;
        return Color.fromRGBO(
          (a.red + (b.red - a.red) * t).round(),
          (a.green + (b.green - a.green) * t).round(),
          (a.blue + (b.blue - a.blue) * t).round(), 1);
      }
    }
    return const Color(0xFF8E44AD);
  }

  @override
  void paint(Canvas canvas, Size size) {
    final dist = distance.toInt();
    final bw = size.width / BOARDS;
    final fh = size.height / dist;

    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height),
      Paint()..color = const Color(0xFF222222));

    if (oilMatrix.isEmpty) return;

    for (int f = 0; f < dist; f++) {
      for (int b = 0; b < BOARDS && b < oilMatrix.length; b++) {
        final v = oilMatrix[b][f];
        if (v > 0.01) {
          canvas.drawRect(
            Rect.fromLTWH(b * bw, (dist - f - 1) * fh, bw + 0.5, fh + 0.5),
            Paint()..color = _oilColor(v),
          );
        }
      }
    }

    for (int f = 0; f < dist; f += 5) {
      canvas.drawLine(Offset(0, (dist - f) * fh), Offset(size.width, (dist - f) * fh),
        Paint()..color = Colors.white.withOpacity(0.12)..strokeWidth = 0.5);
      _drawText(canvas, '${f}ft', Offset(2, (dist - f) * fh - 10), 9, Colors.white.withOpacity(0.55));
    }

    _drawText(canvas, name, Offset(size.width / 2 - name.length * 3, 4), 11, Colors.white.withOpacity(0.8));
  }

  void _drawText(Canvas canvas, String text, Offset offset, double size, Color color) {
    final tp = TextPainter(
      text: TextSpan(text: text, style: TextStyle(fontSize: size, color: color)),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, offset);
  }

  @override
  bool shouldRepaint(OilMapPainter old) => old.oilMatrix != oilMatrix;
}

class CrossSectionPainter extends CustomPainter {
  final List<List<double>>? oilMatrix;
  const CrossSectionPainter({this.oilMatrix});

  static const int BOARDS = 39;
  static const int LANE_FT = 60;

  @override
  void paint(Canvas canvas, Size size) {
    if (oilMatrix == null) return;

    const padL = 30.0, padR = 8.0, padT = 10.0, padB = 22.0;
    final cW = size.width - padL - padR;
    final cH = size.height - padT - padB;

    // Totals per board
    final tot = List<double>.filled(BOARDS, 0);
    for (int b = 0; b < BOARDS; b++) {
      for (int f = 0; f < LANE_FT; f++) {
        tot[b] += oilMatrix![b][f];
      }
    }
    final maxT = tot.reduce(max).clamp(0.001, double.infinity);
    final bw = cW / BOARDS;

    // Grid
    for (int i = 0; i <= 4; i++) {
      final y = padT + cH * (1 - i / 4);
      canvas.drawLine(Offset(padL, y), Offset(padL + cW, y),
        Paint()..color = Colors.grey.withOpacity(0.3)..strokeWidth = 0.5);
      _drawText(canvas, '${(maxT * i / 4).toStringAsFixed(0)}',
        Offset(0, y - 5), 8, Colors.grey);
    }

    // Fill shape
    final fillPath = Path()..moveTo(padL, padT + cH);
    for (int b = 0; b < BOARDS; b++) {
      fillPath.lineTo(padL + b * bw + bw / 2, padT + cH - (tot[b] / maxT) * cH);
    }
    fillPath
      ..lineTo(padL + cW, padT + cH)
      ..close();
    canvas.drawPath(fillPath, Paint()..color = const Color(0xFFB04A4A).withOpacity(0.45));

    // Outline
    final outPath = Path()..moveTo(padL, padT + cH);
    for (int b = 0; b < BOARDS; b++) {
      outPath.lineTo(padL + b * bw + bw / 2, padT + cH - (tot[b] / maxT) * cH);
    }
    canvas.drawPath(outPath, Paint()..color = const Color(0xFFC85050).withOpacity(0.9)
      ..style = PaintingStyle.stroke..strokeWidth = 1.5);

    // Board labels
    for (final b in [1, 5, 10, 15, 20, 25, 30, 35, 39]) {
      _drawText(canvas, '$b', Offset(padL + (b - 1) * bw - 3, size.height - 12),
        8, Colors.grey);
    }
  }

  void _drawText(Canvas canvas, String text, Offset offset, double size, Color color) {
    final tp = TextPainter(
      text: TextSpan(text: text, style: TextStyle(fontSize: size, color: color)),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, offset);
  }

  @override
  bool shouldRepaint(CrossSectionPainter old) => old.oilMatrix != oilMatrix;
}
