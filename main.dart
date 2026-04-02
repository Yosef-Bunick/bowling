



"""
AI Makeup Engine v5.0 - Main Application
=========================================
Bridge between GUI and Engine
- Camera capture + pose validation
- 3-view capture workflow
- Generates face analysis + step-by-step instructions
- Renders per-step makeup overlay previews

This version keeps the 1.15 engine calls, but uses a new 4-panel GUI layout
matching the 4.3 workflow.
"""

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import tkinter as tk

from engine import (
    MakeupEngine,
    generate_instructions,
)
from gui import MakeupGUI, SUCCESS_COLOR, WARNING_COLOR, TEXT_COLOR


# =============================================================================
# CAPTURE STATE
# =============================================================================


@dataclass
class CaptureState:
    """Track 3-view capture progress"""

    views: List[str] = field(default_factory=lambda: ["LEFT", "CENTER", "RIGHT"])
    captured: List[bool] = field(default_factory=lambda: [False, False, False])
    current_index: int = 0

    @property
    def current_view(self) -> str:
        if self.current_index < len(self.views):
            return self.views[self.current_index]
        return "DONE"

    @property
    def all_captured(self) -> bool:
        return all(self.captured)

    def mark_captured(self):
        if self.current_index < len(self.captured):
            self.captured[self.current_index] = True
            self.current_index += 1

    def reset(self):
        self.captured = [False, False, False]
        self.current_index = 0


# =============================================================================
# POSE VALIDATION
# =============================================================================


@dataclass
class PoseConfig:
    yaw_left: float = 25.0
    yaw_right: float = -25.0
    yaw_center_tol: float = 10.0
    pitch_tol: float = 20.0
    yaw_hard_limit: float = 60.0
    pitch_hard_limit: float = 40.0


def check_pose(view: str, yaw: float, pitch: float, cfg: PoseConfig) -> Tuple[bool, str]:
    """Check if current pose matches required view"""
    if abs(yaw) > cfg.yaw_hard_limit:
        return False, f"Turn less ({abs(yaw):.0f}° > {cfg.yaw_hard_limit:.0f}°)"
    if abs(pitch) > cfg.pitch_hard_limit:
        return False, f"Level head ({abs(pitch):.0f}° > {cfg.pitch_hard_limit:.0f}°)"

    if abs(pitch) > cfg.pitch_tol:
        return False, f"Level head (pitch: {pitch:+.0f}°)"

    if view == "LEFT":
        if yaw >= cfg.yaw_left:
            return True, "✓ LEFT view OK"
        return False, f"Turn RIGHT more ({yaw:+.0f}° < {cfg.yaw_left:+.0f}°)"

    if view == "RIGHT":
        if yaw <= cfg.yaw_right:
            return True, "✓ RIGHT view OK"
        return False, f"Turn LEFT more ({yaw:+.0f}° > {cfg.yaw_right:+.0f}°)"

    if view == "CENTER":
        if abs(yaw) <= cfg.yaw_center_tol:
            return True, "✓ CENTER view OK"
        return False, f"Face forward ({yaw:+.0f}°)"

    return False, "Unknown view"


# =============================================================================
# APPLICATION CONTROLLER
# =============================================================================


STEP_NAMES = [
    "All",
    "Concealer",
    "Contour",
    "Blush",
    "Highlight",
    "Bronzer",
    "Brows",
    "Eyeshadow",
    "Eyeliner",
    "Lips",
]

STEP_TO_PRODUCTS: Dict[str, List[str]] = {
    "Concealer": ["concealer"],
    "Contour": ["contour"],
    "Blush": ["blush"],
    "Highlight": ["highlight"],
    "Bronzer": ["bronzer"],
    "Brows": ["brow"],
    "Eyeshadow": ["eyeshadow"],
    "Eyeliner": ["eyeliner"],
    "Lips": ["lip", "lip_gloss"],
}


class MakeupApp:
    def __init__(self):
        self.root = tk.Tk()
        self.gui = MakeupGUI(self.root)
        self.engine = MakeupEngine()

        self.capture_state = CaptureState()
        self.pose_config = PoseConfig()

        # Camera
        self.cap: Optional[cv2.VideoCapture] = None
        self.camera_running = False
        self.camera_thread: Optional[threading.Thread] = None

        # Shared camera data (thread-safe)
        self._lock = threading.Lock()
        self._latest_display: Optional[np.ndarray] = None
        self._latest_result: Optional[Dict] = None
        self._latest_fps: float = 0.0
        self._ui_after_id: Optional[str] = None
        self._last_frame_time = time.time()

        # Current frame/result
        self.current_frame: Optional[np.ndarray] = None
        self.current_result: Optional[Dict] = None

        self.processed = False

        self._bind_callbacks()
        self._set_default_instructions()

    # ---------------------------------------------------------------------
    # GUI bindings
    # ---------------------------------------------------------------------

    def _bind_callbacks(self):
        self.gui.on_start_camera = self.start_camera
        self.gui.on_stop_camera = self.stop_camera
        self.gui.on_capture = self.capture_view
        self.gui.on_reset = self.reset_captures
        self.gui.on_process = self.process_captures
        self.gui.on_style_change = self.change_style
        self.gui.on_celebrity_select = self.select_celebrity
        self.gui.on_export_image = self.export_image
        self.gui.on_export_guide = self.export_guide

    def _set_default_instructions(self):
        self.gui.set_instructions(
            [
                {
                    "step": 1,
                    "name": "Capture",
                    "description": "Start camera → capture LEFT, CENTER, RIGHT → Process.",
                    "tips": ["Make sure your face is centered", "Good lighting helps"],
                }
            ]
        )

    # ---------------------------------------------------------------------
    # Camera controls
    # ---------------------------------------------------------------------

    def start_camera(self):
        if self.camera_running:
            return

        import platform

        backends = [cv2.CAP_ANY]
        if platform.system() == "Windows":
            backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        elif platform.system() == "Linux":
            backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        elif platform.system() == "Darwin":
            backends = [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]

        self.cap = None
        for backend in backends:
            for idx in [0, 1, 2]:
                try:
                    cap = cv2.VideoCapture(idx, backend)
                    if cap.isOpened():
                        ok, _ = cap.read()
                        if ok:
                            self.cap = cap
                            break
                    cap.release()
                except Exception:
                    continue
            if self.cap and self.cap.isOpened():
                break

        if not self.cap or not self.cap.isOpened():
            self.gui.show_message("Error", "No camera found (tried indices 0-2)", "error")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        try:
            self.engine.detector.reset_smoothing()
        except Exception:
            pass

        self.camera_running = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()

        # Start UI tick (throttled) – avoids Tk event-queue flooding
        self._schedule_ui_tick()

        self.gui.update_status("Camera: ON", SUCCESS_COLOR)

    def stop_camera(self):
        self.camera_running = False

        if self._ui_after_id is not None:
            try:
                self.root.after_cancel(self._ui_after_id)
            except Exception:
                pass
            self._ui_after_id = None

        if self.camera_thread:
            self.camera_thread.join(timeout=1.0)
            self.camera_thread = None

        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        self.gui.update_status("Camera: OFF", WARNING_COLOR)

    def _camera_loop(self):
        while self.camera_running and self.cap:
            try:
                ok, frame = self.cap.read()
                if not ok:
                    continue

                frame = cv2.flip(frame, 1)
                self.current_frame = frame.copy()

                try:
                    result = self.engine.process_frame(frame)
                except Exception:
                    result = None

                self.current_result = result

                # FPS
                now = time.time()
                dt = now - self._last_frame_time
                self._last_frame_time = now
                fps = 1.0 / max(dt, 1e-3)

                display = self._draw_overlay(frame, result)

                with self._lock:
                    self._latest_display = display
                    self._latest_result = result
                    # simple EMA fps smoothing
                    self._latest_fps = 0.85 * self._latest_fps + 0.15 * fps
            except Exception:
                continue

    def _schedule_ui_tick(self):
        self._ui_after_id = self.root.after(33, self._ui_tick)  # ~30fps

    def _ui_tick(self):
        if not self.camera_running:
            return

        with self._lock:
            frame = None if self._latest_display is None else self._latest_display.copy()
            result = self._latest_result
            fps = float(self._latest_fps)

        if frame is not None:
            self.gui.update_video(frame)
            self.gui.update_fps(fps)
            self.gui.update_captures(self.capture_state.captured)

            if result:
                yaw = result.get("yaw", 0.0)
                pitch = result.get("pitch", 0.0)
                view = self.capture_state.current_view
                self.gui.update_pose(yaw, pitch, view)

        self._schedule_ui_tick()

    def _draw_overlay(self, frame: np.ndarray, result: Optional[Dict]) -> np.ndarray:
        display = frame.copy()
        h, w = display.shape[:2]

        if result:
            pts = result["pts"]
            yaw = float(result["yaw"])
            pitch = float(result["pitch"])

            # sparse face mesh
            for i in range(0, len(pts), 10):
                x, y = int(pts[i][0]), int(pts[i][1])
                cv2.circle(display, (x, y), 1, (0, 255, 0), -1)

            view = self.capture_state.current_view
            ok, msg = check_pose(view, yaw, pitch, self.pose_config)
            color = (0, 255, 0) if ok else (0, 165, 255)

            cv2.putText(display, f"View: {view}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display, msg, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            cv2.putText(display, f"Yaw: {yaw:+.1f}", (w - 130, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display, f"Pitch: {pitch:+.1f}", (w - 130, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            arrow_x = int(w / 2 + yaw * 2)
            arrow_y = int(h / 2 + pitch * 2)
            cv2.arrowedLine(display, (w // 2, h // 2), (arrow_x, arrow_y), color, 2, tipLength=0.3)
        else:
            cv2.putText(display, "No face detected", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # capture status
        status = []
        for v, c in zip(["L", "C", "R"], self.capture_state.captured):
            status.append(f"[{v}{'✓' if c else ' '}]")
        cv2.putText(display, " ".join(status), (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        return display

    # ---------------------------------------------------------------------
    # Capture controls
    # ---------------------------------------------------------------------

    def capture_view(self):
        if self.capture_state.all_captured:
            self.gui.show_message("Info", "All views captured. Press Process or Reset.")
            return

        if self.current_frame is None or self.current_result is None:
            self.gui.show_message("Warning", "No frame available", "warning")
            return

        view = self.capture_state.current_view
        yaw = float(self.current_result.get("yaw", 0.0))
        pitch = float(self.current_result.get("pitch", 0.0))

        ok, msg = check_pose(view, yaw, pitch, self.pose_config)
        if not ok:
            self.gui.show_message("Pose Error", f"Cannot capture: {msg}", "warning")
            return

        success = self.engine.capture_view(self.current_frame, view)
        if success:
            self.capture_state.mark_captured()
            self.gui.update_captures(self.capture_state.captured)
            self.gui.update_status(f"Captured {view}!", SUCCESS_COLOR)
            if self.capture_state.all_captured:
                self.gui.update_status("All views captured! Press Process.", SUCCESS_COLOR)
        else:
            self.gui.show_message("Error", "Capture failed", "error")

    def reset_captures(self):
        self.capture_state.reset()
        self.engine.reset()
        self.processed = False

        try:
            self.engine.captures.clear()
            self.engine.fused_depth = None
            self.engine.face_metrics = None
            self.engine.skin_tone = None
        except Exception:
            pass

        self.gui.update_captures(self.capture_state.captured)
        self.gui.update_status("Captures reset", TEXT_COLOR)
        self._set_default_instructions()
        self.gui.set_step_overlays({})

    # ---------------------------------------------------------------------
    # Processing + rendering
    # ---------------------------------------------------------------------

    def process_captures(self):
        if not self.capture_state.all_captured:
            self.gui.show_message("Warning", "Capture all 3 views first", "warning")
            return

        self.gui.update_status("Processing...", WARNING_COLOR)

        try:
            ok = self.engine.process_captures()
            if not ok:
                self.gui.show_message("Error", "Processing failed", "error")
                self.gui.update_status("Processing failed", WARNING_COLOR)
                return

            self.processed = True

            # Update analysis
            metrics = self.engine.face_metrics
            if metrics:
                shape_weights = {
                    "round": metrics.shape_round,
                    "oval": metrics.shape_oval,
                    "square": metrics.shape_square,
                    "heart": metrics.shape_heart,
                    "oblong": metrics.shape_oblong,
                }
                feature_scores = {
                    "cheekbone_prominence": metrics.cheekbone_prominence,
                    "jaw_sharpness": metrics.jaw_sharpness,
                    "eye_roundness": metrics.eye_roundness,
                    "brow_arch": metrics.brow_arch,
                    "lip_fullness": metrics.lip_fullness,
                    "forehead_height_score": metrics.forehead_height_score,
                }
                ratios = {
                    "face_width_ratio": metrics.face_width_ratio,
                    "jaw_width_ratio": metrics.jaw_width_ratio,
                    "forehead_ratio": metrics.forehead_ratio,
                    "eye_distance_ratio": metrics.eye_distance_ratio,
                    "nose_length_ratio": metrics.nose_length_ratio,
                    "lip_width_ratio": metrics.lip_width_ratio,
                }
                self.gui.update_analysis(shape_weights, feature_scores, ratios)

            skin = self.engine.skin_tone
            if skin:
                palette = self.engine.get_color_palette()
                concealer_color = palette.get("concealer", (200, 180, 160))
                self.gui.update_skin(skin.undertone, skin.depth, concealer_color)
                self.gui.update_palette(palette)

            if metrics and skin:
                instructions = generate_instructions(metrics, self.engine.current_style, skin)
                self.gui.set_instructions(instructions)

            self.gui.update_celebrities(self.engine.get_celebrity_matches())

            self._regen_step_overlays()

            self.gui.update_status("Processing complete!", SUCCESS_COLOR)
        except Exception as e:
            self.gui.show_message("Error", f"Processing error: {str(e)}", "error")
            self.gui.update_status("Processing failed", WARNING_COLOR)

    def _compute_masks_colors_for_view(self, idx: int) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, Tuple[int, int, int]]]:
        """Compute projected + aggregated masks and a compatible color dict for capture idx."""
        cap = self.engine.captures[idx]
        frame = cap.original_bgr.copy()

        masks_face = self.engine.generate_masks(cap.pts_pixel, cap.axes)
        masks_img_detail = self.engine.project_masks_to_image(masks_face, cap.axes, frame.shape[:2])
        masks = self.engine.aggregate_masks(masks_img_detail)

        palette = self.engine.get_color_palette()
        colors = dict(palette)
        # map eyeshadow -> base shade
        if "eyeshadow" not in colors:
            colors["eyeshadow"] = colors.get("eyeshadow_base", (160, 130, 160))
        # simple gloss color if missing
        if "lip_gloss" not in colors:
            lip = np.array(colors.get("lip", (180, 80, 80)), dtype=np.float32)
            gloss = tuple(np.clip(lip * 0.6 + 255 * 0.4, 0, 255).astype(np.uint8).tolist())
            colors["lip_gloss"] = gloss

        return frame, masks, colors

    def _regen_step_overlays(self):
        if not self.processed or len(self.engine.captures) < 2:
            return

        # Build previews for each of the 3 captures (L/C/R)
        view_map = {"L": 0, "C": 1, "R": 2}

        # Cache per-view computed masks/colors once
        per_view: Dict[str, Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, Tuple[int, int, int]]]] = {}
        for v, i in view_map.items():
            if i >= len(self.engine.captures) or self.engine.captures[i] is None:
                continue
            per_view[v] = self._compute_masks_colors_for_view(i)

        step_images: Dict[str, Dict[str, np.ndarray]] = {}

        # All
        step_images["All"] = {}
        for v, (frame, masks, colors) in per_view.items():
            step_images["All"][v] = self.engine.apply_makeup(frame.copy(), masks, colors)

        # Each step
        for step in STEP_NAMES:
            if step == "All":
                continue
            step_images[step] = {}
            prods = STEP_TO_PRODUCTS.get(step, [])
            for v, (frame, masks, colors) in per_view.items():
                step_masks = {k: masks[k] for k in prods if k in masks}
                if not step_masks:
                    step_images[step][v] = frame.copy()
                else:
                    step_images[step][v] = self.engine.apply_makeup(frame.copy(), step_masks, colors)

        self.gui.set_step_overlays(step_images)

    # ---------------------------------------------------------------------
    # Style controls
    # ---------------------------------------------------------------------

    def change_style(self, style_name: str):
        self.engine.set_style(style_name)
        if not self.processed:
            return

        metrics = self.engine.face_metrics
        skin = self.engine.skin_tone
        if metrics and skin:
            self.gui.set_instructions(generate_instructions(metrics, self.engine.current_style, skin))

        self._regen_step_overlays()

    def select_celebrity(self, celebrity_name: str):
        self.engine.set_celebrity(celebrity_name)

        if self.processed:
            self._regen_step_overlays()

        celeb = self.engine.current_celebrity
        if celeb:
            self.gui.show_message("Style Applied", f"Applied {celeb.name}'s makeup style")

    # ---------------------------------------------------------------------
    # Export
    # ---------------------------------------------------------------------

    def export_image(self):
        if not self.processed:
            self.gui.show_message("Warning", "Process captures first", "warning")
            return

        filepath = self.gui.ask_save_file(
            "Save Image",
            [("PNG files", "*.png"), ("JPEG files", "*.jpg")],
            ".png",
        )
        if not filepath:
            return

        frame, masks, colors = self._compute_masks_colors_for_view(1)
        out = self.engine.apply_makeup(frame, masks, colors)
        cv2.imwrite(filepath, out)
        self.gui.show_message("Success", f"Image saved to {filepath}")

    def export_guide(self):
        if not self.processed:
            self.gui.show_message("Warning", "Process captures first", "warning")
            return

        filepath = self.gui.ask_save_file(
            "Save Guide",
            [("JSON files", "*.json"), ("Text files", "*.txt")],
            ".json",
        )
        if not filepath:
            return

        metrics = self.engine.face_metrics
        skin = self.engine.skin_tone
        if not metrics or not skin:
            self.gui.show_message("Error", "Missing face metrics/skin tone", "error")
            return

        instructions = generate_instructions(metrics, self.engine.current_style, skin)

        guide = {
            "style": self.engine.current_style.name,
            "skin_tone": {"undertone": skin.undertone, "depth": skin.depth},
            "face_shape": {
                "round": metrics.shape_round,
                "oval": metrics.shape_oval,
                "square": metrics.shape_square,
                "heart": metrics.shape_heart,
                "oblong": metrics.shape_oblong,
            },
            "instructions": instructions,
            "colors": {k: list(v) for k, v in self.engine.get_color_palette().items()},
        }

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(guide, f, indent=2)

        self.gui.show_message("Success", f"Guide saved to {filepath}")

    # ---------------------------------------------------------------------
    # Main
    # ---------------------------------------------------------------------

    def run(self):
        try:
            self.gui.run()
        finally:
            self.stop_camera()
            self.engine.close()


def main():
    app = MakeupApp()
    app.run()


if __name__ == "__main__":
    main()
