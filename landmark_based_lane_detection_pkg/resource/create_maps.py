"""
Interactive cone map generator with load, strong smoothing, and export-transform metadata.

Features:
- Load existing maps and continue editing: reads cones plus metadata from JSON.
- Strong, configurable smoothing:
  * "Smooth iters": number of Chaikin iterations (0..30).
  * "Smooth alpha": Chaikin corner-cut ratio (0.10..0.49). Larger => slicker.
  * Smoothing is circular; the start/end are not fixed and are smoothed as well.
- Canvas is a 15x15 field grid with integer intersections in [-2..13] for both axes.
  The (0,0) grid intersection is highlighted explicitly.
- JSON save includes full metadata and the export transform (start, rotation).
  On load, if export-transform metadata is present, cones are inverse-transformed
  back to the originally drawn pose before reconstructing the centerline.

Export format:
{
  "pylones": [{"x": float, "y": float, "is_right": bool}, ...],
  "metadata": {
    "width": float,
    "spacing": float,
    "clockwise": bool,
    "smoothing_iters": int,
    "chaikin_alpha": float,
    "canvas": {"xmin": int, "xmax": int, "ymin": int, "ymax": int},
    "export_transform": {
      "start_point": [float, float],              # start point prior to export alignment
      "rotation_applied_rad": float               # rotation applied to align +y heading
    }
  }
}
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox
import numpy as np


# ------------------------------ Data model ------------------------------ #

@dataclass
class Config:
    """Configuration editable in the UI."""
    width: float = 1.0
    spacing: float = 0.5
    smoothing_iters: int = 4
    chaikin_alpha: float = 0.25
    clockwise: bool = True
    outfile: str = "map.json"
    infile: str = "map.json"
    # Canvas bounds: 15x15 fields, grid intersections at integers in [-2, 13].
    xmin: int = -2
    xmax: int = 13
    ymin: int = -2
    ymax: int = 13


# ------------------------------ Geometry utils ------------------------------ #

def close_loop(poly: np.ndarray) -> np.ndarray:
    """
    Return a closed polyline by appending the start point to the end if needed.
    """
    if poly.shape[0] < 2:
        return poly
    if np.linalg.norm(poly[0] - poly[-1]) > 0.0:
        return np.vstack([poly, poly[0]])
    return poly

def chaikin_circular(poly: np.ndarray, iterations: int = 1, alpha: float = 0.25) -> np.ndarray:
    """
    Circular Chaikin smoothing on a closed ring (no fixed endpoints).

    Parameters
    ----------
    poly : (N,2) array
        Closed ring WITHOUT duplicated final point; the algorithm treats connectivity cyclically.
    iterations : int
        Number of Chaikin passes.
    alpha : float
        Corner-cut ratio in (0, 0.5). Typical 0.25. Higher => stronger smoothing.

    Returns
    -------
    (M,2) array closed ring WITHOUT duplicated endpoint. Use close_loop() for drawing if needed.
    """
    pts = np.asarray(poly, dtype=float)
    if pts.shape[0] < 3 or iterations <= 0:
        return pts

    a = float(alpha)
    b = 1.0 - a

    for _ in range(int(iterations)):
        n = pts.shape[0]
        q = b * pts + a * np.roll(pts, -1, axis=0)
        r = a * pts + b * np.roll(pts, -1, axis=0)
        out = np.empty((n * 2, 2), dtype=float)
        out[0::2] = q
        out[1::2] = r
        pts = out

    return pts

def resample_by_arclength(poly: np.ndarray, step: float) -> np.ndarray:
    """
    Resample a closed polyline at near-uniform arc-length spacing. Expects closed WITH duplicated endpoint.
    """
    diffs = np.diff(poly, axis=0)
    seglen = np.sqrt((diffs ** 2).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = s[-1]
    n = max(int(np.round(total / step)), 3)
    s_new = np.linspace(0.0, total, n + 1)
    out = np.zeros((n + 1, 2), dtype=float)
    j = 0
    for i, si in enumerate(s_new):
        while j < len(s) - 1 and s[j + 1] < si:
            j += 1
        denom = s[j + 1] - s[j]
        t = 0.0 if denom == 0.0 else (si - s[j]) / denom
        out[i] = poly[j] * (1.0 - t) + poly[j + 1] * t
    return out

def stations_along(poly: np.ndarray, spacing: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Return points and unit tangents at stations 0, spacing, ... up to length.
    Expects closed WITH duplicated endpoint.
    """
    diffs = np.diff(poly, axis=0)
    seglen = np.sqrt((diffs ** 2).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = s[-1]
    s_query = np.arange(0.0, total, max(spacing, 1e-6))
    pts = np.zeros((len(s_query), 2), dtype=float)
    tan = np.zeros((len(s_query), 2), dtype=float)
    j = 0
    for i, si in enumerate(s_query):
        while j < len(seglen) and s[j + 1] < si:
            j += 1
        denom = seglen[j]
        t = 0.0 if denom == 0.0 else (si - s[j]) / denom
        p0 = poly[j]
        p1 = poly[j + 1]
        pts[i] = p0 * (1.0 - t) + p1 * t
        d = p1 - p0
        nrm = np.linalg.norm(d)
        tan[i] = np.array([1.0, 0.0]) if nrm == 0.0 else d / nrm
    return pts, tan

def unit_tangents(poly: np.ndarray) -> np.ndarray:
    """
    Compute unit tangents per vertex for a closed polyline WITH duplicated endpoint.
    """
    core = poly[:-1]
    diffs = np.roll(core, -1, axis=0) - np.roll(core, 1, axis=0)
    norms = np.linalg.norm(diffs, axis=1, keepdims=True)
    norms[norms == 0.0] = 1.0
    T = diffs / norms
    return np.vstack([T, T[0]])

def offset_lines(centerline: np.ndarray, half_width: float, clockwise: bool) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute continuous left/right offset polylines for preview. Expects closed WITH duplicated endpoint.
    """
    T = unit_tangents(centerline)
    n_right = np.column_stack([T[:, 1], -T[:, 0]]) if clockwise else np.column_stack([-T[:, 1], T[:, 0]])
    n_left = -n_right
    return centerline + half_width * n_left, centerline + half_width * n_right

def generate_cones(centerline: np.ndarray, width: float, spacing: float, clockwise: bool) -> List[Tuple[float, float, bool]]:
    """
    Generate cone tuples (x, y, is_right) at equal stations from a centerline.
    Expects closed WITH duplicated endpoint.
    """
    half = width / 2.0
    pts, tan = stations_along(centerline, spacing)
    if clockwise:
        n_right = np.column_stack([tan[:, 1], -tan[:, 0]])
    else:
        n_right = np.column_stack([-tan[:, 1], tan[:, 0]])
    n_left = -n_right
    left_cones = pts + half * n_left
    right_cones = pts + half * n_right
    cones: List[Tuple[float, float, bool]] = []
    for p in left_cones:
        cones.append((float(p[0]), float(p[1]), False))
    for p in right_cones:
        cones.append((float(p[0]), float(p[1]), True))
    return cones

def rotate(points: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rotate 2D points around the origin by angle_rad."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    R = np.array([[c, -s], [s, c]], dtype=float)
    return points @ R.T

def rotate_cone_list(cones: List[Tuple[float, float, bool]], angle_rad: float) -> List[Tuple[float, float, bool]]:
    """Rotate list of cone positions around the origin by angle_rad."""
    if not cones:
        return cones
    xy = np.array([[x, y] for x, y, _ in cones], dtype=float)
    xy_r = rotate(xy, angle_rad)
    out: List[Tuple[float, float, bool]] = []
    for (xr, yr), (_, _, is_right) in zip(xy_r, cones):
        out.append((float(xr), float(yr), is_right))
    return out

def translate_cone_list(cones: List[Tuple[float, float, bool]], dx: float, dy: float) -> List[Tuple[float, float, bool]]:
    """Translate list of cone positions by (dx, dy)."""
    return [(float(x + dx), float(y + dy), is_right) for x, y, is_right in cones]

def round_cones(cones: List[Tuple[float, float, bool]], ndigits: int = 2) -> List[Tuple[float, float, bool]]:
    """Round coordinates for export."""
    return [(round(x, ndigits), round(y, ndigits), is_right) for x, y, is_right in cones]

def cones_from_json(path: str) -> Tuple[List[Tuple[float, float, bool]], Dict[str, Any]]:
    """
    Load cones and optional metadata from a JSON file.

    Returns
    -------
    cones : list[(x,y,is_right)]
    metadata : dict (may be empty)
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    items = data.get("pylones", [])
    cones: List[Tuple[float, float, bool]] = []
    for it in items:
        x = float(it["x"])
        y = float(it["y"])
        is_right = bool(it.get("is_right", False))
        cones.append((x, y, is_right))
    metadata = data.get("metadata", {}) if isinstance(data, dict) else {}
    return cones, metadata

def infer_centerline_from_cones(cones: List[Tuple[float, float, bool]]) -> Tuple[np.ndarray, float, float]:
    """
    Infer centerline polyline, track width, and spacing from cones.

    Assumptions:
    - Cones carry 'is_right' flags preserving per-side order along the track.
    - If flags look unusable, fallback splits the list into two halves.
    """
    if len(cones) < 6:
        return np.empty((0, 2), dtype=float), 1.0, 0.5

    left = [(x, y) for x, y, is_right in cones if not is_right]
    right = [(x, y) for x, y, is_right in cones if is_right]

    if len(left) == len(right) and len(left) >= 3:
        L = np.array(left, dtype=float)
        R = np.array(right, dtype=float)
    else:
        n = len(cones) // 2
        L = np.array([[cones[i][0], cones[i][1]] for i in range(n)], dtype=float)
        R = np.array([[cones[i][0], cones[i][1]] for i in range(n, 2 * n)], dtype=float)

    C = 0.5 * (L + R)
    C = close_loop(C)

    width = float(np.mean(np.linalg.norm(R - L, axis=1)))
    seg = np.sqrt(((C[1:] - C[:-1]) ** 2).sum(axis=1))
    spacing = float(np.median(seg[seg > 1e-6])) if np.any(seg > 1e-6) else 0.5

    return C, width, spacing


class Drawer:
    """
    Interactive drawing app with parameter sliders, load/save, smoothing, and export alignment.
    """

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg

        # Figure and axes
        self.fig = plt.figure(figsize=(9.2, 9.8))

        # Reserve a taller bottom panel so UI never overlaps the plot
        self.ax = self.fig.add_axes([0.06, 0.32, 0.88, 0.66])
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(self.cfg.xmin, self.cfg.xmax)
        self.ax.set_ylim(self.cfg.ymin, self.cfg.ymax)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title("Draw the centerline (press and drag). Release to finish.")
        self._draw_grid()

        # Plots
        self.line_raw, = self.ax.plot([], [], linewidth=1.2, label="raw")
        self.centerline_plot, = self.ax.plot([], [], linewidth=2.0, label="centerline")
        self.left_plot, = self.ax.plot([], [], linestyle=":", label="left")
        self.right_plot, = self.ax.plot([], [], linestyle=":", label="right")
        self.cone_scatter = self.ax.scatter([], [])
        self.origin_axes, = self.ax.plot([0], [0], marker="o", markersize=5, label="(0,0)")

        # --- Controls: dedicated bottom panel (no overlap with plot) ---

        # Sliders (left column) - wide enough so value readouts stay inside the slider
        self.s_width = Slider(self.fig.add_axes([0.06, 0.23, 0.36, 0.035]), "Width [m]", 0.2, 5.0, valinit=self.cfg.width)
        self.s_spacing = Slider(self.fig.add_axes([0.06, 0.185, 0.36, 0.035]), "Spacing [m]", 0.2, 2.0, valinit=self.cfg.spacing)
        self.s_smooth = Slider(self.fig.add_axes([0.06, 0.14, 0.36, 0.035]), "Smooth iters", 0, 30, valinit=self.cfg.smoothing_iters, valstep=1)
        self.s_alpha = Slider(self.fig.add_axes([0.06, 0.095, 0.36, 0.035]), "Smooth alpha", 0.10, 0.49, valinit=self.cfg.chaikin_alpha)

        # File I/O (right column)
        self.tb_in  = TextBox(self.fig.add_axes([0.48, 0.205, 0.32, 0.040]), "Load:", initial=self.cfg.infile)
        self.tb_out = TextBox(self.fig.add_axes([0.48, 0.155, 0.32, 0.040]), "Save:", initial=self.cfg.outfile)

        # Buttons - two rows, clear of slider readouts
        self.btn_apply = Button(self.fig.add_axes([0.48, 0.105, 0.14, 0.045]), "Apply")
        self.btn_load  = Button(self.fig.add_axes([0.66, 0.105, 0.14, 0.045]), "Load")
        self.btn_save  = Button(self.fig.add_axes([0.84, 0.105, 0.10, 0.045]), "Save")

        self.btn_redo  = Button(self.fig.add_axes([0.48, 0.055, 0.14, 0.045]), "Redo")
        self.btn_quit  = Button(self.fig.add_axes([0.66, 0.055, 0.14, 0.045]), "Quit")

        self.btn_apply.on_clicked(self.on_apply)
        self.btn_load.on_clicked(self.on_load)
        self.btn_save.on_clicked(self.on_save)
        self.btn_redo.on_clicked(self.on_redo)
        self.btn_quit.on_clicked(self.on_quit)

        # Short help text for smoothing parameters (requested descriptions)
        self.fig.text(
            0.06, 0.042,
            "Smoothing: 'iters' = Chaikin passes; 'alpha' = corner-cut ratio (0.10..0.49).\n"
            "Higher iters/alpha => stronger smoothing (more rounded, denser polyline).",
            fontsize=9
        )

        # Mouse events
        self.cid_press = self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.cid_move = self.fig.canvas.mpl_connect("motion_notify_event", self.on_move)
        self.cid_release = self.fig.canvas.mpl_connect("button_release_event", self.on_release)

        # State
        self.drawing = False
        self.raw: List[Tuple[float, float]] = []
        self.centerline: np.ndarray | None = None
        self.left_line: np.ndarray | None = None
        self.right_line: np.ndarray | None = None
        self.cones_preview: List[Tuple[float, float, bool]] | None = None
        self.cones_export: List[Tuple[float, float, bool]] | None = None

        # Export transform state (for metadata)
        self.last_start_point: Tuple[float, float] | None = None
        self.last_rotation_applied: float | None = None

    # ----- helpers ----- #

    def _draw_grid(self) -> None:
        """Draw a 1 m grid from cfg.(xmin,ymin) to cfg.(xmax,ymax)."""
        for x in range(self.cfg.xmin, self.cfg.xmax + 1):
            self.ax.axvline(x, color="0.85", linewidth=0.5)
        for y in range(self.cfg.ymin, self.cfg.ymax + 1):
            self.ax.axhline(y, color="0.85", linewidth=0.5)

    # ----- events ----- #

    def on_press(self, event) -> None:
        if event.inaxes != self.ax:
            return
        self.drawing = True
        self.raw = [(event.xdata, event.ydata)]
        self.line_raw.set_data([event.xdata], [event.ydata])
        self.fig.canvas.draw_idle()

    def on_move(self, event) -> None:
        if not self.drawing or event.inaxes != self.ax:
            return
        self.raw.append((event.xdata, event.ydata))
        xs, ys = zip(*self.raw)
        self.line_raw.set_data(xs, ys)
        self.fig.canvas.draw_idle()

    def on_release(self, event) -> None:
        if not self.drawing or event.inaxes != self.ax:
            return
        self.drawing = False
        self._regenerate_from_raw()

    def on_apply(self, _event) -> None:
        if self.raw:
            self._regenerate_from_raw()
        else:
            self.ax.set_title("Parameters updated. Draw the centerline.")
            self.fig.canvas.draw_idle()

    def on_load(self, _event) -> None:
        """
        Load cones JSON, inverse-transform to original drawn pose if metadata exists,
        infer centerline, set sliders from metadata (or estimates), and regenerate preview.
        """
        path = self.tb_in.text.strip()
        self.cfg.infile = path
        if not path or not os.path.exists(path):
            self.ax.set_title(f"Load failed: file not found '{path}'.")
            self.fig.canvas.draw_idle()
            return

        cones, meta = cones_from_json(path)

        # If export-transform metadata exists, inverse-transform cones to drawn pose.
        et = meta.get("export_transform", {}) if isinstance(meta, dict) else {}
        if "start_point" in et and "rotation_applied_rad" in et:
            sx, sy = float(et["start_point"][0]), float(et["start_point"][1])
            rot = float(et["rotation_applied_rad"])
            # Inverse: rotate by -rot, then translate by +start
            cones = rotate_cone_list(cones, -rot)
            cones = translate_cone_list(cones, sx, sy)

        # Infer centerline and estimates
        C, width_est, spacing_est = infer_centerline_from_cones(cones)
        if C.size == 0:
            self.ax.set_title("Load failed: not enough cones in file.")
            self.fig.canvas.draw_idle()
            return

        # Update sliders from metadata when available, else from estimates
        width_val = float(meta.get("width", width_est))
        spacing_val = float(meta.get("spacing", spacing_est))
        smooth_iters_val = int(meta.get("smoothing_iters", self.cfg.smoothing_iters))
        alpha_val = float(meta.get("chaikin_alpha", self.cfg.chaikin_alpha))
        clockwise_val = bool(meta.get("clockwise", self.cfg.clockwise))

        self.cfg.clockwise = clockwise_val
        self.s_width.set_val(np.clip(width_val, self.s_width.valmin, self.s_width.valmax))
        self.s_spacing.set_val(np.clip(spacing_val, self.s_spacing.valmin, self.s_spacing.valmax))
        self.s_smooth.set_val(np.clip(smooth_iters_val, self.s_smooth.valmin, self.s_smooth.valmax))
        self.s_alpha.set_val(np.clip(alpha_val, self.s_alpha.valmin, self.s_alpha.valmax))

        # Optional: update canvas bounds from metadata
        if "canvas" in meta:
            cmeta = meta["canvas"]
            self.cfg.xmin = int(cmeta.get("xmin", self.cfg.xmin))
            self.cfg.xmax = int(cmeta.get("xmax", self.cfg.xmax))
            self.cfg.ymin = int(cmeta.get("ymin", self.cfg.ymin))
            self.cfg.ymax = int(cmeta.get("ymax", self.cfg.ymax))
            self.ax.set_xlim(self.cfg.xmin, self.cfg.xmax)
            self.ax.set_ylim(self.cfg.ymin, self.cfg.ymax)

        # Use the inferred centerline as 'raw' for continued editing
        self.raw = [(float(p[0]), float(p[1])) for p in C]
        xs, ys = zip(*self.raw)
        self.line_raw.set_data(xs, ys)

        self.ax.set_title(f"Loaded '{os.path.basename(path)}'. Adjust and Apply, or Redo to redraw.")
        self._regenerate_from_raw()

    def on_save(self, _event) -> None:
        if not self.cones_export:
            self.ax.set_title("Nothing to save. Draw/Load and Apply first.")
            self.fig.canvas.draw_idle()
            return
        out_path = self.tb_out.text.strip() or self.cfg.outfile
        self.cfg.outfile = out_path

        metadata = {
            "width": float(self.cfg.width),
            "spacing": float(self.cfg.spacing),
            "clockwise": bool(self.cfg.clockwise),
            "smoothing_iters": int(self.cfg.smoothing_iters),
            "chaikin_alpha": float(self.cfg.chaikin_alpha),
            "canvas": {
                "xmin": int(self.cfg.xmin),
                "xmax": int(self.cfg.xmax),
                "ymin": int(self.cfg.ymin),
                "ymax": int(self.cfg.ymax),
            },
            "export_transform": {
                "start_point": [float(self.last_start_point[0]) if self.last_start_point else 0.0,
                                float(self.last_start_point[1]) if self.last_start_point else 0.0],
                "rotation_applied_rad": float(self.last_rotation_applied if self.last_rotation_applied is not None else 0.0),
            },
        }

        data = {
            "pylones": [{"x": x, "y": y, "is_right": is_right} for x, y, is_right in self.cones_export],
            "metadata": metadata,
        }
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self.ax.set_title(f"Saved to {out_path}.")
        self.fig.canvas.draw_idle()

    def on_redo(self, _event) -> None:
        self.raw = []
        self.centerline = None
        self.left_line = None
        self.right_line = None
        self.cones_preview = None
        self.cones_export = None
        self.last_start_point = None
        self.last_rotation_applied = None
        self.line_raw.set_data([], [])
        self.centerline_plot.set_data([], [])
        self.left_plot.set_data([], [])
        self.right_plot.set_data([], [])
        self.cone_scatter.remove()
        self.cone_scatter = self.ax.scatter([], [])
        self.ax.set_title("Draw the centerline (press and drag). Release to finish.")
        self.fig.canvas.draw_idle()

    def on_quit(self, _event) -> None:
        plt.close(self.fig)

    # ----- core regeneration ----- #

    def _regenerate_from_raw(self) -> None:
        """
        Regenerate smoothed centerline, preview lines, and export cones.
        """
        # Update config from sliders
        self.cfg.width = float(self.s_width.val)
        self.cfg.spacing = float(self.s_spacing.val)
        self.cfg.smoothing_iters = int(self.s_smooth.val)
        self.cfg.chaikin_alpha = float(self.s_alpha.val)

        if len(self.raw) < 3:
            self.ax.set_title("Too few points. Redraw or Load a map.")
            self.fig.canvas.draw_idle()
            return

        # Build closed ring WITHOUT duplicate for smoothing
        pts = np.array(self.raw, dtype=float)
        if np.linalg.norm(pts[0] - pts[-1]) > 0.0:
            pts_ring = pts
        else:
            pts_ring = pts[:-1]

        # Circular Chaikin smoothing, then close (duplicate) for downstream ops
        sm_ring = chaikin_circular(pts_ring, iterations=self.cfg.smoothing_iters, alpha=self.cfg.chaikin_alpha)
        sm_closed = close_loop(sm_ring)

        # Uniform resampling for clean tangents
        centerline = resample_by_arclength(sm_closed, step=0.05)

        # Preview: offset lines and cones at drawn/loaded position
        left_line, right_line = offset_lines(centerline, self.cfg.width / 2.0, self.cfg.clockwise)
        cones_preview = generate_cones(centerline, self.cfg.width, self.cfg.spacing, self.cfg.clockwise)

        # Export: align start to origin and heading to +y
        start = centerline[0].copy()
        pts_s, tan_s = stations_along(centerline, max(0.5, self.cfg.spacing))
        t0 = tan_s[0] if len(tan_s) > 0 else np.array([0.0, 1.0])
        ang = math.atan2(t0[1], t0[0])
        rot = math.pi / 2.0 - ang

        shifted = centerline - start
        cones_export = generate_cones(shifted, self.cfg.width, self.cfg.spacing, self.cfg.clockwise)
        cones_export = rotate_cone_list(cones_export, rot)
        cones_export = round_cones(cones_export, ndigits=2)

        # Store for metadata
        self.last_start_point = (float(start[0]), float(start[1]))
        self.last_rotation_applied = float(rot)

        # Store for drawing
        self.centerline = centerline
        self.left_line = left_line
        self.right_line = right_line
        self.cones_preview = cones_preview
        self.cones_export = cones_export

        # Draw preview (no translation or rotation)
        self.centerline_plot.set_data(self.centerline[:, 0], self.centerline[:, 1])
        self.left_plot.set_data(self.left_line[:, 0], self.left_line[:, 1])
        self.right_plot.set_data(self.right_line[:, 0], self.right_line[:, 1])

        self.cone_scatter.remove()
        cone_xy = np.array([[x, y] for x, y, _ in self.cones_preview], dtype=float)
        self.cone_scatter = self.ax.scatter(cone_xy[:, 0], cone_xy[:, 1], s=15)

        self.ax.set_title("Preview. Adjust sliders and Apply, or Save/Redo/Quit.")
        self.fig.canvas.draw_idle()

    # ----- run ----- #

    def run(self) -> None:
        plt.show()


def main() -> None:
    cfg = Config()
    Drawer(cfg).run()


if __name__ == "__main__":
    main()
