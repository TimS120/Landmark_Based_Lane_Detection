#!/usr/bin/env python3
"""Interactive cone map generator with UI params and export-only alignment.

This version implements the user's latest requirements:

- Loop closure is **always** enforced by adding a straight line segment from
  the end to the start (no distance threshold).
- Parameters (track width, cone spacing, max deviation) are adjustable via
  UI sliders **before and after** drawing. "Apply" regenerates the result.
- The preview keeps the course at the **drawn position** (no translation).
- The start orientation is computed from the first 0.5 m of the centerline.
  The **exported JSON** is translated so that the start is at (0, 0) and
  rotated so the forward tangent at s=0 points along the +y-axis.
- Cones lie ON the offset lines at width/2 per side; cones are paired at equal
  stations starting at s=0.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import numpy as np


# ------------------------------ Data model ------------------------------ #

@dataclass
class Config:
    """Configuration editable in the UI."""
    width: float = 1.0
    spacing: float = 0.5
    deviation_max: float = 0.25
    canvas_w: int = 10
    canvas_h: int = 10
    clockwise: bool = True
    outfile: str = "map.json"


# ------------------------------ Geometry utils ------------------------------ #

def close_loop_always(poly: np.ndarray) -> np.ndarray:
    """Return a closed polyline by appending the start point to the end.

    A straight segment from last to first is always used to close the loop.
    """
    if poly.shape[0] < 2:
        return poly
    if np.linalg.norm(poly[0] - poly[-1]) > 0.0:
        return np.vstack([poly, poly[0]])
    return poly

def chaikin_smooth(poly: np.ndarray, iterations: int = 1) -> np.ndarray:
    """Apply one or more iterations of Chaikin's corner-cutting to a closed polyline."""
    pts = poly.copy()
    for _ in range(iterations):
        q = 0.75 * pts[:-1] + 0.25 * pts[1:]
        r = 0.25 * pts[:-1] + 0.75 * pts[1:]
        pts = np.empty((q.shape[0] + r.shape[0], 2), dtype=float)
        pts[0::2] = q
        pts[1::2] = r
        pts = np.vstack([pts, pts[0]])
    return pts

def resample_by_arclength(poly: np.ndarray, step: float) -> np.ndarray:
    """Resample a closed polyline at near-uniform arc-length spacing."""
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

def max_deviation(ref: np.ndarray, cur: np.ndarray) -> float:
    """Approximate maximum pointwise distance between two closed polylines."""
    step = max(1, len(cur) // 500)
    max_d = 0.0
    for p in cur[::step]:
        d = np.sqrt(((ref - p) ** 2).sum(axis=1)).min()
        if d > max_d:
            max_d = float(d)
    return max_d

def bounded_smooth(poly: np.ndarray, deviation_max: float) -> np.ndarray:
    """Smooth a closed polyline with a deviation cap using Chaikin + resample."""
    base = resample_by_arclength(close_loop_always(poly), step=0.05)
    best = base
    for _ in range(1, 8):
        cand = chaikin_smooth(best, iterations=1)
        cand = resample_by_arclength(cand, step=0.05)
        if max_deviation(base, cand) <= deviation_max:
            best = cand
        else:
            break
    return best

def unit_tangents(poly: np.ndarray) -> np.ndarray:
    """Compute unit tangents per vertex for a closed polyline."""
    diffs = np.roll(poly, -1, axis=0) - np.roll(poly, 1, axis=0)
    norms = np.linalg.norm(diffs, axis=1, keepdims=True)
    norms[norms == 0.0] = 1.0
    return diffs / norms

def stations_along(poly: np.ndarray, spacing: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return points and unit tangents at stations 0, spacing, ... up to length."""
    diffs = np.diff(poly, axis=0)
    seglen = np.sqrt((diffs ** 2).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = s[-1]
    s_query = np.arange(0.0, total, spacing)
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

def offset_lines(centerline: np.ndarray, half_width: float, clockwise: bool) -> Tuple[np.ndarray, np.ndarray]:
    """Compute continuous left/right offset polylines for preview."""
    T = unit_tangents(centerline)
    n_right = np.column_stack([T[:, 1], -T[:, 0]]) if clockwise else np.column_stack([-T[:, 1], T[:, 0]])
    n_left = -n_right
    return centerline + half_width * n_left, centerline + half_width * n_right

def generate_cones(centerline: np.ndarray, width: float, spacing: float, clockwise: bool) -> List[Tuple[float, float, bool]]:
    """Generate cone tuples (x, y, is_right) at equal stations from a centerline."""
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

def translate_cone_list(cones: List[Tuple[float, float, bool]], t: np.ndarray) -> List[Tuple[float, float, bool]]:
    """Translate list of cone positions by vector t."""
    tx, ty = t.tolist()
    return [(x - tx, y - ty, is_right) for x, y, is_right in cones]

def round_cones(cones: List[Tuple[float, float, bool]], ndigits: int = 2) -> List[Tuple[float, float, bool]]:
    """Round coordinates for export."""
    return [(round(x, ndigits), round(y, ndigits), is_right) for x, y, is_right in cones]


# ------------------------------ UI application ------------------------------ #

class Drawer:
    """Interactive drawing app with parameter sliders and export-only alignment."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg

        # Figure and axes
        self.fig = plt.figure(figsize=(8.6, 8.6))
        self.ax = self.fig.add_axes([0.08, 0.20, 0.86, 0.75])
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(0, cfg.canvas_w)
        self.ax.set_ylim(0, cfg.canvas_h)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title("Draw the centerline (press and drag). Release to finish.")
        self._draw_grid()

        # Plots
        self.line_raw, = self.ax.plot([], [], linewidth=1.2)
        self.centerline_plot, = self.ax.plot([], [], linewidth=2.0)
        self.left_plot, = self.ax.plot([], [], linestyle=":")
        self.right_plot, = self.ax.plot([], [], linestyle=":")
        self.cone_scatter = self.ax.scatter([], [])
        self.origin_marker, = self.ax.plot([], [], marker="o", markersize=6)

        # Sliders
        self.s_width = Slider(self.fig.add_axes([0.08, 0.13, 0.30, 0.03]), "Width [m]", 0.2, 5.0, valinit=self.cfg.width)
        self.s_spacing = Slider(self.fig.add_axes([0.08, 0.09, 0.30, 0.03]), "Spacing [m]", 0.2, 2.0, valinit=self.cfg.spacing)
        self.s_dev = Slider(self.fig.add_axes([0.08, 0.05, 0.03, 0.03]), "Dev", 0.0, 1.0, valinit=self.cfg.deviation_max)

        # Buttons
        self.btn_apply = Button(self.fig.add_axes([0.42, 0.13, 0.12, 0.06]), "Apply")
        self.btn_save = Button(self.fig.add_axes([0.56, 0.13, 0.12, 0.06]), "Save")
        self.btn_redo = Button(self.fig.add_axes([0.70, 0.13, 0.12, 0.06]), "Redo")
        self.btn_quit = Button(self.fig.add_axes([0.84, 0.13, 0.12, 0.06]), "Quit")

        self.btn_apply.on_clicked(self.on_apply)
        self.btn_save.on_clicked(self.on_save)
        self.btn_redo.on_clicked(self.on_redo)
        self.btn_quit.on_clicked(self.on_quit)

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

    # ----- helpers ----- #

    def _draw_grid(self) -> None:
        for x in range(0, self.cfg.canvas_w + 1):
            self.ax.axvline(x, color="0.85", linewidth=0.5)
        for y in range(0, self.cfg.canvas_h + 1):
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

    def on_save(self, _event) -> None:
        if not self.cones_export:
            self.ax.set_title("Nothing to save. Draw and Apply first.")
            self.fig.canvas.draw_idle()
            return
        data = {"pylones": [{"x": x, "y": y, "is_right": is_right} for x, y, is_right in self.cones_export]}
        with open(self.cfg.outfile, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self.ax.set_title(f"Saved to {self.cfg.outfile}.")
        self.fig.canvas.draw_idle()

    def on_redo(self, _event) -> None:
        self.raw = []
        self.centerline = None
        self.left_line = None
        self.right_line = None
        self.cones_preview = None
        self.cones_export = None
        self.line_raw.set_data([], [])
        self.centerline_plot.set_data([], [])
        self.left_plot.set_data([], [])
        self.right_plot.set_data([], [])
        self.cone_scatter.remove()
        self.cone_scatter = self.ax.scatter([], [])
        self.origin_marker.set_data([], [])
        self.ax.set_title("Draw the centerline (press and drag). Release to finish.")
        self.fig.canvas.draw_idle()

    def on_quit(self, _event) -> None:
        plt.close(self.fig)

    # ----- core regeneration ----- #

    def _regenerate_from_raw(self) -> None:
        # Update config from sliders
        self.cfg.width = float(self.s_width.val)
        self.cfg.spacing = float(self.s_spacing.val)
        self.cfg.deviation_max = float(self.s_dev.val)

        if len(self.raw) < 3:
            self.ax.set_title("Too few points. Redraw.")
            self.fig.canvas.draw_idle()
            return

        # Close, smooth, resample centerline (kept at drawn pose for preview)
        pts = np.array(self.raw, dtype=float)
        pts = close_loop_always(pts)
        smoothed = bounded_smooth(pts, deviation_max=self.cfg.deviation_max)
        centerline = resample_by_arclength(smoothed, step=0.05)

        # Preview cones and offset lines at drawn position
        left_line, right_line = offset_lines(centerline, self.cfg.width / 2.0, self.cfg.clockwise)
        cones_preview = generate_cones(centerline, self.cfg.width, self.cfg.spacing, self.cfg.clockwise)

        # Compute export cones with start-at-origin and +y heading
        start = centerline[0].copy()
        # Tangent from first 0.5 m (or spacing, whichever is larger)
        _, tan_s = stations_along(centerline, max(0.5, self.cfg.spacing))
        t0 = tan_s[0] if len(tan_s) > 0 else np.array([0.0, 1.0])
        ang = math.atan2(t0[1], t0[0])
        rot = math.pi / 2.0 - ang
        cones_export = generate_cones(centerline - start, self.cfg.width, self.cfg.spacing, self.cfg.clockwise)
        cones_export = rotate_cone_list(cones_export, rot)
        cones_export = round_cones(cones_export, ndigits=2)

        # Store
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
        self.origin_marker.set_data([self.centerline[0, 0]], [self.centerline[0, 1]])

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
