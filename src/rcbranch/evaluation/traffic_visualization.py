from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Iterable

import numpy as np

from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.geometry.reference_path import ReferencePath


@dataclass(slots=True)
class VehicleTrajectory:
    """A vehicle trajectory expressed as progress along a reference path."""

    obstacle_id: int
    ref_path: ReferencePath
    s: np.ndarray
    times: np.ndarray | None = None
    dt: float | None = None
    v: np.ndarray | None = None
    a: np.ndarray | None = None
    label: str | None = None
    color: str | None = None
    length: float = 4.5
    width: float = 2.0

    def __post_init__(self) -> None:
        self.s = np.asarray(self.s, dtype=float)
        if self.s.ndim != 1 or len(self.s) == 0:
            raise ValueError("VehicleTrajectory.s must be a nonempty 1D array.")
        if self.times is None:
            dt = 1.0 if self.dt is None else float(self.dt)
            if dt <= 0.0:
                raise ValueError("dt must be positive.")
            self.times = np.arange(len(self.s), dtype=float) * dt
        else:
            self.times = np.asarray(self.times, dtype=float)
        if self.times.ndim != 1 or self.times.shape != self.s.shape:
            raise ValueError("VehicleTrajectory.times must match s shape.")
        if np.any(np.diff(self.times) < 0.0):
            raise ValueError("VehicleTrajectory.times must be sorted.")
        if self.v is not None:
            self.v = np.asarray(self.v, dtype=float)
            if self.v.shape != self.s.shape:
                raise ValueError("VehicleTrajectory.v must match s shape.")
        if self.a is not None:
            self.a = np.asarray(self.a, dtype=float)
            if self.a.shape not in (self.s.shape, (len(self.s) - 1,)):
                raise ValueError("VehicleTrajectory.a must match s shape or have one fewer sample.")
        if self.label is None:
            self.label = f"vehicle {self.obstacle_id}"

    @property
    def t_min(self) -> float:
        return float(self.times[0])

    @property
    def t_max(self) -> float:
        return float(self.times[-1])

    def s_at(self, t: float) -> float:
        return float(np.interp(t, self.times, self.s))

    def pose_at(self, t: float) -> tuple[float, float, float]:
        s_now = self.s_at(t)
        x, y, theta, _ = self.ref_path.interpolate_xytheta(s_now)
        return float(x), float(y), float(theta)

    def speed_at(self, t: float) -> float | None:
        if self.v is None:
            return None
        return float(np.interp(t, self.times, self.v))

    def path_xy(self) -> np.ndarray:
        x, y, _, _ = self.ref_path.interpolate_xytheta(self.s)
        return np.column_stack([x, y])


def vehicle_footprint(
    x: float,
    y: float,
    theta: float,
    *,
    length: float,
    width: float,
) -> np.ndarray:
    """Return four footprint corners for a vehicle rectangle."""

    half_l = 0.5 * length
    half_w = 0.5 * width
    local = np.array(
        [
            [half_l, half_w],
            [half_l, -half_w],
            [-half_l, -half_w],
            [-half_l, half_w],
        ],
        dtype=float,
    )
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.array([x, y])


class TrafficVisualizer:
    """Interactive matplotlib visualizer with a time seekbar and play controls."""

    def __init__(
        self,
        trajectories: Iterable[VehicleTrajectory],
        *,
        conflicts: Iterable[ConflictRegion] | None = None,
        title: str = "Traffic through intersection",
        tail_seconds: float = 2.0,
        timer_interval_ms: int = 50,
        playback_speed: float = 1.0,
        loop: bool = False,
    ):
        self.trajectories = list(trajectories)
        if not self.trajectories:
            raise ValueError("At least one trajectory is required.")
        self.conflicts = list(conflicts or [])
        self.title = title
        self.tail_seconds = max(float(tail_seconds), 0.0)
        self.playback_speed = float(playback_speed)
        self.loop = loop
        self.current_time = min(traj.t_min for traj in self.trajectories)
        self.t_min = self.current_time
        self.t_max = max(traj.t_max for traj in self.trajectories)
        self.time_step = self._infer_time_step()
        self._playing = False
        self._updating_slider = False

        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as MplPolygon
        from matplotlib.widgets import Button, Slider

        self._plt = plt
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.subplots_adjust(bottom=0.18)
        self.ax.set_title(title)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.25)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self._draw_paths_and_conflicts()
        self._set_axis_bounds()

        self.vehicle_patches: dict[int, MplPolygon] = {}
        self.tail_lines = {}
        self.label_texts = {}
        colors = plt.get_cmap("tab10")
        for idx, traj in enumerate(self.trajectories):
            color = traj.color or colors(idx % 10)
            x, y, theta = traj.pose_at(self.current_time)
            patch = MplPolygon(
                vehicle_footprint(x, y, theta, length=traj.length, width=traj.width),
                closed=True,
                facecolor=color,
                edgecolor="black",
                linewidth=1.0,
                alpha=0.9,
                zorder=5,
            )
            self.ax.add_patch(patch)
            (tail_line,) = self.ax.plot([], [], color=color, linewidth=2.0, alpha=0.45, zorder=4)
            text = self.ax.text(
                x,
                y,
                str(traj.label),
                fontsize=9,
                color="black",
                ha="center",
                va="bottom",
                zorder=6,
            )
            self.vehicle_patches[traj.obstacle_id] = patch
            self.tail_lines[traj.obstacle_id] = tail_line
            self.label_texts[traj.obstacle_id] = text

        self.time_text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            ha="left",
            va="top",
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.85},
        )

        slider_ax = self.fig.add_axes([0.20, 0.07, 0.54, 0.035])
        self.slider = Slider(
            slider_ax,
            "t [s]",
            self.t_min,
            self.t_max,
            valinit=self.current_time,
            valfmt="%.2f",
        )
        self.slider.on_changed(self._on_slider_changed)

        self.play_button = Button(self.fig.add_axes([0.77, 0.065, 0.08, 0.045]), "Play")
        self.play_button.on_clicked(self._toggle_play)
        self.back_button = Button(self.fig.add_axes([0.07, 0.065, 0.05, 0.045]), "<")
        self.back_button.on_clicked(lambda _event: self.step(-1))
        self.forward_button = Button(self.fig.add_axes([0.88, 0.065, 0.05, 0.045]), ">")
        self.forward_button.on_clicked(lambda _event: self.step(1))

        self.timer = self.fig.canvas.new_timer(interval=int(timer_interval_ms))
        self.timer.add_callback(self._on_timer)
        self.set_time(self.current_time)

    @property
    def playing(self) -> bool:
        return self._playing

    def set_time(self, t: float, *, update_slider: bool = True) -> None:
        self.current_time = float(np.clip(t, self.t_min, self.t_max))
        for traj in self.trajectories:
            x, y, theta = traj.pose_at(self.current_time)
            self.vehicle_patches[traj.obstacle_id].set_xy(
                vehicle_footprint(x, y, theta, length=traj.length, width=traj.width)
            )
            history = self._tail_xy(traj, self.current_time)
            self.tail_lines[traj.obstacle_id].set_data(history[:, 0], history[:, 1])
            label = self.label_texts[traj.obstacle_id]
            label.set_position((x, y + 0.65 * traj.width))
            speed = traj.speed_at(self.current_time)
            if speed is None:
                label.set_text(str(traj.label))
            else:
                label.set_text(f"{traj.label}\n{speed:.1f} m/s")
        self.time_text.set_text(f"t = {self.current_time:.2f} s")
        if update_slider:
            self._updating_slider = True
            self.slider.set_val(self.current_time)
            self._updating_slider = False
        self.fig.canvas.draw_idle()

    def step(self, direction: int = 1) -> None:
        self.pause()
        self.set_time(self.current_time + direction * self.time_step)

    def play(self) -> None:
        if self._playing:
            return
        self._playing = True
        self.play_button.label.set_text("Pause")
        self.timer.start()

    def pause(self) -> None:
        if not self._playing:
            return
        self._playing = False
        self.play_button.label.set_text("Play")
        self.timer.stop()

    def show(self) -> None:
        self._plt.show()

    def save_frame(self, path: str, *, t: float | None = None, dpi: int = 150) -> None:
        if t is not None:
            self.set_time(t)
        self.fig.savefig(path, dpi=dpi, bbox_inches="tight")

    def _on_slider_changed(self, value: float) -> None:
        if self._updating_slider:
            return
        self.pause()
        self.set_time(float(value), update_slider=False)

    def _toggle_play(self, _event) -> None:
        if self._playing:
            self.pause()
        else:
            self.play()

    def _on_timer(self) -> None:
        if not self._playing:
            return
        next_time = self.current_time + self.time_step * self.playback_speed
        if next_time > self.t_max:
            if self.loop:
                next_time = self.t_min
            else:
                next_time = self.t_max
                self.set_time(next_time)
                self.pause()
                return
        self.set_time(next_time)

    def _infer_time_step(self) -> float:
        diffs: list[float] = []
        for traj in self.trajectories:
            if len(traj.times) > 1:
                diff = np.diff(traj.times)
                diffs.extend(diff[diff > 0.0].tolist())
        if not diffs:
            return 0.1
        return float(np.median(diffs))

    def _tail_xy(self, traj: VehicleTrajectory, t: float) -> np.ndarray:
        if self.tail_seconds <= 0.0:
            x, y, _ = traj.pose_at(t)
            return np.array([[x, y]], dtype=float)
        start = max(traj.t_min, t - self.tail_seconds)
        samples = traj.times[(traj.times >= start) & (traj.times <= t)]
        if len(samples) == 0 or samples[-1] < t:
            samples = np.append(samples, t)
        if samples[0] > start:
            samples = np.insert(samples, 0, start)
        s_values = np.interp(samples, traj.times, traj.s)
        x, y, _, _ = traj.ref_path.interpolate_xytheta(s_values)
        return np.column_stack([x, y])

    def _draw_paths_and_conflicts(self) -> None:
        for traj in self.trajectories:
            self.ax.plot(
                traj.ref_path.xy[:, 0],
                traj.ref_path.xy[:, 1],
                color="0.35",
                linestyle="--",
                linewidth=1.2,
                alpha=0.75,
                zorder=1,
            )
        for conflict in self.conflicts:
            self._draw_polygon(conflict.polygon)

    def _draw_polygon(self, polygon) -> None:
        if polygon is None or getattr(polygon, "is_empty", False):
            return
        if hasattr(polygon, "geoms"):
            for geom in polygon.geoms:
                self._draw_polygon(geom)
            return
        if not hasattr(polygon, "exterior"):
            return
        x, y = polygon.exterior.xy
        self.ax.fill(x, y, color="tab:red", alpha=0.18, edgecolor="tab:red", linewidth=1.0, zorder=2)

    def _set_axis_bounds(self) -> None:
        points = []
        for traj in self.trajectories:
            points.append(traj.ref_path.xy)
            points.append(traj.path_xy())
        for conflict in self.conflicts:
            bounds = getattr(conflict.polygon, "bounds", None)
            if bounds:
                minx, miny, maxx, maxy = bounds
                points.append(np.array([[minx, miny], [maxx, maxy]], dtype=float))
        all_points = np.vstack(points)
        min_xy = np.min(all_points, axis=0)
        max_xy = np.max(all_points, axis=0)
        span = np.maximum(max_xy - min_xy, 1.0)
        pad = 0.12 * span + 2.0
        self.ax.set_xlim(min_xy[0] - pad[0], max_xy[0] + pad[0])
        self.ax.set_ylim(min_xy[1] - pad[1], max_xy[1] + pad[1])


def trajectories_from_mpc_solution(solution, vehicles: Iterable[object], *, dt: float = 0.2, branch: int = 0):
    """Build `VehicleTrajectory` objects from an MPC solution and vehicle metadata."""

    trajectories: list[VehicleTrajectory] = []
    for vehicle in vehicles:
        obstacle_id = int(getattr(vehicle, "obstacle_id"))
        if hasattr(solution, "s"):
            s_values = solution.s[obstacle_id]
            v_values = solution.v.get(obstacle_id)
            a_values = solution.a.get(obstacle_id)
        else:
            s_values = solution.branch_s[branch][obstacle_id]
            v_values = solution.branch_v[branch].get(obstacle_id)
            a_values = solution.branch_a[branch].get(obstacle_id)
        label = "ego" if getattr(vehicle, "is_ego", False) else f"vehicle {obstacle_id}"
        trajectories.append(
            VehicleTrajectory(
                obstacle_id=obstacle_id,
                ref_path=getattr(vehicle, "ref_path"),
                s=s_values,
                dt=dt,
                v=v_values,
                a=a_values,
                label=label,
                length=float(getattr(vehicle, "length", 4.5)),
                width=float(getattr(vehicle, "width", 2.0)),
            )
        )
    return trajectories


def visualize_traffic_scene(
    trajectories: Iterable[VehicleTrajectory],
    *,
    conflicts: Iterable[ConflictRegion] | None = None,
    show: bool = True,
    **kwargs,
) -> TrafficVisualizer:
    visualizer = TrafficVisualizer(trajectories, conflicts=conflicts, **kwargs)
    if show:
        visualizer.show()
    return visualizer


def conflict_graph_from_trajectories(trajectories: Iterable[VehicleTrajectory]) -> list[ConflictRegion]:
    from rcbranch.geometry.conflict_regions import build_conflict_graph

    vehicles = [
        SimpleNamespace(
            obstacle_id=traj.obstacle_id,
            ref_path=traj.ref_path,
            width=traj.width,
        )
        for traj in trajectories
    ]
    return build_conflict_graph(vehicles)
