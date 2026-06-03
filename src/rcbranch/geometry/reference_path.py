from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


def _as_xy_array(xy: Iterable[Iterable[float]] | np.ndarray) -> np.ndarray:
    arr = np.asarray(xy, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("ReferencePath.xy must have shape (M, 2).")
    if arr.shape[0] < 2:
        raise ValueError("A reference path needs at least two points.")
    return arr


def _drop_duplicate_points(xy: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    keep = [0]
    for idx in range(1, len(xy)):
        if np.linalg.norm(xy[idx] - xy[keep[-1]]) > tol:
            keep.append(idx)
    if len(keep) < 2:
        raise ValueError("Reference path degenerates to a single point.")
    return xy[keep]


def _arc_length(xy: np.ndarray) -> np.ndarray:
    segment_lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(segment_lengths)])


def _heading_and_curvature(xy: np.ndarray, s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    deltas = np.diff(xy, axis=0)
    segment_theta = np.arctan2(deltas[:, 1], deltas[:, 0])
    theta = np.unwrap(np.concatenate([segment_theta, [segment_theta[-1]]]))
    if len(theta) < 3 or np.isclose(s[-1], 0.0):
        return theta, np.zeros_like(theta)
    dtheta_ds = np.gradient(theta, s, edge_order=1)
    return theta, dtheta_ds


@dataclass(slots=True)
class ReferencePath:
    """Polyline reference path with arc-length projection and interpolation."""

    xy: np.ndarray
    s: np.ndarray
    theta: np.ndarray
    kappa: np.ndarray

    @classmethod
    def from_xy(cls, xy: Iterable[Iterable[float]] | np.ndarray) -> "ReferencePath":
        points = _drop_duplicate_points(_as_xy_array(xy))
        s = _arc_length(points)
        theta, kappa = _heading_and_curvature(points, s)
        return cls(xy=points, s=s, theta=theta, kappa=kappa)

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def project_xy_to_s(self, xy: Iterable[float] | np.ndarray) -> float:
        """Project a point onto the nearest path segment and return arc length."""

        point = np.asarray(xy, dtype=float).reshape(2)
        starts = self.xy[:-1]
        ends = self.xy[1:]
        seg = ends - starts
        seg_len_sq = np.einsum("ij,ij->i", seg, seg)
        valid = seg_len_sq > 0.0
        if not np.any(valid):
            raise ValueError("Reference path contains no nonzero-length segments.")

        rel = point - starts
        t = np.zeros(len(seg), dtype=float)
        t[valid] = np.einsum("ij,ij->i", rel[valid], seg[valid]) / seg_len_sq[valid]
        t = np.clip(t, 0.0, 1.0)
        closest = starts + t[:, None] * seg
        d2 = np.einsum("ij,ij->i", closest - point, closest - point)
        best = int(np.argmin(d2))
        return float(self.s[best] + t[best] * np.sqrt(seg_len_sq[best]))

    def interpolate_xytheta(
        self, s_query: float | np.ndarray,
    ) -> tuple[float | np.ndarray, float | np.ndarray, float | np.ndarray, float | np.ndarray]:
        """Interpolate x, y, heading, and curvature at one or more arc lengths."""

        query = np.asarray(s_query, dtype=float)
        clipped = np.clip(query, 0.0, self.length)
        x = np.interp(clipped, self.s, self.xy[:, 0])
        y = np.interp(clipped, self.s, self.xy[:, 1])
        theta = np.interp(clipped, self.s, np.unwrap(self.theta))
        kappa = np.interp(clipped, self.s, self.kappa)
        if np.isscalar(s_query):
            return float(x), float(y), float(theta), float(kappa)
        return x, y, theta, kappa

    def sample(self, step: float = 0.25) -> tuple[np.ndarray, np.ndarray]:
        if step <= 0.0:
            raise ValueError("step must be positive.")
        samples = np.arange(0.0, self.length + 0.5 * step, step)
        samples[-1] = min(samples[-1], self.length)
        x, y, _, _ = self.interpolate_xytheta(samples)
        return samples, np.column_stack([x, y])
