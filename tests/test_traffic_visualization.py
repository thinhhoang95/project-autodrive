import matplotlib

matplotlib.use("Agg", force=True)

import matplotlib.pyplot as plt
import numpy as np
import pytest

from rcbranch.evaluation.traffic_visualization import (
    TrafficVisualizer,
    VehicleTrajectory,
    conflict_graph_from_trajectories,
    vehicle_footprint,
)
from rcbranch.geometry import ReferencePath


def test_vehicle_footprint_is_centered_and_oriented():
    corners = vehicle_footprint(2.0, 3.0, 0.0, length=4.0, width=2.0)

    assert corners.mean(axis=0).tolist() == pytest.approx([2.0, 3.0])
    assert corners[0].tolist() == pytest.approx([4.0, 4.0])
    assert corners[2].tolist() == pytest.approx([0.0, 2.0])


def test_visualizer_slider_updates_vehicle_patch_position():
    path = ReferencePath.from_xy([[0.0, 0.0], [10.0, 0.0]])
    trajectory = VehicleTrajectory(
        obstacle_id=1,
        ref_path=path,
        s=np.array([0.0, 5.0, 10.0]),
        times=np.array([0.0, 1.0, 2.0]),
        label="ego",
    )
    visualizer = TrafficVisualizer([trajectory], conflicts=[], tail_seconds=1.0)

    visualizer.slider.set_val(1.0)

    assert visualizer.current_time == pytest.approx(1.0)
    patch_xy = visualizer.vehicle_patches[1].get_xy()[:4]
    center = patch_xy.mean(axis=0)
    assert center.tolist() == pytest.approx([5.0, 0.0])
    assert visualizer.time_text.get_text() == "t = 1.00 s"
    plt.close(visualizer.fig)


def test_conflict_graph_from_trajectories_builds_intersection_conflict():
    east_west = ReferencePath.from_xy([[-10.0, 0.0], [10.0, 0.0]])
    south_north = ReferencePath.from_xy([[0.0, -10.0], [0.0, 10.0]])
    times = np.array([0.0, 1.0])
    trajectories = [
        VehicleTrajectory(1, east_west, np.array([0.0, 20.0]), times=times),
        VehicleTrajectory(2, south_north, np.array([0.0, 20.0]), times=times),
    ]

    conflicts = conflict_graph_from_trajectories(trajectories)

    assert len(conflicts) == 1
    assert {conflicts[0].i, conflicts[0].j} == {1, 2}
