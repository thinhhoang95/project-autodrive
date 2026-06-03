import numpy as np
import pytest

from rcbranch.geometry.reference_path import ReferencePath
from rcbranch.scenarios import west_to_north_turn_path


def test_projection_round_trip_on_straight_path():
    path = ReferencePath.from_xy([[0.0, 0.0], [10.0, 0.0]])

    s = path.project_xy_to_s(np.array([4.2, 1.5]))
    x, y, theta, kappa = path.interpolate_xytheta(s)

    assert s == pytest.approx(4.2)
    assert x == pytest.approx(4.2)
    assert y == pytest.approx(0.0)
    assert theta == pytest.approx(0.0)
    assert kappa == pytest.approx(0.0)


def test_interpolation_clamps_to_path_extent():
    path = ReferencePath.from_xy([[0.0, 0.0], [3.0, 4.0]])

    x, y, _, _ = path.interpolate_xytheta(99.0)

    assert x == pytest.approx(3.0)
    assert y == pytest.approx(4.0)


def test_projection_round_trip_on_90_degree_turn_path():
    path = west_to_north_turn_path()
    s_mid_turn = 24.0 + 0.5 * np.pi * 8.0 / 2.0
    x, y, theta, kappa = path.interpolate_xytheta(s_mid_turn)

    projected = path.project_xy_to_s([x, y])

    assert projected == pytest.approx(s_mid_turn, abs=0.15)
    assert theta == pytest.approx(0.25 * np.pi, abs=0.15)
    assert kappa > 0.0
