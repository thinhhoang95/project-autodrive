import numpy as np
import pytest

from rcbranch.geometry.reference_path import ReferencePath


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
