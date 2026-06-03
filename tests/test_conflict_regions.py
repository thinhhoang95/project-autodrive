import pytest

from rcbranch.geometry.conflict_regions import compute_conflict_region
from rcbranch.geometry.reference_path import ReferencePath
from rcbranch.geometry import build_conflict_graph
from rcbranch.scenarios import build_turning_crossroad_scene


def test_conflict_region_for_perpendicular_crossroad():
    east_west = ReferencePath.from_xy([[-10.0, 0.0], [10.0, 0.0]])
    south_north = ReferencePath.from_xy([[0.0, -10.0], [0.0, 10.0]])

    conflict = compute_conflict_region(
        east_west,
        south_north,
        i=1,
        j=2,
        half_width_i=1.0,
        half_width_j=1.0,
        margin=0.0,
        sample_step=0.05,
    )

    assert conflict is not None
    assert conflict.s_i_in == pytest.approx(9.0, abs=0.1)
    assert conflict.s_i_out == pytest.approx(11.0, abs=0.1)
    assert conflict.s_j_in == pytest.approx(9.0, abs=0.1)
    assert conflict.s_j_out == pytest.approx(11.0, abs=0.1)
    assert conflict.polygon.area > 0.0


def test_conflict_region_for_turning_crossroad():
    vehicles = build_turning_crossroad_scene()

    conflicts = build_conflict_graph(vehicles, sample_step=0.05)

    assert len(conflicts) == 1
    conflict = conflicts[0]
    assert {conflict.i, conflict.j} == {1, 2}
    assert conflict.s_i_in == pytest.approx(26.85, abs=0.2)
    assert conflict.s_i_out == pytest.approx(29.7, abs=0.2)
    assert conflict.s_j_in == pytest.approx(21.7, abs=0.2)
    assert conflict.s_j_out == pytest.approx(24.55, abs=0.2)
    assert conflict.polygon.area > 0.0
