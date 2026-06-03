import casadi as ca
import numpy as np
import pytest

from rcbranch.mpc.nlp_builder import ConstraintMeta, NLPBuilder


def test_toy_multiplier_matches_active_speed_cap():
    v_des = 10.0
    v_safe = 7.0
    builder = NLPBuilder()
    v = builder.add_var("v", 1, init=0.0)
    builder.J = 0.5 * (v[0] - v_des) ** 2
    builder.add_con(v[0] - v_safe, -ca.inf, 0.0, ConstraintMeta("toy_safe_speed"))

    solver, _, _ = builder.build_solver("toy_multiplier")
    sol = solver(**builder.arrays())
    x = np.asarray(sol["x"], dtype=float).reshape(-1)
    lam = abs(float(np.asarray(sol["lam_g"]).reshape(-1)[0]))

    assert builder.value("v", x)[0] == pytest.approx(v_safe, abs=1e-5)
    assert lam == pytest.approx(v_des - v_safe, abs=1e-4)


def test_toy_multiplier_zero_when_constraint_inactive():
    v_des = 10.0
    v_safe = 12.0
    builder = NLPBuilder()
    v = builder.add_var("v", 1, init=0.0)
    builder.J = 0.5 * (v[0] - v_des) ** 2
    builder.add_con(v[0] - v_safe, -ca.inf, 0.0, ConstraintMeta("toy_safe_speed"))

    solver, _, _ = builder.build_solver("toy_multiplier_inactive")
    sol = solver(**builder.arrays())
    lam = abs(float(np.asarray(sol["lam_g"]).reshape(-1)[0]))

    assert lam == pytest.approx(0.0, abs=1e-5)
