from __future__ import annotations

from rcbranch.mpc.branching import BranchScore


def plot_branch_score(score: BranchScore):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    ax.plot(score.psi_by_k, label="psi_k")
    ax.set_xlabel("horizon step")
    ax.set_ylabel("dual-priced ambiguity")
    ax.legend()
    return fig
