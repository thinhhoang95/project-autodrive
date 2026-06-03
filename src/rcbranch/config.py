from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class BranchingConfig:
    """Typed subset of the YAML config used by the planner loop."""

    mpc: dict[str, Any] = field(default_factory=dict)
    vehicle: dict[str, Any] = field(default_factory=dict)
    objective: dict[str, Any] = field(default_factory=dict)
    belief: dict[str, Any] = field(default_factory=dict)
    branching: dict[str, Any] = field(default_factory=dict)


def load_config(path: str | Path = "configs/default.yaml") -> BranchingConfig:
    with Path(path).open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return BranchingConfig(
        mpc=dict(raw.get("mpc", {})),
        vehicle=dict(raw.get("vehicle", {})),
        objective=dict(raw.get("objective", {})),
        belief=dict(raw.get("belief", {})),
        branching=dict(raw.get("branching", {})),
    )
