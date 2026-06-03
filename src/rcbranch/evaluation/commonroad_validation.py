from __future__ import annotations


def require_drivability_checker():
    try:
        import commonroad_dc  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "CommonRoad drivability validation requires commonroad-drivability-checker."
        ) from exc


def validation_available() -> bool:
    try:
        require_drivability_checker()
    except ImportError:
        return False
    return True
