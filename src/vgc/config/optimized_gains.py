"""Optimized nominal gain registry."""
from .manual_gains import manual_gains


def optimized_gains(mode: str, coriolis: str) -> dict:
    if str(mode).lower() != "nominal":
        raise ValueError(f"Only nominal gains are available, got {mode!r}")
    return manual_gains("nominal", coriolis)
