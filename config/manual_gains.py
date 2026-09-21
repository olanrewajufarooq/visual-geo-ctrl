"""Root configuration module re-exporting manual gain registry."""

from agc.config.manual_gains import manual_gains, general_default

__all__ = ["manual_gains", "general_default"]
