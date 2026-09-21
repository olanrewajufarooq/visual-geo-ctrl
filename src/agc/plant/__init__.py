"""Physical simulation plant models for AGC."""

from .compound_pi import compound_pi

__all__ = ["PyBulletPlant", "compound_pi"]


def __getattr__(name: str):
    if name == "PyBulletPlant":
        from .pybullet_plant import PyBulletPlant

        return PyBulletPlant
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
