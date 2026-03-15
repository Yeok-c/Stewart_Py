"""Stewart Platform inverse kinematics library."""

__version__ = "0.2.0"

from stewart_py.platform import IKResult, PlatformConfig, StewartPlatform
from stewart_py.rotation import rotX, rotY, rotZ
from stewart_py.visualization import plot_platform

__all__ = [
    "StewartPlatform",
    "PlatformConfig",
    "IKResult",
    "rotX",
    "rotY",
    "rotZ",
    "plot_platform",
]
