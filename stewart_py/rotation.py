"""Rotation matrix constructors for 3D transformations."""

import numpy as np
from numpy.typing import NDArray


def rotX(phi: float) -> NDArray[np.float64]:
    """Rotation matrix about the X axis.

    Args:
        phi: Rotation angle in radians.
    """
    return np.array([
        [1,            0,             0],
        [0,  np.cos(phi), -np.sin(phi)],
        [0,  np.sin(phi),  np.cos(phi)],
    ])


def rotY(theta: float) -> NDArray[np.float64]:
    """Rotation matrix about the Y axis.

    Args:
        theta: Rotation angle in radians.
    """
    return np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [             0, 1,             0],
        [-np.sin(theta), 0, np.cos(theta)],
    ])


def rotZ(psi: float) -> NDArray[np.float64]:
    """Rotation matrix about the Z axis.

    Args:
        psi: Rotation angle in radians.
    """
    return np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi),  np.cos(psi), 0],
        [          0,            0, 1],
    ])
