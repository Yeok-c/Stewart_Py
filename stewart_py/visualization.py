"""3D visualization for Stewart Platform poses."""

from __future__ import annotations

from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from numpy.typing import NDArray

from stewart_py.platform import IKResult


def _plot3D_lines(
    ax: plt.Axes,
    origins: NDArray[np.float64],
    dests: NDArray[np.float64],
    color: str,
) -> None:
    """Draw 3D line segments between corresponding columns of *origins* and *dests*."""
    for i in range(origins.shape[1]):
        ax.plot(
            [origins[0, i], dests[0, i]],
            [origins[1, i], dests[1, i]],
            [origins[2, i], dests[2, i]],
            color=color,
        )


def plot_platform(
    B: NDArray[np.float64],
    result: IKResult,
    ax: Optional[plt.Axes] = None,
    limits: tuple[float, float, float, float, float, float] = (
        -100, 100, -100, 100, 0, 200,
    ),
) -> plt.Axes:
    """Plot the Stewart Platform in 3D.

    Args:
        B: Base anchor positions, shape ``(3, 6)``.
        result: IK calculation result containing positions.
        ax: Optional existing 3D axes. A new figure is created if ``None``.
        limits: Axis limits as ``(xmin, xmax, ymin, ymax, zmin, zmax)``.

    Returns:
        The matplotlib 3D axes used for plotting.
    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

    xmin, xmax, ymin, ymax, zmin, zmax = limits
    ax.set_xlim3d(xmin, xmax)
    ax.set_ylim3d(ymin, ymax)
    ax.set_zlim3d(zmin, zmax)
    ax.set_xlabel("x-axis")
    ax.set_ylabel("y-axis")
    ax.set_zlabel("z-axis")

    # Base polygon (green)
    ax.add_collection3d(
        Poly3DCollection(
            [list(np.transpose(B))], facecolors="green", alpha=0.25,
        )
    )

    # Platform polygon (blue)
    ax.add_collection3d(
        Poly3DCollection(
            [list(np.transpose(result.leg_positions))],
            facecolors="blue",
            alpha=0.25,
        )
    )

    # Servo horns (red), rods (black), virtual legs (orange)
    _plot3D_lines(ax, B, result.horn_positions, "red")
    _plot3D_lines(ax, result.horn_positions, result.leg_positions, "black")
    _plot3D_lines(ax, B, result.leg_positions, "orange")

    return ax
