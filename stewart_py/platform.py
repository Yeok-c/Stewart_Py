"""Stewart Platform inverse kinematics solver.

See 01_Stewart_Py_Inverse_Kinematics.ipynb for the mathematical derivation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from stewart_py.rotation import rotX, rotY, rotZ


@dataclass(frozen=True)
class PlatformConfig:
    """Geometric parameters defining a Stewart Platform.

    Args:
        r_B: Radius of the circumscribed circle for base anchor points.
        r_P: Radius of the circumscribed circle for platform anchor points.
        horn_length: Length of the servo horn (|h| in the derivation).
        rod_length: Length of the connecting rod (|d| in the derivation).
        gamma_B: Half-angle between adjacent base anchor pairs (radians).
        gamma_P: Half-angle between adjacent platform anchor pairs (radians).
        ref_rotation: Reference rotation offset about the Z axis (radians).
    """

    r_B: float
    r_P: float
    horn_length: float
    rod_length: float
    gamma_B: float
    gamma_P: float
    ref_rotation: float = 5 * np.pi / 6


@dataclass
class IKResult:
    """Result of an inverse kinematics calculation.

    Attributes:
        angles: Servo angles for all 6 legs in radians. Shape ``(6,)``.
        leg_vectors: Leg vectors in the local frame. Shape ``(3, 6)``.
        leg_lengths: Leg magnitudes. Shape ``(6,)``.
        leg_positions: Leg endpoint positions in the global frame. Shape ``(3, 6)``.
        horn_positions: Servo horn tip positions in the global frame. Shape ``(3, 6)``.
        reachable: ``True`` if all servo angles have real solutions.
    """

    angles: NDArray[np.float64]
    leg_vectors: NDArray[np.float64]
    leg_lengths: NDArray[np.float64]
    leg_positions: NDArray[np.float64]
    horn_positions: NDArray[np.float64]
    reachable: bool


class StewartPlatform:
    """Inverse kinematics solver for a 6-DOF Stewart Platform with rotational servos.

    Computes servo angles given desired translation and rotation of the platform.

    Args:
        config: Platform geometry configuration.
    """

    def __init__(self, config: PlatformConfig) -> None:
        self.config = config
        pi = np.pi

        # Servo shaft orientation angles
        beta = np.array([
            pi / 2 + pi,
            pi / 2,
            2 * pi / 3 + pi / 2 + pi,
            2 * pi / 3 + pi / 2,
            4 * pi / 3 + pi / 2 + pi,
            4 * pi / 3 + pi / 2,
        ])

        # psi_B: polar coordinates of base anchor points
        psi_B = np.array([
            -config.gamma_B,
            config.gamma_B,
            2 * pi / 3 - config.gamma_B,
            2 * pi / 3 + config.gamma_B,
            2 * pi / 3 + 2 * pi / 3 - config.gamma_B,
            2 * pi / 3 + 2 * pi / 3 + config.gamma_B,
        ])

        # psi_P: polar coordinates of platform anchor points
        psi_P = np.array([
            pi / 3 + 2 * pi / 3 + 2 * pi / 3 + config.gamma_P,
            pi / 3 + -config.gamma_P,
            pi / 3 + config.gamma_P,
            pi / 3 + 2 * pi / 3 - config.gamma_P,
            pi / 3 + 2 * pi / 3 + config.gamma_P,
            pi / 3 + 2 * pi / 3 + 2 * pi / 3 - config.gamma_P,
        ])

        # Apply reference rotation offset
        psi_B = psi_B + config.ref_rotation
        psi_P = psi_P + config.ref_rotation
        beta = beta + config.ref_rotation

        # B: base anchor positions (3x6)
        B = config.r_B * np.array([
            [np.cos(psi_B[i]), np.sin(psi_B[i]), 0] for i in range(6)
        ])
        B = np.transpose(B)

        # P: platform anchor positions (3x6)
        P = config.r_P * np.array([
            [np.cos(psi_P[i]), np.sin(psi_P[i]), 0] for i in range(6)
        ])
        P = np.transpose(P)

        # Home position: platform height when all servos are at zero
        # z[i] should be identical for all 6 legs by symmetry; use first element.
        z = np.sqrt(
            config.rod_length ** 2
            + config.horn_length ** 2
            - (P[0] - B[0]) ** 2
            - (P[1] - B[1]) ** 2
        )
        home_pos = np.array([0.0, 0.0, z[0]])

        self.beta = beta
        self.psi_B = psi_B
        self.psi_P = psi_P
        self.B = B
        self.P = P
        self.home_pos = home_pos

    def calculate(
        self,
        trans: NDArray[np.float64],
        rotation: NDArray[np.float64],
    ) -> IKResult:
        """Compute servo angles for a desired platform pose.

        Args:
            trans: Translation vector ``[tx, ty, tz]``, shape ``(3,)``.
            rotation: Euler angles ``[roll, pitch, yaw]`` in radians, shape ``(3,)``.

        Returns:
            An :class:`IKResult` with servo angles and intermediate geometry.

        Raises:
            ValueError: If *trans* or *rotation* do not have shape ``(3,)``.
        """
        trans = np.asarray(trans, dtype=np.float64)
        rotation = np.asarray(rotation, dtype=np.float64)
        if trans.shape != (3,):
            raise ValueError(f"trans must have shape (3,), got {trans.shape}")
        if rotation.shape != (3,):
            raise ValueError(f"rotation must have shape (3,), got {rotation.shape}")

        # Rotation matrix: R = RotZ * RotY * RotX
        R = np.matmul(np.matmul(rotZ(rotation[2]), rotY(rotation[1])), rotX(rotation[0]))

        # Leg vectors for each of 6 legs
        l = (
            np.repeat(trans[:, np.newaxis], 6, axis=1)
            + np.repeat(self.home_pos[:, np.newaxis], 6, axis=1)
            + np.matmul(R, self.P)
            - self.B
        )
        lll = np.linalg.norm(l, axis=0)

        # Leg positions in global frame
        L = l + self.B

        # Components split for clarity
        lx = l[0, :]
        ly = l[1, :]
        lz = l[2, :]

        # Auxiliary quantities g, e (vectorised), fk (per-leg)
        g = lll ** 2 - (self.config.rod_length ** 2 - self.config.horn_length ** 2)
        e = 2 * self.config.horn_length * lz

        angles = np.zeros(6)
        H = np.zeros((3, 6))
        reachable = True

        for k in range(6):
            fk = 2 * self.config.horn_length * (
                np.cos(self.beta[k]) * lx[k] + np.sin(self.beta[k]) * ly[k]
            )

            # Servo angle from the analytical IK solution
            arcsin_arg = g[k] / np.sqrt(e[k] ** 2 + fk ** 2)
            if abs(arcsin_arg) > 1.0:
                reachable = False
                arcsin_arg = np.clip(arcsin_arg, -1.0, 1.0)

            angles[k] = np.arcsin(arcsin_arg) - np.arctan2(fk, e[k])

            # Servo horn tip position (spherical joint)
            H[:, k] = np.array([
                self.config.horn_length * np.cos(angles[k]) * np.cos(self.beta[k]) + self.B[0, k],
                self.config.horn_length * np.cos(angles[k]) * np.sin(self.beta[k]) + self.B[1, k],
                self.config.horn_length * np.sin(angles[k]),
            ])

        return IKResult(
            angles=angles,
            leg_vectors=l,
            leg_lengths=lll,
            leg_positions=L,
            horn_positions=H,
            reachable=reachable,
        )

    def __repr__(self) -> str:
        return f"StewartPlatform(config={self.config!r})"
