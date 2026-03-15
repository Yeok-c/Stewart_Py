"""Tests for rotation matrix functions."""

import numpy as np
import pytest

from stewart_py import rotX, rotY, rotZ


class TestRotationIdentity:
    """rotX/Y/Z(0) should return the identity matrix."""

    def test_rotX_zero(self):
        np.testing.assert_allclose(rotX(0), np.eye(3), atol=1e-15)

    def test_rotY_zero(self):
        np.testing.assert_allclose(rotY(0), np.eye(3), atol=1e-15)

    def test_rotZ_zero(self):
        np.testing.assert_allclose(rotZ(0), np.eye(3), atol=1e-15)


class TestRotationKnownAngles:
    """Verify specific rotations produce expected results."""

    def test_rotZ_90_maps_x_to_y(self):
        R = rotZ(np.pi / 2)
        result = R @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-15)

    def test_rotX_90_maps_y_to_z(self):
        R = rotX(np.pi / 2)
        result = R @ np.array([0.0, 1.0, 0.0])
        np.testing.assert_allclose(result, [0.0, 0.0, 1.0], atol=1e-15)

    def test_rotY_90_maps_z_to_x(self):
        R = rotY(np.pi / 2)
        result = R @ np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(result, [1.0, 0.0, 0.0], atol=1e-15)

    def test_rotZ_180(self):
        R = rotZ(np.pi)
        result = R @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(result, [-1.0, 0.0, 0.0], atol=1e-15)


class TestRotationProperties:
    """Rotation matrices must be orthogonal with det=1."""

    @pytest.mark.parametrize("angle", [0.1, 0.5, 1.0, np.pi / 4, np.pi])
    def test_orthogonality_rotX(self, angle):
        R = rotX(angle)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-14)

    @pytest.mark.parametrize("angle", [0.1, 0.5, 1.0, np.pi / 4, np.pi])
    def test_orthogonality_rotY(self, angle):
        R = rotY(angle)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-14)

    @pytest.mark.parametrize("angle", [0.1, 0.5, 1.0, np.pi / 4, np.pi])
    def test_orthogonality_rotZ(self, angle):
        R = rotZ(angle)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-14)

    @pytest.mark.parametrize("angle", [0.3, 1.2, -0.7])
    def test_determinant_is_one(self, angle):
        for rot_fn in (rotX, rotY, rotZ):
            assert abs(np.linalg.det(rot_fn(angle)) - 1.0) < 1e-14


class TestRotationComposition:
    """Composition of rotations around the same axis should be additive."""

    def test_rotZ_composition(self):
        a, b = 0.3, 0.7
        np.testing.assert_allclose(rotZ(a) @ rotZ(b), rotZ(a + b), atol=1e-14)

    def test_rotX_composition(self):
        a, b = 0.2, 0.5
        np.testing.assert_allclose(rotX(a) @ rotX(b), rotX(a + b), atol=1e-14)

    def test_rotY_composition(self):
        a, b = -0.4, 1.1
        np.testing.assert_allclose(rotY(a) @ rotY(b), rotY(a + b), atol=1e-14)
