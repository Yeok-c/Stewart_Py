"""Tests for StewartPlatform inverse kinematics solver."""

import numpy as np
import pytest

from stewart_py import StewartPlatform, PlatformConfig


def _make_platform(**overrides) -> StewartPlatform:
    defaults = dict(
        r_B=6.2,
        r_P=5.0,
        horn_length=3.0,
        rod_length=12.0,
        gamma_B=0.2269,
        gamma_P=0.82,
    )
    defaults.update(overrides)
    return StewartPlatform(PlatformConfig(**defaults))


class TestHomePosition:
    """At zero translation and zero rotation, symmetry should hold."""

    def test_home_angles_all_equal(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.zeros(3))
        assert result.reachable
        # All 6 angles should be identical by symmetry
        np.testing.assert_allclose(
            result.angles, result.angles[0] * np.ones(6), atol=1e-10,
        )

    def test_home_no_nan(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.zeros(3))
        assert not np.any(np.isnan(result.angles))

    def test_home_leg_lengths_positive(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.zeros(3))
        assert np.all(result.leg_lengths > 0)


class TestPureTranslation:
    def test_pure_z_translation_symmetric(self):
        """A pure Z translation should produce 6 identical servo angles."""
        platform = _make_platform()
        result = platform.calculate(np.array([0.0, 0.0, 1.0]), np.zeros(3))
        assert result.reachable
        # All angles should be the same (platform stays level, just moves up)
        np.testing.assert_allclose(
            result.angles, result.angles[0] * np.ones(6), atol=1e-10,
        )

    def test_pure_x_translation_breaks_symmetry(self):
        """An X translation should break the 6-fold symmetry."""
        platform = _make_platform()
        result = platform.calculate(np.array([1.0, 0.0, 0.0]), np.zeros(3))
        assert result.reachable
        # Not all angles should be equal
        assert not np.allclose(result.angles, result.angles[0] * np.ones(6), atol=1e-6)

    def test_pure_y_translation_breaks_symmetry(self):
        platform = _make_platform()
        result = platform.calculate(np.array([0.0, 1.0, 0.0]), np.zeros(3))
        assert result.reachable
        assert not np.allclose(result.angles, result.angles[0] * np.ones(6), atol=1e-6)


class TestPureRotation:
    def test_pure_roll(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.array([0.1, 0.0, 0.0]))
        assert result.reachable
        assert not np.any(np.isnan(result.angles))

    def test_pure_pitch(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.array([0.0, 0.1, 0.0]))
        assert result.reachable
        assert not np.any(np.isnan(result.angles))

    def test_pure_yaw(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.array([0.0, 0.0, 0.1]))
        assert result.reachable
        assert not np.any(np.isnan(result.angles))


class TestCombinedPose:
    def test_translation_and_rotation(self):
        platform = _make_platform()
        result = platform.calculate(
            np.array([1.0, 0.5, 0.5]),
            np.array([0.05, 0.1, 0.05]),
        )
        assert result.reachable
        assert not np.any(np.isnan(result.angles))

    def test_result_shapes(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.zeros(3))
        assert result.angles.shape == (6,)
        assert result.leg_vectors.shape == (3, 6)
        assert result.leg_lengths.shape == (6,)
        assert result.leg_positions.shape == (3, 6)
        assert result.horn_positions.shape == (3, 6)


class TestReachability:
    def test_unreachable_pose_no_nan(self):
        """An absurdly large translation should be unreachable but not NaN."""
        platform = _make_platform()
        result = platform.calculate(np.array([0.0, 0.0, 1000.0]), np.zeros(3))
        assert not result.reachable
        assert not np.any(np.isnan(result.angles))

    def test_unreachable_large_rotation(self):
        platform = _make_platform()
        result = platform.calculate(np.zeros(3), np.array([np.pi, 0.0, 0.0]))
        # Full 180-degree roll is likely unreachable for most geometries
        assert not np.any(np.isnan(result.angles))


class TestSmallPerturbations:
    def test_small_perturbation_produces_small_change(self):
        """Small input changes should produce small output changes (continuity)."""
        platform = _make_platform()
        result0 = platform.calculate(np.zeros(3), np.zeros(3))
        result1 = platform.calculate(np.array([0.001, 0.0, 0.0]), np.zeros(3))
        angle_diff = np.max(np.abs(result1.angles - result0.angles))
        assert angle_diff < 0.1  # Small perturbation -> small angle change


class TestInputValidation:
    def test_wrong_trans_shape(self):
        platform = _make_platform()
        with pytest.raises(ValueError, match="trans"):
            platform.calculate(np.array([1.0, 2.0]), np.zeros(3))

    def test_wrong_rotation_shape(self):
        platform = _make_platform()
        with pytest.raises(ValueError, match="rotation"):
            platform.calculate(np.zeros(3), np.array([1.0, 2.0, 3.0, 4.0]))

    def test_accepts_lists(self):
        """Should accept plain lists, not just numpy arrays."""
        platform = _make_platform()
        result = platform.calculate([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        assert result.reachable


class TestRepr:
    def test_repr_contains_class_name(self):
        platform = _make_platform()
        assert "StewartPlatform" in repr(platform)
        assert "PlatformConfig" in repr(platform)
