"""Tests for PlatformConfig."""

import numpy as np
import pytest

from stewart_py import PlatformConfig


def _default_config(**overrides) -> PlatformConfig:
    defaults = dict(
        r_B=6.2,
        r_P=5.0,
        horn_length=3.0,
        rod_length=12.0,
        gamma_B=0.2269,
        gamma_P=0.82,
    )
    defaults.update(overrides)
    return PlatformConfig(**defaults)


class TestPlatformConfigCreation:
    def test_valid_config(self):
        cfg = _default_config()
        assert cfg.r_B == 6.2
        assert cfg.horn_length == 3.0
        assert cfg.rod_length == 12.0

    def test_default_ref_rotation(self):
        cfg = _default_config()
        assert abs(cfg.ref_rotation - 5 * np.pi / 6) < 1e-14

    def test_custom_ref_rotation(self):
        cfg = _default_config(ref_rotation=0.0)
        assert cfg.ref_rotation == 0.0


class TestPlatformConfigFrozen:
    def test_cannot_mutate(self):
        cfg = _default_config()
        with pytest.raises(AttributeError):
            cfg.r_B = 10.0

    def test_cannot_add_field(self):
        cfg = _default_config()
        with pytest.raises(AttributeError):
            cfg.new_field = 42
