"""Smoke tests for visualization functions."""

import matplotlib
matplotlib.use("Agg")  # Non-interactive backend for testing

import matplotlib.pyplot as plt
import numpy as np
import pytest

from stewart_py import StewartPlatform, PlatformConfig, plot_platform


@pytest.fixture
def platform_and_result():
    config = PlatformConfig(
        r_B=6.2, r_P=5.0, horn_length=3.0, rod_length=12.0,
        gamma_B=0.2269, gamma_P=0.82,
    )
    platform = StewartPlatform(config)
    result = platform.calculate(np.zeros(3), np.zeros(3))
    return platform, result


class TestPlotPlatform:
    def test_returns_axes(self, platform_and_result):
        platform, result = platform_and_result
        ax = plot_platform(platform.B, result)
        assert ax is not None
        plt.close("all")

    def test_custom_limits(self, platform_and_result):
        platform, result = platform_and_result
        limits = (-50, 50, -50, 50, 0, 100)
        ax = plot_platform(platform.B, result, limits=limits)
        assert ax.get_xlim3d() == (-50, 50)
        plt.close("all")

    def test_accepts_existing_axes(self, platform_and_result):
        platform, result = platform_and_result
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        returned_ax = plot_platform(platform.B, result, ax=ax)
        assert returned_ax is ax
        plt.close("all")
