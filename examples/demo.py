"""Demonstrate Stewart Platform inverse kinematics."""

import numpy as np
import matplotlib.pyplot as plt

from stewart_py import StewartPlatform, PlatformConfig, plot_platform


def main() -> None:
    config = PlatformConfig(
        r_B=132 / 2,
        r_P=100 / 2,
        horn_length=30,
        rod_length=130,
        gamma_B=0.2269,
        gamma_P=0.82,
    )
    platform = StewartPlatform(config)

    # Sweep pitch angle from -20 to +20 degrees
    for ix in range(-20, 20):
        angle = np.pi * ix / 180
        result = platform.calculate(
            trans=np.array([2.0, 1.0, 0.0]),
            rotation=np.array([0.0, angle, 0.0]),
        )
        print(f"pitch={ix:+3d}°  angles(rad)={np.round(result.angles, 4)}  reachable={result.reachable}")

        ax = plot_platform(platform.B, result)
        plt.pause(0.1)
        plt.clf()

    plt.show()


if __name__ == "__main__":
    main()
