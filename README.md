# Stewart_Py

 [<img src="/doc/readme_resources/ezgif-7-487de93db9.gif">](/doc/readme_resources/ezgif-7-487de93db9.gif)

 Python implementation and step-by-step inverse kinematic explanation of stewart platform

[<img src="/doc/readme_resources/tutorial_ss.png">](/doc/readme_resources/tutorial_ss.png)


### Installation

```bash
pip install .
```

For development (editable install with test dependencies):

```bash
pip install -e ".[dev]"
```

### Usage

```python
import numpy as np
from stewart_py import StewartPlatform, PlatformConfig, plot_platform

# Define platform geometry
config = PlatformConfig(
    r_B=132/2,          # Base anchor circle radius
    r_P=100/2,          # Platform anchor circle radius
    horn_length=30,     # Servo horn length
    rod_length=130,     # Connecting rod length
    gamma_B=0.2269,     # Half-angle between base anchor pairs (rad)
    gamma_P=0.82,       # Half-angle between platform anchor pairs (rad)
)
platform = StewartPlatform(config)

# Solve inverse kinematics
result = platform.calculate(
    trans=np.array([0.0, 0.0, 0.0]),        # Translation [X, Y, Z]
    rotation=np.array([0.0, 0.0, 0.0]),     # Rotation [Roll, Pitch, Yaw] (rad)
)

print(result.angles)       # Servo angles in radians
print(result.reachable)    # True if pose is achievable

# 3D visualization
plot_platform(platform.B, result)
```

For a step-by-step mathematical tutorial, see the [Jupyter notebook](01_Stewart_Py_Inverse_Kinematics.ipynb).

### Running Tests

```bash
pytest
```

#### Sources and additional reading
Robert Eisele's Explanation and js implementation:
https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
https://github.com/infusion/Stewart

hbartle's MATLAB implementation
https://github.com/hbartle/Stewart_Platform/

Others resources
https://github.com/NicHub/stewart-platform-esp32
https://github.com/daniel-s-ingram/stewart
https://github.com/felixros2401/Stewart-Platform
