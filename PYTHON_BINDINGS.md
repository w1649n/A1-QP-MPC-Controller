# Python Bindings for A1 MPC Controller

This package provides Python bindings for the A1 quadruped MPC controller, enabling RL-MPC integration with Isaac Lab.

## Features

- Direct Python access to the C++ MPC solver
- Dynamic weight setting for RL-MPC integration
- Simplified state interface without ROS dependencies
- High-performance QP solver (OSQP)

## Installation

### Prerequisites

1. **System dependencies:**
   ```bash
   sudo apt-get update
   sudo apt-get install -y \
       cmake \
       libeigen3-dev \
       libosqp-dev \
       python3-dev \
       python3-pip
   ```

2. **OsqpEigen:**
   ```bash
   git clone https://github.com/robotology/osqp-eigen.git
   cd osqp-eigen
   mkdir build && cd build
   cmake ..
   make
   sudo make install
   ```

3. **pybind11:**
   ```bash
   pip install pybind11
   ```

### Build and Install

From the repository root:

```bash
pip install -e .
```

This will:
1. Configure CMake with `BUILD_PYTHON_BINDINGS=ON`
2. Build the C++ extension module
3. Install it in editable mode

## Usage

### Basic Example

```python
import numpy as np
import mpc_controller

# Create MPC instance
mpc = mpc_controller.ConvexMpc()

# Set weights (from RL policy or manually)
q_weights = np.array([80.0, 80.0, 1.0, 0.0, 0.0, 270.0, 
                      1.0, 1.0, 20.0, 20.0, 20.0, 20.0, 0.0])
r_weights = np.array([1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6,
                      1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6])
mpc.set_weights(q_weights, r_weights)

# Create robot state
state = mpc_controller.MPCState()
state.root_euler = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
state.root_pos = np.array([0.0, 0.0, 0.3])     # x, y, z
state.root_ang_vel = np.array([0.0, 0.0, 0.0]) # body frame
state.root_lin_vel = np.array([0.0, 0.0, 0.0]) # body frame

# Foot positions (3x4 matrix: [x,y,z] for 4 legs)
state.foot_pos = np.array([
    [0.17, 0.17, -0.17, -0.17],   # x: FL, FR, RL, RR
    [0.15, -0.15, 0.15, -0.15],   # y
    [-0.05, -0.05, -0.05, -0.05]  # z
])

# Contact states (True = stance, False = swing)
state.contacts = [True, True, True, True]

# Desired trajectory
state.root_pos_d = np.array([0.0, 0.0, 0.3])
state.root_euler_d = np.array([0.0, 0.0, 0.0])
state.root_lin_vel_d = np.array([0.5, 0.0, 0.0])  # desired velocity
state.root_ang_vel_d = np.array([0.0, 0.0, 0.0])

# Solve MPC
grf = mpc.solve(state, dt=0.0025)  # Returns 12-dim GRF vector
print(f"Ground Reaction Forces: {grf}")
```

### RL-MPC Integration

The package is designed for RL-MPC architectures where a reinforcement learning policy outputs MPC weights:

```python
import mpc_controller
import numpy as np

class RLMPCController:
    def __init__(self):
        self.mpc = mpc_controller.ConvexMpc()
        self.rl_policy = YourRLPolicy()  # Your RL policy
        
    def step(self, observation):
        # RL policy outputs MPC weights
        q_weights, r_weights = self.rl_policy.predict(observation)
        
        # Update MPC weights
        self.mpc.set_weights(q_weights, r_weights)
        
        # Create state from observation
        state = mpc_controller.MPCState()
        state.root_pos = observation['root_pos']
        state.root_euler = observation['root_euler']
        # ... set other state variables
        
        # Solve MPC for ground reaction forces
        grf = self.mpc.solve(state)
        
        return grf
```

## API Reference

### `MPCState`

Robot state structure for MPC input.

**Attributes:**
- `root_euler` (ndarray[3]): Euler angles [roll, pitch, yaw] in radians
- `root_pos` (ndarray[3]): Position [x, y, z] in meters
- `root_ang_vel` (ndarray[3]): Angular velocity in body frame (rad/s)
- `root_lin_vel` (ndarray[3]): Linear velocity in body frame (m/s)
- `foot_pos` (ndarray[3,4]): Foot positions in absolute frame (meters)
- `contacts` (list[bool]): Contact states for 4 legs [FL, FR, RL, RR]
- `robot_mass` (float): Robot mass in kg (default: 15.0)
- `trunk_inertia` (ndarray[3,3]): Trunk inertia matrix
- `root_euler_d` (ndarray[3]): Desired euler angles
- `root_pos_d` (ndarray[3]): Desired position
- `root_ang_vel_d` (ndarray[3]): Desired angular velocity
- `root_lin_vel_d` (ndarray[3]): Desired linear velocity

### `ConvexMpc`

MPC controller class.

**Methods:**

#### `__init__()`
Create a new MPC controller with default weights.

#### `set_weights(q_weights, r_weights)`
Set MPC cost function weights.

**Parameters:**
- `q_weights` (ndarray[13]): State cost weights for [roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz, gravity]
- `r_weights` (ndarray[12]): Control cost weights for ground reaction forces

#### `solve(state, dt=0.0025) -> ndarray[12]`
Solve MPC optimization problem.

**Parameters:**
- `state` (MPCState): Current robot state
- `dt` (float): Time step in seconds (default: 0.0025)

**Returns:**
- `grf` (ndarray[12]): Ground reaction forces in body frame [FL_xyz, FR_xyz, RL_xyz, RR_xyz]

#### `reset()`
Reset the MPC solver internal state.

## MPC Weight Parameters

The 13-dimensional `q_weights` vector corresponds to:

```python
# Index:  0      1       2      3   4    5    6   7   8    9   10  11   12
q_weights = [roll, pitch, yaw, x,  y,   z,  wx, wy, wz,  vx, vy, vz, gravity]
```

Example weight configurations:

**Default (position control):**
```python
q_weights = [80.0, 80.0, 1.0, 0.0, 0.0, 270.0, 1.0, 1.0, 20.0, 20.0, 20.0, 20.0, 0.0]
```

**RL-MPC (from silvery107/rl-mpc-locomotion):**
```python
q_weights = [1.0, 1.5, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.1, 1.0, 1.0, 0.1, 0.0]
```

The 12-dimensional `r_weights` vector is typically small to penalize control effort:
```python
r_weights = [1e-5] * 12
```

## Testing

Run the test script:

```bash
python test_mpc_bindings.py
```

## Reference Architecture

This implementation follows the RL-MPC architecture from:
- [silvery107/rl-mpc-locomotion](https://github.com/silvery107/rl-mpc-locomotion)

## Troubleshooting

### Import Error

If you get `ImportError: No module named 'mpc_controller'`:

1. Make sure you built with Python bindings enabled
2. Check that pybind11 is installed: `pip list | grep pybind11`
3. Reinstall: `pip install -e . --force-reinstall`

### Build Errors

If you get CMake errors:

1. Ensure all dependencies are installed (see Prerequisites)
2. Check that OsqpEigen is properly installed
3. Try building manually:
   ```bash
   cd src/a1_cpp
   mkdir -p build && cd build
   cmake .. -DBUILD_PYTHON_BINDINGS=ON
   make
   ```

### ROS Dependency Issues

The Python bindings are designed to work without ROS. However, the build system still uses catkin. To build in a non-ROS environment, you may need to modify the CMakeLists.txt to make catkin optional.

## License

See the main repository LICENSE file.
