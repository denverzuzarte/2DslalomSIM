# 2D Slalom Simulator - Implementation Summary

## Changes Implemented

### 1. Base IMU Localization Node (NEW)
**File:** `slalom_simulator/base_imu_localization_node.py`

- Created abstract base class `BaseIMULocalizationNode` for IMU sensors
- Common functionality:
  - Subscribes to clean acceleration from simulator on `/{imu_name}/accel`
  - Publishes State messages on `/{imu_name}/state`
  - Adds measurement noise (R matrix) and bias to clean acceleration
  - Performs double integration (acceleration → velocity → position)
  - Causes drift due to bias + noise accumulation

- Abstract methods (to be implemented by subclasses):
  - `_declare_imu_parameters()`: Declare IMU-specific parameters
  - `_load_noise_model()`: Load R matrix
  - `_load_bias()`: Load bias vector

### 2. IMU1 Localization Node (UPDATED)
**File:** `slalom_simulator/imu1_localization_node.py`

- Now extends `BaseIMULocalizationNode`
- Parameters:
  - R matrix: `imu1_R.xx`, `imu1_R.xy`, `imu1_R.xt`, `imu1_R.yy`, `imu1_R.yt`, `imu1_R.tt`
  - Bias: `imu1_bias` = [0.01, 0.01, 0.001]
- Default R matrix is diagonal with values [0.01, 0.01, 0.001]
- Publishes to `/imu1/state`

### 3. IMU2 Localization Node (UPDATED)
**File:** `slalom_simulator/imu2_localization_node.py`

- Now extends `BaseIMULocalizationNode`
- Parameters:
  - R matrix: `imu2_R.xx`, `imu2_R.xy`, `imu2_R.xt`, `imu2_R.yy`, `imu2_R.yt`, `imu2_R.tt`
  - Bias: `imu2_bias` = [0.05, 0.05, 0.005]
- Default R matrix is diagonal with values [0.05, 0.05, 0.005] (5x worse than IMU1)
- Publishes to `/imu2/state`

### 4. Controller Node (UPDATED)
**File:** `slalom_simulator/controller_node.py`

Complete rewrite to support dual-mode operation:

#### Mode 1: Direct Force Control
- Subscribe to `/controller/force` (Vector3)
- Directly applies commanded force to vehicle
- Overrides setpoint mode when force command is received

#### Mode 2: PID Setpoint Control
- Subscribe to `/controller/setpoint` (Setpoint message: x, y, yaw)
- Computes control force using PID to reach setpoint
- Only controls x, y position (yaw control is TODO)
- PID gains: `pid_kp`, `pid_ki`, `pid_kd`

#### Features:
- Publishes to `/control/command` (Vector3)
- Force magnitude limited by `max_control_force`
- Subscribes to state from configurable localization source (`localization_source` parameter)
- Automatically switches modes based on which topic receives commands
- Resets PID integrators when switching to setpoint mode

### 5. Parameters File (UPDATED)
**File:** `config/params.yaml`

New structure:
```yaml
# Process noise (Q matrix) - water unsteadiness
process_noise:
  sigma_ax_squared: 0.01
  sigma_ay_squared: 0.01
  sigma_alpha_squared: 0.001

# IMU1 measurement noise (R matrix) - full covariance support
imu1_R:
  xx: 0.01      # X acceleration variance
  xy: 0.0       # X-Y covariance
  xt: 0.0       # X-Theta covariance
  yy: 0.01      # Y acceleration variance
  yt: 0.0       # Y-Theta covariance
  tt: 0.001     # Theta (angular) acceleration variance
imu1_bias: [0.01, 0.01, 0.001]

# IMU2 measurement noise (R matrix) - different from IMU1
imu2_R:
  xx: 0.05      # 5x worse than IMU1
  xy: 0.0
  xt: 0.0
  yy: 0.05
  yt: 0.0
  tt: 0.005
imu2_bias: [0.05, 0.05, 0.005]

# Camera parameters
camera_fov_degrees: 60.0
camera_fps: 8.0
camera_pixel_variance: 4.0
camera_range_max: 200.0
```

## System Architecture

### Data Flow

1. **Simulator Node**
   - Publishes CLEAN acceleration to `/imu1/accel` and `/imu2/accel` (100 Hz)
   - Publishes ground truth to `/ground_truth` (60 Hz)
   - Publishes camera detections to `/camera/detections` (configurable FPS)
   - Publishes camera frame to `/gamestate/frame` (30 Hz)
   - Subscribes to `/control/command` for control force

2. **IMU1/IMU2 Localization Nodes**
   - Subscribe to clean acceleration
   - Add noise (R matrix) + bias
   - Integrate to estimate state
   - Publish to `/imu1/state` or `/imu2/state`

3. **Controller Node**
   - Subscribes to state from selected localization source
   - Subscribes to `/controller/force` (direct mode)
   - Subscribes to `/controller/setpoint` (PID mode)
   - Publishes to `/control/command`

### ROS2 Topics

#### Inputs to System
- `/controller/force` (Vector3) - Direct force commands
- `/controller/setpoint` (Setpoint) - Position setpoints for PID

#### Simulator Outputs
- `/imu1/accel` (Vector3Stamped) - Clean IMU1 acceleration
- `/imu2/accel` (Vector3Stamped) - Clean IMU2 acceleration
- `/ground_truth` (State) - True vehicle state
- `/camera/detections` (CameraDetections) - Pole detections
- `/gamestate/frame` (Image) - Pygame visualization

#### Localization Outputs
- `/imu1/state` (State) - IMU1 state estimate
- `/imu2/state` (State) - IMU2 state estimate
- `/kalman/state` (State) - Kalman filter estimate (stub)

#### Control Output
- `/control/command` (Vector3) - Control force to simulator

### ROS2 Messages

1. **State.msg** (EXISTING)
   - `x`, `y`, `yaw` - Position
   - `vx`, `vy`, `wyaw` - Velocity
   - `header` - Timestamp

2. **Setpoint.msg** (EXISTING)
   - `x`, `y`, `yaw` - Target position
   - `header` - Timestamp

3. **CameraDetection.msg** (EXISTING)
   - `x`, `y` - Pole position (with pixel noise)
   - `color` - 0=red, 1=white
   - `confidence`

4. **CameraDetections.msg** (EXISTING)
   - `detections[]` - Array of CameraDetection
   - `header`

## Key Features

### Process Noise (Q Matrix)
- Represents water unsteadiness (waves)
- 6x6 covariance matrix for state [x, y, yaw, vx, vy, omega]
- Applied during physics integration in simulator
- Causes continuous drift in TRUE state

### Measurement Noise (R Matrix)
- 3x3 covariance matrix for [ax, ay, alpha]
- Each IMU has its own R matrix (configurable, full covariance)
- Applied by IMU nodes to clean acceleration
- Causes drift in ESTIMATED state

### Bias
- Constant offset added to acceleration measurements
- Each IMU has its own bias vector [ax_bias, ay_bias, alpha_bias]
- Causes linear drift growth in velocity (integrates to quadratic position error)

### Camera
- 60° FOV cone in direction of vehicle heading
- 8 FPS (configurable)
- Detects poles within range (200m default)
- Adds Gaussian pixel noise (variance = 4.0)
- Publishes position and color of visible poles

## Testing

### Manual Testing Commands

```bash
# Build
cd /path/to/2DslalomSIM
colcon build

# Source
source install/setup.bash

# Launch simulator
ros2 launch slalom_simulator slalom.launch.py

# Test direct force control
ros2 topic pub /controller/force geometry_msgs/msg/Vector3 "{x: 1.0, y: 0.0, z: 0.0}"

# Test setpoint control
ros2 topic pub /controller/setpoint slalom_simulator/msg/Setpoint "{x: 300.0, y: 200.0, yaw: 0.0}"

# Monitor topics
ros2 topic echo /imu1/state
ros2 topic echo /imu2/state
ros2 topic echo /ground_truth
ros2 topic echo /camera/detections
```

## Implementation Status

✅ **Completed:**
- Base IMU localization class
- IMU1 localization node (using base class)
- IMU2 localization node (using base class)
- Controller dual-mode (force + PID setpoint)
- Parameters file with full R matrices
- Process noise (Q matrix)
- Camera simulation
- Message definitions

⏳ **TODO:**
- Kalman localization node (currently stub)
- Yaw control in PID mode
- Integration tests
- Documentation

## Notes

1. **Drift is intentional** - Both process noise and measurement noise/bias cause drift over time. This simulates real-world sensor limitations and motivates the need for sensor fusion (Kalman filter).

2. **IMU2 is worse than IMU1** - This demonstrates the effect of different sensor quality on localization accuracy.

3. **Controller mode switching** - The controller automatically switches between force and setpoint modes based on which topic receives commands. Direct force takes precedence.

4. **Camera noise model** - Simplified to Gaussian noise on pole positions. A more realistic model would account for distance, angle, and feature detection errors.

5. **Full covariance support** - All R matrices support full 3x3 covariance (not just diagonal). This allows modeling correlated noise between axes.
