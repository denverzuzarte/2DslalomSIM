# 2D Slalom Simulator - System Architecture

## Node Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SIMULATOR NODE                               │
│                                                                      │
│  • Physics simulation (60 Hz)                                       │
│  • Process noise (Q matrix - water unsteadiness)                    │
│  • Camera simulation (8 FPS)                                        │
│  • Pygame visualization                                             │
└──────────┬──────────────┬──────────────┬──────────────┬────────────┘
           │              │              │              │
           │ /imu1/accel  │ /imu2/accel  │ /ground_truth│ /camera/detections
           │ (clean)      │ (clean)      │ (State)      │ (CameraDetections)
           │ 100Hz        │ 100Hz        │ 60Hz         │ 8Hz
           ▼              ▼              │              │
    ┌──────────┐   ┌──────────┐         │              │
    │   IMU1   │   │   IMU2   │         │              │
    │  NODE    │   │  NODE    │         │              │
    │          │   │          │         │              │
    │ + R1     │   │ + R2     │         │              │
    │ + bias1  │   │ + bias2  │         │              │
    │ integrate│   │ integrate│         │              │
    └────┬─────┘   └────┬─────┘         │              │
         │              │                │              │
         │ /imu1/state  │ /imu2/state    │              │
         │ (State)      │ (State)        │              │
         ▼              ▼                ▼              ▼
    ┌────────────────────────────────────────────────────┐
    │              CONTROLLER NODE                        │
    │                                                     │
    │  Mode 1: Direct Force (/controller/force)          │
    │  Mode 2: PID Setpoint (/controller/setpoint)       │
    │                                                     │
    │  Subscribes to state from: imu1/imu2/kalman        │
    └────────────────────┬───────────────────────────────┘
                         │
                         │ /control/command
                         │ (Vector3)
                         ▼
                   ┌─────────────┐
                   │  SIMULATOR  │
                   │   (physics) │
                   └─────────────┘
```

## Data Flow Details

### 1. Clean Acceleration Generation
```
Simulator Physics Engine
  ├─ Control force input
  ├─ Dynamics: F = ma + damping
  ├─ Process noise (Q matrix)
  │   └─ Represents water waves/unsteadiness
  └─ Clean acceleration output
      ├─► /imu1/accel (Vector3Stamped)
      └─► /imu2/accel (Vector3Stamped)
```

### 2. IMU Processing (Both IMU1 and IMU2)
```
Clean Acceleration
  │
  ├─ Add Bias (constant offset)
  │   └─ IMU1: [0.01, 0.01, 0.001]
  │   └─ IMU2: [0.05, 0.05, 0.005]
  │
  ├─ Add Measurement Noise (R matrix)
  │   └─ IMU1 R: diag([0.01, 0.01, 0.001])
  │   └─ IMU2 R: diag([0.05, 0.05, 0.005])
  │
  ├─ Integrate to Velocity
  │   └─ v(t+dt) = v(t) + a_corrupted * dt
  │
  └─ Integrate to Position
      └─ p(t+dt) = p(t) + v(t) * dt

Result: Drifting State Estimate → /imuX/state
```

### 3. Controller Dual-Mode Operation
```
┌─────────────────────────────────────┐
│        CONTROLLER NODE              │
│                                     │
│  ┌───────────────────────────────┐ │
│  │   Input Selection             │ │
│  │                               │ │
│  │  /controller/force (Vector3)  │ │
│  │         OR                    │ │
│  │  /controller/setpoint         │ │
│  │        (Setpoint)             │ │
│  └──────────┬────────────────────┘ │
│             │                      │
│             ▼                      │
│  ┌──────────────────────┐         │
│  │  Mode = Force?       │         │
│  └─────┬──────────┬─────┘         │
│        │ Yes      │ No             │
│        ▼          ▼                │
│  ┌─────────┐  ┌──────────────┐   │
│  │ Direct  │  │ PID Control  │   │
│  │ Force   │  │              │   │
│  │ Pass-   │  │ P: Kp * err  │   │
│  │ through │  │ I: Ki * ∫err │   │
│  │         │  │ D: Kd * derr │   │
│  └────┬────┘  └──────┬───────┘   │
│       │              │            │
│       └──────┬───────┘            │
│              ▼                    │
│       Limit Magnitude             │
│              │                    │
│              ▼                    │
│       /control/command            │
└──────────────┬────────────────────┘
               │
               ▼
          Simulator
```

### 4. Camera Simulation
```
Simulator (every 1/8 second)
  │
  ├─ Get vehicle pose (x, y, yaw)
  ├─ Calculate FOV cone (yaw ± 30°)
  │
  └─ For each pole:
      │
      ├─ Distance check (< 200m)
      ├─ Angle check (within FOV)
      │
      └─ If visible:
          │
          ├─ Add pixel noise
          │   └─ N(0, σ²=4)
          │
          └─ Publish detection
              └─ /camera/detections
```

## Noise and Drift Sources

### Process Noise (Q Matrix)
- **Where**: Applied in simulator during state update
- **What**: 6x6 covariance matrix on [x, y, θ, vx, vy, ω]
- **Effect**: Continuous random perturbations to TRUE state
- **Physical**: Water waves, currents, external disturbances

### Measurement Noise (R Matrix)
- **Where**: Applied in IMU nodes to clean acceleration
- **What**: 3x3 covariance matrix on [ax, ay, α]
- **Effect**: Random noise in acceleration measurements
- **Physical**: Sensor quantization, electronic noise, vibration

### Bias
- **Where**: Added in IMU nodes (constant)
- **What**: Constant offset [ax_bias, ay_bias, α_bias]
- **Effect**: Linear drift in velocity, quadratic in position
- **Physical**: Sensor calibration error, temperature drift, aging

### Drift Accumulation
```
Time 0: Perfect match
   ↓
   ├─ Process noise affects TRUE state
   ├─ Bias causes linear velocity error
   └─ Measurement noise integrates (random walk)
   ↓
Time T: Large position error
   │
   └─ IMU estimate diverges from ground truth
```

## Message Types

### State.msg
```
float64 x, y, yaw        # Position
float64 vx, vy, wyaw     # Velocity
std_msgs/Header header
```

### Setpoint.msg
```
float64 x, y, yaw        # Target position (no velocity)
std_msgs/Header header
```

### CameraDetection.msg
```
float64 x, y             # Pole position (with noise)
uint8 color              # 0=red, 1=white
float64 confidence
```

### CameraDetections.msg
```
CameraDetection[] detections
std_msgs/Header header
```

## Launch Configuration

### Default Launch
```bash
ros2 launch slalom_simulator slalom.launch.py
```

### With Different Localization Source
```bash
ros2 launch slalom_simulator slalom.launch.py localization:=imu2
```

### Running Individual Nodes
```bash
# Simulator
ros2 run slalom_simulator simulator_node

# IMU nodes
ros2 run slalom_simulator imu1_localization_node
ros2 run slalom_simulator imu2_localization_node

# Controller
ros2 run slalom_simulator controller_node
```

## Key Parameters

### Simulator
- `pool_width`, `pool_height`: Environment size
- `vehicle_size`, `vehicle_start_x/y`: Vehicle config
- `process_noise.sigma_*_squared`: Q matrix variances
- `camera_fov_degrees`, `camera_fps`, `camera_pixel_variance`: Camera config

### IMU1/IMU2
- `imu1_R.xx/xy/xt/yy/yt/tt`: R matrix elements
- `imu1_bias`: [ax, ay, α] bias vector
- `imu2_R.xx/xy/xt/yy/yt/tt`: Different R matrix
- `imu2_bias`: Different bias

### Controller
- `localization_source`: "imu1", "imu2", or "kalman"
- `pid_kp`, `pid_ki`, `pid_kd`: PID gains
- `max_control_force`: Force saturation limit

## Testing Scenarios

### Scenario 1: Force Control
```bash
# Constant forward force
ros2 topic pub /controller/force geometry_msgs/msg/Vector3 "{x: 2.0, y: 0.0, z: 0.0}"

# Observe drift between ground truth and IMU estimates
ros2 topic echo /ground_truth
ros2 topic echo /imu1/state
ros2 topic echo /imu2/state
```

### Scenario 2: Setpoint Control
```bash
# Command position setpoint
ros2 topic pub /controller/setpoint slalom_simulator/msg/Setpoint "{x: 300.0, y: 200.0, yaw: 0.0}"

# Watch PID controller drive to setpoint using drifting IMU
```

### Scenario 3: Compare IMU Quality
```bash
# Monitor both IMUs
ros2 topic echo /imu1/state &
ros2 topic echo /imu2/state &

# IMU2 should drift faster (5x worse noise + bias)
```

### Scenario 4: Camera Detections
```bash
# View camera detections
ros2 topic echo /camera/detections

# Visualize frame
ros2 run rqt_image_view rqt_image_view /gamestate/frame
```
