# 2D Slalom Simulator with ROS2

A Pygame-based 2D simulator for autonomous underwater vehicle (AUV) slalom navigation with multi-sensor fusion and ROS2 integration.

## Overview

This simulator models a vehicle navigating through a 3-gate slalom course using multiple IMU sensors and Kalman filter-based localization. All communication is handled via ROS2 nodes, enabling modular development and testing of different control and localization strategies.

## Physical Specifications

### World Parameters
- **Pool Dimensions**: 600×800 pixels (configurable)
- **Scale**: 1 pixel = 1 cm
- **Background**: Blue color representing water

### Vehicle Specifications
- **Size**: 40×40 pixels
- **Color**: Dark grey (#505050)
- **Starting Position**: (400, 500) pixels
- **State Variables**: x, y, yaw

### Slalom Gate Configuration

The course consists of **3 gates** arranged in a zig-zag pattern.

#### Gate Structure
Each gate contains:
- **1 Red Pole**: Positioned at gate center point Gᵢ
- **1 White Pole**: Offset perpendicular to gate orientation

#### Gate Geometry
For gate i with center Gᵢ ∈ ℝ² and orientation θᵢ:

**Local Coordinate Frame:**
- Forward axis: f̂ᵢ = (cos θᵢ, sin θᵢ)
- Right axis: r̂ᵢ = (-sin θᵢ, cos θᵢ)

**Pole Positions:**
- Red pole: Pᵣₑd = Gᵢ
- White pole: Pwhᵢₜₑ = Gᵢ + dᵣw · r̂ᵢ

**Gate Dimensions:**
- Distance red-to-white: dᵣw ~ N(100, 5²) pixels
- Gate width (passage): dₘₐₜₑ ~ N(70, 5²) pixels
- Poles are parallel within each gate

#### Zig-Zag Constraint
The white pole alternates sides across consecutive gates:
- Gate 1: white pole on +r̂₁ (right side)
- Gate 2: white pole on -r̂₂ (left side)  
- Gate 3: white pole on +r̂₃ (right side)

This creates the characteristic slalom weaving pattern.

**Note**: All random parameters are sampled once at initialization.

## Sensor System

### IMU Configuration
The system includes **2 IMU sensors** providing measurements at ~100 Hz:

**IMU 1 (Primary):**
- Lower covariance (more accurate)
- Provides: acceleration in x, y, and angular acceleration
- Has bias vector: bias₁ = [bₓ₁, bᵧ₁, bω₁]
- Measurement noise covariance: R₁ (3×3 matrix)

**IMU 2 (Secondary):**
- Higher covariance (less accurate)
- Provides: acceleration in x, y, and angular acceleration
- Has bias vector: bias₂ = [bₓ₂, bᵧ₂, bω₂]
- Measurement noise covariance: R₂ (3×3 matrix)

### Process Noise
- Process noise covariance matrix: Q (3×3 matrix)
- Represents uncertainty in vehicle dynamics

## ROS2 Architecture

### Nodes to Implement

#### 1. `simulator_node`
**Publishes:**
- `/imu1/data` (sensor_msgs/Imu) - IMU 1 measurements at 100 Hz
- `/imu2/data` (sensor_msgs/Imu) - IMU 2 measurements at 100 Hz
- `/ground_truth` (geometry_msgs/PoseStamped) - True vehicle state
- `/camera/image` (sensor_msgs/Image) - Game frame as ROS2 Image
- `/control/force` (geometry_msgs/Vector3) - Applied control force

**Subscribes:**
- `/control/command` (geometry_msgs/Vector3) - Control force commands

**Functionality:**
- Renders Pygame visualization
- Simulates vehicle dynamics
- Generates IMU measurements with noise and bias
- Publishes game frames as ROS2 images

#### 2. `imu1_localization_node`
**Subscribes:**
- `/imu1/data` (sensor_msgs/Imu)

**Publishes:**
- `/imu1/pose` (geometry_msgs/PoseStamped) - Position estimate from IMU1 integration

**Functionality:**
- Integrates IMU1 accelerations to estimate position
- Dead-reckoning from starting position

#### 3. `imu2_localization_node`
**Subscribes:**
- `/imu2/data` (sensor_msgs/Imu)

**Publishes:**
- `/imu2/pose` (geometry_msgs/PoseStamped) - Position estimate from IMU2 integration

**Functionality:**
- Integrates IMU2 accelerations to estimate position
- Dead-reckoning from starting position

#### 4. `kalman_localization_node` (PLACEHOLDER)
**Subscribes:**
- `/imu1/data` (sensor_msgs/Imu)
- `/imu2/data` (sensor_msgs/Imu)

**Publishes:**
- `/kalman/pose` (geometry_msgs/PoseStamped) - Fused position estimate

**Functionality:**
- **This node should be left as a stub/placeholder file**
- Will implement Unscented Kalman Filter (UKF) to fuse both IMU measurements
- To be implemented separately by the user

#### 5. `controller_node`
**Subscribes:**
- `/imu1/pose` OR `/imu2/pose` OR `/kalman/pose` (selectable via parameter)
- `/slalom/gates` (custom message) - Gate positions

**Publishes:**
- `/control/command` (geometry_msgs/Vector3) - Control force commands

**Parameters:**
- `localization_source` (string): "imu1", "imu2", or "kalman"

**Functionality:**
- PID controller with Artificial Potential Field (APF)
- Selectable localization source via launch parameter/flag
- Generates attractive forces toward next gate
- Generates repulsive forces from poles
- Outputs control force vector

## Visualization

The Pygame window displays:

1. **Main Vehicle** (dark grey, 40×40 px) - Ground truth position
2. **IMU1 Ghost** (semi-transparent blue-grey) - Labeled "IMU1"
3. **IMU2 Ghost** (semi-transparent blue-grey) - Labeled "IMU2"
4. **KF Ghost** (semi-transparent blue-grey) - Labeled "KF"
5. **Control Force Arrow** - Emanating from vehicle center
6. **Slalom Gates** - Red and white poles
7. **Blue Background** - Representing water

### Ghost Vehicles
- Rendered with transparency (alpha ~128)
- Color: Blue-grey (#708090 with alpha)
- Display their source label ("IMU1", "IMU2", "KF") on top
- Show estimated position from respective localization nodes

## Configuration

### Configurable Parameters
```python
# Pool dimensions
POOL_WIDTH = 600  # pixels
POOL_HEIGHT = 800  # pixels

# IMU parameters
IMU1_COVARIANCE = [[...]]  # 3x3 matrix
IMU2_COVARIANCE = [[...]]  # 3x3 matrix
IMU1_BIAS = [bx1, by1, bw1]
IMU2_BIAS = [bx2, by2, bw2]
PROCESS_NOISE_Q = [[...]]  # 3x3 matrix

# Vehicle parameters
VEHICLE_SIZE = 40  # pixels
VEHICLE_START_X = 400
VEHICLE_START_Y = 500

# Gate parameters
NUM_GATES = 3
GATE_WIDTH_MEAN = 70
GATE_WIDTH_STD = 5
POLE_SEPARATION_MEAN = 100
POLE_SEPARATION_STD = 5
```

## Building and Running

### Prerequisites
```bash
# ROS2 (Humble recommended)
# Python 3.8+
# Pygame
# NumPy
# rclpy
# cv_bridge (for image publishing)
```

### Installation
```bash
cd ~/ros2_ws/src
git clone <repository_url> slalom_simulator
cd ~/ros2_ws
colcon build --packages-select slalom_simulator
source install/setup.bash
```

### Launch
```bash
# Launch full system with IMU1 localization
ros2 launch slalom_simulator slalom.launch.py localization:=imu1

# Launch with Kalman filter localization
ros2 launch slalom_simulator slalom.launch.py localization:=kalman

# Launch with IMU2 localization
ros2 launch slalom_simulator slalom.launch.py localization:=imu2
```

## Development Notes

### For the User
The `kalman_localization_node.py` file will be created as a **placeholder/stub** with:
- Proper ROS2 node structure
- Subscriptions to both IMU topics
- Publisher for the fused pose estimate
- Empty callback functions for you to implement UKF logic

You will need to implement:
- Unscented Kalman Filter (UKF) algorithm
- State prediction using process model
- Measurement update using both IMU measurements
- Proper covariance propagation

### Code Structure
```
slalom_simulator/
├── slalom_simulator/
│   ├── __init__.py
│   ├── simulator_node.py          # Main Pygame simulator
│   ├── imu1_localization_node.py  # IMU1 integration
│   ├── imu2_localization_node.py  # IMU2 integration
│   ├── kalman_localization_node.py # UKF stub (to be implemented)
│   ├── controller_node.py         # PID + APF controller
│   └── utils.py                   # Shared utilities
├── launch/
│   └── slalom.launch.py           # Launch file
├── config/
│   └── params.yaml                # Configuration parameters
├── package.xml
├── setup.py
└── README.md
```

## Physics & Dynamics

### Vehicle Dynamics
Simple point-mass model with force control:
- State: [x, y, yaw, vₓ, vᵧ, ω]
- Control input: Force vector [Fₓ, Fᵧ]
- Integration: Semi-implicit Euler or RK4

### IMU Measurement Model
```
z_imu = a_true + bias + noise
where:
  a_true = vehicle acceleration
  bias ~ fixed per IMU
  noise ~ N(0, R)
```

## Performance Requirements
- Simulation rate: 60 FPS (Pygame)
- IMU publishing rate: ~100 Hz
- ROS2 node communication: Real-time capable

## Troubleshooting

### Common Issues
1. **Localization drift**: Expected behavior for pure IMU integration
2. **Ghost vehicles diverging**: Normal - shows sensor fusion benefits
3. **Control instability**: Tune PID gains and APF parameters

## License
[Specify your license]

## Contributors
[Your name/team]

## References
- ROS2 Documentation: https://docs.ros.org/
- Pygame Documentation: https://www.pygame.org/docs/
- Kalman Filter Tutorial: [Add relevant references]
