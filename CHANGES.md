# 2D→3D Upgrade: Changes Made & Remaining Work

## Summary
Converted `slalom_simulator` from a 2D simulation (state `[x,y,yaw,vx,vy,wyaw]`)
to a **3D AUV simulation** (state `[x,y,z,yaw,vx,vy,vz,wyaw]`) while keeping the
Pygame display as a 2D top-down view. Replaced all `slalom_interfaces` imports with
`auv_msgs`. Added a pressure sensor node. Added Z process/sensor noise.

---

## COMPLETED CHANGES

### `slalom_simulator/slalom_simulator/utils.py`
- Removed unused `Tuple` import
- `build_Q_matrix`: expanded from **6×6** to **8×8** for state
  `[x, y, z, yaw, vx, vy, vz, wyaw]`. Added `sigma_az2` parameter for Z noise.
- Added `load_covariance_matrix_4x4()` replacing `load_covariance_matrix_3x3()`.
  Keys: `xx`, `yy`, `zz`, `tt` (diagonal only).
- Removed unused `forward_axis` variable in `generate_gates()`.

### `slalom_simulator/slalom_simulator/simulator_node.py`  *(full rewrite)*
- **Imports**: removed `slalom_interfaces.msg`; added
  `auv_msgs.msg.AuvState, Orientation, ObjectDetections, ObjectDetected, PsData`
- **State**: `[x, y, z, yaw, vx, vy, vz, wyaw]` (8D vs old 6D)
- **New params**: `vehicle_start_z`, `process_noise.sigma_az_squared`,
  `buoyancy_accel` (default −0.10 cm/s²)
- **`update_dynamics()`**: Added Z axis physics with buoyancy when `z > 0`;
  Z clamped to 0 at surface. Q matrix is now 8×8. `last_accel` is 4-element
  `[ax, ay, az, alpha]`.
- **`imu_publishing_loop()`**: publishes clean linear accel `[ax,ay,az]` on
  `/imuN/accel` AND clean angular accel `[alpha]` on `/imuN/accel_angular`.
  Angular publishers are created in `main()` to avoid ordering issue.
- **`publish_ground_truth()`**: uses `AuvState` with `position.{x,y,z}`,
  `velocity.{x,y,z}`, `orientation.{roll,pitch,yaw}`, `angular_velocity.z`,
  `depth`, `acceleration.{x,y,z}`.
- **`publish_camera_detections()`**: uses `ObjectDetections` / `ObjectDetected`.
  Detection fields: `name` (pole color), `glob_midpoint.{x,y,z}`.
- **`draw_ghost()`**: reads `AuvState.position.{x,y}` and `AuvState.orientation.yaw`.
- **`render()`**: added Z depth HUD text display.
- **Publishers**: `/ground_truth` → `AuvState`; `/camera/detections` →
  `ObjectDetections`; `/auv/frame` (was `/gamestate/frame`); `/auv/gates`
  (was `/slalom/gates`).
- **Subscribers**: `/imu1/state`, `/imu2/state`, `/kalman/state` all use `AuvState`.
- **Control**: `control_force` is now 3-element `[fx,fy,fz]`.

### `slalom_simulator/slalom_simulator/base_imu_localization_node.py`  *(full rewrite)*
- **Imports**: removed `slalom_interfaces`; added `auv_msgs.msg.AuvState, Orientation`
- **State**: 8D `[x, y, z, yaw, vx, vy, vz, wyaw]`
- **New subscriber**: `/imuN/accel_angular` → caches `alpha` from `vector.x`
- **`accel_callback()`**: assembles 4-element vector `[ax,ay,az,alpha]`, applies
  4×4 R noise + 4-element bias, integrates 3D state. Z floor clamp at surface.
- **`publish_state()`**: publishes `AuvState` with full 3D fields.
- **`_load_noise_model()`**: now returns 4×4 matrix.
- **`_load_bias()`**: now returns 4-element array.
- Added `vehicle_start_z` parameter.

### `slalom_simulator/slalom_simulator/imu1_localization_node.py`  *(full rewrite)*
- R params: `imu1_R.{xx,yy,zz,tt}` (was `xx,xy,xt,yy,yt,tt`)
- Bias: `imu1_bias` is now 4-element `[ax,ay,az,alpha]`
- Uses `load_covariance_matrix_4x4()` instead of `load_covariance_matrix_3x3()`

### `slalom_simulator/slalom_simulator/imu2_localization_node.py`  *(full rewrite)*
- Same changes as IMU1 but 5× higher noise/bias values. Z noise = 0.025, bias z = 0.025.

### `slalom_simulator/slalom_simulator/kalman_localization_node.py`  *(full rewrite)*
- Removed old broken `sensor_msgs/Imu` + `geometry_msgs/PoseStamped` usage
- **Imports**: `auv_msgs.msg.AuvState, Orientation`
- **Subscribes to**: `/imu1/state`, `/imu2/state` (both `AuvState`) instead of raw IMU
- **State**: 8D `[x, y, z, yaw, vx, vy, vz, wyaw]`
- **`_fuse()`**: naive average of IMU1+IMU2 `AuvState` fields (placeholder for UKF)
- **Publishes**: `/kalman/state` as `AuvState`
- UKF stub methods `predict()` and `update()` removed (were empty pass stubs);
  replaced by single `_fuse()` method clearly marked as placeholder.

### `slalom_simulator/slalom_simulator/pressure_sensor_node.py`  *(NEW FILE)*
- New node: `pressure_sensor_node`
- Subscribes to `/ground_truth` (`AuvState`), reads `msg.depth`
- Publishes `auv_msgs/PsData` to `/pressure_sensor/data` at configurable Hz
- Noise model: `depth_measured = true_z + N(0, ps_noise_std²)`, **bias = 0**
- Params: `ps_freq` (default 10 Hz), `ps_noise_variance` (default 0.01 cm²)

### `slalom_simulator/slalom_simulator/controller_node.py`  *(full rewrite)*
- **Imports**: removed `slalom_interfaces`; added `auv_msgs.msg.AuvState`
- **State subscription**: uses `AuvState` (reads `position.x/y`)
- **Setpoint**: changed from custom `Setpoint` msg to `geometry_msgs/Vector3`
  (x=target_x, y=target_y)
- **Control output**: `control_force` is 3D `[fx,fy,fz]`; fz=0 unless direct force

### `slalom_simulator/config/params.yaml`  *(updated)*
- Added `vehicle_start_z: 0.0`
- Added `buoyancy_accel: -0.10`
- Added `process_noise.sigma_az_squared: 0.005`
- `imu1_R`/`imu2_R`: replaced `{xy,xt,yt}` keys with `zz` key
- `imu1_bias`/`imu2_bias`: expanded to 4-element arrays `[ax,ay,az,alpha]`
- Added `ps_freq: 10.0` and `ps_noise_variance: 0.01`

### `slalom_simulator/setup.py`
- Added entry point: `pressure_sensor_node = slalom_simulator.pressure_sensor_node:main`

### `slalom_simulator/launch/slalom.launch.py`
- Added `pressure_sensor_node` launch entry with `params_file`

### `slalom_simulator/package.xml`
- Added `<depend>auv_msgs</depend>`
- Removed rosidl generator/runtime deps (not needed — package generates no msgs)
- Updated description

---

## REMAINING WORK (for next session)

### ~~1. Fix `imu_publishing_loop()` publisher ordering bug~~ ✅ FIXED
Both angular accel publishers moved into `__init__()` before the IMU thread starts.

### 1. `auv_msgs/CMakeLists.txt` — add new messages if needed
`PsData.msg` currently has only `float64 depth` (no header). The pressure sensor
node publishes it without a header — that's fine, but if you want timestamped PS
data, add a header field:
```
std_msgs/Header header
float64 depth
```
Then update `pressure_sensor_node.py` to fill the header.

### 3. `auv_msgs/CMakeLists.txt` — `CameraDetection` / `CameraDetections` msgs  
The camera detections now use `ObjectDetected` / `ObjectDetections` from auv_msgs.
This works but the `name` field carries pole color ("red_pole" / "white_pole") as
a string. If a strongly-typed detection message is preferred, add new msgs to
auv_msgs (not requested here).

### 4. Kalman node — implement actual UKF
Currently a naive average. The node is structurally correct with 8D AuvState.
Implement the predict/update UKF with 8D state `[x,y,z,yaw,vx,vy,vz,wyaw]`.

### 5. Build verification
```bash
cd /home/ubuntu/land_value/tree_count/2DslalomSIM
colcon build --packages-select auv_msgs slalom_simulator
source install/setup.bash
ros2 launch slalom_simulator slalom.launch.py
```
Check for any import errors or missing message fields at runtime.

### 6. Topic renames to note
Old → New:
- `/gamestate/frame` → `/auv/frame`
- `/slalom/gates` → `/auv/gates`
- Controller setpoint: was `slalom_interfaces/Setpoint` → now `geometry_msgs/Vector3`

---

## Topic Graph (After Changes)

```
SimulatorNode (60 Hz physics, 2D Pygame display)
  Publishes:
    /imu1/accel          Vector3Stamped  [ax, ay, az]      100 Hz (thread)
    /imu1/accel_angular  Vector3Stamped  [alpha, 0, 0]     100 Hz (thread)
    /imu2/accel          Vector3Stamped  [ax, ay, az]      100 Hz (thread)
    /imu2/accel_angular  Vector3Stamped  [alpha, 0, 0]     100 Hz (thread)
    /ground_truth        AuvState        true 3D state      60 Hz
    /camera/detections   ObjectDetections pole detections   8 Hz
    /auv/frame           Image           pygame view        30 Hz
    /control/force       Vector3         current force      60 Hz
    /auv/gates           MarkerArray     gate markers        1 Hz
  Subscribes:
    /control/command     Vector3         [fx,fy,fz]
    /imu1/state          AuvState
    /imu2/state          AuvState
    /kalman/state        AuvState

IMU1LocalizationNode (100 Hz via callbacks)
  Subscribes: /imu1/accel, /imu1/accel_angular
  Publishes:  /imu1/state  AuvState  (3D dead-reckoning + noise + bias)

IMU2LocalizationNode (100 Hz via callbacks)
  Subscribes: /imu2/accel, /imu2/accel_angular
  Publishes:  /imu2/state  AuvState  (3D dead-reckoning, 5x worse)

KalmanLocalizationNode
  Subscribes: /imu1/state, /imu2/state  (AuvState)
  Publishes:  /kalman/state  AuvState   (naive average — replace with UKF)

PressureSensorNode (10 Hz)
  Subscribes: /ground_truth  AuvState
  Publishes:  /pressure_sensor/data  PsData  (noisy depth, bias=0)

ControllerNode (20 Hz)
  Subscribes: /{localization_source}/state  AuvState
              /controller/force  Vector3   (direct force mode)
              /controller/setpoint  Vector3  (PID target [x,y])
  Publishes:  /control/command  Vector3  [fx,fy,fz]
```
