

## FIX 3 — Rebuild và test sensors

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select anhc_robot_description
source install/setup.bash
```

**Terminal 1** — restart simulation:
```bash
source ~/robot_ws/install/setup.bash
ros2 launch anhc_robot_bringup anhc_simulation.launch.py
```

**Terminal 2** — đợi 15 giây rồi verify:
```bash
source ~/robot_ws/install/setup.bash

# Test sensors trực tiếp trong Gazebo trước
gz topic -e -t /anhc/scan
# Phải thấy data với ranges: [...] ngay lập tức
```

```bash
# Nếu gz topic có data → test ROS bridge
ros2 topic hz /anhc/scan
ros2 topic hz /anhc/lidar_3d/points

# Test TF
ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=false
```

---

## Verify hoàn chỉnh — checklist

```bash
# 1. Controllers active
ros2 control list_controllers
# expect: joint_state_broadcaster active, anhc_diff_drive_controller active

# 2. Topics đầy đủ
ros2 topic list | grep anhc
# expect: /anhc/scan, /anhc/odom, /anhc/lidar_3d/points, /anhc/cmd_vel

# 3. Sensor rates
ros2 topic hz /anhc/scan            # ~15 Hz
ros2 topic hz /anhc/lidar_3d/points # ~10 Hz
ros2 topic hz /anhc/odom            # ~17-30 Hz

# 4. TF tree đầy đủ
ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=false
# expect: translation [-7, 0, 0.075]

# 5. Drive test
ros2 topic pub --once /anhc/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
# Robot phải di chuyển trong Gazebo
```
