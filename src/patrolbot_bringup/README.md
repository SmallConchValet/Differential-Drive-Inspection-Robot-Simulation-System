# PatrolBot

ROS 2 Humble + Gazebo Classic 自主巡检考核工程。实现纯 P 航点跟踪、模式切换、Action 任务、激光减速/保持急停、人工恢复、速度仲裁、TF 监控及状态发布；不使用 Nav2/SLAM/MoveIt。电池按考核允许固定为 100%。

## 构建与启动

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch patrolbot_bringup patrolbot.launch.py
```

## 任务

```bash
ros2 service call /set_patrol_mode patrolbot_interfaces/srv/SetPatrolMode "{mode: 1}"
ros2 action send_goal /patrol patrolbot_interfaces/action/Patrol "{waypoints: [{x: 2.0, y: 4.5, yaw: 0.0}, {x: 4.0, y: 3.0, yaw: 0.0}, {x: 6.0, y: 1.0, yaw: 0.0}, {x: 1.0, y: 0.5, yaw: 0.0}]}" --feedback
```

QoS：scan 使用 SensorDataQoS；模式、急停和限速为 Reliable/TransientLocal/KeepLast(1)；状态为 Reliable/Volatile。Reliable 只提供 DDS 传输可靠性，不保证进程崩溃后的应用层不丢失。
