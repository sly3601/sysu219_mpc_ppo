# SYSU219 纯 PPO 真机与 Gazebo

本工程使用 ROS 2 Humble。真机和 Gazebo 共用 v3 的 71 维纯 PPO 策略 `policy:=local`。策略读取 IMU、关节反馈和本地摆足参考，运行时跳过里程计。真机硬件读写为 500 Hz，主控按 250 Hz 刷新关节目标，PPO 按 50 Hz 推理。

## 编译

在新终端进入本工程的 `ros2_ws`。现有环境使用 ONNX Runtime 1.18.1（`/opt/onnxruntime`）、ROS 2 `serial` 库（`/usr/local`）、Eigen3 和 yaml-cpp。Gazebo 还需要 Gazebo Classic、`gazebo_ros` 和 `gazebo_ros2_control`。已有这些依赖的机器可以直接编译。

```bash
cd /home/yzz/sysu219_mpc_ppo/ros2_ws
source /opt/ros/humble/setup.bash
```

下面两条编译命令选一条执行。克隆到其他位置时，将上面的 `cd` 改成实际路径。从旧版升级到 v3 时，先编译本版控制器和描述包，再加载安装树。

### 普通并行编译

```bash
colcon build \
  --packages-up-to ocs2_core leg_pd_controller sysu219_guide_controller sysu219_description keyboard_input hardware_sysu219 \
  --symlink-install \
  --event-handlers console_direct+ \
  --continue-on-error \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 限制编译并行度

沿用原工程的低负载编译方式。每次构建一个包，包内最多运行一个编译任务。

```bash
MAKEFLAGS="-j1 -l1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
colcon build \
  --executor sequential \
  --packages-up-to ocs2_core leg_pd_controller sysu219_guide_controller sysu219_description keyboard_input hardware_sysu219 \
  --symlink-install \
  --event-handlers console_direct+ \
  --continue-on-error \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 加载环境

真机、Gazebo 和键盘各自的终端都先执行。克隆到其他位置时，将第一行改成对应的 `ros2_ws` 路径。

```bash
cd /home/yzz/sysu219_mpc_ppo/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

## 真机

在已加载本工作区的终端执行。

```bash
ros2 launch sysu219_guide_controller robot_hardware.launch.py policy:=local rviz:=true
```

入踏步时会检查 IMU 俯仰和横滚各在 ±5° 内。按 `2` 可回固定站姿。

## Gazebo

在已加载本工作区的终端执行，与真机启动命令二选一。

```bash
ros2 launch sysu219_guide_controller gazebo.launch.py policy:=local
```

## 键盘

另开一个终端，按上面的步骤加载同一工作区，再运行。

```bash
ros2 run keyboard_input keyboard_input
```

每次等姿态切换完成，再按下一次 `2`。从 PASSIVE 开始依次进入全趴、半趴和 FIXEDSTAND。站稳后按 `4` 进入踏步，`W/S` 前后，`A/D` 转向，松开运动键回零速踏步。按 `2` 退出踏步并返回站立。

## 当前模型

真机和 Gazebo 都从本工程的 `ros2_ws/install/sysu219_guide_controller/share/sysu219_guide_controller/model/` 加载 `pure_local_gait_ppo.onnx` 和同名 JSON。控制器启动日志会打印检查点 SHA。训练用仿真真值计算机身稳定奖励和零指令踏步 10 秒的漂移指标。检查点轮次和文件校验值见 [模型清单](ros2_ws/src/quadruped_ros2_control-humble/controllers/sysu219_guide_controller/model/pure_local_gait_ppo.json)。
