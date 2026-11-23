# SYSU219 纯 PPO 真机与 Gazebo

本工程使用 ROS 2 Humble。当前 v4 策略使用 `policy:=hybrid`，在 v3 的观测末尾加入原 InEKF 提供的三轴相对位置。PPO 和 InEKF 共用 250 Hz 主控步态时钟，PPO 按 50 Hz 推理。回看旧版本时，v3 使用 `policy:=local`，第一次真机测试的 v1 使用 `policy:=smooth`。

## 编译

在新终端进入本工程的 `ros2_ws`。现有环境使用 ONNX Runtime 1.18.1（`/opt/onnxruntime`）、ROS 2 `serial` 库（`/usr/local`）、Eigen3 和 yaml-cpp。Gazebo 还需要 Gazebo Classic、`gazebo_ros` 和 `gazebo_ros2_control`。已有这些依赖的机器可以直接编译。

```bash
cd /home/yzz/sysu219_mpc_ppo/ros2_ws
source /opt/ros/humble/setup.bash
```

第一次编译时，从下面两条完整构建命令中选一条执行。克隆到其他位置时，将上面的 `cd` 改成实际路径。

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

### 已有 v3 工作区升级到 v4

将本工程的 v4 源码和模型文件同步到真机电脑。已有其他依赖包的编译产物时，在上面进入的 `ros2_ws` 终端重编控制器和描述包。

```bash
MAKEFLAGS="-j1 -l1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
colcon build --executor sequential \
  --packages-select sysu219_guide_controller sysu219_description \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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
ros2 launch sysu219_guide_controller robot_hardware.launch.py policy:=hybrid rviz:=true
```

入踏步时会检查 IMU 俯仰和横滚各在 ±5° 内。按 `2` 可回固定站姿。

## Gazebo

在已加载本工作区的终端执行，与真机启动命令二选一。

```bash
ros2 launch sysu219_guide_controller gazebo.launch.py policy:=hybrid
```

## 键盘

另开一个终端，按上面的步骤加载同一工作区，再运行。

```bash
ros2 run keyboard_input keyboard_input
```

每次等姿态切换完成，再按下一次 `2`。从 PASSIVE 开始依次进入全趴、半趴和 FIXEDSTAND。站稳后按 `4` 进入踏步，`W/S` 前后，`A/D` 转向，松开运动键回零速踏步。按 `2` 退出踏步并返回站立。

## 当前模型

真机和 Gazebo 在 `policy:=hybrid` 下，从本工程的 `ros2_ws/install/sysu219_guide_controller/share/sysu219_guide_controller/model/` 加载 `pure_hybrid_gait_ppo.onnx` 和同名 JSON。控制器启动日志会打印检查点 SHA，当前检查点轮次和文件校验值见 [v4 模型清单](ros2_ws/src/quadruped_ros2_control-humble/controllers/sysu219_guide_controller/model/pure_hybrid_gait_ppo.json)。

带 InEKF 速度观测的 v4 策略为 77 维，在原有 74 维输入末尾追加三轴速度。从 74 维版本更新时，按本文已有的构建命令重新构建 ROS 2 控制器，再重新加载工作区。两个版本均使用 `policy:=hybrid`。

## 维护

本工程的构建命令、环境加载或启动参数有变化时，同步更新本 README。
