# SYSU219 纯 PPO 真机与 Gazebo

本工程使用 ROS 2 Humble。v4 使用 `policy:=hybrid`，支持 InEKF 三轴相对位置的 74 维版、增加三轴线速度的 77 维版，以及直接读取低通 XY 位置和速度的 75 维版。PPO 和 InEKF 共用 250 Hz 主控步态时钟，PPO 按 50 Hz 推理。回看旧版本时，v3 使用 `policy:=local`，第一次真机测试的 v1 使用 `policy:=smooth`。

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

### 已有工作区更新控制器和模型

将本工程的新源码和模型文件同步到真机电脑。已有其他依赖包的编译产物时，在上面进入的 `ros2_ws` 终端增量构建。

```bash
MAKEFLAGS="-j1 -l1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
colcon build --executor sequential \
  --packages-select sysu219_guide_controller sysu219_description keyboard_input \
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

每次等姿态切换完成，再按下一次 `2`。从 PASSIVE 开始依次进入全趴、半趴和 FIXEDSTAND。用 `policy:=hybrid` 启动时，在 FIXEDSTAND 站稳后按下表的策略键。踏步中按 `2` 回到固定站姿，等站稳再选下一策略。`W/S` 前后，`A/D` 转向，松开运动键回零速踏步。`1` 进被动阻尼，站稳后按 `8` 先半趴再全趴。

## v4 对照策略

| 键 | 版本 | 训练检查点 |
| --- | --- | --- |
| `3` | 基于原 `9` 的 75 维版，读取强低通后的 XY 相对位置和速度 | `2026-11-24_17-04-48_key9_filtered_xy/model_75.pt` |
| `4` | 75 维低通 XY 版，训练时加入 0–18 N 随机后拉力 | `2026-11-24_17-36-14_key9_filtered_xy_rear_force/model_725.pt` |
| `5` | 75 维 XY 里程计闭环，零速时修正前后和横向漂移 | `2026-11-24_18-57-54_key4_xy_odometry_hold/model_175.pt` |
| `6` | 75 维 XY 里程计闭环，0–30 N 随机后拉力训练，轻 yaw 角速度惩罚 | `2026-11-24_20-31-40_key5_xy_hold_30n_soft_yaw/model_375.pt` |
| `7` | `zero_pitch`，俯仰目标 0° | `2026-11-24_12-44-42_zero_pitch/model_999.pt` |
| `9` | 真机历史最好看步态的基准，零速会后退 | `2026-11-23_23-22-04/model_750.pt` |

按 `3`、`4`、`5` 或 `6` 进入踏步后，策略观测会把 InEKF 的 XY 相对位置和速度转到入场航向坐标系，再以 50 Hz 更新两级低通。每级时间常数为 1 秒，每次进入踏步时用首帧初始化。滤波后的四个量直接拼入 PPO 观测。

`5` 和 `6` 另外用三点中值、一个完整步态周期的滑动平均和 0.25 秒低通提取平面慢趋势。进入零指令时锁定当前位置，修正后的 `vx`、`vy` 同时送入 PPO 和足端参考。给出运动指令后，纠偏平滑退回零。

真机和 Gazebo 沿用原来的键盘通道。`3/4/5/6/7/9` 的旧功能映射已取消，启用 `policy:=hybrid` 后专用于选策略，踏步中按这些键保持当前策略和运动指令。每个策略使用安装目录下的 `model/pure_hybrid_gait_key<N>.onnx`，同名 JSON 可核对轮次和校验值。控制器启动时预加载各策略，空位或模型校验失败时对应按键保持固定站姿。更新模型后需重启控制器以加载新文件。

真机的原地采集在收到原有踏步状态后延迟 0.5 秒开始。采集写入 ROS 2 工作区下的 `log/sysu219_hold/`。

## 维护

本工程的构建命令、环境加载或启动参数有变化时，同步更新本 README。
