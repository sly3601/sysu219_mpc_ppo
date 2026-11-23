import hashlib
import json
import os
import sysconfig
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    LogInfo,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def cpu_monitor_actions():
    prefix = Path(get_package_prefix('sysu219_guide_controller'))
    executable = prefix / 'lib' / 'sysu219_guide_controller' / 'sysu219_cpu_monitor'
    workspace = next((str(path.parent) for path in (prefix, *prefix.parents)
                      if path.name == 'install'), '')
    if not executable.is_file() or not os.environ.get('DISPLAY') or not workspace:
        return [LogInfo(msg='处理器监视：无图形桌面或监视程序不可用，已跳过窗口。')]
    return [ExecuteProcess(
        cmd=[str(executable), '--workspace', workspace, '--launch-pid', str(os.getpid())],
        name='sysu219_cpu_monitor',
        output='log',
        sigterm_timeout='2',
        sigkill_timeout='2',
    )]


def prepend_env_value(name, value):
    current_value = os.environ.get(name, "")
    if not current_value:
        return value
    if value in current_value.split(os.pathsep):
        return current_value
    return value + os.pathsep + current_value


def on_success(*actions):
    def on_exit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f'Gazebo 启动步骤失败（退出码 {event.returncode}），停止后续初始化。')]
        return list(actions)
    return on_exit


def optional_sensor_publishers(robot, directories):
    """Drop only unavailable ROS publishers, leaving the Gazebo sensor and ros2_control interfaces in place."""
    optional = {'libgazebo_ros_p3d.so', 'libgazebo_ros_imu_sensor.so'}
    missing = []
    for parent in robot.iter():
        for plugin in parent.findall('plugin'):
            library = plugin.get('filename')
            if library in optional and not any((folder / library).is_file() for folder in directories):
                parent.remove(plugin)
                missing.append(library)
    return missing


def require_hybrid_policy(model):
    manifest = model.with_suffix('.json')
    if not manifest.is_file():
        raise RuntimeError(f'结合版 PPO 缺少导出清单：{manifest}')
    fingerprints = {
        'ef3bfbb7be7b1350eca8b7ef3211e9070947c6635f78384e936877da3fce1825',  # 975 ms, 2/3 support
        '3320576f27f15e63d83b01b13b0f743e3f9c46b4e626a0e8adff40fc345c6d48',  # 650 ms, 1/2 support
        '9f517ad3d0bf8e566e2d365d17c1a3a455db3c4c644ad8711d7eedbb0403c055',  # 975 ms, with velocity
        '4e59ccf367ca6e9de22f18127448059f482187b8da137e26ea765398bc2b61bc',  # 650 ms, with velocity
    }
    try:
        info = json.loads(manifest.read_text(encoding='utf-8'))
        contract = json.dumps(info['contract'], separators=(',', ':'), sort_keys=True).encode()
        valid = (info['contract_sha256'] == hashlib.sha256(contract).hexdigest() in fingerprints
                 and hashlib.sha256(model.read_bytes()).hexdigest() == info['onnx_sha256'])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(f'无法读取结合版 PPO 导出清单：{manifest}') from error
    if not valid:
        raise RuntimeError(f'结合版 PPO 策略与导出清单不匹配：{model}')


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    init_height = context.launch_configurations['height']
    com_history_seconds = float(context.launch_configurations['com_history_seconds'])

    pkg_path = os.path.join(get_package_share_directory(package_description))
    leg_pd_prefix = get_package_prefix('leg_pd_controller')
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'GAZEBO': 'true',
            'CLASSIC': 'true',
        },
    ).toxml()

    root = ET.fromstring(robot_description)
    # Use the same exported Gazebo plugin paths as the Humble gzserver launch.
    from scripts import GazeboRosPaths
    _, exported, _ = GazeboRosPaths.get_paths()
    paths = [exported, os.environ.get('GAZEBO_PLUGIN_PATH', ''), os.environ.get('LD_LIBRARY_PATH', '')]
    plugin_dirs = [Path(part) for value in paths for part in value.split(os.pathsep) if part]
    plugin_dirs.extend([Path(get_package_prefix('gazebo_ros')) / 'lib',
                        Path('/usr/lib') / sysconfig.get_config_var('MULTIARCH') / 'gazebo-11/plugins'])
    missing = optional_sensor_publishers(root, plugin_dirs)

    policy = context.launch_configurations['policy']
    guide_policy_args = []
    if policy in ('pure', 'smooth', 'local', 'hybrid', 'tracking', 'stand'):
        model, overlay = {
            'stand': (None, 'gazebo_stand_only.yaml'),
            'pure': ('pure_gait_ppo.onnx', 'pure_gait_ppo.yaml'),
            'smooth': ('pure_smooth_gait_ppo.onnx', 'pure_smooth_gait_ppo.yaml'),
            'local': ('pure_local_gait_ppo.onnx', 'pure_local_gait_ppo.yaml'),
            'hybrid': ('pure_hybrid_gait_ppo.onnx', 'pure_hybrid_gait_ppo.yaml'),
            'tracking': ('mpc_foot_tracker.onnx', 'mpc_tracking.yaml'),
        }[policy]
        policy_share = Path(get_package_share_directory('sysu219_guide_controller'))
        if model:
            model_path = policy_share / 'model' / model
            if not model_path.is_file():
                raise RuntimeError(f'策略 {policy} 尚无已导出的模型 {model}。请先完成对应任务的训练和导出。')
            if policy == 'local' and not model_path.with_suffix('.json').is_file():
                raise RuntimeError(f'策略 {policy} 缺少模型清单：{model_path.with_suffix(".json")}')
            if policy == 'hybrid':
                require_hybrid_policy(model_path)
        control = root.find("./gazebo/plugin[@name='gazebo_ros2_control']")
        if control is None:
            raise RuntimeError('SYSU219 description is missing its Gazebo ROS control plugin')
        overlay_path = Path(pkg_path) / 'config' / overlay
        if not overlay_path.is_file():
            raise RuntimeError(f'策略 {policy} 的参数文件缺失：{overlay_path}')
        ET.SubElement(control, 'parameters').text = str(overlay_path)
        if policy in ('smooth', 'local', 'hybrid'):
            guide_policy_args = ['--param-file', str(overlay_path)]

    # The encoder system and the optional physics recorder are separate classes in one library.
    physics_debug_path = os.path.join(get_package_prefix('sysu219_guide_controller'),
                                     'lib', 'sysu219_guide_controller', 'libsysu219_physics_debug.so')
    if root.findtext('./ros2_control/hardware/plugin') == 'sysu219_guide_controller/GazeboEncoderSystem' and not os.path.isfile(physics_debug_path):
        raise RuntimeError(f'机器人描述指定的 GazeboEncoderSystem 插件缺失：{physics_debug_path}。'
                           '请在本工作区启用 BUILD_GAZEBO_PHYSICS_DEBUG 并安装 gazebo_ros2_control 开发包。')
    diagnostics_status = '[PHYSICS_DEBUG] disabled'
    if policy in ('flat', 'tracking') and os.path.isfile(physics_debug_path):
        gazebo = ET.SubElement(root, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin',
                               filename=physics_debug_path, name='sysu219_physics_debug')
        ros = ET.SubElement(plugin, 'ros')
        ET.SubElement(ros, 'namespace').text = '/gazebo'
        diagnostics_status = '[PHYSICS_DEBUG] plugin available; enabled by Csv_DebugMode'
    robot_description = ET.tostring(root, encoding='unicode')

    rviz_config_file = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "visualize_urdf.rviz"
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    com_visualizer = Node(
        package='sysu219_guide_controller',
        executable='gazebo_com_visualizer.py',
        output='screen',
        parameters=[{'use_sim_time': True, 'history_seconds': com_history_seconds}],
    )

    gz_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sysu219',
            '-reference_frame', 'world',
            '-x', '0.0',
            '-y', '0.0',
            '-z', init_height,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_sim_time': True,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True,
            }
        ],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "leg_pd_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    sysu219_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "sysu219_guide_controller",
            "--controller-manager",
            "/controller_manager",
            *guide_policy_args,
        ],
    )

    # 与界面 Reset Model Poses 相同的模型复位；控制器启动成功后只调用一次。
    # 不重置仿真时钟，不在控制循环中复位，不添加常驻进程。
    reset_model_poses = ExecuteProcess(
        cmd=['timeout', '--kill-after=2s', '15s',
             'ros2', 'service', 'call', '/reset_world', 'std_srvs/srv/Empty', '{}'],
        name='reset_model_poses',
        output='screen',
    )

    def report_reset_result(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='Gazebo 启动复位完成：已执行一次 Reset Model Poses。')]
        return [LogInfo(msg=f'Gazebo 启动复位失败（退出码 {event.returncode}），请查看复位服务输出。')]

    return [
        *([LogInfo(msg='跳过未安装的附加 Gazebo 话题发布器：' + ', '.join(missing)
                       + '。原生 IMU 传感器和 ROS2 Control 接口仍启用。')] if missing else []),
        LogInfo(msg=diagnostics_status),
        LogInfo(msg=f'Gazebo 生成位置：world，x=0，y=0，z={init_height}，姿态角均为 0。'),
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            prepend_env_value('AMENT_PREFIX_PATH', leg_pd_prefix),
        ),

        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            prepend_env_value('LD_LIBRARY_PATH', os.path.join(leg_pd_prefix, 'lib')),
        ),

        rviz,
        *([com_visualizer] if policy in ('flat', 'tracking') and 'libgazebo_ros_p3d.so' not in missing else []),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('gazebo_ros'),
                            'launch',
                            'gazebo.launch.py',
                        ]
                    )
                ]
            ),
            launch_arguments={
                'verbose': 'true',
                'world': str(Path(__file__).resolve().with_name('slippery_ground.world')),
            }.items(),
        ),

        robot_state_publisher,

        gz_spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=on_success(joint_state_publisher),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=on_success(imu_sensor_broadcaster),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=on_success(leg_pd_controller),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=on_success(sysu219_guide_controller),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sysu219_guide_controller,
                on_exit=on_success(
                    LogInfo(msg='Gazebo 控制器已启动，执行一次模型位姿复位。'),
                    reset_model_poses,
                ),
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_model_poses,
                on_exit=report_reset_result,
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='sysu219_description',
        description='package for robot description',
    )

    height = DeclareLaunchArgument(
        'height',
        # 零角几何的脚底在 base 下方 0.587 m；保留约 6 cm 离地间隙。
        default_value='0.65',
        description='Spawn height in simulation; sysu219 zero-angle geometry needs more than 0.587 m',
    )

    return LaunchDescription(
        [
            pkg_description,
            DeclareLaunchArgument(
                'policy', default_value='pure', choices=['pure', 'smooth', 'local', 'hybrid', 'stand', 'flat', 'tracking'],
                description='SYSU219 gait PPO, hybrid PPO with InEKF, standing check or MPC-backed PPO'),
            height,
            *cpu_monitor_actions(),
            DeclareLaunchArgument(
                'com_history_seconds', default_value='10.0',
                description='Duration of COM comparison traces in simulation seconds'),
            OpaqueFunction(function=launch_setup),
        ]
    )
