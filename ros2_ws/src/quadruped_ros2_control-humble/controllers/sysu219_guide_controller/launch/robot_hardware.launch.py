"""SYSU219 hardware with a checked local pure gait policy."""

import hashlib
import json
import os
import sys
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def local_install():
    # A colcon symlink install can expose either the installed or the source launch path.
    for path in Path(__file__).absolute().parents:
        if path.name == 'install':
            return path.resolve()
        if path.name == 'src':
            return (path.parent / 'install').resolve()
    raise RuntimeError('无法识别此启动文件所属的 ROS 工作区，请加载本工程的 install/local_setup.bash。')


PURE_POLICIES = {
    'pure': ('pure_gait_ppo', 'sysu219.pure_gait50.v2', 50),
    'smooth': ('pure_smooth_gait_ppo', 'sysu219.pure_gait74.v1', 74),
    'local': ('pure_local_gait_ppo', 'sysu219.pure_local_gait71.v1', 71),
    'hybrid': ('pure_hybrid_gait_ppo', 'sysu219.pure_hybrid_gait74.v1', 74),
}


def require_pure_policy(model_dir, kind='pure'):
    name, contract_id, width = PURE_POLICIES[kind]
    model = model_dir / f'{name}.onnx'
    manifest = model.with_suffix('.json')
    if not model.is_file() or model.stat().st_size == 0 or not manifest.is_file():
        raise RuntimeError(f'真机启动前需要本工作区已导出的纯 PPO 策略和清单：{model}、{manifest}。')
    try:
        info = json.loads(manifest.read_text(encoding='utf-8'))
        contract = info['contract']
        contract_bytes = json.dumps(contract, separators=(',', ':'), sort_keys=True).encode()
        identity = (contract['id'], contract['observation_width'])
        expected_identity = (contract_id, width)
        if kind == 'hybrid':
            accepted = {
                'ef3bfbb7be7b1350eca8b7ef3211e9070947c6635f78384e936877da3fce1825': (contract_id, width),
                '3320576f27f15e63d83b01b13b0f743e3f9c46b4e626a0e8adff40fc345c6d48': (contract_id, width),
                '9f517ad3d0bf8e566e2d365d17c1a3a455db3c4c644ad8711d7eedbb0403c055': ('sysu219.pure_hybrid_gait77.v1', 77),
                '4e59ccf367ca6e9de22f18127448059f482187b8da137e26ea765398bc2b61bc': ('sysu219.pure_hybrid_gait77.v1', 77),
            }
            expected_identity = accepted.get(info['contract_sha256'])
        valid = (identity == expected_identity
                 and contract['kp'] == [260.0] * 12 and contract['kd'] == [3.8] * 12
                 and hashlib.sha256(contract_bytes).hexdigest() == info['contract_sha256']
                 and hashlib.sha256(model.read_bytes()).hexdigest() == info['onnx_sha256'])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(f'无法读取纯 PPO 导出清单：{manifest}') from error
    if not valid:
        raise RuntimeError(f'纯 PPO 策略与导出清单不匹配：{model}')


def after_success(process, next_process):
    def handle_exit(event, _context):
        if event.returncode == 0:
            return [next_process]
        return [EmitEvent(event=Shutdown(reason=f'控制器加载失败，退出码 {event.returncode}。'))]
    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=handle_exit))


def launch_setup(context):
    install = local_install()
    description_name = LaunchConfiguration('pkg_description').perform(context)
    kind = LaunchConfiguration('policy').perform(context)
    for name in dict.fromkeys(('sysu219_guide_controller', 'sysu219_description', description_name, 'hardware_sysu219')):
        prefix = Path(get_package_prefix(name)).resolve()
        if prefix != install and install not in prefix.parents:
            raise RuntimeError(f'{name} 解析到了其他工作区：{prefix}。请加载 {install}/local_setup.bash。')
    controller_share = Path(get_package_share_directory('sysu219_guide_controller'))
    require_pure_policy(controller_share / 'model', kind)

    description = Path(get_package_share_directory(description_name))
    config = description / 'config'
    # robot.xacro defaults select hardware_sysu219 and its supplied hardware_config.yaml.
    robot_description = xacro.process_file(str(description / 'xacro/robot.xacro')).toxml()
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        parameters=[{'publish_frequency': 20.0, 'use_tf_static': True, 'ignore_timestamp': True,
                     'robot_description': robot_description}], output='screen',
    )
    policy_config = f'{PURE_POLICIES[kind][0]}.yaml'
    hardware_parameters = [str(config / name) for name in ('robot_control.yaml', 'rl_policy.yaml')]
    # Match the original 500 Hz hardware read/write loop. The spawner applies
    # the policy override to the guide only, which runs at 250 Hz.
    guide_parameters = [*hardware_parameters, str(config / policy_config)]
    manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=hardware_parameters,
        remappings=[('~/robot_description', '/robot_description')], output='screen',
        additional_env={'LD_LIBRARY_PATH': os.pathsep.join(filter(None, (
            '/opt/onnxruntime/lib', os.environ.get('LD_LIBRARY_PATH'))))},
    )

    def spawner(controller, files=()):
        arguments = [controller, '--controller-manager', '/controller_manager']
        for path in files:
            arguments.extend(['--param-file', path])
        return Node(package='controller_manager', executable='spawner', output='screen', arguments=arguments)

    joints = spawner('joint_state_broadcaster')
    imu = spawner('imu_sensor_broadcaster')
    guide = spawner('sysu219_guide_controller', guide_parameters)
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_ocs2', output='screen',
        arguments=['-d', str(config / 'visualize_urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    actions = [
        after_success(joints, imu), after_success(imu, guide),
        publisher, manager, joints, TimerAction(period=8.0, actions=[rviz]),
    ]
    if kind == 'hybrid':
        # Resolve the source next to a symlinked launch file, plus regular package installs.
        script = Path(__file__).resolve().parents[1] / 'scripts' / 'capture_hybrid_hold.py'
        if not script.is_file():
            script = Path(get_package_prefix('sysu219_guide_controller')) / 'lib/sysu219_guide_controller/capture_hybrid_hold.py'
        if script.is_file():
            actions.append(ExecuteProcess(
                cmd=[sys.executable, str(script)], name='sysu219_hybrid_hold_capture', output='screen',
            ))
        else:
            actions.append(LogInfo(msg='SYSU219 原地踏步采集器未安装，跳过采集。'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pkg_description', default_value='sysu219_description'),
        DeclareLaunchArgument('policy', default_value='pure', choices=list(PURE_POLICIES)),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
