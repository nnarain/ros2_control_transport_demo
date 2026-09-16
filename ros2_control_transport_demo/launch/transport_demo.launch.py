# Launch file for the ROS2 control transport demo using the socketcan transport.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ros2_control_transport_demo')
    controllers_yaml = pkg + '/config/transport_demo_controllers.yaml'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(
            pkg + '/description/transport_demo.urdf').read()}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': open(pkg + '/description/transport_demo.urdf').read()},
            controllers_yaml,
            # controller_manager only loads the yaml for itself; each spawned controller
            # node needs its own params_file pointer to receive its section of the yaml.
            {
                'left_wheel_controller.params_file': controllers_yaml,
                'right_wheel_controller.params_file': controllers_yaml,
                'joint_state_broadcaster.params_file': controllers_yaml,
            },
        ],
        output='screen',
    )

    spawn_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessExit(target_action=controller_manager, on_exit=[spawn_jsb])),
    ])
