import os

import ament_index_python.packages
import launch
import launch_ros.actions
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('lidro_isl100')
    use_rviz = LaunchConfiguration('use_rviz')
    use_urdf = LaunchConfiguration('use_urdf')
    use_config = LaunchConfiguration('use_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rvizconfig = LaunchConfiguration('rvizconfig')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config_ifl100.rviz')

    config_directory = os.path.join(
        pkg_share,
        'config')
    params = os.path.join(config_directory, 'lidro.yaml')
    lidro_driver_node = launch_ros.actions.Node(package='lidro_isl100',
                                                   executable='lidro_isl100_node',
                                                   output='both',
                                                   parameters=[params])
    xacro_file = os.path.join(pkg_share,'urdf','robot.urdf.xacro')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
        }],
        condition=launch.conditions.IfCondition(use_urdf)
    )

    rqt_reconfigure = launch_ros.actions.Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        condition=launch.conditions.IfCondition(use_config)
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_rviz', default_value='False',
            description='Flag to enable use_rviz'),
        launch.actions.DeclareLaunchArgument(name='use_urdf', default_value='False',
            description='Flag to enable use_urdf'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='use_config', default_value='False',
            description='Flag to enable use_config'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        lidro_driver_node,
        rqt_reconfigure,
        rviz_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=lidro_driver_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
        )),
    ])
