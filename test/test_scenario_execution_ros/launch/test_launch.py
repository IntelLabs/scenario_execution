from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    test_param = LaunchConfiguration('test_param')
    test_path = LaunchConfiguration('test_path')

    return LaunchDescription([
        DeclareLaunchArgument('test_param', description='test parameter'),
        DeclareLaunchArgument('test_path', description='Test path parameter'),

        Node(
            # condition=IfCondition(scenario_status),
            package='test_scenario_execution_ros',
            executable='test_scenario_execution_ros',
            name='test_scenario_execution_ros',
            parameters=[{
                'test_param': test_param,
                'test_path': test_path,
            }],
            output='screen'
        ),

        # ExecuteProcess(
        #     cmd=['sleep', '5'],
        #     output='screen'),
    ])
