from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('robot_model', default_value='wx200',
                          choices=['wx200'],
                          description='robot_model type of the Interbotix Arm'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model'),
                          description='Robot name'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='launches RViz if set to `true`.'),
    DeclareLaunchArgument('rviz_config',
                          default_value=PathJoinSubstitution([get_package_share_directory('arm_sim_scenario'),
                            'rviz',
                            'xsarm_description.rviz',
                            ]),
                            description='file path to the config file RViz should load.',)
]


def generate_launch_description():
    pkg_arm_sim_scenario = get_package_share_directory('arm_sim_scenario')
    xacro_file = PathJoinSubstitution([pkg_arm_sim_scenario,
                                       'urdf',
                                       'wx200.urdf.xacro'])
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')
    robot_model = LaunchConfiguration('robot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'base_link_frame:=','base_link', ' ',
                'use_gripper:=','true', ' ',
                'show_ar_tag:=','false', ' ',
                'show_gripper_bar:=','true', ' ',
                'show_gripper_fingers:=','true', ' ',
                'use_world_frame:=','true', ' ',
                'robot_model:=',robot_model, ' ',
                'robot_name:=',robot_name, ' ',
                'hardware_type:=','gz_classic']), value_type=str)},
        ],
        namespace=robot_name
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=robot_name
    )

    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name,
        arguments=[
            '-d', rviz_config,
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output={'both': 'log'},
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz2)
    return ld
