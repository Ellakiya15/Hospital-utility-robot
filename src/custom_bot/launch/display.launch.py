#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_path = os.path.join(get_package_share_directory('custom_bot'), 'urdf', 'robo3.xacro') #robot-sim.xacro
    doc = get_xacro_to_doc(xacro_path,{})

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, {'robot_description': doc.toxml()}],
        remappings=[
            ('world/default/model/diffbot/joint_states', '/joint_states')
        ]
    )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}]
    # )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'laser_frame']
    )   

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('custom_bot'), 'rviz', 'rsp.rviz')]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=doc.toxml(),
            description='Robot description in URDF/XACRO format'
        ),
        robot_state_publisher,
        rviz,
        joint_state_publisher,
        static_tf
    ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.substitutions import Command
# from launch_ros.actions import Node

# def generate_launch_description():
#     urdf_file = 'urdf/bot.urdf'
#     package_desc = 'custom_bot'
#     print("URDF LOADING..")
#     robot_desc_path = os.path.join(get_package_share_directory(package_desc), urdf_file)

#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher_node',
#         emulate_tty = True,
#         parameters=[{ 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
#         output = "screen"
#     )
#     rviz_config_dir = os.path.join(get_package_share_directory(package_desc), 'rviz')
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         name='rviz_node',
#         parameters=[{'use_sim_time': True}],
#         arguments=['-d', rviz_config_dir])
    
#     return LaunchDescription(
#         [
#             robot_state_publisher_node,
#             rviz_node
#         ]
#     )