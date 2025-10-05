#!/usr/bin/python3
import os
from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    bot_pkg_path = get_package_share_directory("custom_bot")
    world_file = LaunchConfiguration("world_file", default = join(bot_pkg_path, "worlds", "wardroom.sdf"))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    bot_rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bot_pkg_path, "launch", "display.launch.py")),
        launch_arguments={
            # Pass any arguments if your spawn.launch.py requires
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "diffbot",
            "-allow_renaming", "true",
            "-z", "0.35",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0"
        ]
    )

    start_gazebo_ros_image_bridge_cmd = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=['/camera', '/camera/image_raw'],
    output='screen',
)

    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    #         "/left_wheel_rpm@std_msgs/msg/Float64@gz.msgs.Double",
    #         "/right_wheel_rpm@std_msgs/msg/Float64@gz.msgs.Double",
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #         "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    #         "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    #         # "/joint_states@sensor_msgs/msg/JointState@gz.msgs.JointState",
    #         # "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    #         "/world/default/model/diffbot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"
    #     ],
    #     remappings=[
    #         ('/world/default/model/diffbot/joint_states', 'joint_states'),
    #     ]
    # )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/left_wheel_rpm@std_msgs/msg/Float64@gz.msgs.Double",
            "/right_wheel_rpm@std_msgs/msg/Float64@gz.msgs.Double",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        # parameters=[{
        # # Force QoS for /scan to match Gazebo's default
        # "qos_overrides./scan.publisher.durability": "volatile",
        # "qos_overrides./scan.subscription.durability": "volatile",
        # "qos_overrides./scan.publisher.reliability": "reliable",
        # "qos_overrides./scan.subscription.reliability": "reliable",
        # "qos_overrides./scan.publisher.history": "keep_last",
        # "qos_overrides./scan.subscription.history": "keep_last",
        # }]
        remappings=[
            ('/world/default/model/diffbot/joint_states', 'joint_states'),
            # ('/kinect_camera', 'bcr_bot/kinect_camera')
        ]
    )

    # Alternate node to launch ros-gz-bridge
    # bridge_params = os.path.join(get_package_share_directory("diffbot"),'config','gz_bridge.yaml')
    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         '--ros-args',
    #         '-p',
    #         f'config_file:={bridge_params}',
    #     ]
    # )
    # os.environ['GZ_SIM_RESOURCE_PATH'] = (
    #     '/home/ellakiya/.gz/fuel/fuel.gazebosim.org/googleresearch/models:' +
    #     '/home/ellakiya/.gz/fuel/fuel.gazebosim.org/openrobotics/models:' +
    #     join(bot_pkg_path, "worlds:") +
    #     join(bot_pkg_path, "models:") +
    #     os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    # )

    print("GZ_SIM_RESOURCE_PATH:", os.environ['GZ_SIM_RESOURCE_PATH'])
    return LaunchDescription([

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(bot_pkg_path, "worlds")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(bot_pkg_path, "models")),

        AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=join(bot_pkg_path)),

        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        gz_sim,bot_rsp_node,
        gz_spawn_entity, gz_ros2_bridge,
        start_gazebo_ros_image_bridge_cmd
        
    ])

