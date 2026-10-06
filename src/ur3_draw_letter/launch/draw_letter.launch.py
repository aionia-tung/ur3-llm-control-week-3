from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    prefix = LaunchConfiguration("prefix")
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur3_draw_letter"), "rviz", "ur3_draw_letter.rviz"]
    )
    rviz_prefix = (
        "env -i "
        "PATH=/opt/ros/humble/bin:/usr/bin:/bin "
        "HOME=/home/linh "
        "DISPLAY=:0 "
        "WAYLAND_DISPLAY=wayland-0 "
        "XDG_RUNTIME_DIR=/run/user/1000 "
        "QT_QPA_PLATFORM=xcb "
        "LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/usr/lib/x86_64-linux-gnu "
        "OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0 "
        "ROS_VERSION=2 "
        "ROS_DISTRO=humble "
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
        "AMENT_PREFIX_PATH=/home/linh/workspaces/ur_gz/install:/opt/ros/humble "
        "CMAKE_PREFIX_PATH=/home/linh/workspaces/ur_gz/install:/opt/ros/humble"
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ",
        "name:=ur ",
        "ur_type:=", ur_type,
        " safety_limits:=", safety_limits,
        " prefix:=", prefix,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " name:=ur prefix:=", prefix,
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    simulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "runtime_config_package": "ur3_draw_letter",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur3_draw_letter",
            "description_file": "ur.urdf.xacro",
            "prefix": prefix,
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    simulation = GroupAction(scoped=True, actions=[simulation_include])

    moveit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    moveit = GroupAction(scoped=True, actions=[moveit_include])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        condition=IfCondition(launch_rviz),
        prefix=rviz_prefix,
        arguments=["-d", rviz_config_file],
        parameters=[
            kinematics_yaml,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    draw_letter = Node(
        package="ur3_draw_letter",
        executable="draw_letter_node",
        name="draw_letter",
        output="screen",
        parameters=[
            kinematics_yaml,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                description="UR model to simulate (ur3 or ur3e are suitable here).",
            ),
            DeclareLaunchArgument(
                "safety_limits",
                default_value="true",
                description="Enable the UR safety limits.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo with its GUI.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with the MoveIt configuration.",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value='""',
                description="Optional joint-name prefix for the robot.",
            ),
            simulation,
            moveit,
            TimerAction(period=5.0, actions=[rviz]),
            # Give Gazebo and MoveIt time to publish robot_description and
            # joint states before the first planning request.
            TimerAction(period=20.0, actions=[draw_letter]),
        ]
    )
