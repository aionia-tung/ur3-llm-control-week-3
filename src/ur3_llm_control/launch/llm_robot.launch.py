
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("ur3_llm_control")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "runtime_config_package": "ur3_draw_letter",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur3_draw_letter",
            "description_file": "ur.urdf.xacro",
            "launch_rviz": "false",
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "world_file": PathJoinSubstitution([share, "worlds", "task_world.sdf"]),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "safety_limits": "true",
            "description_package": "ur3_draw_letter",
            "description_file": "ur.urdf.xacro",
            "moveit_config_package": "ur3_draw_letter",
            "moveit_config_file": "ur.srdf.xacro",
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    perception_node = Node(
        package="ur3_llm_control",
        executable="camera_perception_node",
        name="camera_perception",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    task_node = Node(
        package="ur3_llm_control", executable="llm_task_node",
        name="llm_task_planner", output="screen",
        parameters=[{
            "command": LaunchConfiguration("command"),
            "api_key": LaunchConfiguration("api_key"),
            "endpoint": LaunchConfiguration("endpoint"),
            "model": LaunchConfiguration("model"),
            "execute": LaunchConfiguration("execute"),
            "student_id": ParameterValue(LaunchConfiguration("student_id"), value_type=str),
        }],
    )
    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "gripper_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60", "--service-call-timeout", "60",
            "--switch-timeout", "60",
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("command", default_value=""),
        DeclareLaunchArgument("api_key", default_value=""),
        DeclareLaunchArgument("endpoint", default_value="https://9router.com/v1/chat/completions"),
        DeclareLaunchArgument("model", default_value="gpt-4o-mini"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("student_id", default_value="23020770"),
        simulation,
        camera_bridge,
        perception_node,
        moveit,
        TimerAction(period=18.0, actions=[gripper_spawner]),
        TimerAction(period=40.0, actions=[task_node]),
    ])