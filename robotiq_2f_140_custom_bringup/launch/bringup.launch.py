from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_pkg = FindPackageShare("robotiq_2f_140_custom_description")
    bringup_pkg = FindPackageShare("robotiq_2f_140_custom_bringup")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "robotiq_2f_140_custom.ros2_control.xacro"])
    controllers_yaml = PathJoinSubstitution([bringup_pkg, "config", "ros2_controllers.yaml"])
    rviz_config = PathJoinSubstitution([desc_pkg, "rviz", "default.rviz"])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " ", "stroke:=0.07",
            " ", "mesh_scale:=0.001",
            " ", "use_collision:=true",
        ]),
        value_type=str
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([control_node, rsp, jsb, gripper_ctrl, rviz])

