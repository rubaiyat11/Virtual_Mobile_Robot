from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Target X coordinate")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Target Y coordinate")
    z_arg = DeclareLaunchArgument("z", default_value="0.0", description="Target Z coordinate")
    
    target_x_val = LaunchConfiguration("x")
    target_y_val = LaunchConfiguration("y")
    target_z_val = LaunchConfiguration("z")

    mission_manager_node = Node(
        package="mission_system",
        executable="mission_manager_server",
        name="mission_manager_node"
    )

    mission_client_node = Node(
        package="mission_system",
        executable="mission_client_node",
        name="mission_client_node",
        parameters=[{
            "target_x": target_x_val,
            "target_y": target_y_val,
            "target_z": target_z_val
        }]
    )

    planning_node = Node(
        package="planning",
        executable="planner_node",
        name="planner_node"
    )

    plant_node = Node(
        package="robot_core",
        executable="plant_node",
        name="plant_node"
    )

    sensor_node = Node(
        package="robot_core",
        executable="sensor_node",
        name="sensor_node"
    )

    mpc_controller_node = Node(
        package="controllers",
        executable="mpc_controller_node",
        name="mpc_controller_node"
    )

    lqr_controller_node = Node(
        package="controllers",
        executable="lqr_controller_node",
        name="lqr_controller_node"
    )

    pid_controller_node = Node(
        package="controllers",
        executable="pid_controller_node",
        name="pid_controller_node"
    )


    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        mission_manager_node,
        planning_node,
        plant_node,
        sensor_node,
        mission_client_node,
        mpc_controller_node,
        lqr_controller_node,
        pid_controller_node
    ])