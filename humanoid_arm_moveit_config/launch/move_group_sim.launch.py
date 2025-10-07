from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare use_sim_time argument
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("humanoid_arm_5dof", package_name="humanoid_arm_moveit_config")
        .robot_description(file_path="config/humanoid_arm_5dof.urdf.xacro")
        .robot_description_semantic(file_path="config/humanoid_arm_5dof.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "stomp"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            # Reduced from 3.0 - with position control and higher speed limits, faster execution is safe
            {"trajectory_execution.allowed_execution_duration_scaling": 1.0},
            {"trajectory_execution.allowed_goal_duration_margin": 2.0},
            # Start tolerance for position-controlled simulation
            {"trajectory_execution.allowed_start_tolerance": 0.1},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node])