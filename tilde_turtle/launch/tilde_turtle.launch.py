from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
import launch.conditions as conditions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    #pkg_share = get_package_share_directory("turtlebot_flatland")
    pkg_share = FindPackageShare("tilde_turtle")

    world_path = LaunchConfiguration("world_path")
    update_rate = LaunchConfiguration("update_rate")
    step_size = LaunchConfiguration("step_size")
    show_viz = LaunchConfiguration("show_viz")
    viz_pub_rate = LaunchConfiguration("viz_pub_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld = LaunchDescription(
        [
            # ******************** flatland********************
            # You can override these default values:
            #   roslaunch flatland_Server server.launch world_path:="/some/world.yaml" initial_rate:="30.0"
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "tilde_world/world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="100.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_sim_time", default_value="true"),

            SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity} ${time} ${logger}]: ${message}"),

            # launch flatland server
            Node(
                name="flatland_server",
                package="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    # Use the arguments passed into the launchfile for this node
                    {"world_path": world_path},
                    {"update_rate": update_rate},
                    {"step_size": step_size},
                    {"show_viz": show_viz},
                    {"viz_pub_rate": viz_pub_rate},
                    {"use_sim_time": use_sim_time},
                ],
            ),

            # Wall-following node for robot 1
            Node(
                package='tilde_turtle',
                executable='wall_follower_slow',
                name='wall_follower_1',
                # namespace='robot_1',
                output='screen',
                parameters=[{'robot_name': 'robot_1'}]
            ),
            
            Node(
                package='tilde_turtle',
                executable='wall_follower_slow',
                name='wall_follower_2',
                # namespace='robot_2',
                output='screen',
                parameters=[{'robot_name': 'robot_2'}]
            ),
            
            # Node(
            #     package='tilde_turtle',
            #     executable='wall_follower_fast',
            #     name='wall_follower_2',
            #     # namespace='robot_2',
            #     output='screen',
            #     parameters=[{'robot_name': 'robot_2'}]
            # ),
            
            # # Wall-following node for robot 2
            # Node(
            #     package='tilde_turtle',
            #     executable='wall_follower_fast',
            #     name='wall_follower_2',
            #     output='screen',
            #     # parameters=[{'robot_name': 'robot_2'}]
            # ),

            # ****** Maps *****
            Node(
                name="tf",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),

            # **************** Visualisation ****************
            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz/robot_navigation.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=conditions.IfCondition(show_viz),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
