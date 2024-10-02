import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.actions import  RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

 
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'ruka_gz'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py')]),
                    launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
            )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
             )

  # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'ruka_gz'],
        output='screen'
    )


 
    ruka_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ruka_gz_arm_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    delayed_diff_drive_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=spawn_entity,
             on_exit=[ruka_arm_controller_spawner],
         )
    )
    
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        delayed_diff_drive_spawner,
        ruka_arm_controller_spawner,
        joint_broad_spawner
    ])

