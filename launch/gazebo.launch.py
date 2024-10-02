# # Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# # This software may be modified and distributed under the terms of the
# # GNU Lesser General Public License v2.1 or any later version.

# from ament_index_python.packages import get_package_share_directory
# import launch
# from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import launch_ros
# import os
# from launch_ros.actions import Node
# import xacro
# from launch.actions import DeclareLaunchArgument

# def generate_launch_description():

# # /////////////////////////////////////
#     rviz_config_arg = DeclareLaunchArgument(
#         "rviz_config",
#         default_value="moveit.rviz",
#         description="RViz configuration file",
#     )

#     db_arg = DeclareLaunchArgument(
#         "db", default_value="False", description="Database flag"
#     )

#     ros2_control_hardware_type = DeclareLaunchArgument(
#         "ros2_control_hardware_type",
#        default_value="joint_trajectory_controller",
#        description="Specify the hardware type for ros2_control.",
#     )

# # ////////////////////////////////////////



#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     pkg_ruka_gz = get_package_share_directory('ruka')
    
#     pkg_share = launch_ros.substitutions.FindPackageShare(package='ruka').find('ruka')

#     default_model_path = os.path.join(pkg_share, 'config')

#     default_rviz_config_path = os.path.join(default_model_path, 'moveit.rviz')

#     default_urdf_path = os.path.join(default_model_path,"ruka_gz.urdf")

#     robot_urdf_description_file = os.path.join(pkg_ruka_gz, 'config', 'ruka_gz.urdf')

#     with open(robot_urdf_description_file, 'r') as f:
#         robot_description_from_file = f.read()





#     joint_state_publisher_gui_node = launch_ros.actions.Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         name='joint_state_publisher_gui',
#         condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
#     )


#     gazebo_sim_start = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
#         ),
#         launch_arguments={'gz_args': '-r empty.sdf'}.items(),
#     )

#     # robot_state_publisher_node = launch_ros.actions.Node(
#     #     package='robot_state_publisher',
#     #     executable='robot_state_publisher',
#     #     output='both',
#     #     parameters=[robot_description]
#     # )
   

#     rviz_node = launch_ros.actions.Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='log',
#         arguments=['-d', LaunchConfiguration('rvizconfig')],
#     )

#     static_tf_node = Node(
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         name="static_transform_publisher",
#         output="log",
#         arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
#     )
    
#     spawn_model_into_gazebo = Node(
#         package='ros_gz_sim',
#         executable='create',
#         parameters=[{'name': 'ruka',
#                     'file': default_urdf_path,
#                     'topic': '/robot_description'
#                     }],
#         output='both',
#     )

    
#     # ros2_controllers_path = os.path.join(
#     #     get_package_share_directory("ruka_gz"),
#     #     "config",
#     #     "ros_control.yaml",
#     # )
#     # ros2_control_node = Node(
#     #     package="controller_manager",
#     #     executable="ros2_control_node",
#     #     parameters=[ros2_controllers_path],
#     #     remappings=[
#     #         ("/controller_manager/robot_description", "/robot_description"),
#     #     ],
#     #     output="screen",
#     # )

#     # )
#     # joint_state_broadcaster_spawner = Node(
#     #     package="controller_manager",
#     #     executable="spawner",
#     #     arguments=[
#     #         "joint_state_broadcaster",
#     #         "--controller-manager",
#     #         "/controller_manager",
#     #     ],
#     # )

#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="screen",
#         parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_from_file}],
#         arguments=[default_urdf_path]
#         )

#     # ros2_control using FakeSystem as hardware
#     ros2_controllers_path = os.path.join(
#         get_package_share_directory("ruka"),
#         "config",
#         "ros2_controllers.yaml",
#     )

#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[ros2_controllers_path],
#         remappings=[
#             ("/controller_manager/robot_description", "/robot_description"),
#         ],
#         output="both",
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager",
#             "/controller_manager",
#         ],
#     )

#     ruka_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["ruka_arm_controller", "-c", "/controller_manager"],
#     )




#     gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[default_model_path])
#     gazebo_media_path = SetEnvironmentVariable(name='GAZEBO_MEDIA_PATH', value=[default_model_path])

   
#     launch_description = launch.LaunchDescription([
#         # ros2_control_hardware_type,
#         gazebo_model_path,
#         gazebo_media_path,
#         robot_state_publisher,
#         ros2_control_node,
#         ruka_controller_spawner,
#         joint_state_broadcaster_spawner,
#         static_tf_node,

#         launch.actions.DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'),
#         launch.actions.DeclareLaunchArgument(name='gui', default_value='false',
#                                              description='Flag to enable joint_state_publisher_gui'),
#         launch.actions.DeclareLaunchArgument(name='model', default_value=default_urdf_path,
#                                              description='Absolute path to robot urdf file'),
#         launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
#                                              description='Absolute path to rviz config file'),
    
    
#         # robot_state_publisher_node,
        
        
#         rviz_node,
#         # gzclient,
#         gazebo_sim_start,
#         spawn_model_into_gazebo,
        
#         # joint_state_publisher_node,
#         #joint_state_publisher_gui_node,
#     ])

#     # launch_description.add_action(spawn_kinect_launch_description)

#     return launch_description
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    package_name = 'ruka_gz'
    # ros2_control_hardware_type = DeclareLaunchArgument(
    #     "ros2_control_hardware_type",
    #    default_value="joint_trajectory_controller",
    #    description="Specify the hardware type for ros2_control.",
    # )
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ruka_gz'),
                 'config', 'ruka_gz.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare('ruka_gz'),
    #         'config',
    #         'ros2_controllers.yaml',
    #     ]
    # )

    robot_controllers = os.path.join(get_package_share_directory(package_name), 'config', 'shok_controllers.yaml')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ruka_gz', '-allow_renaming', 'true'],
    )
# ///////////////////////////////
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    ruka_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ruka_gz_arm_controller',
             "-c", "/controller_manager",
            ],
    )

    return LaunchDescription([
        # Launch gazebo environment
        # ros2_control_hardware_type,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        ros2_control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ruka_arm_controller_spawner],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])