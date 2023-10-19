import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

MY_ROBOT = os.environ.get('MY_ROBOT', "mp_500")
MY_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_ENVIRONMENT + '.world')  
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    if MY_ROBOT == "mp_500" or MY_ROBOT == "mp_400":
        robot_dir = LaunchConfiguration(
            'robot_dir',
            default=os.path.join(get_package_share_directory('neo_simulation2'),
                'robots/'+MY_ROBOT,
                MY_ROBOT+'.urdf'))

    elif MY_ROBOT == "burger" or MY_ROBOT == "waffle" or MY_ROBOT == "waffe_pi" :
        robot_dir = LaunchConfiguration(
            'robot_dir',
            default=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                'urdf/',
                'turtlebot3_'+MY_ROBOT+'.urdf'))
        print(robot_dir)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    if MY_ROBOT == "mp_500" or MY_ROBOT == "mp_400":
        urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_ROBOT+'/', MY_ROBOT+'.urdf')

    elif MY_ROBOT == "burger" or MY_ROBOT == "waffle" or MY_ROBOT == "waffe_pi":
        model_folder = 'turtlebot3_' + MY_ROBOT
        urdf = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models',
            model_folder,
            'model.sdf'
        )
        #urdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf/', 'turtlebot3_'+ MY_ROBOT+'.urdf')

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', MY_ROBOT, '-file', urdf], output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf])

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items()
        )

    return LaunchDescription([spawn_entity, start_robot_state_publisher_cmd, gazebo])