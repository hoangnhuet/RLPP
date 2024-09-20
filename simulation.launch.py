import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def launch_setup(context: LaunchContext, my_neo_robot_arg):
    # Create a list to hold all the nodes
    launch_actions = []

    # The perform method of a LaunchConfiguration is called to evaluate its value.
    my_neo_robot = my_neo_robot_arg.perform(context)
    use_sim_time = True

    robots = ["mpo_700", "mp_400", "mp_500", "mpo_500"]

    # Checking if the user has selected a robot that is valid
    valid = my_neo_robot in robots
    if not valid:
        # Incase of an invalid selection
        print("Invalid option, setting mpo_700 by default")
        my_neo_robot = "mpo_700"
    
    with open('robot_name.txt', 'w') as file:
        file.write(my_neo_robot)
    
    # Set the world path to 'a.world' always
    world_path = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'worlds',
        'simple.world'
    )

    # Getting the robot description
    robot_description_urdf = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'robots', my_neo_robot,
        'mpo_700.urdf'
    )

    # Setting the world and starting Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    # Spawning the robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', my_neo_robot, '-file', robot_description_urdf], 
        output='screen'
    )

    # Start the robot state publisher node
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[robot_description_urdf]
    )

    # Add nodes to the launch actions
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(gazebo)
    launch_actions.append(spawn_entity)

    # Starting the teleop node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix='xterm -e',
        name='teleop'
    )

    launch_actions.append(teleop)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments 'my_robot' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
        'my_robot', 
        default_value='mpo_700',
        description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"'
    ) 

    # Create launch configuration variables for the robot name
    my_neo_robot_arg = LaunchConfiguration('my_robot')

    ld.add_action(declare_my_robot_arg)

    context_arguments = [my_neo_robot_arg]

    opq_function = OpaqueFunction(
        function=launch_setup, 
        args=context_arguments
    )

    ld.add_action(opq_function)

    return ld

