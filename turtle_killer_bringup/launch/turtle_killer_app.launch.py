from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtle_simulator_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
                {'background_r': 72},
                {'background_g': 34},
                {'background_b': 124}
            ]
    )

    turtle_spawner_node = Node(
        package="turtle_killer_pkg",
        executable="turtle_spawner",
        parameters=[
            {"time_spawner": 1.2},
        ]
    )

    turtle_controller_node = Node(
        package="turtle_killer_pkg",
        executable="turtle_controller",
        parameters=[
            {"turtle_speed": 3.0},
        ]
    )

    ld.add_action(turtle_simulator_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    return ld
