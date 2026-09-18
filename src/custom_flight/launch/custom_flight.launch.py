from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    flight_mode_executor = Node(
        package='custom_flight',
        executable='flight_mode_executor',
        name='flight_mode_executor',
        output='screen',
        parameters=[
            # Add parameters here later
            # {
            #     'takeoff_altitude': -5.0,
            # }
        ],
    )

    return LaunchDescription([
        flight_mode_executor,
    ])
