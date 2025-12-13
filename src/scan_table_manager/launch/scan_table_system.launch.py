from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="scan_table_manager",
            executable="scan_table_manager",
            name="scan_table_manager"
        ),

        Node(package="mock_robot", executable="mock_robot", name="mock_robot"),
        Node(package="mock_scanner", executable="mock_scanner", name="mock_scanner"),
        Node(package="mock_pusher", executable="mock_pusher", name="mock_pusher"),
        Node(package="mock_table_sensor", executable="mock_table_sensor", name="mock_table_sensor"),
    ])
