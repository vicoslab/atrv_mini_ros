import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="atrv_mini_ros",
            executable="atrv_mini_node",
            name="atrv_node",
            output="screen",
            parameters=[{"port": "/dev/ttyUSB0"}]
        )
    ])
