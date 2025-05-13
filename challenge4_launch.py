from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    params_file = PathJoinSubstitution([
        FindPackageShare("challenge_04"),
        "params.yaml"
    ])

    odometry_node = Node(
        package='challenge_04',
        executable='odometry',
        output='screen',
    )

    controller_node = Node(
        package='challenge_04',
        executable='controller10',
        name='controller10',
        parameters=[params_file],
        output='screen',
    )

    path_node = Node(
        package='challenge_04',
        executable='path3',
        name='Generator_node',
        parameters=[params_file],
        output='screen',
    )

    camera_node = Node(
        package='challenge_04',
        executable='cameraPub',
        output='screen',
    )

    semaforo_node = Node(
        package='challenge_04',
        executable='susImage4',
        output='screen',
    )

    return LaunchDescription([
        odometry_node,
        controller_node,
        path_node,
        camera_node,
        semaforo_node,
    ])
