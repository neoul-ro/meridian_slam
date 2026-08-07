from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Package path
    bunker_description_pkg = FindPackageShare('bunker_description')

    # Default paths
    default_urdf_path = PathJoinSubstitution([bunker_description_pkg, 'urdf', 'bunker.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([bunker_description_pkg, 'rviz', 'display.rviz'])

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_path,
        description='Path to the robot URDF model'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    ))

    # Include display.launch.py from a common display launcher (or build a similar one in your own package)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py'])
        ),
        launch_arguments={
            'urdf_package': 'bunker_description',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui'),
        }.items()
    ))

    return ld
