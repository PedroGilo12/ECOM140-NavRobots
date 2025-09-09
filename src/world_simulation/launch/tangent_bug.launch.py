from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('world_simulation')

    image_arg = DeclareLaunchArgument(
        'image',
        default_value=os.path.join(pkg_share, 'worlds', 'mapa.png'),
        description='Imagem do mapa para gerar o mundo'
    )
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='-1.0', description='Posição inicial X')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='-1.0', description='Posição inicial Y')

    image = LaunchConfiguration('image')

    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'image': image,
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    tangent_bug = ExecuteProcess(
        cmd=['ros2', 'run', 'tangent_bug', 'tangent_bug'],
        output='screen'
    )

    image_path = os.path.join(pkg_share, 'worlds', 'mapa.png')
    map_base_name = os.path.splitext(os.path.basename(image_path))[0]

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"{map_base_name}_tangent_bug_{timestamp}"

    rosbag_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_name,
            '/cmd_vel',
            '/odom',
            '/bug_state',
            '/scan',
            '/goal_marker'
        ],
        output='screen'
    )

    return LaunchDescription([
        image_arg,
        x_pose_arg,
        y_pose_arg,
        sim_world,
        tangent_bug,
        rosbag_cmd
    ])
