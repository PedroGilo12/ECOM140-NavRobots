from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
import os

def generate_launch_description():
    # Caminho para o seu pacote
    pkg_share = get_package_share_directory('world_simulation')

    # Argumento para a imagem do mundo
    image_arg = DeclareLaunchArgument(
        'image',
        default_value=os.path.join(pkg_share, 'worlds', 'mapa.png'),
        description='Imagem em preto e branco para gerar o mundo'
    )
    
    # Argumentos para a posição inicial do robô
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')

    # Configurações com base nos argumentos
    image = LaunchConfiguration('image')
    sdf_file = os.path.join(pkg_share, 'worlds', 'meu_mundo.sdf')
    
    # 1. Comando para gerar o arquivo SDF a partir da imagem
    generate_sdf = ExecuteProcess(
        cmd=['ros2', 'run', 'world_simulation', 'img2sdf', image, sdf_file],
        output='screen'
    )

    rviz_exec = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_bringup', 'rviz2.launch.py']
    )

    # 2. Ação para incluir o launch file que spawna o TurtleBot3
    # Encontra o pacote do turtlebot3_gazebo
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    spawn_turtlebot_launch_file = os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_turtlebot_launch_file),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    # 3. Ação temporizada para iniciar o Gazebo e, em seguida, spawnar o robô
    run_simulation_and_spawn_robot = TimerAction(
        period=2.0,  # Espera 2 segundos após a geração do SDF
        actions=[
            # Inicia o Gazebo com o mundo gerado
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', sdf_file], # O '-r' inicia a simulação pausada
                output='screen'
            ),
            # Spawna o robô no mundo
            spawn_robot
        ]
    )

    return LaunchDescription([
        image_arg,
        x_pose_arg,
        y_pose_arg,
        generate_sdf,
        run_simulation_and_spawn_robot,
        set_env_vars_resources,
        rviz_exec,
    ])