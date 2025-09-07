from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
import os

def generate_launch_description():
    # Caminho para o seu pacote
    pkg_share = get_package_share_directory('world_simulation')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

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
    
    # 1. Comando para gerar o arquivo SDF a partir da imagem (Este é o nosso gatilho)
    generate_sdf = ExecuteProcess(
        cmd=['ros2', 'run', 'world_simulation', 'img2sdf', image, sdf_file],
        output='screen'
    )

    # --- Ações que DEPENDEM da geração do SDF ---

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', sdf_file], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_exec = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_bringup', 'rviz2.launch.py']
    )

    # 2. Manipulador de eventos para iniciar o Gazebo e o robô APÓS a criação do SDF
    delay_actions_after_sdf = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_sdf,
            on_exit=[
                gzserver_cmd,
                gzclient_cmd,
                spawn_robot,
                robot_state_publisher_cmd,
                rviz_exec,
            ]
        )
    )

    # Ação para configurar variáveis de ambiente (pode ser executada a qualquer momento)
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    # Lista final de ações para o LaunchDescription
    return LaunchDescription([
        image_arg,
        x_pose_arg,
        y_pose_arg,
        set_env_vars_resources,

        # Inicia a geração do SDF imediatamente
        generate_sdf,
        
        # Registra o evento que irá disparar as outras ações quando generate_sdf terminar
        delay_actions_after_sdf,
    ])