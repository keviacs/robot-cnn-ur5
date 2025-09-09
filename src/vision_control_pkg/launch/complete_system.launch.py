#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch completo del sistema CNN + UR5
    
    Este archivo lanza:
    1. MoveIt con UR5 
    2. Cámara virtual
    3. Nodo de visión CNN
    4. Nodo de control del brazo
    """
    
    # Argumentos del launch
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5',
        description='Tipo de robot UR (ur3, ur5, ur10, etc.)'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Lanzar RViz para visualización'
    )
    
    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='test_images',
        description='Modo de cámara: test_images, webcam, generated'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Usar hardware simulado (fake) en lugar del robot real'
    )
    
    # Configuraciones de launch
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    camera_mode = LaunchConfiguration('camera_mode')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
    # Rutas de paquetes
    ur_moveit_config_package = FindPackageShare('ur_moveit_config')
    
    # 1. LANZAR MOVEIT CON UR5
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                ur_moveit_config_package,
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
            'use_fake_hardware': use_fake_hardware,
        }.items()
    )
    
    # 2. NODO CÁMARA VIRTUAL
    camera_node = Node(
        package='vision_control_pkg',
        executable='camera_publisher',
        name='camera_publisher',
        parameters=[{
            'camera_mode': camera_mode,
        }],
        output='screen'
    )
    
    # 3. NODO DE VISIÓN CNN  
    vision_node = Node(
        package='vision_control_pkg',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )
    
    # 4. NODO DE CONTROL DEL BRAZO
    control_node = Node(
        package='vision_control_pkg', 
        executable='control_node',
        name='control_node',
        output='screen'
    )
    
    # 5. NODO MONITOR DEL SISTEMA
    monitor_node = Node(
        package='vision_control_pkg',
        executable='system_monitor',
        name='system_monitor',
        output='screen'
    )
    
    # 6. TRANSFORMACIONES ESTÁTICAS (para la cámara)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '--x', '0.5',      # 50cm adelante del robot
            '--y', '0.0',      # Centrado
            '--z', '0.8',      # 80cm de altura
            '--roll', '0.0',   # Sin rotación en roll
            '--pitch', '1.57', # Apuntar hacia abajo (90 grados)
            '--yaw', '0.0',    # Sin rotación en yaw
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        ur_type_arg,
        launch_rviz_arg,
        camera_mode_arg,
        use_fake_hardware_arg,
        
        # Nodos y launches
        ur_moveit_launch,
        camera_tf_node,
        camera_node,
        vision_node,
        control_node,
        monitor_node,
    ])
