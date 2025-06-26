import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('psd_perception_camera')
    
    # Paths
    default_params_file = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'config', 'cone_detection_rviz.rviz')
    
    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file'
    )
    
    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value='',
        description='Path to TensorRT engine file (overrides params file)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz configuration file'
    )
    
    # Nodes
    cone_detector_node = Node(
        package='psd_perception_camera',
        executable='camera_node',
        name='camera_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'engine_path': LaunchConfiguration('engine_path'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # # RViz node (conditional)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', LaunchConfiguration('rviz_config')],
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz')),
    #     output='screen',
    # )
    
    return LaunchDescription([
        # Arguments
        params_file_arg,
        engine_path_arg,
        use_rviz_arg,
        rviz_config_arg,
        
        # Nodes
        cone_detector_node,
        #rviz_node,
    ])