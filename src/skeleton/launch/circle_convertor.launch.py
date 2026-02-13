from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_service_in_circle_convertor = DeclareLaunchArgument(
        'use_service',
        default_value='false',
        description='Whether to use simulation time'
    )

    container = ComposableNodeContainer(
        name='CircleConvetor',          
        namespace='',                        
        package='rclcpp_components',         
        executable='component_container_mt', 
        output='screen',

        composable_node_descriptions=[
            ComposableNode(
                package='skeleton',                    
                plugin='SquareArea',      
                name='square_area_node',
                remappings=[
                    ("shape", "/square_shape"),
                ],
            ),

            ComposableNode(
                package='skeleton',
                plugin='CircleGenerator',
                name='circle_generator_node',
                parameters=[
                    {
                        'use_service': LaunchConfiguration('use_service')
                    }
                ],
            ),
        ]
    )

    return LaunchDescription([
        container
    ])