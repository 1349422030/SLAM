from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_subscriber',
            namespace='image_subscriber',
            executable='cam_main',
            name='cam_main',
            parameters=[{
                    'image_base_path': '/home/workspace/Datasets/V1_01_easy',  # /xxx
                    'left_sub_image_path': '/cam0/data',  # /xxx
                    'right_sub_image_path': '/cam1/data'  # /xxx
            }],
            output="screen",
        )
    ])


# def generate_launch_description():

#     container = ComposableNodeContainer(
#         name='image_subscriber',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='image_subscriber',
#                 plugin='cam_node::CamNode',
#                 name='image_subscriber',
#                 parameters=[{
#                     'img': '/home/workspace/datasets/pic/03/IMG_20220319_151322.jpg'
#                 }],
#                 extra_arguments=[{'use_intra_process_comms': True}],
#             )
#         ],
#         output='screen',
#     )

#     return launch.LaunchDescription([container])
