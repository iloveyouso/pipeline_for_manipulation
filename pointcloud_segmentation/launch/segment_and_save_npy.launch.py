# segment_and_save_npy.launch.py

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the segmentation_node
    segmentation_node = Node(
        package='pointcloud_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen'
    )

    # Define the Python script to save point cloud as NumPy
    save_npy = ExecuteProcess(
        cmd=['python3', '/home/biomen/bjkim/pytorch_6dof-graspnet/demo/data/pc2npy_bj.py'],
        name='save_npy',
        output='screen'
    )

    # Delay executing the Python script by 10 seconds to ensure segmentation_node is running
    delayed_save = TimerAction(
        period=10.0,  # seconds
        actions=[save_npy]
    )

    return LaunchDescription([
        segmentation_node,
        delayed_save
    ])
