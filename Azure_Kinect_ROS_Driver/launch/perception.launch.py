# perception.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the Azure Kinect driver launch file
    azure_driver_launch = os.path.join(
        get_package_share_directory('azure_kinect_ros_driver'),
        'launch',
        'driver.launch.py'
    )

    # Include the Azure Kinect driver launch file
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(azure_driver_launch)
    )

    # Define the imu_to_tf_node
    imu_to_tf = Node(
        package='imu_to_tf',
        executable='imu_to_tf_node',
        name='imu_to_tf_node',
        output='screen'
    )

    # Define rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Delay launching imu_to_tf and rviz by 10 seconds to ensure the driver initializes
    delayed_launch = TimerAction(
        period=10.0,  # seconds
        actions=[imu_to_tf, rviz]
    )

    return LaunchDescription([
        driver,
        delayed_launch
    ])
    