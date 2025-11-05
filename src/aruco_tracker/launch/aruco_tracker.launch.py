from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    microxrce_agent_process =  ExecuteProcess(
             cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
             name='microxrce_agent_process'
    )
    return LaunchDescription([
        microxrce_agent_process,
        # Run bridge nodes in separate screen sessions
        # ExecuteProcess(
        #      cmd=['screen', '-dmS', 'bridge', 'bash', '-c',
        #           'MicroXRCEAgent udp4 -p 8888'],
        #             name='microxrce_agent_process'
        # ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            output='screen',
            arguments=[
                '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),
        # ExecuteProcess(
        #      cmd=['screen', '-dmS', 'image_bridge', 'bash', '-c', 
        #           'ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        #             name='image_bridge_process'
        # ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            output='screen',
            arguments=[
                '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        ),
        # ExecuteProcess(
        #      cmd=['screen', '-dmS', 'image_bridge', 'bash', '-c', 
        #           'ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera_info/image@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        #             name='camera_info_bridge_process'
        # ),
        # ExecuteProcess(
        #     cmd=['screen', '-dmS', 'camera_info_bridge', 'bash', '-c', 'ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        #     name='camera_info_bridge_process'
        # ),
        # Continue running Aruco tracker node as a normal ROS node and display output on screen
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ]
        ),
    ])