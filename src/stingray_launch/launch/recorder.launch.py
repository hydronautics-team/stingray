import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Аргументы для первой камеры
        DeclareLaunchArgument('output_width', default_value='640',
                                description='Ширина видео'),
        DeclareLaunchArgument('output_height', default_value='480',
                                description='Высота видео'),
        DeclareLaunchArgument('output_fps', default_value='15',
                                description='Частота кадров видео'),
        DeclareLaunchArgument('output_format', default_value='h264',
                                description='Формат видео (FourCC)'),
        DeclareLaunchArgument('record_dir', default_value='./records/',
                                description='Путь к папке для сохранения записей'),
        # Аргументы для первой камеры
        DeclareLaunchArgument('camera_topic', default_value='/stingray/topics/camera/front',
                                description='Топик с изображениями для первой камеры'),

        # Нода для первой камеры
        Node(
            package='stingray_recorder',  # замените на имя вашего пакета
            executable='video_recorder_node',  # имя исполняемого файла ноды
            name='front_camera_video_recorder',
            parameters=[
                {'source_topic': LaunchConfiguration('camera_topic')},
                {'output_width': LaunchConfiguration('output_width')},
                {'output_height': LaunchConfiguration('output_height')},
                {'output_fps': LaunchConfiguration('output_fps')},
                {'output_format': LaunchConfiguration('output_format')},
                {'record_dir': LaunchConfiguration('record_dir')},
            ]
        )
    ])
