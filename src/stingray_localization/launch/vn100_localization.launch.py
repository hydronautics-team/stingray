from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Аргументы командной строки
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    publish_tf = LaunchConfiguration("publish_tf", default="true")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    # Пути
    pkg_share = FindPackageShare("stingray_localization")
    # ekf_config = PathJoinSubstitution([pkg_share, "config", "ekf.yaml"])
    ekf_config = "/stingray/src/stingray_localization/config/ekf.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Использовать симуляционное время",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Публиковать трансформации",
            ),
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", description="Запускать RViz"
            ),
            # 2. Запуск EKF (robot_localization)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_config, {"use_sim_time": use_sim_time}],
                output="screen",
                remappings=[
                    # Если нужно переименовать топик – оставлено как есть
                    ("/odometry/filtered", "/odometry/filtered"),
                ],
            ),
            # 3. Ваш узел: интеграция IMU, публикация трансформаций и маркеров
            Node(
                package="stingray_localization",
                executable="auv_odometry_tf2_broadcaster",  # имя точки входа в setup.py
                name="auv_odometry_tf2_broadcaster",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            # # 4. Запуск RViz (опционально)
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     arguments=[
            #         "-d",
            #         PathJoinSubstitution([pkg_share, "config", "rviz_config.rviz"]),
            #     ],
            #     condition=IfCondition(launch_rviz),
            # ),
        ]
    )
