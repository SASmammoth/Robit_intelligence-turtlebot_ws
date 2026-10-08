# 카메라 + 차선 + 경로 + 표지판 + 주행 전체 실행
#   ros2 launch tb_line robot.launch.py                  # 전부
#   ros2 launch tb_line robot.launch.py signs:=false     # YOLO 빼고
#   ros2 launch tb_line robot.launch.py drive:=false     # 주행 노드 빼고 (차선·경로만 볼 때)
#
# tb_drive는 drive.enable=false로 뜨므로, 실행해도 GUI에서 켜기 전까지는 움직이지 않음
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def share_or_empty(pkg, *rel):
    # 패키지가 아직 빌드 안 됐어도 해당 기능을 끄고 실행할 수 있게
    try:
        return os.path.join(get_package_share_directory(pkg), *rel)
    except PackageNotFoundError:
        return ''


def generate_launch_description():
    default_engine = share_or_empty('tb_signs', 'models', 'sign.engine')
    drive_yaml = share_or_empty('tb_drive', 'config', 'drive.yaml')

    signs = LaunchConfiguration('signs')
    engine = LaunchConfiguration('engine')
    drive = LaunchConfiguration('drive')

    camera = Node(package='tb_camera', executable='camera_node', name='camera_node', output='screen')

    # 카메라가 장치를 연 뒤 노출 고정
    cam_setup = ExecuteProcess(
        cmd=['bash', os.path.join(get_package_share_directory('tb_lane'), 'scripts', 'cam_setup.sh')],
        output='screen')

    line = Node(package='tb_line', executable='lineDetect_node', name='lineDetect_node', output='screen')
    path = Node(package='tb_path', executable='path_node', name='path_node', output='screen')
    yolo = Node(package='tb_signs', executable='sign_detect_node', name='signs_node', output='screen',
                parameters=[{'engine_path': engine}],
                condition=IfCondition(signs))
    driver = Node(package='tb_drive', executable='drive_node', name='drive_node', output='screen',
                  parameters=[drive_yaml] if drive_yaml else [],
                  condition=IfCondition(drive))

    return LaunchDescription([
        DeclareLaunchArgument('signs', default_value='true'),
        DeclareLaunchArgument('engine', default_value=default_engine),
        DeclareLaunchArgument('drive', default_value='true'),
        camera,
        RegisterEventHandler(OnProcessStart(target_action=camera,
                                            on_start=[TimerAction(period=2.0, actions=[cam_setup])])),
        line,
        path,
        yolo,
        driver,
    ])
