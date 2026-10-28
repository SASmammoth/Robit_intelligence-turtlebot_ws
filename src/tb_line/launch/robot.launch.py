# 카메라 + 차선 + 경로 + 표지판 + 주행 전체 실행
#   ros2 launch tb_line robot.launch.py                  # 전부
#   ros2 launch tb_line robot.launch.py signs:=false     # YOLO 빼고
#   ros2 launch tb_line robot.launch.py yolo_hz:=15      # YOLO 초당 15번만 (GPU·CPU 절약)
#   ros2 launch tb_line robot.launch.py drive:=false     # 주행 노드 빼고 (차선·경로만 볼 때)
#   ros2 launch tb_line robot.launch.py params:=~/tb_params.yaml   # GUI에서 저장한 값으로 시작
#
# 파라미터 적용 순서: 각 패키지 config/*.yaml → params:= 파일 (뒤에 오는 값이 덮어씀)
#
# tb_drive는 drive.enable=false로 뜨므로, 실행해도 GUI에서 켜기 전까지는 움직이지 않음
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
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


def param_files(pkg_yaml, extra):
    # 패키지 기본 yaml + (있으면) GUI에서 저장한 파일. 없는 파일은 건너뜀
    return [f for f in (pkg_yaml, extra) if f and os.path.isfile(f)]


def launch_setup(context):
    drive_yaml = share_or_empty("tb_drive", "config", "drive.yaml")
    line_yaml = share_or_empty("tb_line", "config", "line.yaml")
    path_yaml = share_or_empty("tb_path", "config", "path.yaml")

    params = os.path.expanduser(LaunchConfiguration('params').perform(context))
    if params and not os.path.isfile(params):
        print(f"[robot.launch] params 파일 없음, 무시: {params}")
        params = ''

    signs = LaunchConfiguration('signs')
    engine = LaunchConfiguration('engine')
    drive = LaunchConfiguration('drive')
    brain_on = LaunchConfiguration('brain')

    camera = Node(package='tb_camera', executable='camera_node', name='camera_node', output='screen')

    # 카메라가 장치를 연 뒤 노출 고정
    cam_setup = ExecuteProcess(
        cmd=['bash', os.path.join(get_package_share_directory('tb_line'), 'scripts', 'cam_setup.sh')],
        output='screen')

    line = Node(package='tb_line', executable='lineDetect_node', name='lineDetect_node', output='screen',
                parameters=param_files(line_yaml, params))
    path = Node(package='tb_path', executable='path_node', name='path_node', output='screen',
                parameters=param_files(path_yaml, params))
    # YOLO: params 파일의 /signs_node 섹션(GUI 저장) → yolo_hz 인자를 줬을 때만 그 값으로 덮어씀
    yolo_params = param_files('', params) + [{'engine_path': engine}]
    yolo_hz = LaunchConfiguration('yolo_hz').perform(context).strip()
    if yolo_hz:
        yolo_params.append({'max_hz': float(yolo_hz)})
    yolo = Node(package='tb_signs', executable='sign_detect_node', name='signs_node', output='screen',
                parameters=yolo_params,
                condition=IfCondition(signs))
    driver = Node(package='tb_drive', executable='drive_node', name='drive_node', output='screen',
                  parameters=param_files(drive_yaml, params),
                  condition=IfCondition(drive))
    brain = Node(package='tb_brain', executable='brain_node', name='brain_node', output='screen',
                 parameters=param_files('', params),
                 condition=IfCondition(brain_on))
    # odom.* 값을 바꾸려면 params 파일에 /tb_uart_node 섹션을 넣으면 됨
    uart = Node(
        package="tb_uart",
        executable="tb_uart_node",
        name="tb_uart_node",
        output="screen",
        parameters=param_files('', params),
    )

    return [
        camera,
        RegisterEventHandler(
            OnProcessStart(
                target_action=camera,
                on_start=[TimerAction(period=2.0, actions=[cam_setup])],
            )
        ),
        line,
        path,
        yolo,
        driver,
        uart,
        brain
    ]


def generate_launch_description():
    default_engine = share_or_empty('tb_signs', 'models', 'sign.engine')
    return LaunchDescription(
        [
            # Fast DDS 기본은 동기 발행: publish() 안에서 소켓 전송까지 끝냄.
            # WiFi 너머 노트북(GUI)으로 보내는 영상이 막히면 카메라·차선 노드가 그 자리에서 멈춤
            # → 비동기: publish()는 큐에 넣고 바로 반환, 전송은 Fast DDS 스레드가 함
            SetEnvironmentVariable("RMW_FASTRTPS_PUBLICATION_MODE",
                                   os.environ.get("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")),
            DeclareLaunchArgument("signs", default_value="true"),
            DeclareLaunchArgument("engine", default_value=default_engine),
            DeclareLaunchArgument("yolo_hz", default_value="",
                                  description="YOLO 초당 최대 추론 횟수. 비우면 params 파일 값(없으면 0 = 프레임마다), 예: 15"),
            DeclareLaunchArgument("drive", default_value="true"),
            DeclareLaunchArgument("brain", default_value="true"),
            DeclareLaunchArgument("params", default_value="",
                                  description="GUI에서 저장한 파라미터 yaml (비우면 패키지 기본값만)"),
            OpaqueFunction(function=launch_setup),
        ]
    )
