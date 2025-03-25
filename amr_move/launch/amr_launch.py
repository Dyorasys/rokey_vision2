from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 첫 번째 노드
        Node(
            package='amr_move',         # 패키지 이름
            executable='last_send',     # 실행 파일 이름 (setup.py의 console_scripts와 일치해야 함)
            name='yolo_detect',          # 노드 이름
            output='screen',            # 출력 방식
        ),
        Node(
            package='amr_move',         # 패키지 이름
            executable='last_activate',         # 실행 파일 이름 (setup.py의 console_scripts와 일치해야 함)
            name='navigation',         # 노드 이름
            output='screen',            # 출력 방식
        ),

    ])
