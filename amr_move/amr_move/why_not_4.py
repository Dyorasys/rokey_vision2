'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from std_msgs.msg import String
import time


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.finde_publisher = self.create_publisher(String, '/finde', 10)

        self.is_shutdown = False  # 종료 플래그
        self.c_count = 0  # "c" 메시지 발행 횟수
        self.timer = None  # 타이머 핸들

        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',
            self.trigger_callback,
            10
        )
        self.get_logger().info('Waiting for trigger message "a" on /trigger_topic...')

    def trigger_callback(self, msg):
        if msg.data.lower() == 'a':
            self.get_logger().info('Received trigger message "a". Setting up navigation...')
            self.publish_initial_pose()
            time.sleep(2)  # 초기 위치 설정 후 대기
            self.get_logger().info('Waiting for navigation to be ready...')
            self.wait_for_navigation_ready()
            self.send_goal()
        else:
            self.get_logger().warn(f'Ignored message: {msg.data}')

    def publish_initial_pose(self):
        self.get_logger().info('Publishing initial pose...')
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = 0.026354703738017037
        initial_pose.pose.pose.position.y = -0.023555173715895102
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.07846075992136109,
            w=0.9969172027568601
        )
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
        self.initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published successfully.')

    def wait_for_navigation_ready(self):
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available. Is navigation running?')
            return
        self.get_logger().info('FollowWaypoints action server is ready.')


    def send_goal(self):
        waypoints = []

        # 웨이포인트 정의
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = "map"
        waypoint1.pose.position.x = 0.20045082073635237
        waypoint1.pose.position.y = -0.11526315559203755
        waypoint1.pose.orientation.z = -0.5501947083050299
        waypoint1.pose.orientation.w = 0.835036396184707
        waypoints.append(waypoint1)

        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = "map"
        waypoint2.pose.position.x = 0.2885849106796945
        waypoint2.pose.position.y = -0.7729320617606924
        waypoint2.pose.orientation.z = -0.998553042937628
        waypoint2.pose.orientation.w = 0.053775649136049555
        waypoints.append(waypoint2)

        waypoint3 = PoseStamped()
        waypoint3.header.frame_id = "map"
        waypoint3.pose.position.x = -0.43766810834163267
        waypoint3.pose.position.y = -0.6090071044161587
        waypoint3.pose.orientation.z = 0.9647924028389457
        waypoint3.pose.orientation.w = 0.2630125841556892
        waypoints.append(waypoint3)

        waypoint4 = PoseStamped()
        waypoint4.header.frame_id = "map"
        waypoint4.pose.position.x = -1.024580178661337
        waypoint4.pose.position.y = -0.21175594593394653
        waypoint4.pose.orientation.z = 0.9813011653381158
        waypoint4.pose.orientation.w = 0.19247862973861757
        waypoints.append(waypoint4)

        waypoint5 = PoseStamped()
        waypoint5.header.frame_id = "map"
        waypoint5.pose.position.x = -1.4703407287627268
        waypoint5.pose.position.y = -0.292804214493149
        waypoint5.pose.orientation.z = -0.6196984326209984
        waypoint5.pose.orientation.w = 0.784840017205467
        waypoints.append(waypoint5)

        waypoint6 = PoseStamped()
        waypoint6.header.frame_id = "map"
        waypoint6.pose.position.x = -1.5726163715771126
        waypoint6.pose.position.y = -0.3743943690241698
        waypoint6.pose.orientation.z = -0.7126831075538593
        waypoint6.pose.orientation.w = 0.7014861283071635
        waypoints.append(waypoint6)


        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Sending goal with waypoints...')
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by Action server.')
                return
            self.get_logger().info('Goal accepted by Action server.')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Moving towards Waypoint {feedback.current_waypoint}')

    def get_result_callback(self, future):
        try:
            result = future.result().result
            missed_waypoints = result.missed_waypoints
            if missed_waypoints:
                self.get_logger().warn(f'Missed waypoints: {missed_waypoints}')
            else:
                self.get_logger().info('All waypoints completed successfully!')
        except Exception as e:
            self.get_logger().error(f'Error getting result: {e}')
        finally:
            self.start_finde_publishing()

    def start_finde_publishing(self):
        """/finde 메시지를 5초 동안 5번만 발행"""
        self.get_logger().info('Starting periodic publishing of "c" to /finde...')
        self.c_count = 0
        self.timer = self.create_timer(1.0, self.publish_finde_message)  # 1초 간격으로 발행

    def publish_finde_message(self):
        """5번 메시지를 발행한 후 타이머 중지"""
        if self.c_count < 5:
            msg = String()
            msg.data = 'c'
            self.finde_publisher.publish(msg)
            self.get_logger().info(f'Published "c" to /finde topic. Count: {self.c_count + 1}')
            self.c_count += 1
        else:
            self.get_logger().info('Finished publishing "c" messages.')
            self.timer.cancel()  # 타이머 중지
            self.safe_shutdown()
            
def safe_shutdown(self):
    """노드를 안전하게 종료합니다."""
    if not self.is_shutdown:
        self.get_logger().info('Shutting down node...')
        if self.timer:
            self.timer.cancel()  # 타이머 중지
        try:
            # rclpy 컨텍스트가 활성 상태인지 확인
            if rclpy.ok():
                self.destroy_node()  # 노드 파괴
                self.get_logger().info("Node destroyed successfully.")
                rclpy.shutdown()  # 컨텍스트 종료
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        finally:
            self.is_shutdown = True  # 종료 상태 설정


def main(args=None):
    """메인 함수: 노드를 실행하고 안전하게 종료"""
    rclpy.init(args=args)  # ROS 2 초기화
    node = NavigationNode()  # 네비게이션 노드 생성

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
    finally:
        # 종료 처리
        if not node.is_shutdown:  # 종료 상태 확인
            node.safe_shutdown()

    # 프로그램 종료
    if not rclpy.ok():  # 컨텍스트가 종료된 상태를 확인
        node.get_logger().info("Exiting program safely.")




if __name__ == '__main__':
    main()


'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from std_msgs.msg import String


class NavigationNode(Node):
    """
    ros2 네비게이션 노드:
    - 초기 위치 설정
    - 웨이포인트 네비게이션 실행
    - /find 토픽으로 10초마다 'c' 메시지 발행
    """
    def __init__(self):
        super().__init__('navigation_node')  # 노드 이름

        # 초기 위치 설정 퍼블리셔
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 웨이포인트 네비게이션 액션 클라이언트
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # /find 퍼블리셔
        self.finde_publisher = self.create_publisher(String, '/find', 10)

        # /trigger_topic 구독
        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',
            self.trigger_callback,
            10
        )

        # 10초마다 'c' 메시지 발행 타이머 설정
        self.timer = self.create_timer(10.0, self.publish_finde_message)

        self.get_logger().info('Node initialized. Publishing "c" to /find every 10 seconds.')
        self.get_logger().info('Waiting for trigger message "a" on /trigger_topic...')

    def trigger_callback(self, msg):
        """/trigger_topic 콜백: 'a' 메시지를 받으면 네비게이션 실행"""
        if msg.data.lower() == 'a':
            self.get_logger().info('Received trigger message "a". Setting up navigation...')
            self.publish_initial_pose()  # 초기 위치 설정
            self.wait_for_navigation_ready()  # 네비게이션 준비
            self.send_goal()  # 웨이포인트 전송
        else:
            self.get_logger().warn(f'Ignored message: {msg.data}')

    def publish_initial_pose(self):
        """초기 위치 설정 메시지 발행"""
        self.get_logger().info('Publishing initial pose...')

        # 초기 위치 메시지 생성
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # 초기 위치 좌표 및 방향 설정
        initial_pose.pose.pose.position.x = 0.026354703738017037
        initial_pose.pose.pose.position.y = -0.023555173715895102
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=-0.07846075992136109, w=0.9969172027568601
        )
        initial_pose.pose.covariance = [0.25] * 36  # 예제 값

        # 초기 위치 메시지 발행
        self.initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

    def wait_for_navigation_ready(self):
        """네비게이션 액션 서버 준비 확인"""
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available. Is navigation running?')
        else:
            self.get_logger().info('FollowWaypoints action server is ready.')

    def send_goal(self):
        """웨이포인트 설정 및 액션 서버로 목표 전송"""
        waypoints = []

        # 웨이포인트 정의
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"

        # 예제 웨이포인트들
        coordinates = [
            (0.2, -0.1, -0.55, 0.835),
            (0.3, -0.77, -0.998, 0.053),
            (-0.43, -0.6, 0.964, 0.26),
            (-1.02, -0.21, 0.981, 0.19),
            (-1.47, -0.29, -0.619, 0.784),
        ]

        for x, y, z, w in coordinates:
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.orientation.z = z
            waypoint.pose.orientation.w = w
            waypoints.append(waypoint)

        # 목표 생성 및 전송
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.get_logger().info('Sending goal with waypoints...')
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """네비게이션 피드백 처리"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Moving towards Waypoint {feedback.current_waypoint}')

    def goal_response_callback(self, future):
        """목표가 액션 서버에서 수락되었는지 확인"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by Action server.')
                return
            self.get_logger().info('Goal accepted by Action server.')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')

    def get_result_callback(self, future):
        """네비게이션 완료 후 결과 확인"""
        try:
            result = future.result().result
            missed_waypoints = result.missed_waypoints
            if missed_waypoints:
                self.get_logger().warn(f'Missed waypoints: {missed_waypoints}')
            else:
                self.get_logger().info('All waypoints completed successfully!')
        except Exception as e:
            self.get_logger().error(f'Error getting result: {e}')

    def publish_finde_message(self):
        """10초마다 'c' 메시지 발행"""
        msg = String()
        msg.data = 'c'
        self.finde_publisher.publish(msg)
        self.get_logger().info("Published 'c' to /find topic.")


def main(args=None):
    """ROS 2 실행"""
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Exiting...')
    finally:
        node.destroy_node()  # 노드 정리
        rclpy.shutdown()  # ROS 컨텍스트 종료


if __name__ == '__main__':
    main()
