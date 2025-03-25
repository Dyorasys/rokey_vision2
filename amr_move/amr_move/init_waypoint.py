'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from std_msgs.msg import String
import math


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # Subscribe to /trigger_topic
        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',  # Trigger 토픽 구독
            self.trigger_callback,
            10
        )
        self.get_logger().info('Waiting for trigger message "a" on /trigger_topic...')

    def trigger_callback(self, msg):
        if msg.data.lower() == 'a':  # 메시지가 "a"인지 확인
            self.get_logger().info('Received trigger message "a". Performing navigation...')
            self.publish_initial_pose()
            self.send_goal()
        else:
            self.get_logger().warn(f'Ignored message: {msg.data}')

    def publish_initial_pose(self):
        # 초기 위치를 설정하고 발행
        self.get_logger().info('Publishing initial pose...')
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # 초기 위치 설정
        initial_pose.pose.pose.position.x = -0.017241953009357745
        initial_pose.pose.pose.position.y = 0.030906322294970673
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.001727534551818488,
            w=0.9999985078110728
        )

        # Covariance 설정
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        self.initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published successfully.')

    def send_goal(self):
        # 여러 웨이포인트 정의
        waypoints = []

        # 첫 번째 웨이포인트
        waypoint1 = PoseStamped()
        waypoint1.header.stamp.sec = 0
        waypoint1.header.stamp.nanosec = 0
        waypoint1.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint1.pose.position.x = 0.2914209932659453
        waypoint1.pose.position.y = -0.035827083389437166
        waypoint1.pose.position.z = 0.0
        waypoint1.pose.orientation.x = 0.0
        waypoint1.pose.orientation.y = 0.0
        waypoint1.pose.orientation.z = -0.5397640093892747
        waypoint1.pose.orientation.w = 0.8418163779399964
        waypoints.append(waypoint1)

        # 두 번째 웨이포인트
        waypoint2 = PoseStamped()
        waypoint2.header.stamp.sec = 0
        waypoint2.header.stamp.nanosec = 0
        waypoint2.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint2.pose.position.x = 0.3369757869918463
        waypoint2.pose.position.y = -0.5098393486385897
        waypoint2.pose.position.z = 0.0
        waypoint2.pose.orientation.x = 0.0
        waypoint2.pose.orientation.y = 0.0
        waypoint2.pose.orientation.z = -0.933249104361796
        waypoint2.pose.orientation.w = 0.35922988351180585
        waypoints.append(waypoint2)

        # 세 번째 웨이포인트
        waypoint3 = PoseStamped()
        waypoint3.header.stamp.sec = 0
        waypoint3.header.stamp.nanosec = 0
        waypoint3.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint3.pose.position.x = -0.7423834107933234
        waypoint3.pose.position.y = -0.7337036149646982
        waypoint3.pose.position.z = 0.0
        waypoint3.pose.orientation.x = 0.0
        waypoint3.pose.orientation.y = 0.0
        waypoint3.pose.orientation.z = 0.8352340850356917
        waypoint3.pose.orientation.w = 0.54989455643295
        waypoints.append(waypoint3)

        # 네 번째 웨이포인트
        waypoint4 = PoseStamped()
        waypoint4.header.stamp.sec = 0
        waypoint4.header.stamp.nanosec = 0
        waypoint4.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint4.pose.position.x = -0.978511490652095
        waypoint4.pose.position.y = -0.11199992094129231
        waypoint4.pose.position.z = 0.0
        waypoint4.pose.orientation.x = 0.0
        waypoint4.pose.orientation.y = 0.0
        waypoint4.pose.orientation.z = 0.9923236671749109
        waypoint4.pose.orientation.w = 0.12366785986883035
        waypoints.append(waypoint4)

        # 다섯 번째 웨이포인트
        waypoint5 = PoseStamped()
        waypoint5.header.stamp.sec = 0
        waypoint5.header.stamp.nanosec = 0
        waypoint5.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint5.pose.position.x = -1.4371909391769004
        waypoint5.pose.position.y = -0.14034103530455452
        waypoint5.pose.position.z = 0.0
        waypoint5.pose.orientation.x = 0.0
        waypoint5.pose.orientation.y = 0.0
        waypoint5.pose.orientation.z = -0.8695258986554601
        waypoint5.pose.orientation.w = 0.49388734704122006
        waypoints.append(waypoint5)

        # 여섯 번째 웨이포인트
        waypoint6 = PoseStamped()
        waypoint6.header.stamp.sec = 0
        waypoint6.header.stamp.nanosec = 0
        waypoint6.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint6.pose.position.x = -1.4535345072330572
        waypoint6.pose.position.y = -0.18332471336173378
        waypoint6.pose.position.z = 0.0
        waypoint6.pose.orientation.x = 0.0
        waypoint6.pose.orientation.y = 0.0
        waypoint6.pose.orientation.z = -0.8169457047949036
        waypoint6.pose.orientation.w = 0.5767145874842756
        waypoints.append(waypoint6)

        # FollowWaypoints 액션 목표 생성 및 전송
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        # 액션 서버 준비 대기
        self.get_logger().info('Waiting for Action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is ready.')

        # 목표 전송 및 피드백 콜백 설정
        self.get_logger().info('Sending waypoints...')
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
        self.get_logger().info(f'Received feedback: Current Waypoint Index = {feedback.current_waypoint}')

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
            self.get_logger().info('Mission complete. Node will now shut down.')
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from std_msgs.msg import String
import math
import time


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # 초기 위치를 퍼블리시하기 위한 퍼블리셔 생성
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # FollowWaypoints 액션 클라이언트 생성
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        # /finde 토픽 퍼블리셔 생성
        self.finde_publisher = self.create_publisher(String, '/finde', 10)

        # Trigger 토픽 구독 ("/trigger_topic"에서 메시지를 기다림)
        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',  # Trigger 메시지를 받을 토픽 이름
            self.trigger_callback,  # 메시지가 수신될 때 호출될 콜백 함수
            10  # 큐 사이즈
        )
        self.get_logger().info('Waiting for trigger message "a" on /trigger_topic...')

    def trigger_callback(self, msg):
        """Trigger 메시지를 처리하는 콜백 함수"""
        if msg.data.lower() == 'a':  # 메시지가 'a'인지 확인
            self.get_logger().info('Received trigger message "a". Setting up navigation...')
            self.publish_initial_pose()  # 초기 위치 퍼블리시
            time.sleep(2)  # 초기 위치 설정 후 대기
            self.get_logger().info('Waiting for navigation to be ready...')
            self.wait_for_navigation_ready()  # 네비게이션 준비 대기
            self.send_goal()  # 웨이포인트 목표 전송
        else:
            self.get_logger().warn(f'Ignored message: {msg.data}')

    def publish_initial_pose(self):
        """초기 위치를 설정하고 퍼블리시"""
        self.get_logger().info('Publishing initial pose...')
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'  # 초기 위치 기준 프레임 설정
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # 초기 위치 좌표 및 방향 설정
        initial_pose.pose.pose.position.x = 0.026354703738017037
        initial_pose.pose.pose.position.y = -0.023555173715895102
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.07846075992136109,
            w=0.9969172027568601
        )

        # Covariance 행렬 설정
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # 퍼블리시
        self.initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published successfully.')

    def wait_for_navigation_ready(self):
        """네비게이션이 준비될 때까지 대기"""
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available. Is navigation running?')
            return
        self.get_logger().info('FollowWaypoints action server is ready.')

    def send_goal(self):
        # 여러 웨이포인트 정의
        waypoints = []

        # 첫 번째 웨이포인트
        waypoint1 = PoseStamped()
        waypoint1.header.stamp.sec = 0
        waypoint1.header.stamp.nanosec = 0
        waypoint1.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint1.pose.position.x = 0.20045082073635237
        waypoint1.pose.position.y = -0.11526315559203755
        waypoint1.pose.position.z = 0.0
        waypoint1.pose.orientation.x = 0.0
        waypoint1.pose.orientation.y = 0.0
        waypoint1.pose.orientation.z = -0.5501947083050299
        waypoint1.pose.orientation.w = 0.835036396184707
        waypoints.append(waypoint1)

        # 두 번째 웨이포인트
        waypoint2 = PoseStamped()
        waypoint2.header.stamp.sec = 0
        waypoint2.header.stamp.nanosec = 0
        waypoint2.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint2.pose.position.x = 0.2885849106796945
        waypoint2.pose.position.y = -0.7729320617606924
        waypoint2.pose.position.z = 0.0
        waypoint2.pose.orientation.x = 0.0
        waypoint2.pose.orientation.y = 0.0
        waypoint2.pose.orientation.z = -0.998553042937628
        waypoint2.pose.orientation.w = 0.053775649136049555
        waypoints.append(waypoint2)

        # 세 번째 웨이포인트
        waypoint3 = PoseStamped()
        waypoint3.header.stamp.sec = 0
        waypoint3.header.stamp.nanosec = 0
        waypoint3.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint3.pose.position.x = -0.43766810834163267
        waypoint3.pose.position.y = -0.6090071044161587
        waypoint3.pose.position.z = 0.0
        waypoint3.pose.orientation.x = 0.0
        waypoint3.pose.orientation.y = 0.0
        waypoint3.pose.orientation.z = 0.9647924028389457
        waypoint3.pose.orientation.w = 0.2630125841556892
        waypoints.append(waypoint3)

        # 네 번째 웨이포인트
        waypoint4 = PoseStamped()
        waypoint4.header.stamp.sec = 0
        waypoint4.header.stamp.nanosec = 0
        waypoint4.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint4.pose.position.x = -1.024580178661337
        waypoint4.pose.position.y = -0.21175594593394653
        waypoint4.pose.position.z = 0.0
        waypoint4.pose.orientation.x = 0.0
        waypoint4.pose.orientation.y = 0.0
        waypoint4.pose.orientation.z = 0.9813011653381158
        waypoint4.pose.orientation.w = 0.19247862973861757
        waypoints.append(waypoint4)

        # 다섯 번째 웨이포인트
        waypoint5 = PoseStamped()
        waypoint5.header.stamp.sec = 0
        waypoint5.header.stamp.nanosec = 0
        waypoint5.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint5.pose.position.x = -1.4703407287627268
        waypoint5.pose.position.y = -0.292804214493149
        waypoint5.pose.position.z = 0.0
        waypoint5.pose.orientation.x = 0.0
        waypoint5.pose.orientation.y = 0.0
        waypoint5.pose.orientation.z = -0.6196984326209984
        waypoint5.pose.orientation.w = 0.784840017205467
        waypoints.append(waypoint5)

        # 다섯 번째 웨이포인트
        waypoint6 = PoseStamped()
        waypoint6.header.stamp.sec = 0
        waypoint6.header.stamp.nanosec = 0
        waypoint6.header.frame_id = "map"  # 프레임 ID를 설정
        waypoint6.pose.position.x = -1.5726163715771126
        waypoint6.pose.position.y = -0.3743943690241698
        waypoint6.pose.position.z = 0.0
        waypoint6.pose.orientation.x = 0.0
        waypoint6.pose.orientation.y = 0.0
        waypoint6.pose.orientation.z = -0.7126831075538593
        waypoint6.pose.orientation.w = 0.7014861283071635
        waypoint6.append(waypoint6)


        # FollowWaypoints 액션 목표 생성 및 전송
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        # 서버 연결 대기
        self.action_client.wait_for_server()

        # 목표 전송 및 피드백 콜백 설정
        self.get_logger().info('Sending goal with waypoints...')

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """액션 서버로부터의 응답 처리"""
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
        """웨이포인트 진행 상황 처리"""
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f'Moving towards Waypoint {current_waypoint}')

    def get_result_callback(self, future):
        """최종 결과 처리 및 종료"""
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
            # /finde 토픽에 'c' 메시지 퍼블리시
            self.publish_finde_message()
            self.get_logger().info('Mission complete. Node will now shut down.')
            if rclpy.ok():  # rclpy가 활성 상태인지 확인
                self.destroy_node()
                rclpy.shutdown()

    def publish_finde_message(self):
        """/finde 토픽에 'c' 메시지 퍼블리시"""
        msg = String()
        msg.data = 'c'
        self.finde_publisher.publish(msg)
        self.get_logger().info('Published "c" to /finde topic.')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()




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
            self.publish_finde_message()
            self.safe_shutdown()

    def publish_finde_message(self):
        msg = String()
        msg.data = 'c'
        self.finde_publisher.publish(msg)
        self.get_logger().info('Published "c" to /finde topic.')

    def safe_shutdown(self):
        if not self.is_shutdown:
            if rclpy.ok():
                self.destroy_node()
                rclpy.shutdown()
            self.is_shutdown = True


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Shutting down...')
    finally:
        node.safe_shutdown()


if __name__ == '__main__':
    main()
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

        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',
            self.trigger_callback,
            10
        )
        self.timer = None  # /finde 메시지를 퍼블리시하기 위한 타이머
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
        """/finde 메시지를 주기적으로 발행"""
        self.get_logger().info('Starting periodic publishing of "c" to /finde...')
        self.timer = self.create_timer(2.0, self.publish_finde_message)  # 2초 간격으로 발행

    def publish_finde_message(self):
        """주기적으로 /finde 메시지 발행"""
        msg = String()
        msg.data = 'c'
        self.finde_publisher.publish(msg)
        self.get_logger().info('Published "c" to /finde topic.')

    def safe_shutdown(self):
        if not self.is_shutdown:
            self.get_logger().info('Shutting down node...')
            if self.timer:
                self.timer.cancel()  # 타이머 중지
            if rclpy.ok():
                self.destroy_node()
                rclpy.shutdown()
            self.is_shutdown = True


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Shutting down...')
    finally:
        node.safe_shutdown()


if __name__ == '__main__':
    main()
