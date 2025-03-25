'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import threading
import sys
import select
import termios
import tty

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_initial_pose(self):
        # Create the initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'  # The frame in which the pose is defined
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # /initialpose

        # position:
        #   x: 0.1750425100326538
        #   y: 0.05808566138148308
        #   z: 0.0
        # orientation:
        #   x: 0.0
        #   y: 0.0
        #   z: -0.04688065682721989
        #   w: 0.9989004975549108
  


        # Set the position (adjust these values as needed)
        initial_pose.pose.pose.position.x = -0.017241953009357745 # X-coordinate
        initial_pose.pose.pose.position.y = 0.030906322294970673 # Y-coordinate
        initial_pose.pose.pose.position.z = 0.0  # Z should be 0 for 2D navigation

        # Set the orientation (in quaternion form)
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.001727534551818488,  # 90-degree rotation in yaw (example)
            w=0.9999985078110728  # Corresponding quaternion w component
        )

        # covariance:
        #   - 0.25
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.25
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.06853891909122467

        # Set the covariance values for the pose estimation
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]


        # Publish the initial pose
        self.publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

        # Destroy the node and shutdown rclpy
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'i':
                    node.get_logger().info('Key "i" pressed. Publishing initial pose...')
                    node.publish_initial_pose()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    # Start the keyboard listener in a separate thread
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)


if __name__ == '__main__':
    main()


#---------------------------------------------------------------------------------------------------
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import String


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.trigger_publisher = self.create_publisher(String, '/status_topic', 10)  # 새로운 토픽 생성
        
        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',
            self.trigger_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.timer = None  # Timer를 초기화

    def trigger_callback(self, msg):
        if msg.data == 'a':
            self.get_logger().info('Received trigger message "a". Publishing initial pose...')
            self.publish_initial_pose()
            # 타이머를 시작하여 반복적으로 'b' 메시지를 보냄
            if self.timer is None:
                self.start_status_timer()

    def publish_initial_pose(self):
        # Create the initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # Set the position
        initial_pose.pose.pose.position.x = -0.017241953009357745
        initial_pose.pose.pose.position.y = 0.030906322294970673
        initial_pose.pose.pose.position.z = 0.0

        # Set the orientation (in quaternion form)
        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.001727534551818488,
            w=0.9999985078110728
        )

        # Set the covariance values for the pose estimation
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # Publish the initial pose
        self.initialpose_publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

    def start_status_timer(self):
        # 타이머를 1초 간격으로 설정 (간격은 필요에 따라 조정 가능)
        self.timer = self.create_timer(1.0, self.publish_status_message)

    def publish_status_message(self):
        # Create and publish the 'b' message
        status_msg = String()
        status_msg.data = 'b'
        self.trigger_publisher.publish(status_msg)
        self.get_logger().info('Status message "b" published on /status_topic.')
    

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


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
            self.send_waypoints()

    def publish_initial_pose(self):
        # 초기 위치를 설정하고 발행
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
        self.get_logger().info('Initial pose published.')

    def send_waypoints(self):
        # 웨이포인트 정의
        waypoints = []

        # 첫 번째 웨이포인트
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = "map"
        waypoint1.pose.position.x = 0.2914209932659453
        waypoint1.pose.position.y = -0.035827083389437166
        waypoint1.pose.orientation.z = -0.5397640093892747
        waypoint1.pose.orientation.w = 0.8418163779399964
        waypoints.append(waypoint1)

        # 두 번째 웨이포인트
        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = "map"
        waypoint2.pose.position.x = 0.3369757869918463
        waypoint2.pose.position.y = -0.5098393486385897
        waypoint2.pose.orientation.z = -0.933249104361796
        waypoint2.pose.orientation.w = 0.35922988351180585
        waypoints.append(waypoint2)

        # 세 번째 웨이포인트
        waypoint3 = PoseStamped()
        waypoint3.header.frame_id = "map"
        waypoint3.pose.position.x = -0.7423834107933234
        waypoint3.pose.position.y = -0.7337036149646982
        waypoint3.pose.orientation.z = 0.8352340850356917
        waypoint3.pose.orientation.w = 0.54989455643295
        waypoints.append(waypoint3)

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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')
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
'''
