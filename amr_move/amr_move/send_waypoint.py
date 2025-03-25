
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String  # Ensure String is imported
import math
import threading
import sys
import select
import termios
import tty



class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # Subscribe to /trigger_topic
        self.subscription = self.create_subscription(
            String,
            '/trigger_topic',
            self.trigger_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def trigger_callback(self, msg):
        if msg.data == 'b':
            self.get_logger().info('Received trigger message "b". Sending waypoints...')
            self.send_goal()

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Attempting to cancel the goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    # def cancel_done_callback(self, future):
    #     cancel_response = future.result()
    #     if cancel_response.accepted:
    #         self.get_logger().info('Goal cancellation accepted. Exiting program...')
    #         self.destroy_node()
    #         rclpy.shutdown()
    #         sys.exit(0)  # Exit the program after successful cancellation
    #     else:
    #         self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info('Goal cancellation accepted. Exiting program...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)  # Exit the program after successful cancellation
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'g':
                    node.get_logger().info('Key "g" pressed. Sending goal...')
                    node.send_goal()
                elif key.lower() == 's':
                    node.get_logger().info('Key "s" pressed. Cancelling goal...')
                    node.cancel_goal()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

'''
#def main(args=None):
#    rclpy.init(args=args)
#    node = WaypointFollower()
#    rclpy.spin(node)
#    node.destroy_node()
#    rclpy.shutdown()
'''

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()
    
    rclpy.spin(node)


if __name__ == '__main__':
    main()
'''



import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
import math
import threading
import sys
import select
import termios
import tty

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

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

        # 서버 연결 대기
        self.action_client.wait_for_server()

        self.get_logger().info('Sending goal with waypoints...')
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
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

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'g':
                    node.get_logger().info('Key "g" pressed. Sending goal...')
                    node.send_goal()
                elif key.lower() == 's':
                    node.get_logger().info('Key "s" pressed. Stopping node...')
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)
    node.destroy_node()
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

    def send_waypoints(self):
        # 웨이포인트 정의
        self.get_logger().info('Preparing waypoints...')
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
'''