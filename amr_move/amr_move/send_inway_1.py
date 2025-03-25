'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep

class TriggerPublisher(Node):
    def __init__(self):
        super().__init__('trigger_publisher')
        self.publisher = self.create_publisher(String, '/trigger_topic', 10)
        self.timer = self.create_timer(1.0, self.send_trigger)  # Timer to call send_trigger every second

    def send_trigger(self):
        msg = String()
        msg.data = 'a'
        self.publisher.publish(msg)
        self.get_logger().info('Sent trigger message "a".')

def main(args=None):
    rclpy.init(args=args)
    node = TriggerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TriggerPublisher(Node):
    def __init__(self):
        super().__init__('trigger_publisher')
        self.publisher = self.create_publisher(String, '/trigger_topic', 10)
        self.counter = 0  # 메시지 발행 횟수를 추적
        self.max_count = 3  # 최대 발행 횟수
        self.state = 'waiting'  # 현재 상태 ('waiting', 'sending')
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 상태 확인

    def timer_callback(self):
        if self.state == 'waiting':
            # 첫 번째 메시지를 보내기 전에 5초 대기
            if self.counter == 0:
                self.get_logger().info('Waiting 5 seconds before sending the first message...')
                self.state = 'sending'
                self.create_timer(5.0, self.send_trigger)
            elif self.counter < self.max_count:
                self.get_logger().info(f'Waiting 10 seconds before sending the next message {self.counter + 1}...')
                self.state = 'sending'
                self.create_timer(10.0, self.send_trigger)
            else:
                # 모든 메시지 발행 완료 후 종료
                self.get_logger().info('All messages sent. Shutting down...')
                self.destroy_node()
                rclpy.shutdown()

    def send_trigger(self):
        """메시지 발행"""
        msg = String()
        msg.data = 'a'
        self.publisher.publish(msg)
        self.counter += 1
        self.get_logger().info(f'Sent trigger message "a". Count: {self.counter}')
        self.state = 'waiting'


def main(args=None):
    rclpy.init(args=args)
    node = TriggerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


