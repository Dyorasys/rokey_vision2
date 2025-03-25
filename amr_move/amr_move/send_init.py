import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TriggerPublisher(Node):
    def __init__(self):
        super().__init__('trigger_publisher')
        self.publisher = self.create_publisher(String, '/trigger_topic', 10)
        self.counter = 0  # 메시지 발행 횟수를 추적
        self.timer = self.create_timer(10.0, self.send_trigger)  # 10초에 한 번씩 호출

    def send_trigger(self):
        if self.counter < 3:  # 메시지를 최대 3번만 발행
            msg = String()
            msg.data = 'a'
            self.publisher.publish(msg)
            self.get_logger().info(f'Sent trigger message "a". Count: {self.counter + 1}')
            self.counter += 1
        else:
            self.get_logger().info('Sent trigger message 3 times. Stopping timer.')
            self.timer.cancel()  # 타이머 중지

def main(args=None):
    rclpy.init(args=args)
    time.sleep(3)
    node = TriggerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

