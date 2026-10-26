import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

default_topic = 'TB_Uart_TX'          # subscribe하여 터미널에 출력할 토픽 (파라미터 topic)
default_type = 'std_msgs/msg/String'  # 메시지 타입 (파라미터 type)


class SubNode(Node):
    def __init__(self):
        super().__init__('sub_node')
        # 실행 시 --ros-args -p topic:=/mission/pause -p type:=std_msgs/msg/Bool 처럼 변경
        self.topic = self.declare_parameter('topic', default_topic).value
        self.type_name = self.declare_parameter('type', default_type).value
        self.sub = self.create_subscription(get_message(self.type_name), self.topic, self.on_rx, 10)
        self.get_logger().info(f'Subscribing to "{self.topic}" ({self.type_name})')

    def on_rx(self, msg):
        if self.type_name == 'std_msgs/msg/String':
            print(f'[{self.topic}] {msg.data}')
        else:
            print(f'[{self.topic}] {json.dumps(message_to_ordereddict(msg), ensure_ascii=False)}')


def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
