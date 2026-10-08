import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rx_topic = 'TB_Uart_TX'  # subscribe하여 터미널에 출력할 토픽


class SubNode(Node):
    def __init__(self):
        super().__init__('sub_node')
        self.sub = self.create_subscription(String, rx_topic, self.on_rx, 10)
        self.get_logger().info(f'Subscribing to "{rx_topic}"')

    def on_rx(self, msg: String):
        print(f'[{rx_topic}] {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
