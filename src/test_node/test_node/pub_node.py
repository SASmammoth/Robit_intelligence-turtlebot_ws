import rclpy
from rclpy.node import Node
from std_msgs.msg import String

tx_topic = 'TB_Uart_RX'  # 터미널 입력을 publish할 토픽


class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')
        self.pub = self.create_publisher(String, tx_topic, 10)
        self.get_logger().info(f'Publishing terminal input to "{tx_topic}"')

    def run(self):
        while rclpy.ok():
            try:
                text = input()
            except EOFError:
                break
            msg = String()
            msg.data = text
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
