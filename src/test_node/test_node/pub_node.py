import rclpy
import yaml
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message

default_topic = 'TB_Uart_RX'          # 터미널 입력을 publish할 토픽 (파라미터 topic)
default_type = 'std_msgs/msg/String'  # 메시지 타입 (파라미터 type)


class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')
        # 실행 시 --ros-args -p topic:=/brain/run -p type:=std_msgs/msg/String 처럼 변경
        self.topic = self.declare_parameter('topic', default_topic).value
        self.type_name = self.declare_parameter('type', default_type).value
        self.msg_type = get_message(self.type_name)
        self.pub = self.create_publisher(self.msg_type, self.topic, 10)
        self.get_logger().info(f'Publishing terminal input to "{self.topic}" ({self.type_name})')

    def run(self):
        while rclpy.ok():
            try:
                text = input()
            except EOFError:
                break
            msg = self.msg_type()
            try:
                if self.type_name == 'std_msgs/msg/String':
                    msg.data = text  # String은 입력 그대로
                else:
                    # 그 외 타입은 YAML로 입력. 예) {data: true}, {detections: [{class_id: 7}]}
                    set_message_fields(msg, yaml.safe_load(text) or {})
            except Exception as e:
                self.get_logger().warn(f'invalid input: {e}')
                continue
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
