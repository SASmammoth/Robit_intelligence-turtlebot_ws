import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ---- Topic 이름 상수 ----
TX_TOPIC = 'TB_Uart_RX'   # 발행 (노드 -> 로봇)
RX_TOPIC = 'TB_Uart_TX'   # 수신 (로봇 -> 노드)

# ---- 메시지 상수 ----
MSG_START = 'start'
MSG_QUIT = 'quit'
MSG_DIRECTION = 'direction 1 -1'
MSG_STOP = 'velocity 0 0'

KEY_MAP = {
    'w': 'velocity 50 50',
    'a': 'velocity 0 50',
    's': 'velocity -50 -50',
    'd': 'velocity 50 0',
}

# 터미널은 key release 이벤트가 없으므로, 일정 시간 키 입력이 없으면
# "아무 키도 안 눌린 상태"로 간주한다. 타임아웃을 두 단계로 나눈다.
#  - INITIAL_TIMEOUT: 키 반복이 시작되기 전. OS의 반복 시작 지연(보통 0.25~0.66초)보다 길어야 함
#  - REPEAT_TIMEOUT : 키 반복이 확인된 뒤. 반복 주기(보통 30~40ms)보다 약간 길게
INITIAL_TIMEOUT = 0.6
REPEAT_TIMEOUT = 0.1
REPEAT_GAP_MAX = 0.12      # 같은 키 입력 간격이 이 값보다 짧으면 "반복 중"으로 판단
LOOP_PERIOD = 0.005        # 메인 루프에서 키보드를 확인하는 주기
START_RETRY_PERIOD = 1.0   # "start" 재발행 주기 (초)


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.pub = self.create_publisher(String, TX_TOPIC, 10)
        self.sub = self.create_subscription(String, RX_TOPIC, self.rx_callback, 10)

        self.started = False        # "start"를 수신했는지
        self.last_cmd = None        # 마지막으로 발행한 velocity 명령
        self.last_key = None
        self.last_key_time = 0.0
        self.repeating = False      # 키 반복(auto-repeat)이 확인되었는지
        self.quit_requested = False

        self.fd = sys.stdin.fileno()
        self.old_attr = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        # 노드 시작 시 "start" 발행, 응답이 없으면 1초마다 재발행
        self.publish(MSG_START)
        self.start_timer = self.create_timer(START_RETRY_PERIOD, self.retry_start)

    def publish(self, text):
        self.pub.publish(String(data=text))
        self.get_logger().info(f'TX: {text}')

    def retry_start(self):
        if self.started:
            self.start_timer.cancel()
            return
        self.publish(MSG_START)

    def rx_callback(self, msg):
        self.get_logger().info(f'RX: {msg.data}')
        if self.started:
            return
        if msg.data.strip() == MSG_START:
            self.started = True
            self.start_timer.cancel()
            self.publish(MSG_DIRECTION)
            termios.tcflush(self.fd, termios.TCIFLUSH)  # 대기 중 입력 버림
            # 초기 상태는 "정지"로 간주 -> start 직후 "velocity 0 0"은 발행하지 않음
            self.last_cmd = MSG_STOP

    def read_keys(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if not ch:
                break
            keys.append(ch.lower())
        return keys

    def poll_keyboard(self):
        keys = self.read_keys()
        if not self.started:
            return  # start 수신 전에는 키 입력 무시

        now = time.monotonic()
        for k in keys:
            if k == 'q':
                self.publish(MSG_QUIT)
                self.quit_requested = True
                return
            if k in KEY_MAP:
                same_key = (k == self.last_key)
                self.repeating = same_key and (now - self.last_key_time) < REPEAT_GAP_MAX
                self.last_key = k
                self.last_key_time = now

        if self.last_key is not None:
            timeout = REPEAT_TIMEOUT if self.repeating else INITIAL_TIMEOUT
            if now - self.last_key_time > timeout:
                self.last_key = None
                self.repeating = False

        cmd = KEY_MAP.get(self.last_key, MSG_STOP)
        if cmd != self.last_cmd:  # 입력이 바뀔 때만 발행
            self.publish(cmd)
            self.last_cmd = cmd

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_attr)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=LOOP_PERIOD)
            node.poll_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()