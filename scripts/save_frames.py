# save_frames.py
# 사용: python3 save_frames.py --ros-args -p out_dir:=~/dataset_raw/run1 -p interval:=0.5
import os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # 카메라가 best effort로 발행
from sensor_msgs.msg import CompressedImage

class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        self.out = os.path.expanduser(self.declare_parameter('out_dir', '~/dataset_raw').value)
        self.interval = self.declare_parameter('interval', 0.5).value   # 초 단위 저장 간격
        os.makedirs(self.out, exist_ok=True)
        self.last = 0.0
        self.count = 0
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'saving to {self.out} every {self.interval}s')

    def cb(self, msg):
        now = time.time()
        if now - self.last < self.interval:
            return
        self.last = now
        path = os.path.join(self.out, f'{int(now*1000)}.jpg')
        with open(path, 'wb') as f:
            f.write(bytes(msg.data))
        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(f'{self.count} frames saved')

def main():
    rclpy.init()
    rclpy.spin(FrameSaver())

if __name__ == '__main__':
    main()