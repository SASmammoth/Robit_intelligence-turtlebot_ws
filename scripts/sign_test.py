# YOLO26 ONNX 테스트: /camera/image_raw/compressed → 검출 → /sign/debug/compressed
import ast, os, time
import cv2, numpy as np, onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class SignTest(Node):
    def __init__(self):
        super().__init__('sign_test')
        model = self.declare_parameter('model', os.path.expanduser('~/models/sign_v1.onnx')).value
        self.conf = self.declare_parameter('conf', 0.5).value
        self.sess = ort.InferenceSession(model, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0]
        self.size = self.inp.shape[2]
        meta = self.sess.get_modelmeta().custom_metadata_map
        self.names = ast.literal_eval(meta['names']) if 'names' in meta else {}
        self.get_logger().info(f'input {self.inp.shape}, classes {self.names}')
        self.pub = self.create_publisher(CompressedImage, '/sign/debug/compressed', 1)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.cb, 1)
        self.first = True

    def letterbox(self, img):
        h, w = img.shape[:2]
        r = min(self.size / h, self.size / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        px, py = (self.size - nw) // 2, (self.size - nh) // 2
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[py:py+nh, px:px+nw] = cv2.resize(img, (nw, nh))
        return canvas, r, px, py


    def decode(self, out):
        o = out[0]
        if o.ndim == 2 and o.shape[-1] == 6:          # end-to-end: [N, 6] x1 y1 x2 y2 score cls
            return [d for d in o if d[4] >= self.conf]
        if o.shape[0] < o.shape[1]:                   # [4+nc, 8400] -> [8400, 4+nc]
            o = o.T
        scores = o[:, 4:]
        cls = scores.argmax(1)
        conf = scores.max(1)
        keep = conf >= self.conf
        b, cls, conf = o[keep, :4], cls[keep], conf[keep]
        boxes = [[float(cx - w/2), float(cy - h/2), float(w), float(h)] for cx, cy, w, h in b]
        idx = cv2.dnn.NMSBoxes(boxes, conf.tolist(), self.conf, 0.5)
        res = []
        for i in np.array(idx).flatten():
            x, y, w, h = boxes[i]
            res.append((x, y, x + w, y + h, conf[i], cls[i]))
        return res

    def cb(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        lb, r, px, py = self.letterbox(img)
        x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        t = time.time()
        out = self.sess.run(None, {self.inp.name: x})[0]
        dt = (time.time() - t) * 1000
        if self.first:
            self.get_logger().info(f'output shape {out.shape}')
            self.first = False
        for x1, y1, x2, y2, sc, c in self.decode(out):
            x1, x2 = (x1 - px) / r, (x2 - px) / r
            y1, y2 = (y1 - py) / r, (y2 - py) / r
            label = f'{self.names.get(int(c), int(c))} {sc:.2f}'
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, label, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, f'{dt:.0f} ms', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        out_msg = CompressedImage(header=msg.header, format='jpeg')
        out_msg.data = cv2.imencode('.jpg', img)[1].tobytes()
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(SignTest())

if __name__ == '__main__':
    main()
