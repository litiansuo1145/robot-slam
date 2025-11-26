import os
import time

# 环境配置
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import cv2
import zmq
import base64
import json
import threading
import numpy as np
import subprocess

# ================= 配置区 =================
CSI_SENSOR_ID = 0
DISPLAY_WIDTH = 1920   
DISPLAY_HEIGHT = 1080
USB_CHECK_INTERVAL = 2.0 

VIDEO_PORT = 5555
CTRL_PORT = 5556
# =========================================

def gstreamer_pipeline(sensor_id=0, capture_width=1920, capture_height=1080, display_width=1920, display_height=1080, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=1"
        % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
    )

class DualCameraSender(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.running = True
        
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 5)
        self.socket.bind(f"tcp://*:{VIDEO_PORT}")
        
        # CSI
        print(f"[Video] 正在启动 CSI...")
        pipeline = gstreamer_pipeline(sensor_id=CSI_SENSOR_ID, display_width=DISPLAY_WIDTH, display_height=DISPLAY_HEIGHT)
        self.cam_csi = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        # USB
        self.cam_usb = None
        self.last_usb_check_time = 0

    def scan_and_open_usb(self):
        """自动扫描 USB 摄像头"""
        # 尝试 ID 1 到 4
        for idx in range(1, 5):
            try:
                # 使用 V4L2 后端
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        print(f"[Video] 成功找到 USB 摄像头 -> /dev/video{idx}")
                        return cap
                    else:
                        cap.release()
            except:
                pass
        return None

    def run(self):
        print(f"[Video] 图传服务已就绪")
        while self.running:
            # CSI
            if self.cam_csi.isOpened():
                ret_csi, frame_csi = self.cam_csi.read()
                if ret_csi:
                    self.send_frame('cam0', frame_csi)
            
            # USB (自动重连 + 自动扫描)
            current_time = time.time()
            if self.cam_usb is None:
                if current_time - self.last_usb_check_time > USB_CHECK_INTERVAL:
                    self.cam_usb = self.scan_and_open_usb() # 使用自动扫描函数
                    self.last_usb_check_time = current_time
            else:
                ret_usb, frame_usb = self.cam_usb.read()
                if ret_usb:
                    self.send_frame('cam1', frame_usb)
                else:
                    self.cam_usb.release()
                    self.cam_usb = None # 断开后重置，下次循环会触发重新扫描
            
            time.sleep(0.01)

    def send_frame(self, topic, frame):
        try:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 35])
            jpg_as_text = base64.b64encode(buffer)
            self.socket.send_multipart([topic.encode('utf-8'), jpg_as_text])
        except:
            pass

    def stop(self):
        self.running = False
        if self.cam_csi.isOpened(): self.cam_csi.release()
        if self.cam_usb is not None: self.cam_usb.release()
        self.zmq_context.term()

class RobotControlNode(Node):
    def __init__(self, video_socket):
        super().__init__('usv_main_node')
        self.video_socket = video_socket

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.brush_pub = self.create_publisher(Bool, '/brush_cmd', 10)
        
        # QoS: Transient Local
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        self.zmq_ctrl_context = zmq.Context()
        self.ctrl_socket = self.zmq_ctrl_context.socket(zmq.SUB)
        self.ctrl_socket.bind(f"tcp://*:{CTRL_PORT}")
        self.ctrl_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        
        self.create_timer(0.01, self.check_zmq_ctrl)
        # 修改日志标记版本
        self.get_logger().info(f"[Control] ROS2 节点就绪 (AutoUSB + Map v3)")

    def map_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            img = np.zeros((height, width), dtype=np.uint8)
            img.fill(128) 
            img[data == 0] = 255
            img[data == 100] = 0
            
            img = cv2.flip(img, 0)
            
            _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
            jpg_as_text = base64.b64encode(buffer)
            
            self.video_socket.send_multipart([b'map', jpg_as_text])
        except:
            pass

    def check_zmq_ctrl(self):
        try:
            msg = self.ctrl_socket.recv(flags=zmq.NOBLOCK)
            data = json.loads(msg.decode('utf-8'))
            
            if 'linear' in data:
                twist = Twist()
                twist.linear.x = float(data['linear'])
                twist.angular.z = float(data['angular'])
                self.cmd_vel_pub.publish(twist)

            if 'brush' in data:
                b_msg = Bool()
                b_msg.data = bool(data['brush'])
                self.brush_pub.publish(b_msg)
            
            if 'action' in data and data['action'] == 'save_map':
                self.save_map_file()

        except zmq.Again:
            pass
        except:
            pass

    def save_map_file(self):
        timestamp = int(time.time())
        map_name = f"/home/nvidia/map_{timestamp}"
        self.get_logger().info(f"Saving map: {map_name}")
        cmd = f"ros2 run nav2_map_server map_saver_cli -f {map_name}"
        subprocess.Popen(cmd, shell=True)

    def destroy_node(self):
        self.ctrl_socket.close()
        self.zmq_ctrl_context.term()
        super().destroy_node()

def main(args=None):
    video_thread = DualCameraSender()
    video_thread.start()

    rclpy.init(args=args)
    node = RobotControlNode(video_thread.socket)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        video_thread.stop()
        video_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()