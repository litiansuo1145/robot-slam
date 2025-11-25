import os
import time

# --- 环境配置 ---
# 1. 解决 SSH 下无显示器报错
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

# 【已修改】关闭调试日志，防止刷屏
# os.environ["OPENCV_LOG_LEVEL"] = "debug"
# os.environ["OPENCV_VIDEOIO_DEBUG"] = "1"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
import zmq
import base64
import json
import threading
import numpy as np

# ================= 配置区 =================
# CSI 摄像头配置 (主摄 - IMX477)
CSI_SENSOR_ID = 0
DISPLAY_WIDTH = 1920   
DISPLAY_HEIGHT = 1080

# USB 摄像头配置 (副摄)
# 之前的调试表明你的 USB 摄像头视频流 ID 应该是 1 (2 是元数据)
USB_CAM_ID = 1        
USB_CHECK_INTERVAL = 2.0 

# 端口
VIDEO_PORT = 5555
CTRL_PORT = 5556
# =========================================

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,   # IMX477 必须匹配 1920x1080
    capture_height=1080,
    display_width=320,
    display_height=240,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=1"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class DualCameraSender(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.running = True
        
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{VIDEO_PORT}")
        
        # --- 1. 初始化 CSI 摄像头 (主) ---
        print(f"[Video] 正在启动 CSI (ID: {CSI_SENSOR_ID})...")
        pipeline = gstreamer_pipeline(
            sensor_id=CSI_SENSOR_ID,
            display_width=DISPLAY_WIDTH, 
            display_height=DISPLAY_HEIGHT
        )
        
        self.cam_csi = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if self.cam_csi.isOpened():
            print("[Video] CSI 摄像头启动成功")
        else:
            print("[Video] CSI 摄像头启动失败 (请检查排线或 nvargus-daemon)")

        # --- 2. 初始化 USB 摄像头变量 (副) ---
        self.cam_usb = None
        self.last_usb_check_time = 0

    def check_and_open_usb(self):
        try:
            cap = cv2.VideoCapture(USB_CAM_ID, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    print(f"[Video] USB 摄像头 (ID {USB_CAM_ID}) 已连接")
                    return cap
                else:
                    cap.release()
        except Exception:
            pass
        return None

    def run(self):
        print(f"[Video] 图传服务已就绪 (端口 {VIDEO_PORT})")
        while self.running:
            # A. CSI
            if self.cam_csi.isOpened():
                ret_csi, frame_csi = self.cam_csi.read()
                if ret_csi:
                    self.send_frame('cam0', frame_csi)
            
            # B. USB (带自动重连)
            current_time = time.time()
            if self.cam_usb is None:
                if current_time - self.last_usb_check_time > USB_CHECK_INTERVAL:
                    self.cam_usb = self.check_and_open_usb()
                    self.last_usb_check_time = current_time
            else:
                ret_usb, frame_usb = self.cam_usb.read()
                if ret_usb:
                    self.send_frame('cam1', frame_usb)
                else:
                    # 静默处理断开，只在重连成功时打印
                    self.cam_usb.release()
                    self.cam_usb = None

            time.sleep(0.01)

    def send_frame(self, topic, frame):
        try:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            jpg_as_text = base64.b64encode(buffer)
            self.socket.send_multipart([topic.encode('utf-8'), jpg_as_text])
        except Exception:
            pass

    def stop(self):
        self.running = False
        if self.cam_csi.isOpened(): self.cam_csi.release()
        if self.cam_usb is not None and self.cam_usb.isOpened(): self.cam_usb.release()
        self.zmq_context.term()

# --- ROS2 节点类 ---
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('usv_main_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.brush_pub = self.create_publisher(Bool, '/brush_cmd', 10)
        
        self.zmq_ctrl_context = zmq.Context()
        self.ctrl_socket = self.zmq_ctrl_context.socket(zmq.SUB)
        self.ctrl_socket.bind(f"tcp://*:{CTRL_PORT}")
        self.ctrl_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        
        self.create_timer(0.01, self.check_zmq_ctrl)
        self.get_logger().info(f"[Control] ROS2 控制节点已就绪 (监听 {CTRL_PORT})")

    def check_zmq_ctrl(self):
        try:
            msg = self.ctrl_socket.recv(flags=zmq.NOBLOCK)
            data = json.loads(msg.decode('utf-8'))
            
            if 'linear' in data and 'angular' in data:
                twist = Twist()
                twist.linear.x = float(data['linear'])
                twist.angular.z = float(data['angular'])
                self.cmd_vel_pub.publish(twist)

            if 'brush' in data:
                b_msg = Bool()
                b_msg.data = bool(data['brush'])
                self.brush_pub.publish(b_msg)
        except zmq.Again:
            pass
        except Exception:
            pass

    def destroy_node(self):
        self.ctrl_socket.close()
        self.zmq_ctrl_context.term()
        super().destroy_node()

def main(args=None):
    video_thread = DualCameraSender()
    video_thread.start()

    rclpy.init(args=args)
    node = RobotControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("正在停止程序...")
    finally:
        video_thread.stop()
        video_thread.join()
        node.destroy_node()
        rclpy.shutdown()
        print("程序已退出")

if __name__ == '__main__':
    main()