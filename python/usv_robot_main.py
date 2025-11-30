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
# 两个 USB 摄像头的 ID (请通过 ls -l /dev/video* 确认)
# 通常 video0 是第一个，video2 是第二个 (video1 往往是 video0 的元数据)
USB_CAM0_ID = 0  
USB_CAM1_ID = 2  

# 分辨率 (双USB建议不要太高，640x480 最稳定，1280x720 取决于摄像头性能)
CAM_WIDTH = 640
CAM_HEIGHT = 480

VIDEO_PORT = 5555
CTRL_PORT = 5556
# =========================================

class DualCameraSender(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.running = True
        
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 5)
        self.socket.bind(f"tcp://*:{VIDEO_PORT}")
        
        # 初始化两个摄像头
        print("[Video] 正在初始化双 USB 摄像头...")
        self.cam0 = self.open_usb_cam(USB_CAM0_ID, "Front (Cam0)")
        self.cam1 = self.open_usb_cam(USB_CAM1_ID, "Rear (Cam1)")

    def open_usb_cam(self, dev_id, name):
        """通用 USB 摄像头打开函数 (强制 MJPG)"""
        try:
            # 使用 V4L2 后端
            cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
            
            # 【关键】设置 MJPG 格式，节省 USB 带宽
            # 如果不加这行，两个摄像头同时开大概率会报错
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            if cap.isOpened():
                # 尝试读一帧
                ret, _ = cap.read()
                if ret:
                    print(f"[Video] {name} 启动成功 (ID: {dev_id})")
                    return cap
                else:
                    print(f"[Warn] {name} 打开了但读不到数据")
                    cap.release()
            else:
                print(f"[Warn] {name} 无法打开 (ID: {dev_id})")
        except Exception as e:
            print(f"[Error] 初始化 {name} 失败: {e}")
        return None

    def run(self):
        print(f"[Video] 图传服务运行中...")
        while self.running:
            # --- 读取 Cam 0 ---
            if self.cam0 and self.cam0.isOpened():
                ret0, frame0 = self.cam0.read()
                if ret0:
                    self.send_frame('cam0', frame0)
                else:
                    # 如果掉线，尝试重连逻辑可以加在这里，暂时略过
                    pass

            # --- 读取 Cam 1 ---
            if self.cam1 and self.cam1.isOpened():
                ret1, frame1 = self.cam1.read()
                if ret1:
                    self.send_frame('cam1', frame1)
            
            # 控制帧率，防止 CPU 满载
            time.sleep(0.015) 

    def send_frame(self, topic, frame):
        try:
            # 压缩发送
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            jpg_as_text = base64.b64encode(buffer)
            self.socket.send_multipart([topic.encode('utf-8'), jpg_as_text])
        except:
            pass

    def stop(self):
        self.running = False
        if self.cam0 and self.cam0.isOpened(): self.cam0.release()
        if self.cam1 and self.cam1.isOpened(): self.cam1.release()
        self.zmq_context.term()

# --- ROS2 节点 (保持不变，只为了完整性列出) ---
class RobotControlNode(Node):
    def __init__(self, video_socket):
        super().__init__('usv_main_node')
        self.video_socket = video_socket

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.brush_pub = self.create_publisher(Bool, '/brush_cmd', 10)
        
        # QoS: Transient Local 用于地图
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
        self.get_logger().info(f"[Control] ROS2 双USB节点就绪")

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