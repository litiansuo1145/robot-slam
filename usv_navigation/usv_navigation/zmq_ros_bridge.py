import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import zmq
import json

class ZMQToROSBridge(Node):
    def __init__(self):
        super().__init__('zmq_ros_bridge')
        
        # 1. 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.brush_pub = self.create_publisher(Bool, '/brush_cmd', 10)
        
        # 2. 初始化 ZeroMQ 接收端
        # 【修改点】这里改名为 self.zmq_context，避免和 ROS2 Node 的 self.context 冲突
        self.zmq_context = zmq.Context()
        
        # 使用 SUB 模式订阅消息
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.socket.bind("tcp://*:5556") # 绑定端口
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
        
        self.get_logger().info("ZMQ Bridge Started. Listening on port 5556...")

        # 3. 使用定时器定期检查 ZMQ 消息
        self.timer = self.create_timer(0.01, self.check_zmq_messages) # 100Hz 检查

    def check_zmq_messages(self):
        try:
            # 非阻塞接收
            msg_bytes = self.socket.recv(flags=zmq.NOBLOCK)
            data = json.loads(msg_bytes.decode('utf-8'))
            
            # --- 处理移动指令 ---
            if 'linear' in data and 'angular' in data:
                twist = Twist()
                twist.linear.x = float(data['linear'])
                twist.angular.z = float(data['angular'])
                self.cmd_vel_pub.publish(twist)
                # 调试时可以取消下面这行的注释
                self.get_logger().info(f"Pub Move: {data}")

            # --- 处理滚刷指令 ---
            if 'brush' in data:
                bool_msg = Bool()
                bool_msg.data = bool(data['brush'])
                self.brush_pub.publish(bool_msg)
                self.get_logger().info(f"Pub Brush: {bool_msg.data}")

        except zmq.Again:
            pass # 没收到消息，跳过
        except Exception as e:
            self.get_logger().error(f"Error processing ZMQ: {e}")

    def destroy_node(self):
        # 关闭时清理 ZMQ 资源
        self.socket.close()
        self.zmq_context.term()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZMQToROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()