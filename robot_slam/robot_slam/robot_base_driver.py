import rclpy
from rclpy.node import Node
from std_msgs.msg import String # 我们现在使用更简单的String消息来发送指令
import serial
import threading
import time

class RobotBaseDriverNode(Node):
    def __init__(self):
        super().__init__('robot_base_driver_node')
        self.get_logger().info("Advanced Robot Base Driver Node Started")

        # --- 参数 ---
        self.declare_parameter('serial_port', '/dev/ttyACM0') # 使用您确认的正确端口
        self.declare_parameter('baud_rate', 115200)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # --- 串口初始化 ---
        try:
            # 添加 dtr=False 来尝试禁用自动复位
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1.0, dtr=False)
            self.get_logger().info(f"Successfully connected to serial port {serial_port}.")
            # 即使禁用了DTR，也保留一个小的延时以确保稳定
            time.sleep(1.0)
            # 初始设置
            self.serial_connection.write(b'S\r\n') # 设置当前航向
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            rclpy.shutdown()
            return

        # --- ROS 接口 ---
        # 订阅指令话题
        self.command_sub = self.create_subscription(String, '/robot_command', self.command_callback, 10)
        # 发布来自单片机的状态数据
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # --- 启动独立的串口读取线程 ---
        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.get_logger().info("Driver is ready.")

    def command_callback(self, msg: String):
        """当收到指令时，通过串口发送"""
        command = msg.data
        try:
            # 您的协议似乎需要 \r\n 结尾
            self.serial_connection.write((command + '\r\n').encode('utf-8'))
            self.get_logger().info(f'Sent command: "{command}"')
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")
    
    def serial_read_loop(self):
        """在一个独立的线程中持续读取来自单片机的数据并发布"""
        while rclpy.ok():
            try:
                # 读取一行数据，直到遇到 \n
                line = self.serial_connection.readline().decode('utf-8').strip()
                if line: # 如果读到了非空行
                    # 直接将单片机发来的原始状态字符串发布出去
                    status_msg = String()
                    status_msg.data = line
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f'Received from MCU: "{line}"')
            except Exception as e:
                # 忽略偶尔的解析错误
                pass

    def on_shutdown(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write('0\r\n'.encode('utf-8')) # 发送停止指令
            self.get_logger().info("Sent final stop command '0' and closing serial port.")
            self.serial_connection.close()

def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()