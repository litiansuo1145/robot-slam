import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('base_controller_node')
        self.get_logger().info("Base Controller Node Started")

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1.0)
            self.get_logger().info(f"Successfully connected to serial port {serial_port} at {baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port {serial_port}: {e}")
            self.serial_connection = None
            return

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info("Ready to receive velocity commands.")

    def cmd_vel_callback(self, msg: Twist):
        if self.serial_connection is None:
            self.get_logger().warn("Serial port not available. Cannot send command.")
            return

        # --- 关键修改在这里 ---
        command = '0' # 默认命令是停止 (Stop), 根据您的发现修改为 '0'

        if msg.linear.x > 0.1:
            command = 'A'
        elif msg.linear.x < -0.1:
            command = 'B'
        elif msg.angular.z > 0.1:
            command = 'C'
        elif msg.angular.z < -0.1:
            command = 'D'
        
        try:
            self.serial_connection.write(command.encode('utf-8'))
            self.get_logger().info(f'Received Twist command -> Sent serial command: "{command}"')
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")

    def on_shutdown(self):
        if self.serial_connection and self.serial_connection.is_open:
            # 发送最后的停止指令，确保安全
            self.serial_connection.write('0'.encode('utf-8'))
            self.get_logger().info("Sent final stop command '0' and closing serial port.")
            self.serial_connection.close()

def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
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