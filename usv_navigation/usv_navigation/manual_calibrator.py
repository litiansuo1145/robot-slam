import rclpy
from rclpy.node import Node
import serial
import tty
import termios
import sys

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# 【修正】这里的描述是给您看的，按键是您习惯的，但发送的指令是修正后的
key_map = {
    'a': '前进',
    'b': '后退',
    'd': '左转',
    'c': '右转',
    's': '停止'
}
# 【修正】实际发送的命令，已应用反转逻辑
command_map = {
    'a': 'B', # 按'a'前进，发送'B'
    'b': 'A', # 按'b'后退，发送'A'
    'd': 'C', # 按'd'左转，发送'C'
    'c': 'D', # 按'c'右转，发送'D'
    's': '0'
}

def get_key():
    # ... (这个函数不需要修改) ...
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class ManualCalibrator(Node):
    def __init__(self):
        super().__init__('manual_calibrator')
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"成功连接到串口 {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口 {SERIAL_PORT}: {e}")
            sys.exit(1)
        
        self.get_logger().info("手动校准节点已启动 (已应用电机反转修正)。按键盘控制：")
        for key, desc in key_map.items():
            self.get_logger().info(f"  '{key}': {desc}")
        self.get_logger().info("按 'q' 键退出。")

    def run(self):
        while rclpy.ok():
            key = get_key()
            if key == 'q':
                break
            
            if key in command_map:
                command_to_send = command_map[key]
                self.get_logger().info(f"按键 '{key}' -> 发送修正后命令: '{command_to_send}'")
                self.ser.write(command_to_send.encode())
            else:
                stop_command = command_map['s']
                self.get_logger().info(f"无效按键，发送停止命令 '{stop_command}'")
                self.ser.write(stop_command.encode())

def main(args=None):
    rclpy.init(args=args)
    calibrator = ManualCalibrator()
    calibrator.run()
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()