import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # 【新增】引入布尔消息类型
import serial
import time

# ==================== 配置区 ====================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 物理参数 ---
WHEEL_BASE = 0.25  
SPEED_TO_PWM_SCALE = 700.0  

# 电机PWM范围
PWM_MIN = 1100
PWM_NEUTRAL = 1500
PWM_MAX = 1900
# ==============================================

class USVController(Node):
    def __init__(self):
        super().__init__('usv_controller')
        
        # 1. 订阅移动指令 /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # 2. 【新增】订阅滚刷控制指令 /brush_cmd
        # 发送 True 开启，发送 False 关闭
        self.brush_subscription = self.create_subscription(
            Bool,
            '/brush_cmd',
            self.brush_callback,
            10)
        
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"成功连接到串口 {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口 {SERIAL_PORT}: {e}")
            rclpy.shutdown()
        
        self.last_sent_command = ""
        self.get_logger().info("USV控制器就绪: /cmd_vel (移动), /brush_cmd (滚刷)")

    # --- 【新增】滚刷回调函数 ---
    def brush_callback(self, msg):
        if msg.data:
            # 收到 True -> 发送开启指令 'O'
            command = "O\n"
            log_msg = "发送指令: 滚刷开启 (ON)"
        else:
            # 收到 False -> 发送关闭指令 'C'
            command = "C\n"
            log_msg = "发送指令: 滚刷关闭 (OFF)"
            
        try:
            self.ser.write(command.encode())
            self.get_logger().info(log_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"串口发送失败: {e}")

    # --- 移动回调函数 (保持不变) ---
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 计算 PWM
        drive_pwm_diff = linear_x * SPEED_TO_PWM_SCALE
        turn_pwm_diff = (angular_z * WHEEL_BASE / 2.0) * SPEED_TO_PWM_SCALE
        
        left_pwm_base = PWM_NEUTRAL + drive_pwm_diff - turn_pwm_diff
        right_pwm_base = PWM_NEUTRAL + drive_pwm_diff + turn_pwm_diff
        
        final_left_pwm = 3000 - left_pwm_base
        final_right_pwm = 3000 - right_pwm_base
        
        final_left_pwm = max(PWM_MIN, min(PWM_MAX, int(final_left_pwm)))
        final_right_pwm = max(PWM_MIN, min(PWM_MAX, int(final_right_pwm)))

        command = f"M,{final_left_pwm},{final_right_pwm}\n"

        if command != self.last_sent_command:
            try:
                self.ser.write(command.encode())
                # 只有PWM变化很大时才打印，避免日志刷屏，或者保持你原来的逻辑
                self.get_logger().info(f"CmdVel -> PWM(L:{final_left_pwm}, R:{final_right_pwm})")
                self.last_sent_command = command
            except serial.SerialException as e:
                self.get_logger().error(f"串口发送失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    usv_controller = USVController()
    rclpy.spin(usv_controller)
    usv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()