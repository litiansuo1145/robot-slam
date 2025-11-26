import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import time
import zmq 

# ==================== 配置区 ====================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 物理参数 ---
WHEEL_BASE = 0.25  
SPEED_TO_PWM_SCALE = 400.0 

# 电机PWM范围
PWM_MIN = 1100
PWM_NEUTRAL = 1500
PWM_MAX = 1900
# ==============================================

class USVController(Node):
    def __init__(self):
        super().__init__('usv_controller')
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.brush_subscription = self.create_subscription(Bool, '/brush_cmd', self.brush_callback, 10)
        
        # ZMQ 发布者 (回传 PWM)
        self.zmq_context = zmq.Context()
        self.pwm_pub = self.zmq_context.socket(zmq.PUB)
        self.pwm_pub.bind("tcp://*:5557") 
        
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"成功连接到串口 {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口 {SERIAL_PORT}: {e}")
            rclpy.shutdown()
        
        self.last_sent_command = ""
        self.get_logger().info("USV控制器就绪 (直驱模式)...")

    def brush_callback(self, msg):
        if msg.data:
            command = "O\n"
            log_msg = "发送指令: 滚刷开启 (ON)"
        else:
            command = "C\n"
            log_msg = "发送指令: 滚刷关闭 (OFF)"
            
        try:
            self.ser.write(command.encode())
            self.get_logger().info(log_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"串口发送失败: {e}")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 1. 计算差速 PWM 增量
        drive_pwm_diff = linear_x * SPEED_TO_PWM_SCALE
        turn_pwm_diff = (angular_z * WHEEL_BASE / 2.0) * SPEED_TO_PWM_SCALE
        
        # 2. 计算基准 PWM (1500为中点)
        left_pwm_val = PWM_NEUTRAL + drive_pwm_diff - turn_pwm_diff
        right_pwm_val = PWM_NEUTRAL + drive_pwm_diff + turn_pwm_diff
        
        # 【修改】: 删除了之前的 3000 - x 逻辑，现在是直驱
        # 如果前进 (linear_x > 0)，PWM > 1500
        # 如果后退 (linear_x < 0)，PWM < 1500
        
        # 3. 限制范围 (1100 - 1900)
        final_left_pwm = max(PWM_MIN, min(PWM_MAX, int(left_pwm_val)))
        final_right_pwm = max(PWM_MIN, min(PWM_MAX, int(right_pwm_val)))

        # 4. 组装指令 "M,1550,1550"
        command = f"M,{final_left_pwm},{final_right_pwm}\n"

        if command != self.last_sent_command:
            try:
                self.ser.write(command.encode())
                self.get_logger().info(f"CmdVel -> PWM(L:{final_left_pwm}, R:{final_right_pwm})")
                
                # 回传给 PC 端显示
                feedback = f"{final_left_pwm},{final_right_pwm}"
                self.pwm_pub.send_string(feedback)
                
                self.last_sent_command = command
            except serial.SerialException as e:
                self.get_logger().error(f"串口发送失败: {e}")

    def destroy_node(self):
        self.pwm_pub.close()
        self.zmq_context.term()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    usv_controller = USVController()
    rclpy.spin(usv_controller)
    usv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()