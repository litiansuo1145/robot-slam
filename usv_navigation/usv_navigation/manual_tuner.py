import serial
import time
import threading
import sys
import os
import rclpy
import tty
import termios

class ManualTuner:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # --- [1] 基础物理参数 ---
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        self.PWM_ADJUST_STEP = 1

        # --- [2] 核心可调参数 ---
        self.wheel_base = 0.4
        self.speed_to_pwm_scale = 400.0

        # --- [3] 运行时变量 ---
        self.pwm_drive_diff = 100
        self.pwm_turn_diff = 100
        
        # 状态记录
        self.brush_active = False 
        self.debug_last_key = "None" # 用于调试按键
        
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.running = False
        self.lock = threading.Lock()
        
        # 【修复】分离通用指令记录和移动指令记录
        self.last_sent_display = "" # 用于界面显示
        self.last_move_cmd = ""     # 专门用于移动指令去重
        self.last_drive_offset = 0
        self.last_turn_offset = 0

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print("正在连接...")
            time.sleep(2)
            self.connected = True
            self.running = True
            
            # 启动读取线程（只读取不打印，防止刷屏影响界面）
            self.read_thread = threading.Thread(target=self._read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()
            return True
        except serial.SerialException as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                # 退出时复位
                with self.lock: 
                    self.ser.write(f"M,{self.PWM_NEUTRAL},{self.PWM_NEUTRAL}\n".encode())
                    time.sleep(0.05)
                    self.ser.write("C\n".encode())
            except Exception: pass
            self.ser.close()
        self.connected = False

    def _read_serial(self):
        while self.running and self.ser.is_open:
            try:
                self.ser.readline() # 读掉缓冲区数据，但不打印
            except: break

    # --- 发送移动指令 (修复了覆盖逻辑) ---
    def send_pwm_command(self, left_pwm, right_pwm):
        if not self.connected or not self.running: return
        
        command = f"M,{left_pwm},{right_pwm}\n"
        
        # 【核心修复】只跟上一次的“移动指令”做对比
        # 这样即使刚发了 'O'，如果移动没变，就不会发 'M' 来刷新屏幕
        if command != self.last_move_cmd:
            try:
                with self.lock: self.ser.write(command.encode())
                self.last_move_cmd = command
                self.last_sent_display = command.strip() # 更新界面显示
            except serial.SerialException: self.running = False

    # --- 发送滚刷指令 ---
    def send_brush_command(self, enable):
        if not self.connected or not self.running: return
        
        cmd_char = "O" if enable else "C"
        command = f"{cmd_char}\n"
        
        try:
            with self.lock: self.ser.write(command.encode())
            
            # 更新状态
            self.brush_active = enable
            
            # 强制更新界面显示，让用户看到 "Brush: O"
            self.last_sent_display = f"Brush CMD: {cmd_char} (Sent)"
        except serial.SerialException: 
            self.running = False

    def _print_hud(self):
        os.system('clear')
        print("="*60)
        print("        无人船控制面板 (修复版)")
        print("="*60)
        print(f"【按键调试】检测到键值: {self.debug_last_key}")
        print("-" * 60)
        print("【滚刷控制】")
        print(" 按 'O' (字母O) -> 开启 | 按 'C' -> 关闭")
        print(f" 当前状态: {'🟢 开启 (Active)' if self.brush_active else '🔴 关闭 (Inactive)'}")
        print("-" * 60)
        print("【移动控制】W/A/S/D (空格停车)")
        print(f" 油门: {self.pwm_drive_diff} | 转向: {self.pwm_turn_diff}")
        print("="*60)
        print(f"串口发送内容: {self.last_sent_display}")
        print("="*60)

    def run_control_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self._print_hud()
            
            while self.running:
                # 读取一个字符
                char = sys.stdin.read(1).lower()
                
                # 记录按键用于调试 (显示 ASCII 码)
                self.debug_last_key = f"'{char}' (Ascii: {ord(char)})"

                is_movement_key = True
                
                # --- 1. 移动逻辑 ---
                if char == 'w':
                    self.last_drive_offset = self.pwm_drive_diff
                    self.last_turn_offset = 0
                elif char == 's':
                    self.last_drive_offset = -self.pwm_drive_diff
                    self.last_turn_offset = 0
                elif char == 'a':
                    self.last_turn_offset = -self.pwm_turn_diff
                    self.last_drive_offset = 0
                elif char == 'd':
                    self.last_turn_offset = self.pwm_turn_diff
                    self.last_drive_offset = 0
                elif char == ' ':
                    self.last_drive_offset = 0
                    self.last_turn_offset = 0
                else:
                    is_movement_key = False

                # --- 2. 滚刷与参数 ---
                if not is_movement_key:
                    # 【重要】确保是字母 'o' 不是数字 '0'
                    if char == 'o': 
                        self.send_brush_command(True)
                    elif char == 'c':
                        self.send_brush_command(False)
                    
                    # 调参键
                    elif char in ['=', '+']: self.pwm_drive_diff = min(400, self.pwm_drive_diff + self.PWM_ADJUST_STEP)
                    elif char == '-': self.pwm_drive_diff = max(0, self.pwm_drive_diff - self.PWM_ADJUST_STEP)
                    elif char == 'q': self.running = False; continue

                # --- 3. 计算并发送PWM ---
                left_pwm_base = self.PWM_NEUTRAL + self.last_drive_offset - self.last_turn_offset
                right_pwm_base = self.PWM_NEUTRAL + self.last_drive_offset + self.last_turn_offset
                
                left_pwm = left_pwm_base
                right_pwm = right_pwm_base

                left_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(left_pwm)))
                right_pwm = max(self.PWM_MIN, min(self.PWM_MAX, int(right_pwm)))
                
                self.send_pwm_command(left_pwm, right_pwm)
                self._print_hud()
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.disconnect()

def main(args=None):
    rclpy.init(args=args)
    port = input("输入串口 (回车默认 /dev/ttyACM0): ").strip() or '/dev/ttyACM0'
    tuner = ManualTuner(port=port)
    if tuner.connect():
        tuner.run_control_loop()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
