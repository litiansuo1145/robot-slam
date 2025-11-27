import serial
import time
import threading
import sys
import os
import tty
import termios

class ManualTuner:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.PWM_NEUTRAL = 1500
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.lock = threading.Lock()
        self.brush_pwm = 0 # 当前滚刷PWM值

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print("连接成功...")
            time.sleep(2)
            self.running = True
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def send_brush_raw(self, pwm):
        if not self.running: return
        cmd = f"B,{pwm}\n"
        with self.lock:
            self.ser.write(cmd.encode())
        self.brush_pwm = pwm

    def _print_menu(self):
        os.system('clear')
        print("="*40)
        print("   滚刷 PWM 直通测试工具")
        print("="*40)
        print(f"当前滚刷 PWM: 【 {self.brush_pwm} 】")
        print("-" * 40)
        print(" U: +1000  |  J: -1000")
        print(" I: +100   |  K: -100")
        print(" O: +10    |  L: -10")
        print(" 0: 急停 (设为0)")
        print(" Q: 退出")
        print("="*40)

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self._print_menu()
            
            while self.running:
                char = sys.stdin.read(1).lower()
                
                update = False
                if char == 'u': self.brush_pwm += 1000; update = True
                elif char == 'j': self.brush_pwm -= 1000; update = True
                elif char == 'i': self.brush_pwm += 100; update = True
                elif char == 'k': self.brush_pwm -= 100; update = True
                elif char == 'o': self.brush_pwm += 10; update = True
                elif char == 'l': self.brush_pwm -= 10; update = True
                elif char == '0': self.brush_pwm = 0; update = True
                elif char == 'q': self.running = False; break
                
                if update:
                    # 限制范围防止溢出
                    self.brush_pwm = max(0, min(20000, self.brush_pwm))
                    self.send_brush_raw(self.brush_pwm)
                    self._print_menu()
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            if self.ser: self.ser.close()

if __name__ == '__main__':
    tuner = ManualTuner()
    if tuner.connect():
        tuner.run()
        