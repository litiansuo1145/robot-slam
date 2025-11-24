import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
---------------------------
Simple Keyboard Teleop (Stateful)
---------------------------
Moving around:
       w
    a  s  d

space key, s : force stop
---------------------------
CTRL-C to quit
---------------------------
"""

move_bindings = {
    'w': (1.0, 0.0), 's': (-1.0, 0.0),
    'a': (0.0, 1.0), 'd': (0.0, -1.0),
}

def get_key(settings):
    if sys.platform == 'win32': return None
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist: key = sys.stdin.read(1)
    else: key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.2
        self.turn = 0.5
        
        # --- 这是关键修改：引入状态变量 ---
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        print(msg)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = get_key(self.settings)

        # --- 关键逻辑修改：只在有按键时更新状态 ---
        if key in move_bindings.keys():
            self.target_linear_x = move_bindings[key][0]
            self.target_angular_z = move_bindings[key][1]
        elif key == ' ' or key == 's':
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
        elif (key == '\x03'):
            self.restore_terminal_settings()
            rclpy.shutdown()
            return
        
        # --- 关键逻辑修改：总是使用最后记住的状态来发布 ---
        twist = Twist()
        twist.linear.x = self.target_linear_x * self.speed
        twist.angular.z = self.target_angular_z * self.turn
        self.publisher_.publish(twist)

    # ... (restore_terminal_settings 和 main 函数保持不变) ...
    def restore_terminal_settings(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = None
    try:
        node = TeleopKeyboardNode()
        rclpy.spin(node)
    except termios.error:
        print("Could not get terminal attributes. Is this running in a proper terminal?")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if node:
            node.restore_terminal_settings()
        else:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
