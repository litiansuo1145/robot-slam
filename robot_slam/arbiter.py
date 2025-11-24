import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ArbiterNode(Node):
    def __init__(self):
        super().__init__('arbiter_node')
        self.get_logger().info("Intelligent Arbiter Node (The Helmsman) has started.")

        # 使用与默认发布者兼容的QoS配置
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # --- 订阅者 ---
        self.planner_sub = self.create_subscription(Twist, '/cmd_vel_planner', self.planner_callback, qos_profile)
        self.avoider_sub = self.create_subscription(Twist, '/cmd_vel_avoider', self.avoider_callback, qos_profile)
        
        # --- 发布者 ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 状态变量 ---
        self.planner_cmd = None
        self.avoider_cmd = None
        self.last_planner_time = self.get_clock().now() # 新增：大脑指令的时间戳
        self.last_avoider_time = self.get_clock().now() # 反射神经指令的时间戳

        # --- 决策定时器 ---
        self.create_timer(0.05, self.decide_and_publish) # 20Hz决策频率
        self.get_logger().info("Arbiter is now listening with command timeout.")

    def planner_callback(self, msg):
        """当收到大脑的指令时，存储它并更新时间戳"""
        self.planner_cmd = msg
        self.last_planner_time = self.get_clock().now()

    def avoider_callback(self, msg):
        """当收到反射神经的指令时，存储它并更新时间戳"""
        self.avoider_cmd = msg
        self.last_avoider_time = self.get_clock().now()

    def decide_and_publish(self):
        """
        核心决策逻辑：基于优先级和“保鲜期”进行仲裁。
        """
        final_cmd = Twist() # 默认是停止指令
        now = self.get_clock().now()
        
        # --- 指令的保鲜期设为0.5秒 ---
        cmd_timeout_ns = 0.5 * 1e9

        # --- 检查指令是否“新鲜” ---
        is_avoider_fresh = (now - self.last_avoider_time).nanoseconds < cmd_timeout_ns
        is_planner_fresh = (now - self.last_planner_time).nanoseconds < cmd_timeout_ns

        # --- 优先级决策 ---
        if is_avoider_fresh and self.avoider_cmd:
            # 规则1：如果反射神经的指令是新鲜的，无条件听从它
            self.get_logger().warn("Arbiter: Prioritizing FRESH Avoider command!", throttle_duration_sec=1)
            final_cmd = self.avoider_cmd
        elif is_planner_fresh and self.planner_cmd:
            # 规则2：否则，如果大脑的指令是新鲜的，就听从大脑的
            self.get_logger().info("Arbiter: Following FRESH Planner command.", throttle_duration_sec=1)
            final_cmd = self.planner_cmd
        else:
            # 规则3：如果所有指令都“过时”了，就什么都不做，final_cmd保持为停止
            self.get_logger().info("Arbiter: All commands are stale. Stopping.", throttle_duration_sec=1)
        
        # 发布最终的决策
        self.cmd_vel_pub.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
