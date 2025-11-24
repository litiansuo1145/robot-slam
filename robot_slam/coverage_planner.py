import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool # 导入布尔消息类型
import time
import math

class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner_node')
        self.get_logger().info("Coverage Planner Node (Brain) Started")

        # --- 参数 ---
        self.declare_parameter('pool_length', 5.0)
        self.declare_parameter('pool_width', 3.0)
        self.declare_parameter('robot_linear_speed', 0.2)
        self.declare_parameter('robot_angular_speed', 0.1)
        self.declare_parameter('robot_coverage_width', 0.5)

        self.pool_length = self.get_parameter('pool_length').get_parameter_value().double_value
        self.pool_width = self.get_parameter('pool_width').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('robot_linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('robot_angular_speed').get_parameter_value().double_value
        self.coverage_width = self.get_parameter('robot_coverage_width').get_parameter_value().double_value
        
        # --- 发布者 ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 订阅者 ---
        self.obstacle_subscriber = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.is_obstacle_detected = False # 内部状态变量

        # 创建一个一次性定时器，5秒后启动任务
        self.create_timer(5.0, self.start_coverage_path)

    def obstacle_callback(self, msg: Bool):
        """当收到反射神经的状态时，更新自己的内部状态"""
        if msg.data and not self.is_obstacle_detected:
            self.get_logger().warn("Received obstacle signal from reflex node.")
        self.is_obstacle_detected = msg.data

    def start_coverage_path(self):
        """执行完整的“之”字形覆盖路径"""
        self.get_logger().info("Executing Z-shaped coverage path...")
        
        num_lanes = math.ceil(self.pool_length / self.coverage_width)
        self.get_logger().info(f"Calculated number of lanes: {num_lanes}")

        for i in range(num_lanes):
            self.get_logger().info(f"--- Starting Lane {i+1} ---")
            self.move_straight(self.pool_width)

            # 如果不是最后一条泳道，则需要平移到下一条
            if i < num_lanes - 1:
                self.get_logger().info("Transitioning to the next lane...")
                # 我们假设避障已经帮我们完成了掉头，现在只需要横向平移
                self.move_straight(self.coverage_width)
        
        self.get_logger().info("Coverage path completed!")
        rclpy.shutdown()

    def move_straight(self, distance):
        """持续发布前进指令，直到达到距离或被反射神经打断"""
        duration = distance / self.linear_speed
        self.get_logger().info(f"Moving straight for {distance:.2f} meters (or until obstacle)...")
        
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        
        start_time = time.time()
        while (time.time() - start_time < duration) and not self.is_obstacle_detected:
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1) # 使用spin_once来处理回调
        
        if self.is_obstacle_detected:
            self.get_logger().warn("Obstacle detected! Yielding control to avoider. Waiting...")
            # 停止发布前进指令，并耐心等待
            self.stop_robot()
            
            # 持续检查，直到反射神经说“安全了”
            while self.is_obstacle_detected:
                self.get_logger().info("Avoider is active, planner is waiting...", throttle_duration_sec=1)
                rclpy.spin_once(self, timeout_sec=0.1)
            
            self.get_logger().info("Avoider has finished. Planner resuming control.")
        
        self.stop_robot()
    
    def stop_robot(self):
        """发布停止指令并等待"""
        self.get_logger().info("Stopping robot...")
        twist_msg = Twist()
        self.publisher_.publish(twist_msg)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
