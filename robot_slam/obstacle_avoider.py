

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        self.get_logger().info("Obstacle Avoider Node (Reflex) Started")

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
        
        self.status_publisher_ = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.cmd_publisher_ = self.create_publisher(Twist, '/cmd_vel_avoider', 10)

        self.declare_parameter('avoidance_distance', 0.5)
        self.declare_parameter('turn_speed', 0.5)
        self.avoidance_distance = self.get_parameter('avoidance_distance').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

    def scan_callback(self, msg: LaserScan):
        obstacle_detected = False
        angle_range_deg = 30 
        center_index = len(msg.ranges) // 2
        index_range = int(math.radians(angle_range_deg / 2) / msg.angle_increment)
        start_index = center_index - index_range
        end_index = center_index + index_range

        for i in range(start_index, end_index):
            distance = msg.ranges[i]
            if distance > 0.01 and distance < self.avoidance_distance:
                obstacle_detected = True
                break

        status_msg = Bool()
        status_msg.data = obstacle_detected
        self.status_publisher_.publish(status_msg)

        if obstacle_detected:
            self.get_logger().warn("Obstacle Detected! Publishing avoidance turn.", throttle_duration_sec=1)
            twist_msg = Twist()
            twist_msg.angular.z = -self.turn_speed
            self.cmd_publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()