import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import time

class DummyMapPublisher(Node):
    def __init__(self):
        super().__init__('dummy_map_publisher')

        # 关键：定义一个与 map_server 完全相同的 QoS Profile
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布者
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", qos_profile)
        
        # 创建一个定时器，在1秒后发布一次地图，然后什么都不做
        self.timer = self.create_timer(1.0, self.publish_map_once)
        
        self.get_logger().info('Dummy Map Publisher has started.')

    def publish_map_once(self):
        if self.timer.is_canceled():
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = 1.0
        msg.info.width = 10
        msg.info.height = 10
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        
        # 创建一个简单的 10x10 地图数据 (0=free, 100=occupied, -1=unknown)
        map_data = [-1] * 100
        map_data[55] = 100 # 在中间放一个障碍物
        msg.data = map_data
        
        self.map_publisher.publish(msg)
        self.get_logger().info('<<<<< DUMMY MAP HAS BEEN PUBLISHED! >>>>>')
        
        # 发布后取消定时器，确保只发布一次
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = DummyMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()