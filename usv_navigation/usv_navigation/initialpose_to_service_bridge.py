import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import Relocalize
import math

# 用于将四元数转换为欧拉角(特别是yaw)的辅助函数
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initialpose_bridge')

        # 声明一个参数用于接收地图路径
        self.declare_parameter('pcd_path', '/home/nvidia/my_slam_map/map.pcd')

        # 1. 创建 /initialpose 话题的订阅者
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10)

        # 2. 创建 /localizer/relocalize 服务的客户端
        self.cli = self.create_client(Relocalize, '/localizer/relocalize')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /localizer/relocalize not available, waiting again...')
        
        self.get_logger().info('Initial Pose to Relocalize Service Bridge is ready.')

    def pose_callback(self, msg):
        self.get_logger().info('Received pose from RViz on /initialpose topic.')

        # 从消息中提取位置和姿态
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        # 将姿态从四元数转换为欧拉角
        roll, pitch, yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)

        # 准备服务请求
        req = Relocalize.Request()
        req.pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        req.x = position.x
        req.y = position.y
        req.z = position.z # 通常在2D地图上，z可以保持为0
        req.roll = roll
        req.pitch = pitch
        req.yaw = yaw
        
        self.get_logger().info(f"Calling /localizer/relocalize service with yaw: {yaw:.2f}")

        # 异步调用服务
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully called relocalize service: {response.message}")
            else:
                self.get_logger().error(f"Failed to call relocalize service: {response.message}")
        except Exception as e:
            self.get_logger().error(f'Service call failed {e!r}')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = InitialPoseBridge()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    