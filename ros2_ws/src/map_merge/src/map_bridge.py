import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapBridge(Node):
    def __init__(self):
        super().__init__('map_bridge_node')
        
        # 定义机器人列表
        self.robots = ['robot1', 'robot2', 'robot3']
        self.subs = []
        self.pubs = []

        # 配置 QoS：订阅时必须是 Transient Local (为了听 SLAM)
        qos_in = QoSProfile(depth=1)
        qos_in.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_in.reliability = ReliabilityPolicy.RELIABLE

        # 配置 QoS：发布时必须是 Volatile (为了喂给 Map Merge)
        qos_out = QoSProfile(depth=1)
        qos_out.durability = DurabilityPolicy.VOLATILE
        qos_out.reliability = ReliabilityPolicy.RELIABLE

        for robot in self.robots:
            # 输入话题 (SLAM)
            topic_in = f'/{robot}/map'
            # 输出话题 (给 Map Merge) -> 注意这里加了后缀 _volatile
            topic_out = f'/{robot}/map_volatile'

            self.get_logger().info(f'Bridging {topic_in} -> {topic_out}')

            # 创建发布者
            pub = self.create_publisher(OccupancyGrid, topic_out, qos_out)
            self.pubs.append(pub)

            # 创建订阅者 (使用 lambda 捕获当前的 pub 变量)
            # 注意：python 闭包陷阱，需要用默认参数 pub=pub 锁定变量
            sub = self.create_subscription(
                OccupancyGrid, 
                topic_in, 
                lambda msg, p=pub: p.publish(msg), 
                qos_in
            )
            self.subs.append(sub)

def main(args=None):
    rclpy.init(args=args)
    node = MapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()