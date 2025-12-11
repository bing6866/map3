import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback) # 10Hz

        # 这里填入你在 multi_bot.launch.py 里定义的坐标
        self.robots = [
            {'name': 'robot1', 'x': 0.0,  'y': 0.0,  'yaw': 0.0},
            {'name': 'robot2', 'x': 2.0,  'y': -5.0, 'yaw': 0.0},
            {'name': 'robot3', 'x': -4.0, 'y': 3.0,  'yaw': 0.0}
        ]
        self.get_logger().info("Dynamic TF Broadcaster Started")

    def broadcast_timer_callback(self):
        time_now = self.get_clock().now().to_msg()
        
        for robot in self.robots:
            t = TransformStamped()
            t.header.stamp = time_now
            t.header.frame_id = 'world'
            t.child_frame_id = f"{robot['name']}/map"

            t.transform.translation.x = float(robot['x'])
            t.transform.translation.y = float(robot['y'])
            t.transform.translation.z = 0.0

            # 简单的 Yaw 转 Quaternion
            yaw = robot['yaw']
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)

            self.br.sendTransform(t)

def main():
    rclpy.init()
    node = DynamicTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()