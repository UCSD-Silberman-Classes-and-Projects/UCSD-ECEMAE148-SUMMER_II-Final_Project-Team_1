#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AvoidanceTick(Node):
    def __init__(self):
        super().__init__('manta_avoidance_tick')
        self.pub = self.create_publisher(
            Float32, '/manta/avoidance/control_tick', 10
        )
        self.create_timer(0.05, self.tick)

    def tick(self):
        self.pub.publish(Float32(data=0.0))

def main():
    rclpy.init()
    node = AvoidanceTick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
