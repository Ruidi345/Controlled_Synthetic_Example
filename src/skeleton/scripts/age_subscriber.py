#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from skeleton.msg import Age  

class AgeSubscriber(Node):
    def __init__(self):
        super().__init__('age_sub_node')
        self.sub = self.create_subscription(
            Age,
            'person_age',     
            self.callback_on_age,
            10         
        )

    def callback_on_age(self, msg):
        self.get_logger().info(f"Received: years={msg.age}")

def main():
    rclpy.init()
    node = AgeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# remember: chmod +x scripts/age_subscriber.py
# run: ros2 run skeleton age_subscriber.py
if __name__ == '__main__':
    main()