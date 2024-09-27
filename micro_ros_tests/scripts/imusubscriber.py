#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu

class IMUsub(Node):
    def __init__(self):
        super().__init__("IMUSubscriber")
        self.sub = self.create_subscription(Imu, "imu/data", self.subscriber_callback, 10)
    def subscriber_callback(self, msg):
        print("Linear acceleration x: " + str(msg.linear_acceleration.x))
        print("Linear acceleration y: " + str(msg.linear_acceleration.y))
        print("Linear acceleration z: " + str(msg.linear_acceleration.z))
        print("Angular velocity x: " + str(msg.angular_velocity.x))
        print("Angular velocity y: " + str(msg.angular_velocity.y))
        print("Angular velocity z: " + str(msg.angular_velocity.z))

def main(arge=None):
    rclpy.init()
    mysub = IMUsub()
    print("Waiting for data to be published..")
    try:
        rclpy.spin(mysub)
    except KeyboardInterrupt:
        print("Terminating node...")
        mysub.destroy_node()

if __name__ == "__main__":
    main()