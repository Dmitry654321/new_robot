#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped


class tof_node(Node):
    def __init__(self):
        super().__init__('tof')
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.user = os.getenv("USER_NAME")

        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.checkRange, 10) #might be wrong topic name
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
    
    def checkRange(self, msg):
        range = msg.range
        if range >= 0.2:
            self.moveForward()
        else:
            self.stop()
    
    def moveForward(self):
        self.run_wheels('forward_callback', 0.5, 0.5)
    
    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)

        
    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)



def main():
    rclpy.init()
    tof = tof_node()
    rclpy.spin(tof)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
