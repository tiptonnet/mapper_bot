import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import sys
import time

class CommandControl(Node):

    def __init__(self,args):
        self.args = args
        self.iterations = 0
        super().__init__('CommandControl')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
            """
            Send comand velocities. Must be in GUIDED mode. Assumes angular
            velocities are zero by default.
            """
            cmd_vel = Twist()

            cmd_vel.linear.x = vx
            cmd_vel.linear.y = vy
            cmd_vel.linear.z = vz

            cmd_vel.angular.x = avx
            cmd_vel.angular.y = avy
            cmd_vel.angular.z = avz

            self.publisher_.publish(cmd_vel)
    
def main(args=None):
    if len(sys.argv) < 2:
        sim=False
    else:
        sim = sys.argv[1]
    #print(sim)
    rclpy.init()
    reading_laser = CommandControl(sim)                  
    reading_laser.get_logger().info("Control system launched")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
