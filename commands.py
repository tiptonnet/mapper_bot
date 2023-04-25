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

class ReadingCommands(Node):

    def __init__(self,args):
        self.args = args
        self.iterations = 0
        super().__init__('reading_commands')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        timer_period = 0.5  # seconds
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

    def set_vel(self, vx, vy, vz, avx=0.0, avy=0.0, avz=0.0):
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
    
    def GetCommand(self, vx, vy, vz, avx=0.0, avy=0.0, avz=0.0):
        self.set_vel(vx, vy, vz, avx, avy, avz)

def main(args=None):
    if len(sys.argv) < 2:
        sim=False
    else:
        sim = sys.argv[1]
    #print(sim)
    rclpy.init()
    reading_commands = ReadingCommands(sim)                  
    rclpy.spin(reading_commands)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_commands.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()