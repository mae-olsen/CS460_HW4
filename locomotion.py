import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator') # Node name
        self.setpoint = 1.2 # Distance from wall and obstacles
        self.Kp = 1.0  #proportional gain
        self.Kd = 0.0  #derivative gain
        self.previous_error = 0.0 # previous error initial value

        self.last_time = time.time()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.sensor_callback,
            qos_profile_sensor_data)
        self.get_logger().info('Navigator node (start) has been started.')

    def sensor_callback(self, msg):
        data = msg.ranges
        #print("I am alive!")
        move_cmd = Twist()
        rightSide = data[225:280] # sensor values on right side of bot, [225:280] of: [80:135]
        minRightSide = min(rightSide)

        error = self.setpoint - minRightSide
        current_time = time.time()
        dt = current_time - self.last_time
        print(error)

        P = self.Kp * error
        if dt > 0:
            D = self.Kd * (error - self.previous_error) / dt
        else:
            D = 0
        controlSignal = P + D

        move_cmd.linear.x = 0.7
        move_cmd.angular.z = -controlSignal
        print(controlSignal)
        #Front turn
        avgFront = np.mean(data[45:325])
        print("Front:", avgFront)
        # minFront = data[180]
        #if avgFront < 0.5:
         #   move_cmd.linear.x = 0.0
          #  move_cmd.angular.z = 2.0
           # self.previous_error = 0
        
        self.cmd_pub.publish(move_cmd)
        self.previous_error = error
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    navigator_node = Navigator()
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
