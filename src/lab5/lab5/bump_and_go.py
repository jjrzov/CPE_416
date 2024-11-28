import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix
from tf2_ros import TransformException

import numpy as np
import time

class BumpAndGoNode(Node):

    def __init__(self):
        super().__init__('bump_and_go')

        # Publisher to move robot
        self.publisher = self.create_publisher(Twist, 'diff_drive/cmd_vel', 10)
        
        # Subscriber to LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            'diff_drive/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(0.1, self.timer_callback)    # Set timer for state switching
        self.state = 'FORWARD'
        self.back_start_time = 0
        self.latest_laser = LaserScan() # Store LIDAR variable

        # Set Movements for the robot
        self.twist = Twist()
        self.linear_velocity = 0.5  # Move forward speed
        self.angular_velocity = 0.35    # Turning speed

        self.get_logger().info('Information for running node')
        self.get_logger().info('Implements a rooba like bump and go movement')
        self.get_logger().info('If no obstacle detected => move forward')
        self.get_logger().info('If obstacle detected => back 2s then turn')
        self.get_logger().info('Note: Robot stops if LIDAR fails')

    def scan_callback(self, msg):
        # Copy the data from the laser scanner into member variable
        self.latest_laser = msg

    def get_lidar_data(self):
        # Returns the average of the LIDAR beams in front of the robot
        avg_distance = 0
        x_axis_index = (len(self.latest_laser.ranges) // 2)
        
        angle_range = np.deg2rad(60)    # Want 30 degrees to front of robot
        indices = int(angle_range // self.latest_laser.angle_increment)   # Find how many indices for 30 degs
        start_index = x_axis_index - indices

        for i in range((2 * indices) + 1):
            distance = float(self.latest_laser.ranges[start_index + i])
            avg_distance += distance
        
        return float(avg_distance / ((2 * indices) + 1))

    def timer_callback(self):
        # FSM for controlling the robot
        # x_axis_index = len(self.latest_laser.ranges) // 2
        
        if self.state == 'FORWARD':
            self.get_logger().info('GOING FORWARD')
            self.twist.linear.x = self.linear_velocity  # Move robot forward
            self.twist.angular.z = 0.0  # No turning
            
            if not self.latest_laser.ranges:
                self.state = 'STOP' # LIDAR stopped working
                return

            # distance = float(self.latest_laser.ranges[x_axis_index])
            distance = float(self.get_lidar_data())
            if distance < 5:
                # There is an object in front of robot
                self.state = 'BACK' # Keep turning
                self.back_start_time = time.time()
            else:
                # There is no object in front of robot
                self.state = 'FORWARD'
                self.publisher.publish(self.twist)

        elif self.state == 'BACK':
            self.get_logger().info('BACKING UP')
            self.twist.linear.x = -self.linear_velocity # Move robot backwards
            self.twist.angular.z = 0.0 # No turning
            
            if ((time.time() - self.back_start_time) < 2.0):
                self.state = 'BACK'
                self.publisher.publish(self.twist)
            else:
                self.state = 'TURN'

        elif self.state == 'TURN':
            self.get_logger().info('TURNING')
            self.twist.linear.x = 0.0 # Do not move forwards / backwards
            self.twist.angular.z = -self.angular_velocity   # Turn right until no obstacle
            
            if not self.latest_laser.ranges:
                self.state = 'STOP' # LIDAR stopped working
                return

            # distance = float(self.latest_laser.ranges[x_axis_index])
            distance = self.get_lidar_data()
            if distance < 5:
                # There is an object in front of robot
                self.publisher.publish(self.twist)
                self.state = 'TURN' # Keep turning
            else:
                # There is no object in front of robot
                self.state = 'FORWARD'

        elif self.state == 'STOP':
            self.get_logger().info('STOPPED')
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)

            self.state = 'FORWARD'

        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)

            self.state = 'STOP'



def main(args=None):
    rclpy.init(args=args)
    node = BumpAndGoNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()