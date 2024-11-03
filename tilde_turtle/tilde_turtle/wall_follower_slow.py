#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowerSlow(Node):
    def __init__(self, robot_name):
        super().__init__('wall_follower_' + robot_name)
        self.cmd_vel_pub = self.create_publisher(Twist, f'{robot_name}/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, f'{robot_name}/scan', self.scan_callback, 10)
        

    def scan_callback(self, msg):
        twist = Twist()
        # Simple wall-following algorithm based on laser scan data
        # This is a basic placeholder; adapt it to your needs
        front_distance = min(msg.ranges[0:30] + msg.ranges[330:359])
        side_distance = min(msg.ranges[60:120])

        if front_distance < 0.5:  # Too close to the wall in front
            twist.angular.z = 0.5  # Turn left
        elif side_distance > 0.5:  # Too far from the wall on the side
            twist.angular.z = -0.5  # Turn right
        else:
            twist.linear.x = 0.2  # Move forward

        self.cmd_vel_pub.publish(twist)
        
class WallFollowerFast(Node):
    def __init__(self, robot_name):
        super().__init__('wall_follower_' + robot_name)
        self.cmd_vel_pub = self.create_publisher(Twist, f'{robot_name}/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, f'{robot_name}/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        twist = Twist()
        # Simple wall-following algorithm based on laser scan data
        # This is a basic placeholder; adapt it to your needs
        front_distance = min(msg.ranges[0:30] + msg.ranges[330:359])
        side_distance = min(msg.ranges[60:120])

        if front_distance < 0.5:  # Too close to the wall in front
            twist.angular.z = 0.5  # Turn left
        elif side_distance > 0.5:  # Too far from the wall on the side
            twist.angular.z = -0.5  # Turn right
        else:
            twist.linear.x = 0.2  # Move forward

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    # Create a temporary node to read the parameter
    temp_node = Node('temporary_node')
    robot_name = temp_node.declare_parameter('robot_name', 'robot_1').value
    temp_node.destroy_node()  # Destroy the temporary node once the parameter is retrieved

    # Instantiate the correct wall follower node based on the parameter
    if robot_name == "robot_1":
        node = WallFollowerSlow(robot_name)
    else:
        node = WallFollowerFast(robot_name)

    # Spin the chosen node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# def main(args=None):
#     # rclpy.init(args=args)
    
#     # # Create the node
#     # node = WallFollowerSlow("robot_1")  # Default robot name for this example
#     # node2 = WallFollowerFast("robot_2")  # Default robot name for this example

#     # # Declare the parameter and get the robot name
#     # node.declare_parameter("robot_name", "robot_1")  # Set default if not provided
#     # node2.declare_parameter("robot_name", "robot_2")  # Set default if not provided
#     # robot_name = node.get_parameter("robot_name").value  # Retrieve the robot name

#     # # # Set the robot name in the WallFollowerSlow
#     # # node = WallFollowerSlow(robot_name)

#     # rclpy.spin(node)
#     # rclpy.spin(node2)
#     # rclpy.shutdown()
    
#     rclpy.init(args=args)

#     node1 = WallFollowerSlow("robot_1")
#     node2 = WallFollowerFast("robot_2")

#     try:
#         rclpy.spin(node1)
#         rclpy.spin(node2)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node1.destroy_node()
#         node2.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
