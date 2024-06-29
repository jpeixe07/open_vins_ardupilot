import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix

class OdomRepublisher(Node):
    def __init__(self, input_topic, output_topic, is_correct):
        super().__init__('odom_republisher')
        self.is_correct = is_correct
        self.pub = self.create_publisher(Odometry, output_topic, 1)
        self.subscription = self.create_subscription(Odometry, input_topic, self.callback, 1)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz rate

    def callback(self, msg):
        if not self.is_correct:
            q = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            rotation_matrix = quaternion_matrix(q)

            flip_matrix = np.array([
                [0, -1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1]
            ])
            
            new_rotation_matrix = np.dot(flip_matrix, rotation_matrix)

            new_q = quaternion_from_matrix(new_rotation_matrix)
            
            msg.pose.pose.orientation.x = -new_q[0]
            msg.pose.pose.orientation.y = -new_q[1]
            msg.pose.pose.orientation.z = -new_q[2]
            msg.pose.pose.orientation.w = new_q[3]

            msg.pose.pose.position.x = -msg.pose.pose.position.x
            msg.pose.pose.position.y = -msg.pose.pose.position.y

            msg.twist.twist.linear.x = -msg.twist.twist.linear.y
            msg.twist.twist.linear.y = -msg.twist.twist.linear.x
            msg.twist.twist.linear.z = -msg.twist.twist.linear.z
            msg.twist.twist.angular.x = -msg.twist.twist.angular.y
            msg.twist.twist.angular.y = -msg.twist.twist.angular.x
            msg.twist.twist.angular.z = -msg.twist.twist.angular.z
        
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.pub.publish(msg)

    def timer_callback(self):
        pass  # This function is here to satisfy the timer callback, but we don't need to do anything in it

def main(args=None):
    rclpy.init(args=args)
    odom_republisher = OdomRepublisher('/ov_msckf/odomimu', '/mavros/odometry/out', False)
    rclpy.spin(odom_republisher)
    odom_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
