import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf.transformations import quaternion_from_matrix, quaternion_matrix, quaternion_multiply, quaternion_inverse
import numpy as np
 
class OdomRepublisher:
    def __init__(self, input_topic, output_topic, is_correct):
        self.sub = rospy.Subscriber(input_topic, Odometry, self.callback, queue_size=1)
        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=1)
        self.is_correct = is_correct
        self.rate = rospy.Rate(30)
 
    def callback(self, data):

        if not self.is_correct:
            q = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
            rotation_matrix = quaternion_matrix(q)
    
            # matriz de transformação 
            flip_matrix = np.array([
                [0, -1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1]
            ])
            # flip_matrix = np.array([
            #     [0.0031375042558366272, 0.9999942155251319, 0.0013133862039308268, 0],
            #     [0.9999916038526114, -0.0031340285803495727, -0.0026400926384573383, 0],
            #     [-0.0026359611770076655, 0.0013216584784356434, -0.9999956524543192, 0],
            #     [0.0, 0.0, 0.0, 1.0]
            # ])
            new_rotation_matrix = np.dot(flip_matrix, rotation_matrix)
    
            # Converter a matriz de rotação de volta para um quaternião
            new_q = quaternion_from_matrix(new_rotation_matrix)
            data.pose.pose.orientation = Quaternion(*new_q)
            data.pose.pose.orientation.x = -data.pose.pose.orientation.x
            data.pose.pose.orientation.y = -data.pose.pose.orientation.y
            data.pose.pose.orientation.z = -data.pose.pose.orientation.z
            # Inverter os sentidos dos eixos x e y da posição
            #data.pose.pose.position.x = -data.pose.pose.position.x
            #data.pose.pose.position.y = -data.pose.pose.position.y

            #vins fusion only stereo
            #x cresce pra esquerda
            #y cresce pra tras
            data.pose.pose.position.x = -data.pose.pose.position.x
            data.pose.pose.position.y = -data.pose.pose.position.y
            #d1ata.pose.pose.position.z = -data.pose.pose.position.z

            # Inverter os sentidos dos eixos x e y da velocidade linear e angular
            data.twist.twist.linear.x = -data.twist.twist.linear.y
            data.twist.twist.linear.y = -data.twist.twist.linear.x
            data.twist.twist.linear.z = -data.twist.twist.linear.z
            data.twist.twist.angular.x = -data.twist.twist.angular.y
            data.twist.twist.angular.y = -data.twist.twist.angular.x
            data.twist.twist.angular.z = -data.twist.twist.angular.z
            # Transform the position using the flip matrix
            #position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 1.0])
            #new_position = np.dot(flip_matrix, position)
            #print(new_position)
            #data.pose.pose.position.x = new_position[0]
            #data.pose.pose.position.y = new_position[1]
            #data.pose.pose.position.z = new_position[2]
        
         
        #Atualizar o header 
        data.header.frame_id = 'odom'
        data.child_frame_id = 'base_link'
        #data.header.stamp = rospy.Time.now()
        self.pub.publish(data)
        self.rate.sleep()
 
if __name__ == '__main__':
    rospy.init_node('odom_republisher')
    odom_republisher = OdomRepublisher('/ov_msckf/odomimu', '/mavros/odometry/out', False)
    rospy.spin()


    