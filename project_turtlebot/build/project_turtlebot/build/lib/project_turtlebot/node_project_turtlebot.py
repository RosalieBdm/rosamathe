import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from turtlesim.msg import Pose
import numpy as np
import math
#import custom_msg
from std_msgs.msg import String
from dataclasses import dataclass
from custom_msg.msg import message_communication


# -------------------
# TODO : mettre le publisher dans le subscriber pour avoir q'un seul node 
# -------------------
# Subscribe au topic /tf pour récupérer la position

ANGLE_TOLERANCE = 1
DISTANCE_TOLERANCE = 0.5
RAYON_VISIBILITE = 5
ID = 0


class SubscriberPublisher(Node):

    def __init__(self):
        super().__init__('node_projet')
        self.subscriptionTurtle = self.create_subscription(
            Odometry,     # type du message odom = Odometry.msg
            '/odom',  # topic = odom
            self.listener_callback,
            10) 
   
        self.publisherTurtle= self.create_publisher(Twist, '/cmd_vel', 10)  #topic = nom du topic
        
        self.subscriptionComm = self.create_subscription(
            message_communication,     # type du message odom = Odometry.msg
            '/turtle_com',  # topic = odom
            self.communication_callback,
            10) 
        
        self.publisherComm= self.create_publisher(message_communication, '/turtle_com', 10)  #topic = nom du topic

        self.pointList = [(0.0,0.0),(1.0,1.0),(-1.0,1.0)]
        self.index = 0
        self.currentPoint = self.pointList[0] # prochain objectif
        self.angleRobotPoint = 0.0 #orientation robot par rapport au point
        self.currentPosition = [] #position du robot
        self.orientation = 0.0
        self.glissement = 0.0
        self.id = ID


    def listener_callback(self, msg):
        
        #self.get_logger().info('callback')
        # mise à jour des variables globales
        self.currentPosition = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.angleRobotPoint = calcul_alignement(msg.pose.pose, self.currentPoint)
        
        
        if (point_atteint(self.currentPosition,self.currentPoint)):
            
            self.get_logger().info('Ponit Atteint!')
            self.get_logger().info("Entrez la prochaine coordonnée x")
            x = float(input())
            self.get_logger().info("Entrez la prochaine coordonnée y")
            y = float(input())
            self.currentPoint = [x,y]

            #VERSION AVEC LISTE DE POINTS
            """
            self.index += 1
            if (self.index >= len(self.pointList)):
                self.index= 0
                self.currentPoint = self.pointList[self.index]
            else:
                self.currentPoint = self.pointList[self.index]
            """
            

        else :
            
            if (abs(self.angleRobotPoint) < 1) : # si on est quasiment alignés avec le point
                #print("je suis aligné")
                msg = Twist()
                msg.linear.x = 0.1
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = 0.0
                self.publisherTurtle.publish(msg) # publish tout droit 
            else :
                self.get_logger().info('Pas aligné' + str(self.angleRobotPoint))
                #print("pas aligné" , self.angleRobotPoint)
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = self.angleRobotPoint * 0.2
                
                self.publisherTurtle.publish(msg) # publish angle
        
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)

        msgComm = MessageCommunication()
        msgComm.id = ID
        msgComm.position = self.currentPosition
        msgComm.point = self.currentPoint
        msgComm.orientation = self.orientation
        self.publisherComm.publish(msgComm)

    def communication_callback(self, msg):
        if ( msg.id != self.id ):
            if (detection_voisin(self.currentPosition, msg.position) and calcul_croisement(msg.position, msg.point, self.currentPosition, self.currentPoint)):
                self.glissement = calcul_zeghal(msg.orientation, msg.position, self.orientation, self.currentPosition)
            else:
                self.glisssement = 0



def main(args=None):
    rclpy.init(args=args)

    subscriberPublisher = SubscriberPublisher()

    rclpy.spin(subscriberPublisher) # executer les callback
    print("Noeuds créés")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriberPublisher.destroy_node()
    rclpy.shutdown()

def calcul_alignement(pose, point):
    #calculer
    orientation_robot = euler_from_quaternion(pose.orientation)[2]
    vecteur_robot_point = [point[0] - pose.position.x, point[1] - pose.position.y]
    anglePoint = math.atan2(vecteur_robot_point[1] - 0, vecteur_robot_point[0] ) # angle par rapoport à l'axe x autour de l'axe z
    return anglePoint - orientation_robot # angle depuis l'orientation du robot vers le vecteur point-robot 


def point_atteint(position, point):
    distance = math.sqrt((position[0] - point[0])**2 + (position[1] - point[1])**2)
    print("distance :", distance)
    if (distance < DISTANCE_TOLERANCE) :
        return True
    return False


def ccw(A,B,C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

# return true si les trajectoires se croisent
def calcul_croisement(pos_voisin, point_voisin, pos_robot, point_robot):
    return ccw(pos_voisin, pos_robot, point_robot) != ccw(point_voisin, pos_robot, point_robot) and ccw(pos_voisin, point_voisin, pos_robot) != ccw(pos_voisin, point_voisin, point_robot)


def calcul_zeghal(orientation_voisin, pos_voisin, orientation_robot, pos_robot):
    angle_robot_voisin = calcul_alignement(pos_robot, pos_voisin) # angle du vecteur robot-voisin avec comme référentiel l'orientation du robot
    angle_orientation_robot_voisin = orientation_robot - orientation_voisin #différence entre l'orientation du robot et l'orientation du voisin 
    return angle_robot_voisin - angle_orientation_robot_voisin

def detection_voisin(pos_robot, pos_voisin):
    distance = math.sqrt((pos_robot[0] - pos_voisin[0])**2 + (pos_robot[1] - pos_voisin[1])**2)

    if (distance < RAYON_VISIBILITE) :
        return True
    return False


def euler_from_quaternion(quaternion):
    x = quaternion.x 
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    t0= +2.0*(w*x+y*z)
    t1=+1.0-2.0*(x*x+y*y)
    roll_x= math.atan2(t0,t1)
    t2 = +2.0 * (w*y-z*x)
    t2= +1.0 if t2 > +1.0 else t2
    t2= -1.0 if t2< -1.0 else t2
    pitch_y=math.asin(t2)
    t3=+2.0 * (w*z + x*y)
    t4 = +1.0 - 2.0 * (y*y + z*z)
    yaw_z= math.atan2(t3,t4)

    return(roll_x, pitch_y, yaw_z)

if __name__ == '__main__':
    main()