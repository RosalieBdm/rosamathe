import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from custom_interfaces.msg import Trajectories
import numpy as np
import math
from std_msgs.msg import String
from dataclasses import dataclass


# -------------------
# TODO : trouver comment lancer le talker avec un paramètre ou créer un autre noeud ou tout mettre dans le meme manager (centralisé
# pour lancer aver plusieurs paramètres, créer un laucnher
# -------------------
# Subscribe au topic /tf pour récupérer la position

ANGLE_TOLERANCE = 0.2
DISTANCE_TOLERANCE = 0.1
RAYON_VISIBILITE = 5


class SubscriberPublisher(Node):
    ID = 0

    def __init__(self):
        super().__init__('node_projet')
        
        self.subscriptionTurtle = self.create_subscription(
            Pose,     # type du message odom = Odometry.msg
            '/turtle1/pose',  # topic = odom
            self.listener_callback,
            10) 
        self.publisherTurtle= self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  #topic = nom du topic
        
        

        self.subscriptionComm = self.create_subscription(
            Trajectories,     # type du message odom = Odometry.msg
            '/turtle_com',  # topic = odom
            self.communication_callback, 
            10) 
        #self.subscription  # prevent unused variable warning
        self.publisherComm= self.create_publisher(Trajectories, '/turtle_com', 10)  #topic = nom du topic

        
        if (SubscriberPublisher.ID == 0) :
            self.pointList = [(3.0,5.0),(7.0,5.0)]
        else :
            self.pointList = [(7.0,4.0),(3.0,6.0)]
        
        self.index = 0
        self.currentPoint = self.pointList[0] # prochain objectif
        self.angleRobotPoint = 0.0 #orientation robot par rapport au point
        self.currentPosition = [] #position du robot
        self.orientation = 0.0
        self.glissement = 0.0
        self.id = SubscriberPublisher.ID
        SubscriberPublisher.ID += 1


    def listener_callback(self, msg):
        
        #self.get_logger().info('callback')
        # mise à jour des variables globales
        self.currentPosition = msg.x, msg.y
        self.angleRobotPoint = calcul_alignement(msg, self.currentPoint)
        self.orientation = msg.theta
        
        if (point_atteint(self.currentPosition,self.currentPoint)):
            """
            self.get_logger().info('Ponit Atteint!')
            self.get_logger().info("Entrez la prochaine coordonnée x")
            x = float(input())
            self.get_logger().info("Entrez la prochaine coordonnée y")
            y = float(input())
            self.currentPoint = [x,y]
            """
            #VERSION AVEC LISTE DE POINTS
            
            self.index += 1
            if (self.index >= len(self.pointList)):
                self.index= 0
                self.currentPoint = self.pointList[self.index]
            else:
                self.currentPoint = self.pointList[self.index]
            
            

        else :
            
            if (abs(self.angleRobotPoint) < ANGLE_TOLERANCE) : # si on est quasiment alignés avec le point
                #print("je suis aligné")
                msg = Twist()
                msg.linear.x = 1.0
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
        
        

        msgComm = Trajectories()
        msgComm.id = self.id
        msgComm.x = self.currentPosition[0]
        msgComm.y = self.currentPosition[1]
        msgComm.px=self.currentPoint[0]
        msgComm.py=self.currentPoint[1]
        self.publisherComm.publish(msgComm)

    def communication_callback(self, msg):
        if ( msg.id != self.id ):
            if (detection_voisin(self.currentPosition, msg.position) and calcul_croisement(msg.position, msg.point, self.currentPosition, self.currentPoint)):
                self.glissement = calcul_zeghal(msg.orientation, msg.position, self.orientation, self.currentPosition)
            else:
                self.glisssement = 0

class SubscriberPublisher2(Node):
    ID = 2

    def __init__(self):
        super().__init__('node_projet')
        
        self.subscriptionTurtle = self.create_subscription(
            Pose,     # type du message odom = Odometry.msg
            '/turtle2/pose',  # topic = odom
            self.listener_callback,
            10) 
        self.publisherTurtle= self.create_publisher(Twist, '/turtle2/cmd_vel', 10)  #topic = nom du topic
        
        

        self.subscriptionComm = self.create_subscription(
            Trajectories,     # type du message odom = Odometry.msg
            '/turtle_com',  # topic = odom
            self.communication_callback, 
            10) 
        #self.subscription  # prevent unused variable warning
        self.publisherComm= self.create_publisher(Trajectories, '/turtle_com', 10)  #topic = nom du topic

        
        if (SubscriberPublisher.ID == 0) :
            self.pointList = [(3.0,5.0),(7.0,5.0)]
        else :
            self.pointList = [(7.0,4.0),(3.0,6.0)]
        
        self.index = 0
        self.currentPoint = self.pointList[0] # prochain objectif
        self.angleRobotPoint = 0.0 #orientation robot par rapport au point
        self.currentPosition = [] #position du robot
        self.orientation = 0.0
        self.glissement = 0.0
        self.id = SubscriberPublisher.ID
        SubscriberPublisher.ID += 1


    def listener_callback(self, msg):
        
        #self.get_logger().info('callback')
        # mise à jour des variables globales
        self.currentPosition = msg.x, msg.y
        self.angleRobotPoint = calcul_alignement(msg, self.currentPoint)
        self.orientation = msg.theta
        
        if (point_atteint(self.currentPosition,self.currentPoint)):
            """
            self.get_logger().info('Ponit Atteint!')
            self.get_logger().info("Entrez la prochaine coordonnée x")
            x = float(input())
            self.get_logger().info("Entrez la prochaine coordonnée y")
            y = float(input())
            self.currentPoint = [x,y]
            """
            #VERSION AVEC LISTE DE POINTS
            
            self.index += 1
            if (self.index >= len(self.pointList)):
                self.index= 0
                self.currentPoint = self.pointList[self.index]
            else:
                self.currentPoint = self.pointList[self.index]
            
            

        else :
            
            if (abs(self.angleRobotPoint) < ANGLE_TOLERANCE) : # si on est quasiment alignés avec le point
                #print("je suis aligné")
                msg = Twist()
                msg.linear.x = 1.0
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
        
        

        msgComm = Trajectories()
        msgComm.id = self.id
        msgComm.x = self.currentPosition[0]
        msgComm.y = self.currentPosition[1]
        msgComm.px=self.currentPoint[0]
        msgComm.py=self.currentPoint[1]
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
    subscriberPublisher2 = SubscriberPublisher2()
    rclpy.spin(subscriberPublisher) # executer les callback
    rclpy.spin(subscriberPublisher2) # executer les callback
    print("Noeuds créés")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriberPublisher.destroy_node()
    rclpy.shutdown()

def calcul_alignement(pose, point):
    #calculer
    orientation_robot = pose.theta
    vecteur_robot_point = [point[0] - pose.x, point[1] - pose.y]
    if (vecteur_robot_point[0] == 0):
        return -1
    anglePoint = math.atan(vecteur_robot_point[1]/vecteur_robot_point[0] ) # angle par rapoport à l'axe x autour de l'axe z

    if ((pose.x - point[0]) > 0):
        anglePoint += math.pi
    
    if anglePoint < 0 :
        anglePoint += 2*math.pi

    if orientation_robot < 0:
        orientation_robot += 2*math.pi

    print("anglePoint : " + str(anglePoint))
    print("orientation : " + str(orientation_robot))
    if (anglePoint - orientation_robot) > math.pi:
        return anglePoint - orientation_robot - 2*math.pi
    elif (anglePoint - orientation_robot) < - math.pi:
        return anglePoint - orientation_robot + 2*math.pi
    return anglePoint - orientation_robot# angle depuis l'orientation du robot vers le vecteur point-robot 


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