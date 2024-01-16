import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

from project_interfaces.msg import Trajectory


ANGLE_DEBUT_ALIGNEMENT = 0.25 #Angle seuil qui déclanche le réalignement 
ANGLE_FIN_ALIGNEMENT = 0.15 #Angle seuil qui stoppe l'alignement 
DISTANCE_TOLERANCE = 0.1 #Distance seuil à laquelle l'objectif est atteint
RAYON_VISIBILITE = 1 #Distance seuil à laquelle les voisins sont visibles
ID = 0


class SubscriberPublisher(Node):

    def __init__(self):
        super().__init__('node_projet')
        
        #---------------------------------------#
        # INITIALISATION SUBSCRIBERS/PUBLISHERS # 
        #---------------------------------------#

        # Subscriber qui récupère l'odométrie du robot 
        self.subscriptionTurtle = self.create_subscription(
            Odometry,     # type du message odom = Odometry.msg
            '/robot_1/odom',  # topic = odom
            self.listener_callback,
            10) 
        
        # Subscriber qui récupère la position des voisins 
        self.subscriptionComm = self.create_subscription(
            Trajectories,     # type du message odom = Odometry.msg
            '/turtle_com',  # topic = odom
            self.communication_callback,
            10) 
        
        # Publisher qui commande le robot
        self.publisherTurtle= self.create_publisher(Twist, '/robot_1/cmd_vel', 10)  #topic = nom du topic
        
        # Publisher qui donne la position du robot aux voisins
        self.publisherComm= self.create_publisher(Trajectories, '/turtle_com', 10)  #topic = nom du topic

        #-----------------------------------------#
        # DECLARATION  DES ATTRIBUTS DE LA CLASSE #
        #-----------------------------------------#
        self.pointList = [(2.0,0.0),(0.0,0.0)] #liste des points à atteindre
        self.index = 0 #index du point actuel 
        self.currentPoint = self.pointList[0] # point actuel à atteindre
        self.angleRobotPoint = 0.0 # orientation robot par rapport au point (entre -pi et pi radians)
        self.currentPosition = [0.0,0.0] # position du robot
        self.orientation = 0.0 # orientation du robot par rapport à son référentiel (entre -pi et pi radians)
        self.glissement = 0.0 # valeur du glissement pour éviter les voisins (pour l'instant , entre -pi et pi radians, à modifier par la suite)
        self.state = "walk" #Etat du robot [walk (avancer tout droit), turn (tourner), stop (s'arrêter)]
        self.id = ID #ID du robot

    #--------------------CALLBACK DE MOUVEMENT-----------------------------#
    #Commande le robot en fonction des données odométriques reçues dans msg
    def listener_callback(self, msg):

        # mise à jour des variables globales
        self.currentPosition = msg.pose.pose.position.x, msg.pose.pose.position.y 
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)[2]
        self.angleRobotPoint = calcul_alignement(self.currentPosition, self.orientation, self.currentPoint)
        self.get_logger().info('glissement: ' + str(self.glissement))

        # Si le point objectif est atteint, modification de l'objectif
        if (point_atteint(self.currentPosition,self.currentPoint)):
            
            """
            # VERSION AVEC LES ENTREES UTILISATEUR #
            # Le robot a pour objectif le point entré en input#
            self.get_logger().info('Point Atteint!')
            self.get_logger().info("Entrez la prochaine coordonnée x")
            x = float(input())
            self.get_logger().info("Entrez la prochaine coordonnée y")
            y = float(input())
            self.currentPoint = [x,y]
            """

            #VERSION AVEC LISTE DE POINTS
            
            self.index += 1
            if (self.index >= len(self.pointList)): #cas où on a atteint la fin de la liste : on recommence depuis le premier élément
                self.index= 0
                self.currentPoint = self.pointList[self.index]
            else:
                self.currentPoint = self.pointList[self.index]
            #self.get_logger().info('Prochain Point: ' + str(self.currentPoint))
            
        # Si le point objectif n'est pas atteint, gestion des états et déplacement 
        else :
            # Etat walk : avance tout droit. Si il n'est pas aligné, ->  turn. Si il croise un voisin, -> stop
            # Etat turn : tourne. Si il est aligné -> stop
            # Etat stop: stop (réduit le problème d'arc de cercle). Si il n'y a plus de voisin dans le périmètre, -> walk

            # Test pour arrêter le robot si il a un voisin dans son périmètre de vision 
            #(Pour l'instant, il reste arrêté indéfiniement, il faut modifier pour qu'il évite le voisin au lieu de s'arrêter)
            if self.glissement!=0:
                self.state="stop"
            # Repart si pas de voisin
            elif self.state =="stop":
                self.state="walk"
            #Arrête de tourner si angle sous le seuil ANGLE_FIN_ALIGNEMENT
            elif ((abs(self.angleRobotPoint) < ANGLE_FIN_ALIGNEMENT) and self.state == "turn") : # si on est quasiment alignés avec le point
                self.state="stop"
            #Commence à tourner si l'angle est au dessus du seuil ANGLE_DEBUT_ALIGNEMENT
            elif ((abs(self.angleRobotPoint) > ANGLE_DEBUT_ALIGNEMENT) and self.state == "walk"):
                self.state="turn"
            #Par défaut, avance 
            else:
                self.state = "walk"
            
            

            #---GESTION DEPLACEMENTS---#
                
            #Avance en ligne droite
            if self.state == "walk":
                msg = Twist()
                msg.linear.x = 0.1
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = 0.0
                self.publisherTurtle.publish(msg)
            #Tourne vers l'objectif 
            elif self.state == "turn":
                #self.get_logger().info('Pas aligné' + str(self.angleRobotPoint))
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = self.angleRobotPoint * 0.5 #Plus l'angle est élevé, plus le robot tourne vite
                self.publisherTurtle.publish(msg)
            #Reste immobile
            else: 
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = 0.0
                self.publisherTurtle.publish(msg)
        
     
        #ENVOI DE LA POSITION AUX VOISINS 
        msgComm = Trajectories()
        msgComm.id = self.id 
        msgComm.x = self.currentPosition[0]
        msgComm.y = self.currentPosition[1]
        msgComm.px=self.currentPoint[0]
        msgComm.py=self.currentPoint[1]
        self.publisherComm.publish(msgComm)

    #------------------------CALLBACK COMMUNICATION VOISINS---------------------#
    #Calcule la valeur du glissement si le voisin est dans le rayon de visibilité et risque de croiser 
    def communication_callback(self, msg):
        if ( msg.id != self.id ): #si le voisin est différent du robot
            """
            #Calcul du glissement avec zeghal, à revoir 
            # détection voisin renvoie true si voisin dans rayon visibilité et calcul_croisement renvoie true si trajectoire voisin croise trajectoire robot (à tester)
            if (detection_voisin(self.currentPosition, (msg.x, msg.y)) and calcul_croisement((msg.x, msg.y), (msg.px, msg.py), self.currentPosition, self.currentPoint)):
                self.glissement = calcul_zeghal(msg.orientation, (msg.x, msg.y), self.orientation, self.currentPosition)
            else:
                self.glissement = 0
            """
            #Test pour déclancher l'état stop si le voisin est dans le périmètre
            if (detection_voisin(self.currentPosition, (msg.x, msg.y))):
                self.glissement = 10
            else:
                self.glissement = 0



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

#calcul de l'angle entre deux vecteurs : vecteur orientation du robot et vecteur robot-point
# position_robot : [float , float] , position du robot
# orientation_robot : float , orientation du robot dans son référentiel, valeur dans [-pi, pi]
# point : [float , float] , point à atteindre 
def calcul_alignement(position_robot, orientation_robot, point):
    # cette fonction est aussi utilisée pour calculer l'angle entre le vecteur orientation du robot et le vecteur robot-voisin
    # dans ce cas, point est la position du voisin 


    #calcul du vecteur robot-point 
    vecteur_robot_point = [point[0] - position_robot[0], point[1] - position_robot[1]]

    # Cas où le robot est exactement sur l'objectif
    if (vecteur_robot_point[0] == 0):
        return -1
    
    # calcul de l'angle du vecteur robot-point dans le référentiel du robot (valeur dans [-pi/2,pi/2])
    anglePoint = math.atan(vecteur_robot_point[1]/vecteur_robot_point[0] )

    # Si le vecteur est dans la moitié gauche du cercle trigo, on ajoute une rotation de pi pour que la valeur soit dans [-pi/2, 3pi/2]
    if (vecteur_robot_point[0] > 0):
        anglePoint += math.pi
    
    # Normalisation des angles sur [0, 2pi]
    if anglePoint < 0 :
        anglePoint += 2*math.pi
    if orientation_robot < 0:
        orientation_robot += 2*math.pi
    
    #print("anglePoint : " + str(anglePoint))
    #print("orientation : " + str(orientation_robot))
    
    # Calcul de l'angle recherché (valeur dans [-2pi, 2pi])
    theta = anglePoint - orientation_robot
    # Normalisation de l'angle sur [-pi, pi]
    if theta > math.pi:
        theta -= 2*math.pi
    elif theta < - math.pi:
       theta += 2*math.pi
    return theta 
    

# Return true si le robot est suffisament proche de l'objectif 
# position_robot : [float , float] , position du robot
# point : [float , float] , point à atteindre 
def point_atteint(position, point):
    # calcul de la distance à l'objectif 
    distance = math.sqrt((position[0] - point[0])**2 + (position[1] - point[1])**2)

    if (distance < DISTANCE_TOLERANCE) :
        return True
    return False

# Fonction utile pour croisement (récupérée sur internet)
def ccw(A,B,C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

# Return true si les trajectoires se croisent (récupérée sur internet)
# pos_voisin : [float , float] , position du voisin
# point_voisin : [float , float] , point à atteindre pour le voisin
# pos_robot : [float , float] , position du robot
# point_robot : [float , float] , point à atteindre pour le robot
def calcul_croisement(pos_voisin, point_voisin, pos_robot, point_robot):
    return ccw(pos_voisin, pos_robot, point_robot) != ccw(point_voisin, pos_robot, point_robot) and ccw(pos_voisin, point_voisin, pos_robot) != ccw(pos_voisin, point_voisin, point_robot)


# Return true si le robot détecte un voisin dans son rayon de visibilité
# pos_robot : [float , float] , position du robot
# pos_voisin : [float , float] , position du voisin
def detection_voisin(pos_robot, pos_voisin):
    distance = math.sqrt((pos_robot[0] - pos_voisin[0])**2 + (pos_robot[1] - pos_voisin[1])**2)

    if (distance < RAYON_VISIBILITE) :
        print("VOISIN")
        return True
    return False


# Calcul les forces de glissement (à compléter)
# pos_voisin : [float , float] , position du voisin
# orientation_voisin : float , orientation du voisin dans son référentiel, valeur dans [-pi, pi]
# pos_robot : [float , float] , position du robot
# orientation_robot : float , orientation du robot dans son référentiel, valeur dans [-pi, pi]
def calcul_zeghal(orientation_voisin, pos_voisin, orientation_robot, pos_robot):
    #calcul de l'angle entre l'orientation du robot et le vecteur robot-voisin
    angle_robot_voisin = calcul_alignement(pos_robot,orientation_robot, pos_voisin) 
    #calcul de la différence entre l'orientation du robot et l'orientation du voisin 
    angle_orientation_robot_voisin = orientation_robot - orientation_voisin 

    #à compléter (pourquoi on retourne ça ?)
    return angle_robot_voisin - angle_orientation_robot_voisin


# Tranforme un Quaternion en angles d'euler (récupérée sur internet)
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