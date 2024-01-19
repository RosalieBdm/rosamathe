import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from custom_interfaces.msg import Trajectories
from .utils import *

ANGLE_DEBUT_ALIGNEMENT = 1 #Angle seuil qui déclanche le réalignement 
ANGLE_FIN_ALIGNEMENT = 0.15 #Angle seuil qui stoppe l'alignement 
RAYON_VISIBILITE = 1.5 #Distance seuil à laquelle les voisins sont visibles



class SubscriberPublisher(Node):

    def __init__(self):
        super().__init__('node_projet')

        self.declare_parameters('', [
            ('id', -1),
            ('checkpoints', [])
        ])
        self.id = self.get_parameter('id').get_parameter_value().integer_value

        # the list of checkpoints is given as a list of floats/doubles in the launch file (no list of tuples type available)
        tmp_list_checkpoints = self.get_parameter('checkpoints').get_parameter_value().double_array_value
        # create the list of checkpoints from this array of doubles
        self.list_checkpoints = [(tmp_list_checkpoints[i], tmp_list_checkpoints[i+1]) for i in range(0, len(tmp_list_checkpoints), 2)]
        
        #---------------------------------------#
        # INITIALISATION SUBSCRIBERS/PUBLISHERS # 
        #---------------------------------------#

        # Subscriber qui récupère l'odométrie du robot 
        self.subscriptionTurtle = self.create_subscription(
            Odometry,     # type du message odom = Odometry.msg
            f'/robot_{self.id}/odom',  # A TESTER, 
            self.listener_callback,
            10) 
        
        # Subscriber qui récupère la position des voisins 
        self.subscriptionComm = self.create_subscription(
            Trajectories,     # type du message odom = Odometry.msg
            '/turtle_com',  # topic = odom
            self.communication_callback,
            10) 
        
        # Publisher qui commande le robot
        self.publisherTurtle= self.create_publisher(Twist, f'/robot_{self.id}/cmd_vel', 10)  #topic = nom du topic
        
        # Publisher qui donne la position du robot aux voisins
        self.publisherComm= self.create_publisher(Trajectories, '/turtle_com', 10)  #topic = nom du topic

        #-----------------------------------------#
        # DECLARATION  DES ATTRIBUTS DE LA CLASSE #
        #-----------------------------------------#
        self.index_checkpoint_to_reach = 0 #index du point actuel 
        self.coords_checkpoint_to_reach = self.list_checkpoints[0] # point actuel à atteindre
        self.angleRobotPoint = 0.0 # orientation robot par rapport au point (entre -pi et pi radians)
        self.currentPosition = (0.0, 0.0) # position du robot
        self.orientation = 0.0 # orientation du robot par rapport à son référentiel (entre -pi et pi radians)
        self.glissement = 0.0 # valeur du glissement pour éviter les voisins (pour l'instant , entre -pi et pi radians, à modifier par la suite)
        self.state = "walk" #Etat du robot [walk (avancer tout droit), turn (tourner), stop (s'arrêter)]

    #--------------------CALLBACK DE MOUVEMENT-----------------------------#
    #Commande le robot en fonction des données odométriques reçues dans msg
    def listener_callback(self, msg: Odometry) -> None:

        # mise à jour des variables globales
        self.currentPosition = msg.pose.pose.position.x, msg.pose.pose.position.y 
        self.orientation = euler_from_quaternion(msg.pose.pose.orientation)[2]
        self.angleRobotPoint = calcul_alignement(self.currentPosition, self.orientation, self.coords_checkpoint_to_reach)
        self.get_logger().info('glissement: ' + str(self.glissement))

        # Si le point objectif est atteint, modification de l'objectif
        if (point_atteint(self.currentPosition,self.coords_checkpoint_to_reach)):

            #VERSION AVEC LISTE DE POINTS
            
            self.index_checkpoint_to_reach += 1
            self.coords_checkpoint_to_reach = self.list_checkpoints[self.index_checkpoint_to_reach % len(self.list_checkpoints)]
            #self.get_logger().info('Prochain Point: ' + str(self.currentPoint))
            
        # Si le point objectif n'est pas atteint, gestion des états et déplacement 
        else :
            # Etat walk : avance tout droit. Si il n'est pas aligné, ->  turn. Si il croise un voisin, -> stop
            # Etat turn : tourne. Si il est aligné -> stop
            # Etat stop: stop (réduit le problème d'arc de cercle). Si il n'y a plus de voisin dans le périmètre, -> walk

            # Test pour arrêter le robot si il a un voisin dans son périmètre de vision 
            #(Pour l'instant, il reste arrêté indéfiniement, il faut modifier pour qu'il évite le voisin au lieu de s'arrêter)
            #if self.glissement!=0:
            #    self.state="stop"
            # Repart si pas de voisin
            
            if self.state =="stop":
                self.state="walk"
            #Arrête de tourner si angle sous le seuil ANGLE_FIN_ALIGNEMENT
            elif ((abs(self.angleRobotPoint+ self.glissement) < ANGLE_FIN_ALIGNEMENT) and self.state == "turn") : # si on est quasiment alignés avec le point
                self.state="stop"
            #Commence à tourner si l'angle est au dessus du seuil ANGLE_DEBUT_ALIGNEMENT
            elif ((abs(self.angleRobotPoint+ self.glissement) > ANGLE_DEBUT_ALIGNEMENT) and self.state == "walk"):
                self.state="walk"
            #Par défaut, avance 

            #---GESTION DEPLACEMENTS---#
                
            #Avance en ligne droite
            if self.state == "walk":
                msg = Twist()
                msg.linear.x = 0.1
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = (self.angleRobotPoint + self.glissement) * 0.5
                self.publisherTurtle.publish(msg)
            #Tourne vers l'objectif 
            elif self.state == "turn":
                #self.get_logger().info('Pas aligné' + str(self.angleRobotPoint))
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = (self.angleRobotPoint + self.glissement) * 0.5 #Plus l'angle est élevé, plus le robot tourne vite
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
        msgComm.px=self.coords_checkpoint_to_reach[0]
        msgComm.py=self.coords_checkpoint_to_reach[1]
        msgComm.orientation=self.orientation
        self.publisherComm.publish(msgComm)

    #------------------------CALLBACK COMMUNICATION VOISINS---------------------#
    #Calcule la valeur du glissement si le voisin est dans le rayon de visibilité
    def communication_callback(self, msg: Trajectories) -> None:
        if ( msg.id != self.id ): #si le voisin est différent du robot
            
            """
            #Calcul du glissement avec zeghal, à revoir 
            # détection voisin renvoie true si voisin dans rayon visibilité et calcul_croisement renvoie true si trajectoire voisin croise trajectoire robot (à tester)
            if (detection_voisin(self.currentPosition, (msg.x, msg.y)) and calcul_croisement((msg.x, msg.y), (msg.px, msg.py), self.currentPosition, self.currentPoint)):
                self.glissement = calcul_zeghal(msg.orientation, (msg.x, msg.y), self.orientation, self.currentPosition)
            else:
                self.glissement = 0
            """
            #Si voisin dans le rayon de visibilité, calcul de zeghal
            if (detection_voisin(self.currentPosition, (msg.x, msg.y), RAYON_VISIBILITE)):
                #calcul du glissement zeghal (angle)
                self.glissement = calcul_zeghal(msg.orientation, (msg.x, msg.y), self.orientation, self.currentPosition)
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


if __name__ == '__main__':
    main()