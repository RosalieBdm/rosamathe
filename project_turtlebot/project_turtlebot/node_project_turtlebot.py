#-----------------------------------------------------------------------------------#
# PRJReb - Projet TURTLEBOT
# 2023-2024
# Rosalie BIEDERMANN _ Mathilde EBER _ Constantin THEBAUDEAU
# INSA LYON 

# Ce code crée un node qui commande un turtlebot et lui permet de 
#  - suivre une liste de checkpoints 
#  - éviter les autres robots grâce aux calculs de glissement de Zeghal 
#-----------------------------------------------------------------------------------#


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from custom_interfaces.msg import Trajectories
from .utils import *



ANGLE_DEBUT_ALIGNEMENT = 1 #Angle seuil qui déclanche le réalignement 
ANGLE_FIN_ALIGNEMENT = 0.15 #Angle seuil qui stoppe l'alignement 
RAYON_VISIBILITE = 1.5 #Distance seuil à laquelle les voisins sont visibles
DISTANCE_TOLERANCE = 0.1 #Distance seuil à laquelle l'objectif est atteint


class SubscriberPublisher(Node):

    def __init__(self):
        super().__init__('node_projet')

        # Ces paramètres sont indiqués dans le launcher 
        self.declare_parameters('', [
            ('id', -1),
            ('checkpoints', []),
            ('offset_position', [])
        ])

        # ID du robot 
        self.id = self.get_parameter('id').get_parameter_value().integer_value

        # Position de départ 
        self.offset_position = self.get_parameter('offset_position').get_parameter_value().double_array_value

        # la liste des checkpoints est donnée en floats/doubles dans le fichier launch (pas de liste de tuples disponible)
        tmp_list_checkpoints = self.get_parameter('checkpoints').get_parameter_value().double_array_value
        # Création de la liste de checkpoints depuis le tableau de doubles 
        self.list_checkpoints = [(tmp_list_checkpoints[i], tmp_list_checkpoints[i+1]) for i in range(0, len(tmp_list_checkpoints), 2)]
        
        #---------------------------------------#
        # INITIALISATION SUBSCRIBERS/PUBLISHERS # 
        #---------------------------------------#

        # Subscriber qui récupère l'odométrie du robot 
        self.subscriptionTurtle = self.create_subscription(
            Odometry,     # type du message odom = Odometry.msg
            f'/robot_{self.id}/odom',  #topic de l'odométrie 
            self.listener_callback, #callback associé
            10) 
        
        # Subscriber qui récupère la position des voisins 
        self.subscriptionComm = self.create_subscription(
            Trajectories,     # type du message défini dans custom_interfaces
            '/turtle_com',  # topic de communication inter-robots
            self.communication_callback, #callback associé
            10) 
        
        # Publisher qui commande le robot
        self.publisherTurtle = self.create_publisher(
            Twist,  # type du message Twist.msg
            f'/robot_{self.id}/cmd_vel', #topic de commande du robot
            10) 
        
        # Publisher qui donne la position du robot aux voisins
        self.publisherComm = self.create_publisher(
            Trajectories, # type du message défini dans custom_interfaces
            '/turtle_com', # topic de communication inter-robots
            10) 


        #-----------------------------------------#
        # DECLARATION  DES ATTRIBUTS DE LA CLASSE #
        #-----------------------------------------#

        self.index_checkpoint_to_reach = 0 #index du point actuel 
        self.coords_checkpoint_to_reach = self.list_checkpoints[0] # point actuel à atteindre
        self.angleRobotPoint = 0.0 # orientation robot par rapport au point (entre -pi et pi radians)
        self.currentPosition = self.offset_position # position du robot
        self.orientation = 0.0 # orientation du robot par rapport à son référentiel (entre -pi et pi radians)
        self.glissement = 0.0 # valeur du glissement pour éviter les voisins 
        self.state = "walk" #Etat du robot [walk (avancer tout droit), turn (tourner)]

    #--------------------CALLBACK DE MOUVEMENT-----------------------------#
    #Commande le robot en fonction des données odométriques reçues dans msg_vel
    def listener_callback(self, msg_vel: Odometry) -> None:

        # mise à jour des variables globales
        self.currentPosition = msg_vel.pose.pose.position.x + self.offset_position[0], msg_vel.pose.pose.position.y + self.offset_position[1] 
        self.orientation = euler_from_quaternion(msg_vel.pose.pose.orientation)[2]
        
        self.get_logger().info('glissement: ' + str(self.glissement))

        # Si le point objectif est atteint, modification de l'objectif
        if (check_distance(self.currentPosition,self.coords_checkpoint_to_reach, DISTANCE_TOLERANCE)):
            self.index_checkpoint_to_reach += 1
            self.coords_checkpoint_to_reach = self.list_checkpoints[self.index_checkpoint_to_reach % len(self.list_checkpoints)]
        
        #calcul de l'alignement avec le prochain point 
        self.angleRobotPoint = calcul_alignement(self.currentPosition, self.orientation, self.coords_checkpoint_to_reach)


        # ------------- GESTION DES ETATS -------------- #
        
        # Etat walk : avance tout droit et se réaligne en même temps. Si réalignement trop important ->  turn
        # Etat turn : s'arrête et tourne. Si il est aligné -> walk

    
        #Etat turn si l'angle est au dessus du seuil ANGLE_DEBUT_ALIGNEMENT
        if ((abs(self.angleRobotPoint+ self.glissement) > ANGLE_DEBUT_ALIGNEMENT) and self.state == "walk"):
            self.state="turn"
        #Etat walk si l'angle est sous le seuil ANGLE_FIN_ALIGNEMENT
        elif ((abs(self.angleRobotPoint+ self.glissement) < ANGLE_FIN_ALIGNEMENT) and self.state == "turn") : # si on est quasiment alignés avec le point
            self.state="walk"
        #Par défaut, walk
        else:
            self.state="walk"
    

        # ------------ GESTION DEPLACEMENTS ------------ #

        msg_command_motor = Twist()
        msg_command_motor.linear.x = 0.0
        msg_command_motor.linear.y = 0.0
        msg_command_motor.linear.z = 0.0
        msg_command_motor.angular.z = 0.0
        #Avance en ligne droite et tourne en même temps 
        if self.state == "walk":
            msg_command_motor.linear.x = 0.1
            msg_command_motor.angular.z = (self.angleRobotPoint + self.glissement) * 0.5 #Plus l'angle est élevé, plus le robot tourne vite
        #Tourne vers l'objectif 
        elif self.state == "turn":
            msg_command_motor.angular.z = (self.angleRobotPoint + self.glissement) * 0.5 #Plus l'angle est élevé, plus le robot tourne vite
            
        self.publisherTurtle.publish(msg_command_motor)
        
     
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
        if ( msg.id != self.id ): #si le message est envoyé par un voisin
            
            positionVoisin = (msg.x, msg.y)

            #Test avec une condition supplémentaire d'arrêt d'évitement (peu concluant)
            #if (check_distance(self.currentPosition, positionVoisin, RAYON_VISIBILITE) and (calcul_distance(self.coords_checkpoint_to_reach, self.currentPosition) < calcul_distance(self.currentPosition, self.coords_checkpoint_to_reach)):
            
            #Si voisin dans le rayon de visibilité, calcul de zeghal
            if (check_distance(self.currentPosition, positionVoisin, RAYON_VISIBILITE)):
                #calcul du glissement zeghal (angle)
                self.glissement = calcul_zeghal(msg.orientation, positionVoisin, self.orientation, self.currentPosition)
            else:
                self.glissement = 0


def main(args=None):
    rclpy.init(args=args)

    subscriberPublisher = SubscriberPublisher()

    rclpy.spin(subscriberPublisher) # exécuter les callback
    print("Noeuds créés")
    
    subscriberPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()