import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from custom_interfaces.msg import Trajectories
from .utils import *

ANGLE_DEBUT_ALIGNEMENT = 1 #Angle seuil qui déclanche le réalignement 
ANGLE_FIN_ALIGNEMENT = 0.15 #Angle seuil qui stoppe l'alignement 
RAYON_VISIBILITE = 1.5#Distance seuil à laquelle les voisins sont visibles



class SubscriberPublisher(Node):

    def __init__(self):
        super().__init__('node_projet')

        self.declare_parameters('', [
            ('id', -1),
            ('checkpoints', []),
            ('offset_position', [])
        ])
        self.id = self.get_parameter('id').get_parameter_value().integer_value

        self.offset_position = self.get_parameter('offset_position').get_parameter_value().double_array_value

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
        self.distanceVoisin=0
        #-----------------------------------------#
        # DECLARATION  DES ATTRIBUTS DE LA CLASSE #
        #-----------------------------------------#
        self.index_checkpoint_to_reach = 0 #index du point actuel 
        self.coords_checkpoint_to_reach = self.list_checkpoints[0] # point actuel à atteindre
        self.angleRobotPoint = 0.0 # orientation robot par rapport au point (entre -pi et pi radians)
        self.currentPosition = self.offset_position # position du robot
        self.orientation = 0.0 # orientation du robot par rapport à son référentiel (entre -pi et pi radians)
        self.glissement = 0.0 # valeur du glissement pour éviter les voisins (pour l'instant , entre -pi et pi radians, à modifier par la suite)
        self.state = "walk" #Etat du robot [walk (avancer tout droit), turn (tourner), stop (s'arrêter)]

    #--------------------CALLBACK DE MOUVEMENT-----------------------------#
    #Commande le robot en fonction des données odométriques reçues dans msg
    def listener_callback(self, msg_vel: Odometry) -> None:

        # mise à jour des variables globales
        self.currentPosition = msg_vel.pose.pose.position.x + self.offset_position[0], msg_vel.pose.pose.position.y + self.offset_position[1] 
        self.orientation = euler_from_quaternion(msg_vel.pose.pose.orientation)[2]
        
        self.get_logger().info('glissement: ' + str(self.glissement))

        # Si le point objectif est atteint, modification de l'objectif
        if (point_atteint(self.currentPosition,self.coords_checkpoint_to_reach)):

            #VERSION AVEC LISTE DE POINTS
            
            self.index_checkpoint_to_reach += 1
            self.coords_checkpoint_to_reach = self.list_checkpoints[self.index_checkpoint_to_reach % len(self.list_checkpoints)]
            #self.get_logger().info('Prochain Point: ' + str(self.currentPoint))
            
        # Si le point objectif n'est pas atteint, gestion des états et déplacement 
        
        # Etat walk : avance tout droit. Si il n'est pas aligné, ->  turn. Si il croise un voisin, -> stop
        # Etat turn : tourne. Si il est aligné -> stop
        # Etat stop: stop (réduit le problème d'arc de cercle). Si il n'y a plus de voisin dans le périmètre, -> walk

        # Test pour arrêter le robot si il a un voisin dans son périmètre de vision 
        #(Pour l'instant, il reste arrêté indéfiniement, il faut modifier pour qu'il évite le voisin au lieu de s'arrêter)
        #if self.glissement!=0:
        #    self.state="stop"
        # Repart si pas de voisin

        self.angleRobotPoint = calcul_alignement(self.currentPosition, self.orientation, self.coords_checkpoint_to_reach)
            
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

        msg_command_motor = Twist()
        msg_command_motor.linear.x = 0.0
        msg_command_motor.linear.y = 0.0
        msg_command_motor.linear.z = 0.0
        msg_command_motor.angular.z = 0.0
        #Avance en ligne droite
        if self.state == "walk":
            msg_command_motor.linear.x = 0.1
            msg_command_motor.angular.z = (self.angleRobotPoint + self.glissement) * 0.5
            #Tourne vers l'objectif 
        elif self.state == "turn":
            #self.get_logger().info('Pas aligné' + str(self.angleRobotPoint))
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
        if ( msg.id != self.id ): #si le voisin est différent du robot
            positionVoisin = (msg.x, msg.y)
            """
            #Calcul du glissement avec zeghal, à revoir 
            # détection voisin renvoie true si voisin dans rayon visibilité et calcul_croisement renvoie true si trajectoire voisin croise trajectoire robot (à tester)
            if (detection_voisin(self.currentPosition, (msg.x, msg.y)) and calcul_croisement((msg.x, msg.y), (msg.px, msg.py), self.currentPosition, self.currentPoint)):
                self.glissement = calcul_zeghal(msg.orientation, (msg.x, msg.y), self.orientation, self.currentPosition)
            else:
                self.glissement = 0
            """
            #Si voisin dans le rayon de visibilité, calcul de zeghal
            #if detection_voisin(self.currentPosition, positionVoisin, RAYON_VISIBILITE):
               
            if (detection_voisin(self.currentPosition, positionVoisin, RAYON_VISIBILITE) and (calcul_distance(self.coords_checkpoint_to_reach, self.currentPosition) < calcul_distance(self.currentPosition, self.coords_checkpoint_to_reach) or calcul_distance((msg.px,msg.py), self.currentPosition) < calcul_distance((msg.px,msg.py), positionVoisin)) ):
                #calcul du glissement zeghal (angle)
                self.get_logger().info('robot_voisin (r1 r2): ' + str(calcul_angle_vecteur(self.currentPosition, positionVoisin)))
                self.get_logger().info('v1 v2: ' + str(self.orientation - msg.orientation) )
                self.get_logger().info('orientation: ' + str(self.orientation ) )
                angle_zeghal = calcul_zeghal(msg.orientation, positionVoisin, self.orientation, self.currentPosition)

                #angle_zeghal < pi -> projection vers le haut 
                if (angle_zeghal) < math.pi :
                    #v1 dans la moitié droite du cercle -> glissement dans le sens trigo 
                    if self.orientation < math.pi/2 or self.orientation > 3*math.pi/2 :
                        self.glissement = 1.0/(calcul_distance(self.currentPosition, positionVoisin))*1.2
                    #v1 dans la moitié gauche du cercle -> glissement dans le sens inverse 
                    else :
                        self.glissement =  - 1.0/(calcul_distance(self.currentPosition, positionVoisin))*1.2
                #angle_zeghal > pi -> projection vers le bas 
                else : 
                    #v1 dans la moitié droite du cercle -> glissement dans le sens inverse 
                    if self.orientation < math.pi/2 or self.orientation > 3*math.pi/2 :

                        self.glissement = - 1.0/(calcul_distance(self.currentPosition, positionVoisin))*1.2
                    #v1 dans la moitié gauche du cercle -> glissement dans le sens trigo 
                    else :
                        self.glissement =   1.0/(calcul_distance(self.currentPosition, positionVoisin))*1.2
                
                self.get_logger().info('zeghal: ' + str(angle_zeghal))
            else:
                self.distanceVoisin = 10
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