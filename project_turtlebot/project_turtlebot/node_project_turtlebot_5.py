import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from typing import Tuple, Iterable

from project_interfaces.msg import Trajectory


ANGLE_DEBUT_ALIGNEMENT = 0.25                                                   #Angle seuil qui déclanche le réalignement 
ANGLE_FIN_ALIGNEMENT = 0.15                                                     #Angle seuil qui stoppe l'alignement 
DISTANCE_CHECKPOINT_THRESHOLD = 0.1                                             #Distance seuil à laquelle l'objectif est atteint
RADIUS_VISIBILITY = 1                                                           #Distance seuil à laquelle les voisins sont visibles
LINEAR_SPEED_DEFAULT = 0.1                                                      # the linear speed


class Turtlebot(Node):

    def __init__(self, id: int, list_checkpoints: Iterable[Tuple[float, float]]):
        super().__init__('node_projet')

        #---------------------------------------#
        # INITIALISATION SUBSCRIBERS/PUBLISHERS # 
        #---------------------------------------#

        self.subscriber_turtlebot_odometry = self.create_subscription(          # Subscriber to get the odometry of the robot
            Odometry,                                                           # message type
            '/robot_5/odom',                                                    # topic to subscribe to
            self.turtlebot_odometry_callback,                                   # callback to handle the message
            10                                                                  # queue size
        ) 

        self.subscriber_turtlebot_trajectories = self.create_subscription(      # Subscriber to get the other robot positions and checkpoints that they want to reach
            Trajectory,                                                         # message type
            '/turtle_com',                                                      # topic to subscribe to
            self.trajectory_callback,                                           # callback to handle the message
            10                                                                  # queue size
        ) 
        
        self.publisher_turtlebot_trajectories = self.create_publisher(          # Publisher to give the robot position and checkpoint to reach to others
            Trajectory,                                                         # message type
            '/turtle_com',                                                      # topic to publish to
            10                                                                  # queue size
        )

        self.publisher_turtlebot_motors_controller = self.create_publisher(     # Publisher to control the robot movements
            Twist,                                                              # message type
            '/robot_5/cmd_vel',                                                 # topic to publish to
            10                                                                  # queue size
        )

        #-----------------------------------------#
        # DECLARATION DES ATTRIBUTS DE LA CLASSE #
        #-----------------------------------------#
        self.list_checkpoints_to_reach = list_checkpoints                       # list of checkpoints to reach
        self.index_checkpoint_to_reach = 0                                      # index of the current checkpoint to reach
        self.checkpoint_to_reach = self.list_checkpoints_to_reach[0]            # current checkpoint to reach
        self.angle_robot_to_checkpoint = 0.0                                    # orientation robot par rapport au point (entre -pi et pi radians)
        self.position = [0.0, 0.0]                                              # position du robot
        self.orientation = 0.0                                                  # orientation du robot par rapport à son référentiel (entre -pi et pi radians)
        self.zeghal_shifting_force = 0.0                                        # valeur du glissement pour éviter les voisins (pour l'instant , entre -pi et pi radians, à modifier par la suite)
        self.state = "walk"                                                     # Etat du robot [walk (avancer tout droit), turn (tourner), stop (s'arrêter)]
        self.id = id                                                            # ID du robot

    #--------------------CALLBACK DE MOUVEMENT-----------------------------#
    #Commande le robot en fonction des données odométriques reçues dans msg
    def turtlebot_odometry_callback(self, odometry_message: Odometry) -> None:
        
        # mise à jour des variables globales
        self.position = odometry_message.pose.pose.position.x, odometry_message.pose.pose.position.y 
        self.orientation = euler_from_quaternion(odometry_message.pose.pose.orientation)[2]
        self.angle_robot_to_checkpoint = calculate_alignment(self.position, self.orientation, self.checkpoint_to_reach)
        self.get_logger().info(f"Zeghal shifting coeficient: {self.zeghal_shifting_force}")

        # Si le point objectif est atteint, modification de l'objectif
        if (checkpoint_reached(self.position, self.checkpoint_to_reach)):
            
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
            
            self.index_checkpoint_to_reach += 1
            self.checkpoint_to_reach = self.list_checkpoints_to_reach[self.index_checkpoint_to_reach % len(self.list_checkpoints_to_reach)]
            
        # Si le point objectif n'est pas atteint, gestion des états et déplacement 
        else :
            # Etat walk : avance tout droit. Si il n'est pas aligné, ->  turn. Si il croise un voisin, -> stop
            # Etat turn : tourne. Si il est aligné -> stop
            # Etat stop: stop (réduit le problème d'arc de cercle). Si il n'y a plus de voisin dans le périmètre, -> walk

            # Test pour arrêter le robot si il a un voisin dans son périmètre de vision 
            #(Pour l'instant, il reste arrêté indéfiniement, il faut modifier pour qu'il évite le voisin au lieu de s'arrêter)
            if self.zeghal_shifting_force != 0:
                self.state = "stop"
            # Repart si pas de voisin
            elif self.state == "stop":
                self.state = "walk"
            #Arrête de tourner si angle sous le seuil ANGLE_FIN_ALIGNEMENT
            elif ((abs(self.angle_robot_to_checkpoint) < ANGLE_FIN_ALIGNEMENT) and self.state == "turn") : # si on est quasiment alignés avec le point
                self.state = "stop"
            #Commence à tourner si l'angle est au dessus du seuil ANGLE_DEBUT_ALIGNEMENT
            elif ((abs(self.angle_robot_to_checkpoint) > ANGLE_DEBUT_ALIGNEMENT) and self.state == "walk"):
                self.state = "turn"
            #Par défaut, avance 
            else:
                self.state = "walk"
            
            #---GESTION DEPLACEMENTS---# 
            motors_command = Twist()
            motors_command.linear.x = 0.0                                               # x linear speed is set to 0
            motors_command.linear.y = 0.0                                               # y linear speed is set to 0
            motors_command.linear.z = 0.0                                               # z linear speed is set to 0
            motors_command.angular.z = 0.0                                              # z angular speed is set to 0

            if self.state == "walk":                                                    # STATE WALK
                motors_command.linear.x = LINEAR_SPEED_DEFAULT                          # movement in a straight line
            elif self.state == "turn":                                                  # STATE TURN
                motors_command.angular.z = self.angle_robot_to_checkpoint * 0.5         # the more the rotation to make, the greater is the angular speed
            self.publisher_turtlebot_motors_controller.publish(motors_command)
     

        #ENVOI DE LA POSITION AUX VOISINS 
        msg_trajectory = Trajectory()
        msg_trajectory.id = self.id 
        msg_trajectory.x = self.position[0]
        msg_trajectory.y = self.position[1]
        msg_trajectory.px=self.checkpoint_to_reach[0]
        msg_trajectory.py=self.checkpoint_to_reach[1]
        self.publisher_turtlebot_trajectories.publish(msg_trajectory)

    #------------------------CALLBACK COMMUNICATION VOISINS---------------------#
    #Calcule la valeur du glissement si le voisin est dans le rayon de visibilité et risque de croiser le robot
    def trajectory_callback(self, trajectory_message: Trajectory) -> None:
        if trajectory_message.id != self.id: #si le voisin est différent du robot
            """
            #Calcul du glissement avec zeghal, à revoir 
            # détection voisin renvoie true si voisin dans rayon visibilité et calcul_croisement renvoie true si trajectoire voisin croise trajectoire robot (à tester)
            if (detection_voisin(self.currentPosition, (msg.x, msg.y)) and calcul_croisement((msg.x, msg.y), (msg.px, msg.py), self.currentPosition, self.currentPoint)):
                self.glissement = calcul_zeghal(msg.orientation, (msg.x, msg.y), self.orientation, self.currentPosition)
            else:
                self.glissement = 0
            """
            #Test pour déclancher l'état stop si le voisin est dans le périmètre
            if (detect_neighbor(self.position, (trajectory_message.x, trajectory_message.y))):
                self.zeghal_shifting_force = 10
            else:
                self.zeghal_shifting_force = 0


def calculate_alignment(coords_robot: Tuple[float, float], bearing_robot: float, coords_destination: Tuple[float, float]) -> float:
    """Calculate the angle between the robot orientationn and the vector robot->destination
    This function is also used to calculate the angle between the robot orientaiton and the vector robot->neighbor
    In this specific case, the coords_destination corresponds to the coordinates of the neighbor.

    Args:
        coords_robot (Tuple[float, float]):         the cartesian coordinates of a robot
        orientation_robot (float):                  the bearing of the robot in radians [-PI, PI] in its own referential
        coords_destination (Tuple[float, float]):   the cartesian coordinates of the destination of the robot

    Returns:
        float: _description_
    """

    #calcul du vecteur robot-point 
    vecteur_robot_point = [coords_destination[0] - coords_robot[0], coords_destination[1] - coords_robot[1]]
    
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
    if bearing_robot < 0:
        bearing_robot += 2*math.pi
    
    #print("anglePoint : " + str(anglePoint))
    #print("orientation : " + str(orientation_robot))
        
    # Calcul de l'angle recherché (valeur dans [-2pi, 2pi])
    theta = anglePoint - bearing_robot
    # Normalisation de l'angle sur [-pi, pi]
    if theta > math.pi:
        theta -= 2*math.pi
    elif theta < - math.pi:
       theta += 2*math.pi
    return theta 
  
def checkpoint_reached(coords_robot: Tuple[float, float], coords_checkpoint: Tuple[float, float]):
    """Check if a checkpoint has been reached based on the coordinates of a checkpoint and the coordinates of a robot.

    Args:
        coords_robot (Tuple[float, float]):         the cartesian coordinates of a robot
        coords_checkpoint (Tuple[float, float]):    the cartesian coordinates of a robot's checkpoint to reach

    Returns:
        bool: True if the checkpoint has been reached, else False
    """
    # calcul de la distance à l'objectif 
    distance = math.sqrt((coords_robot[0] - coords_checkpoint[0])**2 + (coords_robot[1] - coords_checkpoint[1])**2)
    return distance < DISTANCE_CHECKPOINT_THRESHOLD

def ccw(A: Tuple[float, float], B: Tuple[float, float], C: Tuple[float, float]) -> bool:
    """Utility function to find of two trajectories are crossing.
    Found on internet.

    Args:
        A (Tuple[float, float]): TODO
        B (Tuple[float, float]): TODO
        C (Tuple[float, float]): TODO

    Returns:
        bool: TODO
    """
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def detect_possible_collision(coords_neighbor: Tuple[float, float], coords_dest_neighbor: Tuple[float, float], coords_robot: Tuple[float, float], coords_dest_robot: Tuple[float, float]) -> bool:
    """Detect if the trajectories of the two robots are crossing each other

    Args:
        coords_neighbor (Tuple[float, float]):          the cartesian coordinates of a neighbor
        coords_dest_neighbor (Tuple[float, float]):     the cartesian coordinates of the checkpoint to be reached by the neighbor
        coords_robot (Tuple[float, float]):             the cartesian coordinates of the robot
        coords_dest_robot (Tuple[float, float]):        the cartesian coordinates of the checkpint to be reached by the robot

    Returns:
        bool: True if the trajectories are crossing, else False
    """
    return ccw(coords_neighbor, coords_robot, coords_dest_robot) != ccw(coords_dest_neighbor, coords_robot, coords_dest_robot) and ccw(coords_neighbor, coords_dest_neighbor, coords_robot) != ccw(coords_neighbor, coords_dest_neighbor, coords_dest_robot)


def detect_neighbor(coords_robot: Tuple[float, float], coords_neighbor: Tuple[float, float]) -> bool:
    """Detect if another robot is visible by the current robot and if it should be acknowledge

    Args:
        coords_robot (Tuple[float, float]):         the cartesian coordinates of a robot
        coords_neighbor (Tuple[float, float]):      the cartesian coordinates of another robot

    Returns:
        bool: True if the other robot is considered visible, else False
    """
    distance = math.sqrt((coords_robot[0] - coords_neighbor[0])**2 + (coords_robot[1] - coords_neighbor[1])**2)

    return distance < RADIUS_VISIBILITY


def calculate_zeghal(bearing_neighbor: float, coords_neighbor: Tuple[float, float], bearing_robot: float, coords_robot: Tuple[float, float]) -> float:
    """Calculate the zeghal shift force to apply in order to avoid 

    Args:
        bearing_neighbor (float):                   the orientation of a neighbor in radians [-PI, PI] in its own referential
        coords_neighbor (Tuple[float, float]):      the cartesian coordinates of a neighbor
        bearing_robot (float):                      the orientation of the robot in radians [-PI, PI] in its own referential
        coords_robot (Tuple[float, float]):         the cartesian coordinates of the robot

    Returns:
        float: the zeghal shift force
    """
    angle_robot_neighbor = calculate_alignment(coords_robot,bearing_robot, coords_neighbor)     # calculate the angle between the robot orientation and the vector robot-neighbor
    angle_orientation_robot_neighbor = bearing_robot - bearing_neighbor                         # calculate the difference between the robot orientaiton and its neighbor orientation

    # TODO to complete
    # ? why do we return this
    return angle_robot_neighbor - angle_orientation_robot_neighbor


def euler_from_quaternion(quaternion) -> Tuple[float, float, float]:
    """Transform quaternion coordinates to euler coordinates
    Function found on internet.

    Args:
        quaternion (_type_): a quaternion # TODO description to complete and type to find

    Returns:
        Tuple[float, float, float]: the euler coordinates of the quaternion
    """
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

    return roll_x, pitch_y, yaw_z


def main(args=None):
    rclpy.init(args=args)

    turtlebot = Turtlebot(id=5, list_checkpoints=[(0.0, 0.0), (2.0, 0.0)])

    rclpy.spin(turtlebot) # executer les callback
    print("Noeuds créés")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()