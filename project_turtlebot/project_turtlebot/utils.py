#-----------------------------------------------------------------------------------#
# PRJReb - Projet TURTLEBOT
# 2023-2024
# Rosalie BIEDERMANN _ Mathilde EBER _ Constantin THEBAUDEAU
# INSA LYON 

# Ce code contient toutes les fonctions utiles au node node_poject_turtlebot 
#-----------------------------------------------------------------------------------#

import math


# Calcul de l'angle entre deux vecteurs : vecteur orientation du robot et vecteur robot-point
# position_robot : [float , float] , position du robot
# orientation_robot : float , orientation du robot dans son référentiel, valeur dans [-pi; pi]
# point : [float , float] , point à atteindre 
def calcul_alignement(position_robot, orientation_robot, point):

    #calcul du vecteur robot-point 
    vecteur_robot_point = [point[0] - position_robot[0], point[1] - position_robot[1]]
    
    # Cas où le robot est exactement sur l'objectif
    if (vecteur_robot_point[0] == 0):
        return -1
    
    # calcul de l'angle du vecteur robot-point dans le référentiel du robot (valeur dans [0;2pi])
    anglePoint = calcul_angle_vecteur(position_robot, point)

    # Normalisation de l'angle sur [0; 2pi]
    if orientation_robot < 0:
        orientation_robot += 2*math.pi
    
    print("anglePoint : " + str(anglePoint))
    print("orientation : " + str(orientation_robot))
        
    # Calcul de l'angle recherché (valeur dans [-2pi; 2pi])
    theta = anglePoint - orientation_robot
    # Normalisation de l'angle sur [-pi; pi]
    if theta > math.pi:
        theta -= 2*math.pi
    elif theta < - math.pi:
       theta += 2*math.pi
    return theta 
  

# Return true si la distance robot-point est inférieure au seuil
# position_robot : [float , float] , position du robot
# point : [float , float] , point à comparer  
def check_distance(position_robot, point, distance_seuil):
    # Calcul de la distance au point
    distance = math.sqrt((position_robot[0] - point[0])**2 + (position_robot[1] - point[1])**2)

    #Return true si la distance est sous le seuil
    if (distance < distance_seuil) :
        return True
    return False

# Calcul de l'angle d'un vecteur
# point1 : [float , float]
# point2 : [float , float] 
def calcul_angle_vecteur(point1, point2):
    #Calcul du vecteur robot-point 
    vecteur_robot_point = [point1[0] - point2[0], point1[1] - point2[1]]
    
    # Cas où le robot est exactement sur l'objectif
    if (vecteur_robot_point[0] == 0):
        return -1
    
    # Calcul de l'angle du vecteur robot-point dans le référentiel du robot (valeur dans [-pi/2,pi/2])
    angleVecteur = math.atan(vecteur_robot_point[1]/vecteur_robot_point[0]) 

    # Si le vecteur est dans la moitié gauche du cercle trigo, on ajoute une rotation de pi pour que la valeur soit dans [-pi/2, 3pi/2]
    if (vecteur_robot_point[0] > 0):
        angleVecteur += math.pi
    
    # Normalisation de l'angle sur [0, 2pi]
    if angleVecteur < 0 :
        angleVecteur += 2*math.pi
    return angleVecteur
    

# Calcul des forces de glissement selon Zeghal 
# pos_voisin : [float , float] , position du voisin
# orientation_voisin : float , orientation du voisin, valeur dans [-pi, pi]
# pos_robot : [float , float] , position du robot
# orientation_robot : float , orientation du robot, valeur dans [-pi, pi]
def calcul_zeghal(orientation_voisin, pos_voisin, orientation_robot, pos_robot) -> float:
    #Calcul distance robot-voisin
    distance_robot_voisin = calcul_distance(pos_robot, pos_voisin)

    #Calcul de la différence entre l'orientation du robot et l'orientation du voisin (v1 -v2) [-2pi,2pi]
    angle_v1v2 = orientation_robot - orientation_voisin 

    #Normalisation sur [0;2pi]
    if (angle_v1v2 < 0) :
        angle_v1v2 += 2*math.pi

    #Calcul de l'angle de l'axe robot-voisin (r1r2) [0;2pi]
    angle_robot_voisin = calcul_angle_vecteur(pos_robot, pos_voisin)
    
    #Calcul de l'angle à renvoyer [-2pi;2pi]
    angle_zeghal = angle_v1v2 - angle_robot_voisin

    #Normalisation sur [0;2pi]
    if (angle_zeghal < 0) :
        angle_zeghal += 2*math.pi

    #Orientation de v1 (robot) dans le référentiel de l'axe robot-voisin (v1 - r1r2)
    orientation_referentiel_r1r2 = orientation_robot - angle_robot_voisin

    #angle_zeghal < pi -> projection vers pi/2 
    if (angle_zeghal) < math.pi :
        #v1 dans la moitié droite du cercle -> glissement dans le sens inverse 
        if orientation_referentiel_r1r2 < math.pi/2 or orientation_referentiel_r1r2 > 3*math.pi/2 :
            return - ((1.0/(distance_robot_voisin)))
        #v1 dans la moitié gauche du cercle -> glissement dans le sens trigo 
        else :
            return ((1.0/(distance_robot_voisin)))
    #angle_zeghal > pi -> projection vers -pi/2
    else : 
        #v1 dans la moitié droite du cercle -> glissement dans le sens trigo 
        if orientation_referentiel_r1r2 < math.pi/2 or orientation_referentiel_r1r2 > 3*math.pi/2 :

            return ((1.0/(distance_robot_voisin)))
        #v1 dans la moitié gauche du cercle -> glissement dans le sens inverse 
        else :
            return - ((1.0/(distance_robot_voisin)))


# Renvoie la distance entre deux points
# point1 : [float, float]
# point2 : [float, float]
def calcul_distance(point1, point2):
    distance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    return distance


# Tranforme un Quaternion en angles d'euler (récupérée sur internet)
# En 2D, seulement yaw_z est utile 
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