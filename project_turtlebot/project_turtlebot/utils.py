import math


ANGLE_DEBUT_ALIGNEMENT = 1,7 #Angle seuil qui déclanche le réalignement 
ANGLE_FIN_ALIGNEMENT = 0.2 #Angle seuil qui stoppe l'alignement 
DISTANCE_TOLERANCE = 0.1 #Distance seuil à laquelle l'objectif est atteint



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
    anglePoint = calcul_angle_vecteur(position_robot, point)

    # Normalisation des angles sur [0, 2pi]
    if orientation_robot < 0:
        orientation_robot += 2*math.pi
    
    print("anglePoint : " + str(anglePoint))
    print("orientation : " + str(orientation_robot))
        
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
def detection_voisin(pos_robot, pos_voisin, distance_seuil):
    distance = math.sqrt((pos_robot[0] - pos_voisin[0])**2 + (pos_robot[1] - pos_voisin[1])**2)

    if (distance < distance_seuil) :
        print("VOISIN")
        return True
    return False



# calcul de l'angle d'un vecteur
# point1 : [float , float]
# point2 : [float , float] 
def calcul_angle_vecteur(point1, point2):
    #calcul du vecteur robot-point 
    vecteur_robot_point = [point1[0] - point2[0], point1[1] - point2[1]]
    
    # Cas où le robot est exactement sur l'objectif
    if (vecteur_robot_point[0] == 0):
        return -1
    
    # calcul de l'angle du vecteur robot-point dans le référentiel du robot (valeur dans [-pi/2,pi/2])
    angleVecteur = math.atan(vecteur_robot_point[1]/vecteur_robot_point[0]) 

    # Si le vecteur est dans la moitié gauche du cercle trigo, on ajoute une rotation de pi pour que la valeur soit dans [-pi/2, 3pi/2]
    if (vecteur_robot_point[0] > 0):
        angleVecteur += math.pi
    
    # Normalisation de l'angle sur [0, 2pi]
    if angleVecteur < 0 :
        angleVecteur += 2*math.pi
    return angleVecteur
    

# Calcul les forces de glissement 
# pos_voisin : [float , float] , position du voisin
# orientation_voisin : float , orientation du voisin dans son référentiel, valeur dans [-pi, pi]
# pos_robot : [float , float] , position du robot
# orientation_robot : float , orientation du robot dans son référentiel, valeur dans [-pi, pi]
def calcul_zeghal(orientation_voisin, pos_voisin, orientation_robot, pos_robot) -> float:
    #calcul distance robot-voisin
    distance_robot_voisin = calcul_distance(pos_robot, pos_voisin)
    #calcul de la différence entre l'orientation du robot et l'orientation du voisin (v1 -v2) [-2pi,2pi]
    angle_v1v2 = orientation_robot - orientation_voisin 
    #Normalisation sur 0,2pi
    if (angle_v1v2 < 0) :
        angle_v1v2 += 2*math.pi

    print("v1-V2" + str(angle_v1v2))
    #cacul de l'angle de l'axe robot-voisin [0-2pi]
    angle_robot_voisin = calcul_angle_vecteur(pos_robot, pos_voisin)

    print("angle robot voisin " + str(angle_robot_voisin))
    print("zegahl : " + str(angle_v1v2 - angle_robot_voisin))
    
    #calcul de l'angle à renvoyer [-2pi, 2pi]
    angle_zeghal = angle_v1v2 - angle_robot_voisin
    #Normalisation sur 0,2pi
    if (angle_zeghal < 0) :
        angle_zeghal += 2*math.pi

    orientation_referentiel_r1r2 = orientation_robot- angle_robot_voisin
    #angle_zeghal < pi -> projection vers le haut 
    if (angle_zeghal) < math.pi :
        #v1 dans la moitié droite du cercle -> glissement dans le sens trigo 
        if orientation_referentiel_r1r2 < math.pi/2 or orientation_referentiel_r1r2 > 3*math.pi/2 :
            return 1.0/(distance_robot_voisin)*1.2
        #v1 dans la moitié gauche du cercle -> glissement dans le sens inverse 
        else :
            return - 1.0/(distance_robot_voisin)*1.2
    #angle_zeghal > pi -> projection vers le bas 
    else : 
        #v1 dans la moitié droite du cercle -> glissement dans le sens inverse 
        if orientation_referentiel_r1r2 < math.pi/2 or orientation_referentiel_r1r2 > 3*math.pi/2 :

            return - 1.0/(distance_robot_voisin)*1.2
        #v1 dans la moitié gauche du cercle -> glissement dans le sens trigo 
        else :
            return  1.0/(distance_robot_voisin)*1.2


    """
    # si les robots ont fini de se croiser ( v1-v2 pas dans [-pi/2,pi/2]) -> angle_v1v2 [-pi/2, pi/2]
    if (abs(angle_zeghal) > 1.4 ) :
        return (0,angle_zeghal)
    #Glissement inversement proportionnel à la distance
    """

    return angle_zeghal

# Renvoie la distance entre deux points
#point1 : [float, float]
#point2 : [float, float]
def calcul_distance(point1, point2):
    distance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    return distance


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