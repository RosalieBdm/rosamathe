 # Projet turtlebot sur ROS2 dans le cadre du cours projetREB 

## Objectifs:
- déployer une flotte de turtlebots qui se déplacent en suivant une liste de points donnée.
- mettre en place un protocole d'évitement entre les turtlebots

## Hypothèses:
- L'environnement est connu (carte)
- Il n'y a pas d'obstacle entre les points
- Les robots partent du même point
- Les robots se croisent deux par deux 

## Mise en oeuvre:
- Un PC crée un node pour chaque turtlebot et les calculs sont effectués par les nodes (pas centralisé)
- ROS_DOMAIN_ID identique pour tous les turtlebots -> ajout de namespaces /robot_{ID} pour les distinguer 
- communication entre les turtlebots sur le topic /turtle_com
- calculs glissements de Zeghal pour l'évitement
- tests des algorithmes sur turtlesim
- détection des voisins par distance entre robot et voisin 

## Commandes pour lancer les turtlebots:
### Pour chaque turtlebot:
- Dans un premier terminal:
  
      ssh jetson@192.168.1.1{ID}
      sudo chmod 666 /dev/ttyUSB0
      cd ~/kobuki_base
      source install/setup.dev
      export ROS_DOMAIN_ID=1
      ros2 run kobuki_node kobuki_ros_node --ros-args --remap __ns:=/robot_{ID}
  
- Dans un second terminal:
  
      cd ~/rosamathe_ws
      export ROS_DOMAIN_ID=1
      source install/setup.bash
      colcon build
      ros2 run project_turtlebot talker 

### Launcher

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    turtle1_node = Node(
        package='project_turtlebot',
        executable='talker',
        parameters=[
            {'id': 1},
            {'checkpoints': [2.0, 0.0, 0.0, 2.0]}
        ]
    )

    turtle5_node = Node(
        package='project_turtlebot',
        executable='talker',
        parameters=[
            {'id': 5},
            {'checkpoints': [2.0, 0.0, 0.0, 2.0]}
        ]
    )

    return LaunchDescription([
        turtle1_node,
        turtle5_node
    ])
```
  
## Fichiers:
- custom_interfaces : package avec nos formats de messages customisés
- project_turtlebot : package avec les nodes qui commandent les turtlebots (seulement 1 et 5 pour l'instant)
   - project_turtlebot_node : code python qui gère le node
   - utils : code python qui contient les fonctions utiles
- project_turtle_sim: package avec le node qui commande turtlesim

# Avancées:
##   15/01:
  - robots suivent les points
  - robots s'arrètent si voisin détecté
  - launcher pour lancer plusieurs turtlesim
###   A faire:
  - tester calculs de croisements
  - tester calculs de glissements 
  - mettre à jour les calculs de croisements (pour l'instants, les robots sont considérés comme des points)

##   19/01:
  - Les robots suivent les points et s'évitent (coefficient/angle d'évitement à revoir)
  - Le calcul de zeghal est cohérent (youpi)
  - Les robots détectent leurs voisins seulement avec la distance (pas de détection de croisement)
  - Les fonctions ont été mises dans un fichier Utils à part
    
###   A faire:
  - Trouver les bonnes valeurs pour l'angle de glissement et les distances de détection de voisins
  - Lancer le node avec le launcher
  - Tester plein de cas de figure différents
  - Peut-être revoir la gestion des virages ( parfois le robot tourne autour du point objectif)

    


