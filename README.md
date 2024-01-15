 # Projet turtlebot sur ROS2 dans le cadre du cours projetREB 

## Objectifs:
- déployer une flotte de turtlebots qui se déplacent en suivant une liste de points donnée.
- mettre en place un protocole d'évitement entre les turtlebots

## Hypothèses:
- L'environnement est connu (carte)
- Il n'y a pas d'obstacle entre les points

## Mise en oeuvre:
- Les turtlebots sont commandés depuis un PC sur des node différents (calculs effectués par chaque node, pas de calculs centralisés)
- ROS_DOMAIN_ID identique pour tous les turtlebots (ajout de namespaces pour les distinguer)
- communication entre les turtlebots sur le topic /turtle_com
- calculs glissements de Zeghal pour l'évitement
- tests des algorithmes sur turtlesim

## Commandes pour lancer les turtlebots:
### Pour chaque turtlebot:
- Dans un premier terminal:
  
      ssh jetson@192.168.1.1{ID}
      sudo chmod 666 /dev/ttyUSB0
      cd ~/kobuki_base
      source install/setup.dev
      export ROS_DOMAIN_ID=1
      ros2 run kobuki_node kobuki_ros_node __ns:=/robot_{ID}
  
- Dans un second terminal:
  
      cd ~/rosamathe_ws
      export ROS_DOMAIN_ID=1
      source install/setup.bash
      colcon build
      ros2 run project_turtlebot talker_{ID}   
  (pour l'instant, seulement talker et talker_5 disponibles)

# Avancées:
##   15/01:
  - robots suivent les points
  - robots s'arrètent si voisin détecté
  - launcher pour lancer plusieurs turtlesim
##   A faire:
  - tester calculs de croisements
  - tester calculs de glissements 
  - mettre à jour les calculs de croisements (pour l'instants, les robots sont considérés comme des points)
