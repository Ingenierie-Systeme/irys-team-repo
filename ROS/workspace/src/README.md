# Workspace ROS 2

## Architecture du contrôle du robot

- reception des informations des capteurs (codes exemples dans `sensor_subscriber`)
    - IMU : `/imu/data`
    - Caméra : `/camera/image`
    - GNSS : `/gnss/fix`
    - Baromètre : `/baro/data`
- estimation de l'état au fil du temps (package `robot_pose`)
    - filtre de Kalman
    - recalibration au GPS en surface
    - publication sur topic `/robot/pose` (position, vitesse et inclinaison)
- estimation de la sécurité et navigation (package `robot_brain`)
    - estimation heuristique du temps avant collision dans trois secteurs caméra
    - choix d'une direction locale pour éviter un obstacle visible
    - suivi de mur visuel
    - arrêt d'urgence si obstacle trop proche via flux optique (`robot_brain/optical_flow_emergency_stop.py`)
    - publication sur le topic `/robot/direction` (direction vers laquelle le robot doit se diriger, en prenant en compte sa manoeuvrabilité limité)
- commandes des servo-moteurs (package `robot_control`)
    - convertit la direction en consignes pour les servo-moteurs
    - nageoire pour le déplacement : `/servo/nageoire_cmd`
    - masse mobile pour l'inclinaison : `/servo/masse_cmd`

## Comportement du robot

Idéalement / théoriquement : 
- SLAM de grottes sous-marine pour explorer la zone à la place de plongeurs

Compte tenu des limitations du projet (pas de source de lumière embarquée, pas de Lidar):
- tours dans le bassin ROB en autonomie pour tester les lois de commandes, les sécurités et l'assemblage du robot


## ROS-TCP-Endpoint

Le workspace inclut le paquet officiel Unity `ros_tcp_endpoint` comme sous-module
Git, sur la branche ROS 2 `main-ros2`. Après un nouveau clonage du dépôt :

```bash
git submodule update --init --recursive
```

Compiler le workspace depuis `ROS/workspace` après avoir installé et sourcé une
distribution ROS 2 :

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ROS/workspace
colcon build --symlink-install
source install/setup.bash
```

Lancer l'endpoint TCP, les viewers IMU/GNSS/caméra, le moniteur de sécurité
et l'estimateur d'état :

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

L'endpoint écoute par défaut sur `0.0.0.0:10000`. Configurer l'adresse IP de
cette machine et le port `10000` dans le `ROSConnection` de Unity. Les viewers
IMU, GNSS et caméra peuvent être désactivés séparément avec
`start_imu_viewer:=false`, `start_gnss_viewer:=false` ou
`start_camera_viewer:=false`. Le moniteur de sécurité peut être désactivé avec
`start_obstacle_monitor:=false`. L'estimateur d'état peut être désactivé avec
`start_state_estimator:=false`. L'évitement caméra peut être désactivé avec
`start_camera_avoidance:=false`. Le contrôleur servo peut être désactivé avec
`start_servo_controller:=false`. L'adresse et le port peuvent être changés avec
`tcp_ip:=0.0.0.0` et `tcp_port:=10000`.

## Estimation de l'état du robot

Le nœud `robot_pose/state_estimator` fusionne l'IMU `/imu/data` et les fixes
GNSS `/gnss/fix`. Il propage position et vitesse en coordonnées ENU à partir
de l'accélération IMU, puis corrige la position par filtre de Kalman lorsqu'un
fix GNSS valide est disponible. La publication `nav_msgs/msg/Odometry` sur
`/robot/pose` utilise `odom` comme repère monde et `base_link` comme repère
robot. Les coordonnées GNSS sont relatives à l'origine par défaut
(48.39°, -4.48°, 0 m), à configurer pour correspondre à l'origine de la scène
Unity :

```bash
ros2 run robot_pose state_estimator --ros-args \
  -p origin_latitude:=48.39 \
  -p origin_longitude:=-4.48 \
  -p origin_altitude:=0.0
```

Le filtre s'initialise au premier fix GNSS valide. Il continue ensuite par
propagation inertielle quand le GNSS n'émet plus, mais son incertitude augmente
et la dérive n'est pas bornée sans correction de position ou de vitesse. Après
une interruption GNSS de plus de 2 secondes, un fix de retour à moins de 50 m
de la position estimée recalibre la position sans réinitialiser la vitesse ;
ces limites sont configurables par `gnss_reacquisition_timeout` et
`max_gnss_reacquisition_distance`. La pose publiée ne constitue donc pas une
navigation inertielle fiable à long terme. La covariance nulle publiée par le
simulateur IMU signifie « inconnue » ;
le filtre utilise `acceleration_noise_stddev` (0,5 m/s² par défaut) pour son
modèle de bruit. Le paramètre `imu_includes_gravity` vaut `true` par défaut,
conformément à la configuration Unity actuelle.

## Arrêt d'urgence par flux optique

Le nœud `robot_brain/optical_flow_emergency_stop` lit `/camera/image`, estime
le temps avant collision à partir de l'expansion radiale du flux optique et
publie un `std_msgs/msg/Bool` sur `/safety/emergency_stop`. `true` signifie
arrêt actif. Le seuil TTC est configurable (2 secondes par défaut) :

```bash
ros2 run robot_brain optical_flow_emergency_stop --ros-args \
  -p image_topic:=/camera/image \
  -p stop_topic:=/safety/emergency_stop \
  -p ttc_threshold_seconds:=2.0
```

Le nœud publie aussi l'arrêt au démarrage, pendant l'attente de deux images
exploitables, et si le flux caméra expire (0,5 seconde par défaut). Le signal
est fiable et transitoire-local afin que les abonnés tardifs reçoivent le
dernier état. Il revient à `false` quand des images valides indiquent un TTC
supérieur au seuil. Cette estimation est une heuristique qui suppose un point
de fuite au centre et une expansion due à l'approche d'un obstacle fixe ; elle
ne fournit pas une distance métrique et ne remplace pas un dispositif de
sécurité certifié. Aucun contrôleur moteur n'étant actuellement connecté à ce
topic, le nœud publie l'état d'arrêt mais n'immobilise pas seul le robot.

## Estimation des obstacles et choix de direction

Le nœud `robot_brain/camera_obstacle_avoidance` analyse le flux de
`/camera/image` et estime un TTC heuristique dans les secteurs gauche, centre
et droit. Il garde le cap avant tant que le secteur central ne paraît pas
dangereux, puis choisit une diagonale vers le côté dont le TTC paraît le plus
élevé. Si aucun côté ne semble sûr, si la caméra expire ou si
`/safety/emergency_stop` vaut `true`, il publie un vecteur nul.

La consigne est un `geometry_msgs/msg/Vector3Stamped` sur `/robot/direction`,
exprimé dans `base_link` avec `x` vers l'avant, `y` vers la gauche et `z` vers
le haut. Les choix sont avant `(1, 0, 0)`, diagonale `(0.5, ±0.866, 0)` ou arrêt
`(0, 0, 0)`. Le seuil TTC est de 2 secondes par défaut et peut être réglé avec
`minimum_time_to_contact`. Le nœud s'utilise seul ainsi :

```bash
ros2 run robot_brain camera_obstacle_avoidance --ros-args \
  -p image_topic:=/camera/image \
  -p direction_topic:=/robot/direction \
  -p minimum_time_to_contact:=2.0
```

Cette perception caméra seule ne fournit ni distance métrique ni garantie de
dégagement : les TTC reposent sur l'hypothèse d'une expansion radiale autour
du centre de l'image. Le nœud ne suit donc pas une paroi à distance donnée.
Le suivi de mur devra attendre une source de profondeur calibrée (par exemple
le `PointCloud2` simulé sur `/lidar/points`) et l'installation d'un capteur
équivalent sur le robot réel.

## Commande du servo

Le nœud `robot_control/servo_command_publisher` lit le
`geometry_msgs/msg/Vector3Stamped` de `/robot/direction` et publie des
`std_msgs/msg/Float64` sur `/servo/nageoire_cmd` et `/servo/masse_cmd`, à
50 Hz. L'angle de la nageoire suit le cap du vecteur `(x, y)` avec une
oscillation sinusoïdale d'amplitude maximale 40° et de période 5 secondes.
L'amplitude suit la norme du vecteur (les vecteurs directionnels actuels ont
une norme de 1 et l'arrêt une norme de 0) ; la consigne est limitée à ±90°.
L'angle de la masse est proportionnel à `x`, borné à ±50° ; une direction
nulle recentre les deux servos.

```bash
ros2 run robot_control servo_command_publisher
```

## Création d'un package

```bash
ros2 pkg create --build-type ament_python <nom_package>
```