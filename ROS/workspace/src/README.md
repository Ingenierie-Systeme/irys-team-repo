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
    - choix de la trajectoire à adopter en fonction des informations
    - évitement d'obstacles grâce à la caméra (détermination d'un chemin safe)
    - arrêt d'urgence si obstacle trop proche via flux optique (`robot_brain/optical_flow_emergency_stop.py`)
    - publication sur le topic `/robot/direction` (direction vers laquelle le robot doit se diriger, en prenant en compte sa manoeuvrabilité limité)
- commandes des servo-moteurs (package `robot_control`)
    - récupère la consigne de direction et la convertit en commande pour les servo-moteurs, en prennant en compte l'état du robot, son inertie 
    - nageoire pour le déplacement : `/servo/angle_nage`
    - masse mobile pour l'inclinaison : `/servo/angle_mass` (angle déterminé à partir de la position désirée de la masse mobile sur son axe)

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
`start_state_estimator:=false`. L'adresse et le port peuvent être changés avec
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
arrêt actif. Le seuil TTC est configurable (1 seconde par défaut) :

```bash
ros2 run robot_brain optical_flow_emergency_stop --ros-args \
  -p image_topic:=/camera/image \
  -p stop_topic:=/safety/emergency_stop \
  -p ttc_threshold_seconds:=1.0
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

## Commande du servo

Lancer le publisher de consigne sinusoïdale :

```bash
ros2 run robot_control servo_command_publisher
```

Le nœud publie `std_msgs/msg/Float64` sur `/servo_angle_cmd` à 50 Hz, avec une
amplitude de 40 degrés et une période de 5 secondes.

## Création d'un package

```bash
ros2 pkg create --build-type ament_python <nom_package>
```