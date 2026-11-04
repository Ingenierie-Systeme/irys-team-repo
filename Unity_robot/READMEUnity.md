# Simulation Unity

Ce projet Unity représente le robot biomimétique dans un bassin et permet de simuler les mouvements de ses articulations ainsi que les efforts hydrodynamiques appliqués aux nageoires. 
La scène principale est [`SampleScene`](Assets/Scenes/SampleScene.unity). Le projet utilise Unity `6000.6.7f1`.

## Ouvrir la simulation

1. Ouvrir le dossier `Unity_robot` dans Unity Hub avec la version indiquée
   ci-dessus.
2. Ouvrir `Assets/Scenes/SampleScene.unity`.
3. Appuyer sur **Play** pour lancer la scène.

Le bassin 3D est un décor d'expérimentation importé depuis Blender. La surface de l'eau sert aussi de niveau de référence aux calculs physiques ; sa hauteur est réglée à `y = 0` par défaut.

## Objets du robot

Le groupe `Robot` est le modèle articulé principal de la scène :

| Objet | Rôle |
| --- | --- |
| `structure` | Corps du robot et base de l'articulation ; les éléments structurels y sont reliés. |
| `nageoire` | Nageoire caudale montée sur le servo et pilotée par `servo_AB`. |
| `lien Servo-bras`, `bras`, `lien bras_nageoire` | Pièces de transmission entre le servo et la nageoire. |
| `aileron_up_front`, `aileron_down_front`, `aileron_up_back`, `aileron_down_back` | Ailerons disposés autour du corps ; ils sont reliés à la structure par des articulations fixes dans la scène. |
| `MasseMobile` | Masse interne mobile qui déplace le centre de gravité pour agir sur le tangage. |


## Articulations et commandes

La hiérarchie mécanique est construite avec des composants Unity
`ArticulationBody` :

- **Queue — articulation rotative (`RevoluteJoint`)** : le servo fait osciller
  la nageoire caudale. `servo_AB` reçoit des commandes `std_msgs/Float64` sur
  `/output/servos` (`std_msgs/Float32MultiArray`), où l'index `0` contient
  l'angle en degrés, limité par `angleMax` (`±50°` par défaut dans le script,
  `±70°` dans `SampleScene`). Les flèches gauche et droite permettent aussi de
  modifier la consigne au clavier.
- **Masse mobile — articulation prismatique (`PrismaticJoint`)** : la masse se
  déplace sur l'axe local Z, avec une course par défaut de `±0,1 m`.
  `masse_mobile_AB` convertit la valeur à l'index `1` de `/output/servos`
  (degrés) en position dans cette course. En l'absence de commande pendant une
  seconde, la consigne revient progressivement au centre.
- **Liaisons fixes (`FixedJoint`)** : elles maintiennent ensemble les parties
  rigides, notamment les éléments de transmission du servo et les ailerons.

Les paramètres de vitesse, de course, d'amortissement et de limites sont
configurés dans les composants de la scène et dans les scripts
[`servo_AB.cs`](Assets/scripts/Articulation%20body/servo_AB.cs) et
[`masse_mobile_AB.cs`](Assets/scripts/Articulation%20body/masse_mobile_AB.cs).

## Modèle physique

Le script [`hydro_forces_AB.cs`](Assets/scripts/Articulation%20body/hydro_forces_AB.cs)
calcule les efforts sur les panneaux de nageoire immergés : traînée normale et
tangentielle, et portance optionnelle. Les réglages partagés sont dans
[`HydroForcesSettings.asset`](Assets/Settings/HydroForcesSettings.asset) :
six panneaux, densité d'eau de `1000 kg/m³`, niveau d'eau `y = 0` et portance
désactivée par défaut.

Un calcul de flottabilité par échantillonnage du volume existe également dans
`USV_AB.cs`. Il doit être ajouté et configuré sur le corps pour être utilisé ;
il n'est pas activé sur `structure` dans la scène principale actuelle. La
simulation ne modélise donc pas, par défaut, un écoulement complet du bassin ni
des vagues.

Les calculs hydrodynamiques associés aux nageoires sont décrits dans
[`Assets/scripts/Articulation body/README.md`](Assets/scripts/Articulation%20body/README.md).

## Connexion à ROS 2

La communication passe par le package **ROS-TCP-Connector**. Le prefab
[`ROSConnectionPrefab`](Assets/Resources/ROSConnectionPrefab.prefab) est
préconfiguré pour contacter l'endpoint ROS à `127.0.0.1:10000`. Si ROS tourne
sur une autre machine, remplacer cette adresse par celle de la machine qui
héberge l'endpoint, puis démarrer celui-ci côté ROS.

Les scripts de capteurs disponibles publient notamment :

| Capteur | Topic par défaut | Type ROS |
| --- | --- | --- |
| IMU | `/input/imu` | `sensor_msgs/Imu` |
| Pression | `/input/pressure` | `sensor_msgs/FluidPressure` |
| GNSS | `/input/gnss` | `sensor_msgs/NavSatFix` |
| Caméra | `/input/camera` | `sensor_msgs/Image` |

Le magnétomètre `/input/mag` est prévu dans l'architecture ROS, mais la scène
Unity actuelle ne comporte pas de simulateur qui publie ce capteur.

Ces Topics ont été choisi arbitrairement durant le développement de la simulation, mais il peuvent être modifiés pour s'accorder avec le robot réel.

Les topics et fréquences sont modifiables dans les composants Unity concernés.
Un script présent dans le projet n'est actif que s'il est attaché à un objet de
la scène et correctement configuré.

## Scripts de simulation

Deux approches physiques sont conservées dans le projet :

- `Assets/scripts/Articulation body/` : modèle utilisé par le robot articulé de  `SampleScene`, avec `ArticulationBody`.

- `Assets/scripts/Rigid_body/` : implémentation alternative basée sur des
  `Rigidbody` et `ConfigurableJoint`. Première implémentation tentée, mais moins fonctionnelle que l'autre.