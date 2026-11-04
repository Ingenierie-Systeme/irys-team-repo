# irys-team-repo

## Members

Gaspard DELPIANO--MANFRINI
Mateo BRUN
Noé CARRAYROU
Eliaz YAHIA
Clement SARTRE
Lancelot RAMIS

## Présentation et objectifs

L’exploration de grottes sous-marines pose des difficultés d’accessibilité, de visibilité, de
communication et de sécurité. Le projet vise la conception d’un démonstrateur compact de locomotion
biomimétique, susceptible de préfigurer une solution spécialisée pour les milieux confinés.


Les objectifs de cette première réalisation sont la propulsion par nageoire caudale oscillante, le
contrôle du tangage par translation d’une masse interne, la mesure de profondeur et d’attitude, ainsi
que l’enregistrement d’images et de données expérimentales.

## Environnement 3D

![Env3D](images/environnement_3D.png)

## Architecture logicielle

![Archilog](images/architecture_logicielle.png)

## Schéma électrique

![Schema_elec](images/schema_electrique.png)


## Les pièces de CAO

### Bras de transmission du servomoteur nageoire

![Bras_transm](images/bras_transmission1.png)
![Bras_transm2](images/bras_transmission2.png)

### Platine de fixation de la nageoire

![Platine_fixation](images/platine_fixation1.png)
![Platine_fixation2](images/platine_fixation2.png)

### Support servomoteur nageoire

![support_servo](images/support_petit_servomoteur1.png)
![support_servo2](images/support_petit_servomoteur2.png)

### Support masse mobile

![supportmm1](images/support_masse_mobile1.png)
![supportmm2](images/support_masse_mobile2.png)

# Hardware

## Schéma électrique


## Connection à la raspberrypi

La connection ne fonctionne que sur mon partage de connexion ou par ethernet. Je pense que l'adresse mac de notre raspberrypi s'est fait ban du réseau iot.

Pour se connecter à la raspberry pi, brancher d'abord la raspberrypi et connectez votre client au partage de connexion Pixel_4952. Enfin, exécuter dans le shell :

`ssh irysbot@pi4.local`

puis rentrer le mdp : irys112



