# Simulation hydro IRYS — simu-hydro

Ouvrir `Unity_robot` avec Unity 6000.6.5f1, puis `Assets/Scenes/SimulationHydro.unity` et Play.

- Haut/bas ou W/S : intensité de propulsion `u`, mémorisée.
- Gauche/droite ou A/D : virage signé `v`.
- Espace : arrêt progressif du battement.
- Tab : vue piscine / suivi du robot.
- Entrée externe : désactiver `keyboard` sur `HydroRobot`, puis affecter `u` et `v`.

## Calculs

- `RobotHydroDefinition.cs` : dimensions fixes, PVC, rigidité EI, limites du servo.
- `HydroCommand.cs` : filtres et arc amplitude/fréquence, angle moyen de virage.
- `HydroSurface.cs` : six surfaces directionnelles équivalentes par élément. Les aires projetées et leurs centres sont regroupés à l'initialisation à partir de la CAO ; aucune boucle sur les triangles ni découpe à la surface de l'eau pendant la physique. Les sections de palme utilisent directement leurs dimensions.
- `HydroRobot.cs` : application des efforts, servo et flottabilité du corps sur 144 échantillons fixes, au lieu de plusieurs milliers.
- `Editor/HydroSceneTools.cs` : construction des masses, inerties et articulations souples ; ces valeurs sont enregistrées dans la scène. Reconstruire la scène après modification des dimensions ou raideurs, en préservant auparavant la scène existante.

Chaque surface reçoit une résistance normale quadratique et un faible frottement tangent, à partir de sa vitesse relative à l'eau. Les forces sont appliquées à leur position pour transmettre les couples au robot. La fraction immergée est une interpolation sur la hauteur de l'élément. Pour les ailerons fixes, le volume est calculé une fois à partir du maillage fermé, puis multiplié par cette fraction. Les géométries visuelles restent celles de la CAO ; les directions hydrodynamiques sont approximées par les axes locaux.

La palme conserve six articulations élastiques amorties. Le servo conserve ses limites de couple/vitesse et ses butées. Unity résout la mécanique avec `ArticulationBody`, au pas de 2,5 ms. Le modèle vise un mouvement visuellement crédible ; les efforts, surtout lors des rotations et des immersions partielles, sont approximatifs. `u` est une intensité, pas une puissance en watts.

La piscine originale est conservée dans `Assets/Environment/Pool/Piscine_ENSTA_ROB.prefab`. Les fichiers `.poolmesh` et leur importeur permettent son utilisation sans Blender ; les collisions sont limitées aux parois et au fond. Les outils Editor de construction/import sont nécessaires pour reconstruire ces assets, pas pour calculer les efforts en Play.

Les rapports, exports CSV et scripts dédiés à leur validation/génération ont été retirés. La scène n'écrit aucun rapport.
