# PROPOSITION TECHNIQUE ET COMMERCIALE

## Conception et réalisation d'un robot biomimétique pour l'exploration de cavités sous-marines

**Référence :** PTC-ROB-001
**Version :** 0.1 – Ébauche
**Date :** 26 octobre 2026
**Équipe projet :** 6 ingénieurs
**Charge prévisionnelle :** 240 heures-ingénieur

---

# 1. Présentation du projet

## 1.1. Contexte

L'exploration des grottes et cavités sous-marines présente des difficultés importantes liées à l'accessibilité, à la visibilité, à la configuration des passages et aux risques encourus par les plongeurs.

Les robots sous-marins conventionnels, notamment les ROV à hélices, permettent déjà de réaliser de nombreuses missions d'observation. Cependant, certaines contraintes d'encombrement, de mobilité et de risque d'accrochage justifient l'étude de solutions alternatives.

Notre équipe propose le développement d'un prototype de robot biomimétique inspiré de la locomotion des poissons.

## 1.2. Objectif

Concevoir, fabriquer et tester un démonstrateur aquatique compact intégrant :

- Une propulsion par nageoire caudale oscillante.
- Un système de contrôle du tangage par déplacement longitudinal du centre de gravité.
- Une acquisition vidéo embarquée.
- Une mesure de pression et de profondeur.
- Une centrale inertielle permettant de mesurer l'attitude.
- Un système informatique embarqué assurant la commande et l'acquisition des données.

Le démonstrateur constituera une première étape vers le développement d'un robot destiné à l'exploration des milieux sous-marins confinés.

# 2. Étude de marché

## 2.1. Marchés visés

Plusieurs secteurs présentent un intérêt potentiel pour ce type de plateforme.

| Marché                    | Applications                                           |
| ------------------------- | ------------------------------------------------------ |
| Recherche scientifique    | Étude des cavités et des écosystèmes sous-marins       |
| Hydrogéologie             | Reconnaissance des réseaux karstiques                  |
| Exploration spéléologique | Observation préliminaire de passages immergés          |
| Inspection technique      | Inspection de conduites et d'ouvrages immergés         |
| Enseignement supérieur    | Recherche et expérimentation en robotique biomimétique |

## 2.2. Concurrence

Les références du secteur comprennent notamment :

**Blue Robotics – BlueROV2 :** robot sous-marin polyvalent à propulsion par hélices, commercialisé pour l'inspection, la recherche et l'expérimentation.

**SUNFISH :** plateforme autonome spécialisée dans l'exploration d'environnements sous-marins complexes, notamment les grottes.

**UNEXMIN :** projet européen de développement de robots autonomes pour l'exploration et la cartographie de mines inondées.

Ces solutions démontrent l'intérêt de la robotique pour l'exploration d'environnements difficilement accessibles.

## 2.3. Proposition de valeur

Notre solution se distingue par une architecture biomimétique compacte associant une propulsion caudale à un contrôle de l'assiette par déplacement de masse.

Cette conception vise à étudier :

- La possibilité de réduire l'exposition des organes propulsifs aux obstacles.
- L'intérêt d'une architecture allongée dans les passages étroits.
- Les performances énergétiques d'une propulsion oscillante.
- La faisabilité d'un contrôle du tangage sans propulseur vertical.

Ces avantages potentiels devront être validés expérimentalement.

# 3. Description technique

## 3.1. Architecture mécanique

Le corps principal du robot est constitué d'un tube cylindrique transparent en PMMA.

| Caractéristique                 | Spécification                  |
| ------------------------------- | ------------------------------ |
| Matériau                        | PMMA (Plexiglas)               |
| Diamètre extérieur              | 100 mm                         |
| Longueur du tube                | 650 mm                         |
| Appendices                      | Ailerons et nageoire caudale   |
| Accès et fermeture              | Trappes, flasques et joints    |
| Supports et éléments mécaniques | Majoritairement imprimés en 3D |

La structure doit assurer la rigidité de l'ensemble, le positionnement des composants et la protection des éléments électroniques contre l'eau.

## 3.2. Propulsion

Le système propulsif repose sur une nageoire caudale oscillante, entraînée par un servomoteur étanche.

Le système devra permettre de faire varier les paramètres de commande de la nageoire afin d'étudier leurs effets sur le déplacement du robot.

Les essais permettront notamment de caractériser la vitesse de déplacement, la stabilité et la consommation énergétique.

## 3.3. Contrôle du tangage

Le contrôle du tangage repose sur le déplacement longitudinal d'une masse interne comprenant une partie de l'électronique embarquée.

Ce déplacement sera assuré par un système mécanique à courroie, poulies et guidage, actionné par le servomoteur retenu.

Le déplacement de la masse modifie la position du centre de gravité du robot et produit un moment de tangage.

Les essais devront déterminer la course utile, les limites mécaniques et la plage d'assiette effectivement atteignable.

## 3.4. Architecture électronique

| Composant                         | Rôle                                          |
| --------------------------------- | --------------------------------------------- |
| Raspberry Pi 4 Model B – 2 Go     | Calculateur embarqué                          |
| Blue Robotics Navigator           | Interfaces de commande et mesures inertielles |
| Caméra USB                        | Acquisition vidéo                             |
| Capteur de pression Blue Robotics | Mesure de profondeur                          |
| Batterie LiPo 2S – 7,4 V, 42,2 Wh | Alimentation                                  |
| Servomoteur extérieur étanche     | Propulsion caudale                            |
| Actionneur interne retenu         | Déplacement de masse                          |

Le calculateur assurera l'exécution des commandes des actionneurs, l'acquisition des données des capteurs et l'enregistrement des résultats expérimentaux.

L'architecture logicielle sera adaptée aux interfaces disponibles sur la Raspberry Pi et la Navigator.

## 3.5. Alimentation

L'alimentation sera assurée par une batterie LiPo 2S présentant une énergie nominale de 42,2 Wh.

Un système de conversion et de régulation électrique devra assurer la compatibilité avec les tensions d'alimentation des équipements électroniques et des actionneurs.

L'autonomie sera mesurée pendant les essais en fonction du profil d'utilisation.

## 3.6. Limites du prototype

Le prototype constitue un démonstrateur expérimental.

Les fonctionnalités de cartographie, de localisation autonome, d'évitement d'obstacles et de navigation opérationnelle en grotte ne font pas partie des performances garanties à ce stade.

Les modalités de pilotage, de récupération et de sécurisation devront être adaptées aux conditions d'essais.

# 4. Organisation du projet

## 4.1. Ressources humaines

Six ingénieurs participent au développement, pour une durée de 40 heures chacun.

| Fonction                            | Missions principales                    |  Heures |
| ----------------------------------- | --------------------------------------- | ------: |
| Chef de projet / architecte système | Coordination, exigences, intégration    |      40 |
| Ingénieur mécanique                 | CAO, structure, étanchéité              |      40 |
| Ingénieur propulsion                | Nageoire, transmission, dimensionnement |      40 |
| Ingénieur électronique              | Alimentation, intégration électronique  |      40 |
| Ingénieur logiciel                  | Commande, capteurs, acquisition         |      40 |
| Ingénieur essais et validation      | Assemblage, protocoles, mesures         |      40 |
| **Total**                           |                                         | **240** |

Les fonctions représentent une répartition prévisionnelle du travail. Les phases de conception et de validation nécessiteront une collaboration entre les membres de l'équipe.

## 4.2. Réunions

Une réunion de coordination est prévue à chaque séance.

Elle permettra de présenter les avancements, de prendre les décisions techniques, de résoudre les difficultés et d'actualiser le planning.

Chaque réunion donnera lieu à un relevé des décisions et des actions à mener.

Les réunions sont comprises dans les 40 heures de travail individuel.

# 5. Planning prévisionnel

| Phase                                | Charge totale | Objectif                         |
| ------------------------------------ | ------------: | -------------------------------- |
| Analyse des besoins et architecture  |          30 h | Cahier des charges               |
| Conception mécanique et électronique |          60 h | CAO, schémas, dimensionnements   |
| Fabrication et approvisionnement     |          48 h | Pièces et composants disponibles |
| Assemblage et intégration            |          42 h | Prototype assemblé               |
| Expérimentation et validation        |          42 h | Résultats d'essais               |
| Documentation et restitution         |          18 h | Rapport final                    |
| **Total**                            |     **240 h** |                                  |

La planification pourra être adaptée aux contraintes d'approvisionnement et aux résultats des premiers essais.

# 6. Proposition financière

## 6.1. Hypothèses de chiffrage

Le chiffrage repose sur les hypothèses suivantes :

- Six ingénieurs mobilisés pendant 40 heures chacun.
- Coût horaire employeur estimatif : 36 €/h.
- Frais généraux imputés au projet : 15 % de la masse salariale.
- Composants électroniques principaux déjà disponibles.
- Provision pour imprévus matériels : 20 % des achats.
- Marge commerciale cible : 20 % du prix de vente HT.

## 6.2. Masse salariale

| Poste                     | Calcul      |  Montant HT |
| ------------------------- | ----------- | ----------: |
| Ingénieur 1               | 40 h × 36 € |     1 440 € |
| Ingénieur 2               | 40 h × 36 € |     1 440 € |
| Ingénieur 3               | 40 h × 36 € |     1 440 € |
| Ingénieur 4               | 40 h × 36 € |     1 440 € |
| Ingénieur 5               | 40 h × 36 € |     1 440 € |
| Ingénieur 6               | 40 h × 36 € |     1 440 € |
| **Total masse salariale** |             | **8 640 €** |

## 6.3. Frais généraux

Les frais généraux correspondent aux charges de fonctionnement de la structure affectées au projet.

| Poste                               | Coût imputé |
| ----------------------------------- | ----------: |
| Locaux et énergie                   |       350 € |
| Matériel informatique et logiciels  |       300 € |
| Machines, atelier et amortissements |       350 € |
| Assurance et administration         |       296 € |
| **Total**                           | **1 296 €** |

## 6.4. Fournitures et fabrication

Les équipements électroniques déjà disponibles sont considérés, à ce stade, comme mis à disposition sans facturation d'achat au projet.

| Fourniture                        | Coût estimatif HT |
| --------------------------------- | ----------------: |
| Tube PMMA et découpes             |              90 € |
| Flasques, joints et étanchéité    |             110 € |
| Courroie, poulies et guidages     |              55 € |
| Fabrication additive              |             100 € |
| Accessoires des actionneurs       |              50 € |
| Éclairage embarqué                |              40 € |
| Régulation électrique et câblage  |              60 € |
| Visserie et consommables          |              45 € |
| **Total achats**                  |         **550 €** |
| Provision pour imprévus matériels |             110 € |
| **Total budget matériel**         |         **660 €** |

Les montants devront être actualisés à partir des références fournisseurs et des composants effectivement disponibles.

## 6.5. Synthèse du prix de vente

| Poste                             |      Montant HT |
| --------------------------------- | --------------: |
| Masse salariale                   |         8 640 € |
| Frais généraux                    |         1 296 € |
| Fournitures                       |           550 € |
| Provision pour imprévus matériels |           110 € |
| **Coût de revient prévisionnel**  |    **10 596 €** |
| Marge commerciale prévisionnelle  |         2 649 € |
| **PRIX DE VENTE PROPOSÉ**         | **13 245 € HT** |

Le prix de vente prévisionnel est calculé avec une marge cible représentant 20 % du chiffre d'affaires du projet.

# 7. Analyse des risques

| Risque                       | Conséquence                | Mesure envisagée                               |
| ---------------------------- | -------------------------- | ---------------------------------------------- |
| Défaut d'étanchéité          | Endommagement électronique | Essais à sec et tests d'étanchéité             |
| Couple insuffisant           | Propulsion dégradée        | Vérification et essais de charge               |
| Course de masse insuffisante | Tangage limité             | Dimensionnement mécanique                      |
| Instabilité hydrostatique    | Comportement incontrôlé    | Vérification flottabilité et centre de gravité |
| Défaillance électrique       | Arrêt du robot             | Protections et surveillance                    |
| Dépassement de délai         | Prototype incomplet        | Priorisation des fonctions                     |
| Perte du robot               | Perte matérielle           | Essais en milieu contrôlé avec récupération    |

# 8. Livrables

À l'issue du projet, l'équipe fournira :

- Un prototype mécanique et électronique assemblé.
- Les fichiers de conception assistée par ordinateur.
- Les schémas d'intégration.
- Les programmes développés.
- Les protocoles et résultats d'essais.
- Un rapport de synthèse technique et financière.

La validation des performances sera limitée aux essais effectivement réalisés.

# 9. Conclusion

Le projet vise à concevoir un démonstrateur de robot aquatique biomimétique susceptible de contribuer, à terme, au développement de solutions d'exploration de cavités sous-marines.

Son architecture repose sur une propulsion caudale oscillante, un système de contrôle du tangage par déplacement de masse et une électronique embarquée associant Raspberry Pi, Navigator et capteurs.

La réalisation mobilisera six ingénieurs pour un volume total de 240 heures.

Le coût de revient prévisionnel s'élève à **10 596 € HT**, pour une proposition commerciale indicative de **13 245 € HT**.

Cette offre constitue une estimation préliminaire. Elle pourra être révisée à l'issue du dimensionnement et de la consolidation des achats.
