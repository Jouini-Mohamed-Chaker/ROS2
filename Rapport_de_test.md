## Rapport sur les Défis et Solutions rencontrés lors du Projet ROS 2 et Gazebo

### Introduction  
Ce document détaille les principaux défis rencontrés lors de la réalisation d’un projet combinant ROS 2 Humble, Gazebo Fortress, le TurtleBot3, et un robot personnalisé. Il met en lumière les solutions adoptées pour résoudre ces problèmes, afin de faciliter le développement dans des projets similaires.  

---

### Défis et Solutions  

#### 1. Configuration de l’environnement ROS 2  
**Défi :**  
La nécessité d’ajouter plusieurs variables d’environnement pour simplifier le développement n’était pas évidente au début. Parmi ces configurations :  
``` bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1
source /usr/share/gazebo/setup.sh
export GZ_VERSION=fortress
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib
```

**Solution :**  
Ces lignes ont été ajoutées au fichier `~/.bashrc` pour automatiser la configuration à chaque ouverture de terminal, rendant le travail plus fluide.  

---

#### 2. Dysfonctionnements de Gazebo  
**Défi :**  
Gazebo rencontrait des problèmes d’exécution intermittents, avec des pannes imprévisibles. La reconstruction des packages réparait souvent le problème.  

**Solution :**  
Adopter une approche proactive en reconstruisant les packages affectés avec des commandes comme `colcon build --packages-select <package_name>` pour résoudre rapidement les anomalies.  

---

#### 3. Complexité des relations entre composants ROS 2  
**Défi :**  
Les interactions entre fichiers de lancement, URDF, SDF et RViz étaient difficiles à comprendre initialement.  

**Solution :**  
Consacrer du temps à l’étude des guides officiels pour clarifier les rôles de chaque composant. La création de diagrammes pour illustrer les relations et les séquences d’exécution a également aidé à mieux organiser le projet.  

---

#### 4. Problèmes de compatibilité entre fichiers SDF et URDF  
**Défi :**  
L’utilisation de fichiers SDF directement avec ROS 2 posait des problèmes de compatibilité.  

**Solution :**  
Se tourner vers le format SDF, qui est mieux pris en charge par Gazebo, pour garantir une intégration plus harmonieuse des composants.  

---

#### 5. Difficultés avec les fichiers Xacro  
**Défi :**  
La conversion des fichiers Xacro en URDF échouait souvent en raison d’erreurs imprévues, rendant leur utilisation difficile.  

**Solution :**  
Abandonner temporairement les fichiers Xacro pour travailler uniquement avec des fichiers URDF simples, qui se sont révélés plus fiables pour ce projet.  

---

#### 6. Packages manquants dans l’installation de base  
**Défi :**  
Certains packages nécessaires n’étaient pas inclus dans les installations par défaut, ce qui ralentissait le démarrage du projet.  

**Solution :**  
Identifier et installer manuellement les packages manquants à l’aide des commandes `apt` ou `rosdep`, tout en s’appuyant sur la documentation et les forums communautaires.  

---

#### 7. Manque de guides adaptés aux débutants  
**Défi :**  
La documentation et les tutoriels disponibles étaient souvent trop fragmentés ou avancés pour les besoins d’un débutant.  

**Solution :**  
Trouver des ressources spécifiques à la version utilisée (ROS 2 Humble et Gazebo Fortress), puis compiler des notes personnelles pour combler les lacunes et simplifier le processus.  

---

#### 8. Confusion entre différentes versions de Gazebo  
**Défi :**  
Les différences entre Gazebo Classique, Ignition et Gazebo Fortress ont généré des confusions, particulièrement en ce qui concerne la compatibilité avec ROS 2.  

**Solution :**  
Se concentrer uniquement sur Gazebo Fortress, la version officiellement prise en charge avec ROS 2 Humble, et éviter de mélanger plusieurs versions.  

---

#### 9. Absence d’un IDE dédié pour ROS 2  
**Défi :**  
Le manque d’un environnement de développement intégré (IDE) spécifiquement conçu pour ROS 2 a compliqué des tâches comme la gestion des erreurs ou la complétion automatique.  

**Solution :**  
Utiliser des éditeurs de texte polyvalents comme Visual Studio Code avec des extensions adaptées à ROS 2, tout en désactivant les indications d’erreurs inutiles pour un travail plus fluide.  

---

### Conclusion  
Ce projet a permis de relever des défis techniques variés, allant de la configuration de l’environnement à la gestion des composants système et des outils. Grâce à des solutions pragmatiques et une approche méthodique, il a été possible de surmonter les obstacles et de mener à bien le projet. Ces apprentissages serviront de guide pour simplifier et accélérer les futurs développements ROS 2 et Gazebo.

