# Introduction à ROS 2

## Qu’est-ce que ROS 2 ?
ROS 2 (Robot Operating System 2) est un framework open-source qui facilite le développement d’applications robotiques. Créé comme une évolution de ROS 1, ROS 2 répond aux limitations de son prédécesseur, notamment en matière de communication distribuée, de performance en temps réel, et de compatibilité avec des systèmes variés (microcontrôleurs, robots autonomes, ou clusters distribués).

---

## Objectifs principaux de ROS 2
1. **Modularité** : Permet aux développeurs de construire des systèmes robotiques à partir de composants réutilisables et interchangeables.
2. **Interopérabilité** : Fonctionne sur divers systèmes d’exploitation (Linux, Windows, macOS) et architectures matérielles.
3. **Fiabilité** : Fournit des mécanismes robustes pour gérer la communication et les défaillances.
4. **Scalabilité** : S’adapte aussi bien aux petits robots qu’aux systèmes robotiques complexes ou distribués.

---

## Applications courantes
- **Robotique industrielle** : Manipulateurs, bras robotiques.
- **Robots mobiles** : Drones, véhicules autonomes.
- **Recherche académique et prototypage rapide**.
- **Domotique et robots personnels**.

---

## Comment fonctionne ROS 2 ?
ROS 2 repose sur une architecture distribuée où différents composants, appelés **nœuds**, interagissent via un middleware nommé **DDS** (Data Distribution Service). Ce middleware garantit une communication performante et fiable.

### Principes fondamentaux
1. **Décentralisation** : Contrairement à ROS 1, il n’y a pas de maître central. Tous les nœuds sont autonomes et peuvent s’interconnecter dynamiquement.
2. **Communication inter-processus** : Les nœuds utilisent des mécanismes standardisés tels que les **topics**, les **services**, et les **actions**.
3. **Compatibilité en temps réel** : Grâce à DDS, ROS 2 s’adapte aux systèmes nécessitant des garanties strictes de temps de réponse.

### Avantages par rapport à ROS 1
- **Temps réel** : Support intégré pour les systèmes temps réel.
- **Sécurité** : Cryptage et authentification via DDS.
- **Portabilité** : Fonctionne sur diverses architectures matérielles et logicielles, y compris les plateformes embarquées.

---

## Les Nœuds (Nodes)
Un **nœud** est un processus autonome qui accomplit une tâche spécifique dans un système robotique. Ils sont conçus pour être simples et modulaires, ce qui facilite le développement et la maintenance.

### Exemples de nœuds dans un robot mobile
- **Capteurs** : Lit les données d’un capteur (LiDAR, caméra, etc.) et les publie pour d’autres nœuds.
- **Contrôleur de mouvement** : Reçoit des commandes de navigation et contrôle les moteurs.
- **Traitement de données** : Effectue des calculs comme la fusion de capteurs ou la création de cartes SLAM.

### Commandes utiles
- `ros2 node list`
- `ros2 run <nom_du_package> <nom_de_l’executable>`
- `ros2 node info <nom_du_nœud>`

### Relations entre plusieurs nœuds

![Nodes](/assets/Nodes-TopicandService.gif)

---

## Les Topics (Sujets)
Les **topics** permettent un échange asynchrone de données entre les nœuds selon un modèle **publication-abonnement**.

### Fonctionnement
- Un **éditeur** publie des messages sur un topic donné.
- Un ou plusieurs **abonnés** reçoivent ces messages.

### Commandes utiles
- `ros2 topic list`
- `ros2 topic echo <nom_du_topic>`
- `ros2 topic info <nom_du_topic>`
- `ros2 topic pub <nom_du_topic> <type_de_message> ‘<arguments>’`
- `ros2 topic hz <nom_du_topic>`

### Relations entre topics et nœuds

![Topics](/assets/Topic-MultiplePublisherandMultipleSubscriber.gif)

---

## Les Services
Les **services** permettent une interaction synchronisée entre deux nœuds via un modèle requête-réponse.

### Différences entre Services et Topics
- **Topics** : Flux continu de données.
- **Services** : Interactions ponctuelles nécessitant une réponse immédiate.

### Commandes utiles
- `ros2 service list`
- `ros2 service type <nom_du_service>`
- `ros2 service call <service_name> <service_type> <arguments>`

### Relations entre services et nœuds
![Services](/assets/Service-SingleServiceClient.gif)

---

## Les Actions
Les **actions** gèrent des tâches longues ou complexes en permettant :
- Le lancement d’une tâche asynchrone.
- Le suivi de son avancement.
- L’annulation si nécessaire.

### Commandes utiles
- `ros2 action list`
- `ros2 action info <nom_de_l’action>`
- `ros2 action send_goal <nom_de_l’action> <type_de_l’action> <valeurs>` 
*(les valeurs sont en format YAML)*

### Relations entre actions et nœuds

![Actions](/assets/Action-SingleActionClient.gif)

---

## Les Paramètres
Les **paramètres** permettent de configurer les nœuds de manière flexible sans modifier leur code ou nécessiter un redémarrage.

### Commandes utiles
- `ros2 param list`
- `ros2 param get <nom_du_nœud> <nom_de_paramètre>`
- `ros2 param set <nom_du_nœud> <nom_de_paramètre> <valeur>`

---

## Conclusion
ROS 2 est un outil puissant et modulable pour le développement d’applications robotiques. Grâce à ses mécanismes robustes et flexibles, il permet de créer des systèmes scalables, fiables, et adaptés à des environnements variés.
