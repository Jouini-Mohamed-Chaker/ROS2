# 🚀 Installation de ROS 2 Humble Hawksbill sur Ubuntu Jammy 22.04 LTS

## 🧰 Pré-requis
- **Système d'exploitation** : Ubuntu Jammy 22.04 LTS

---

## 🌍 Configuration des paramètres régionaux (locale)

Assurez-vous que votre système utilise les paramètres locaux en UTF-8 :

```bash
locale  # Vérifiez les paramètres actuels

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Vérifiez que la configuration est correcte
```

---

## 🛠️ Installation de ROS 2 Humble Hawksbill

1. **Ajoutez les dépendances nécessaires** :

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ros-archive-keyring.gpg
```

2. **Ajoutez le dépôt ROS 2** :

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
```

3. **Installez ROS 2** :

```bash
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🐢 Installation des paquets TurtleBot3

1. **Installez les dépendances nécessaires** :

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
echo 'export ROS_DOMAIN_ID=30 # TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

2. **Ajoutez les fichiers de configuration au `~/.bashrc`** :

```bash
echo "source ~/setup.bash" >> ~/.bashrc
echo "source ~/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

3. **Clonez les simulations TurtleBot3** :

```bash
mkdir -p ~/turtlebot3_ws/src/
cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
```

---

## 📝 TurtleBot3 package
### 📂 Chaque fois qu'on ouvre un nouveau terminal il fault inclure :
```bash
source install/setup.bash
```

### ⚙️ Lancer les modèles et mondes
- **Burger** :
  ```bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo empty_world.launch.py
  ```
- **Waffle** :
  ```bash
  export TURTLEBOT3_MODEL=waffle
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
- **Waffle Pi** :
  ```bash
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
  ```

### 🗺️ SLAM (Cartographer)
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 🎮 Téléopération
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

### 💾 Sauvegarde de la carte
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 🚦 Navigation
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

> **Note :** Le nœud SLAM doit être actif pour utiliser la navigation.

---

## 🛠️ Étapes de navigation

1. **Estimation de la position initiale** :
    - Cliquez sur le bouton **2D Pose Estimate** dans RViz2.
    - Cliquez sur la carte pour positionner le robot et orientez la flèche verte.
    - Lancez le téléopération pour ajuster manuellement la position :
      ```bash
      ros2 run turtlebot3_teleop teleop_keyboard
      ```
    - Déplacez le robot pour affiner sa position estimée.
    - Terminez le téléopération avec `Ctrl + C` pour éviter des conflits.

2. **Fixez un objectif de navigation** :
    - Cliquez sur **Navigation2 Goal** dans RViz2.
    - Cliquez sur la carte pour définir une destination et orientez la flèche verte.

---

## 🚀 MyBots Package

Le package `MyBots` contient deux sous-packages : `my_bot` et `line_bot`. Pour construire et lancer ces packages, vous pouvez utiliser le script `build.sh`.

### 🛠️ Construction et lancement

1. **Rendre le script exécutable** :

```bash
chmod +x build.sh
```

2. **Construire et lancer un package** :

```bash
./build.sh [nom_du_package]  # Remplacez [nom_du_package] par 'my_bot' ou 'line_bot'
```

### 📦 Packages nécessaires pour `my_bot`

Pour installer les dépendances nécessaires pour le package `my_bot`, utilisez les commandes suivantes :

```bash
sudo apt update
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-rclcpp
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2
sudo apt install libignition-transport11-dev
sudo apt install ros-humble-ros-ign-bridge
```

### 📦 Packages nécessaires pour `line_bot`

Pour installer les dépendances nécessaires pour le package `line_bot`, utilisez les commandes suivantes :

```bash
sudo apt update
sudo apt install ros-humble-ament-cmake
sudo apt install ros-humble-rclcpp
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install libignition-transport11-dev
```

---

Bonne chance avec votre configuration de ROS 2 Humble Hawksbill, TurtleBot3 et MyBots ! 🚀