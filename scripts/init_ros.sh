#!/bin/bash
# init_ros.sh - Instalação do pacote schunk_pg70 e dependências ROS
set -e

# 1. Instalação do serial
# Clona o repositório serial
git clone https://github.com/wjwwood/serial.git /tmp/serial
# Compila e instala a lib serial
cd /tmp/serial
make && make install
# Remove a pasta temporária para deixar o container limpo
rm -rf /tmp/serial

# 2. Servidor de pacotes ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

# 3. Verificação de dependências dos pacotes
echo "### rosdep update ###"
rosdep update
cd  /LabVir_moveit_ur10_lab1/catkin_ws
echo "### rosdep install ###"
rosdep install --from-paths src --ignore-src -r -y

# 4. Compilação
echo "### Compilando pacotes ###"
catkin build
source devel/setup.bash
