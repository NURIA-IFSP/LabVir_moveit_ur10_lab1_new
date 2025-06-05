#!/bin/bash
set -e

echo "### [1/6] Detectando diretório catkin_ws ###"

DIR=$(realpath "$(dirname "${BASH_SOURCE[0]}")")
while [[ "$DIR" != "/" && ! -d "$DIR/catkin_ws/src" ]]; do
    echo "Subindo de $DIR"
    DIR=$(dirname "$DIR")
done

if [[ "$DIR" == "/" ]]; then
    echo "❌ Diretório catkin_ws não encontrado."
    exit 1
fi

CATKIN_WS="$DIR/catkin_ws"
echo "✅ Diretório catkin_ws encontrado em: $CATKIN_WS"


echo "### [2/6] Instalando biblioteca serial ###"
# git clone https://github.com/wjwwood/serial.git /tmp/serial
# cd /tmp/serial
# make && sudo make install
# rm -rf /tmp/serial

sudo apt update
sudo apt install ros-noetic-serial


# echo "### [3/6] Adicionando chave e repositório do ROS ###"

# sudo apt update

echo "### [4/6] Atualizando rosdep ###"
sudo rosdep init || true
rosdep update

echo "### [5/6] Instalando dependências do catkin_ws ###"
cd "$CATKIN_WS"
rosdep install --from-paths src --ignore-src -r -y

echo "### [6/6] Compilando pacotes ###"
catkin build
source devel/setup.bash

echo "### ✅ Script finalizado com sucesso! ###"
