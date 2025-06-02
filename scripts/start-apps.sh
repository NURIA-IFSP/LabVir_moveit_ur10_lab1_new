#!/bin/bash

export DISPLAY=:1
export HOME=/home/${USER:-ubuntu}

# Aguarda até o DISPLAY responder (máx 15 segundos)
for i in {1..15}; do
    if xdpyinfo >/dev/null 2>&1; then
        echo "X display :1 disponível."
        break
    fi
    echo "Aguardando o X display iniciar..."
    sleep 1
done


# Inicia terminal principal
xfce4-terminal --geometry=100x30+100+100 --title="Main Terminal" &

# Terminal com ROS (exemplo)
# xfce4-terminal --geometry=80x24+600+100 --title="ROS Tools" -e "bash -c 'source /opt/ros/noetic/setup.bash; roscore'" &

# Inicia VSCode apontando para o projeto
# code /projeto1_PosDoc &
