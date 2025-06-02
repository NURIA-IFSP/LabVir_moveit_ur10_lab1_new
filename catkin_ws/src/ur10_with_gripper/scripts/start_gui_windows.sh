#!/bin/bash

# Inicia o joint_state_publisher_gui
rosrun joint_state_publisher_gui joint_state_publisher_gui &
sleep 2  # Tempo para abrir a janela

# Inicia o RViz com configuração personalizada
rosrun rviz rviz -d $(rospack find ur10_with_gripper)/config/ur10_with_gripper.rviz &
sleep 3  # Tempo para abrir o RViz

# Move e redimensiona as janelas (resolução: 1920x1080)
# RViz: posição (0,0), tamanho 1280x1040
wmctrl -r "RViz" -e 0,0,0,1280,1040

# Joint State Publisher: posição (1280,0), tamanho 640x1040
wmctrl -r "Joint State Publisher" -e 0,1280,0,640,1040
