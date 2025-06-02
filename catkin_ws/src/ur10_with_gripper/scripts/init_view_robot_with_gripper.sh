#!/bin/bash

echo "Iniciando o robo com garra..."
roslaunch ur10_with_gripper view_robot_move.launch 

sleep 3

echo "Organizando as janelas..."
./start_gui_windows.sh

sleep 3
echo "Tudo pronto!"