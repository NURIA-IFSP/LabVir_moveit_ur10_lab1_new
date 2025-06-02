#!/bin/bash

WALLPAPER_PATH="/usr/share/backgrounds/labvir/wallpaper2_robot_sozinho.jpg"
MONITOR="monitorVNC-0"

echo "Aplicando wallpaper no monitor: $MONITOR"

for WS in 0 1 2 3
do
    echo "Configurando workspace $WS..."

    xfconf-query -c xfce4-desktop -p /backdrop/screen0/$MONITOR/workspace$WS/image-style -n -t int -s 5

    xfconf-query -c xfce4-desktop -p /backdrop/screen0/$MONITOR/workspace$WS/last-image \
        -n -t string -s "$WALLPAPER_PATH"
done

# Atualiza o desktop
xfdesktop --reload
echo "Wallpaper aplicado com sucesso."
