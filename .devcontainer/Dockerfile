FROM osrf/ros:noetic-desktop-full

# Argumentos para criar usuário não-root
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Criação de usuário com permissões sudo
RUN apt-get update && apt-get install -y sudo ca-certificates wget gnupg lsb-release && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -a -G dialout $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Alterna para o novo usuário
USER $USERNAME

# Adiciona ao grupo video
RUN sudo usermod --append --groups video $USERNAME

# Atualiza pacotes e instala dependências ROS
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y git nano python-is-python3 \
    ros-noetic-ros-tutorials \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    liburdfdom-tools \
    ros-noetic-effort-controllers \
    ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-rviz-visual-tools \
    python3-catkin-tools \
    ros-noetic-moveit \
    ros-noetic-catkin \
    libpoco-dev \
    python3-wstool

# Atualiza rosdep (como root)
USER root
RUN rosdep init && \
    rosdep update || true

# Instala XFCE + pacotes gráficos
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    xfce4 xfce4-terminal \
    novnc websockify xterm x11vnc curl net-tools \
    python3-pip libgl1-mesa-dri libgl1-mesa-glx dbus-x11 mesa-utils \
    x11-utils x11-xserver-utils wmctrl xdotool firefox autocutsel && \
    apt-get clean

# Configura o display para o ambiente gráfico
ENV DISPLAY=:1

# Desabilita o aviso de EOL do ROSI
ENV DISABLE_ROS1_EOL_WARNINGS 1

# Instala dependências para o VSCode
RUN apt-get update && apt-get install -y \
    gnome-keyring \
    libsecret-1-0 libsecret-1-dev

# Instala TurboVNC
RUN wget -O /tmp/turbovnc.deb https://github.com/TurboVNC/turbovnc/releases/download/3.1.1/turbovnc_3.1.1_amd64.deb && \
    dpkg -i /tmp/turbovnc.deb || apt-get install -fy && \
    rm /tmp/turbovnc.deb

ENV PATH="/opt/TurboVNC/bin:${PATH}"

# Instala o VSCode
RUN curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor -o /etc/apt/trusted.gpg.d/microsoft.gpg && \
    echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list && \
    apt update && apt install -y code

# Configura VNC com senha fixa
RUN mkdir -p /home/${USERNAME}/.vnc && \
    echo "senha-segura" | /opt/TurboVNC/bin/vncpasswd -f > /home/${USERNAME}/.vnc/passwd && \
    chmod 600 /home/${USERNAME}/.vnc/passwd && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.vnc

# Copia scripts de inicialização para a pasta home/ubuntu
COPY ../scripts/*.sh /home/${USERNAME}/
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/*.sh && \
    chmod +x /home/${USERNAME}/*.sh

# Expõe portas
EXPOSE 5901 6080

# Configura ROS no .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /projeto1_PosDoc/init.bash" >> /home/$USERNAME/.bashrc

# === SETUP MOVEIT WORKSPACE ===
USER root

# Copia o workspace catkin_ws do host para o container
COPY catkin_ws /LabVir_moveit_ur10_lab1/catkin_ws

# Configura ambiente do usuário e permissões
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

# Adiciona script para abrir tutorial e RViz lado a lado
RUN mkdir -p /home/$USERNAME/Desktop && \
    echo '#!/bin/bash\n# Source dos workspaces ROS\nsource /ros/noetic/setup.bash\nsource /home/ubuntu/catkin_ws/devel/setup.bash\n\n# Detecta resolução da tela\nRES=$(xdpyinfo | grep dimensions | awk "{print $2}")\nWIDTH=$(echo $RES | cut -d "x" -f1)\nHEIGHT=$(echo $RES | cut -d "x" -f2)\nHALF_WIDTH=$(($WIDTH / 2))\n\n# Abre navegador na metade esquerda\nxdg-open "https://moveit.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html" &\nsleep 3\nwmctrl -r :ACTIVE: -e 0,0,0,$HALF_WIDTH,$HEIGHT\n\n# Abre RViz na metade direita\nxfce4-terminal --title="RViz Tutorial" --geometry=100x30 --hold -e "bash -c 'roslaunch panda_moveit_config demo.launch rviz_tutorial:=true'"&\nsleep 5\nwmctrl -r RViz -e 0,$HALF_WIDTH,0,$HALF_WIDTH,$HEIGHT\n' > /home/ubuntu/start_tutorial_and_rviz.sh && \
    chmod +x /home/ubuntu/start_tutorial_and_rviz.sh && \
    chown ubuntu:ubuntu /home/ubuntu/start_tutorial_and_rviz.sh

# Adiciona atalho na área de trabalho para o script init_ros.sh
RUN echo '[Desktop Entry]\nVersion=1.0\nType=Application\nName=Init ROS\nComment=Inicializa ambiente ROS\nExec=/home/ubuntu/init_ros.sh\nIcon=utilities-terminal\nTerminal=true\nCategories=Development;' > /home/ubuntu/Desktop/init_ros.desktop && \
    chmod +x /home/ubuntu/Desktop/init_ros.desktop && \
    chown ubuntu:ubuntu /home/ubuntu/Desktop/init_ros.desktop

# Adiciona atalho na área de trabalho para o script Tutorial + RViz
RUN echo '[Desktop Entry]\nVersion=1.0\nType=Application\nName=Tutorial + RViz\nComment=Abre navegador com o tutorial e o RViz lado a lado\nExec=/home/ubuntu/start_tutorial_and_rviz.sh\nIcon=applications-internet\nTerminal=false\nCategories=Development;' > /home/ubuntu/Desktop/tutorial_and_rviz.desktop && \
    chmod +x /home/ubuntu/Desktop/tutorial_and_rviz.desktop && \
    chown ubuntu:ubuntu /home/ubuntu/Desktop/tutorial_and_rviz.desktop

# === CONFIGURAÇÃO DE IMAGENS ===
# Copia wallpapers para o container
COPY ../images/wallpapers/ /usr/share/backgrounds/labvir/
RUN chmod 644 /usr/share/backgrounds/labvir/*

# Cria diretório de autostart e adiciona script do wallpaper
USER $USERNAME
RUN mkdir -p /home/$USERNAME/.config/autostart && \
    echo "[Desktop Entry]\n\
Type=Application\n\
Name=Set Wallpaper\n\
Exec=/home/ubuntu/set-wallpaper.sh\n\
Terminal=false\n\
X-GNOME-Autostart-enabled=true" > /home/$USERNAME/.config/autostart/set-wallpaper.desktop && \
    chmod +x /home/$USERNAME/.config/autostart/set-wallpaper.desktop


# Retorna para o usuário normal
USER $USERNAME

# Comando padrão
CMD ["/home/ubuntu/start-vnc.sh"]
