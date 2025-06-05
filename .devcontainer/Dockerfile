FROM osrf/ros:noetic-desktop-full

# === CONFIGURAÇÃO DE USUÁRIO NÃO-ROOT ===
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

# === INSTALA FERRAMENTAS BÁSICAS E REMOVE REPOSITÓRIO ANTIGO DO ROS ===
RUN sed -i '/packages.ros.org/d' /etc/apt/sources.list.d/ros1-latest.list \
    && apt-get update \
    && apt-get install -y curl wget gnupg2 lsb-release ca-certificates sudo

# === INSTALA NOVA CHAVE GPG DO ROS ===
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# === RECONFIGURA O REPOSITÓRIO DO ROS ===
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros1-latest.list

# === CRIA USUÁRIO ===
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo,dialout,video $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# === ATUALIZA SISTEMA E INSTALA DEPENDÊNCIAS DO ROS ===
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
    ros-noetic-serial \
    ros-noetic-rqt-joint-trajectory-controller \
    libpoco-dev \
    python3-wstool

# === CONFIGURA ROSDEP (COMO ROOT) ===
USER root
RUN rosdep init && \
    rosdep update || true

# === INSTALA XFCE + VNC ===
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    xfce4 xfce4-terminal \
    novnc websockify xterm x11vnc net-tools \
    python3-pip libgl1-mesa-dri libgl1-mesa-glx dbus-x11 mesa-utils \
    x11-utils x11-xserver-utils wmctrl xdotool firefox autocutsel && \
    apt-get clean

# === CONFIGURA DISPLAY ===
ENV DISPLAY=:1

# === DESABILITA AVISO DE EOL DO ROS ===
ENV DISABLE_ROS1_EOL_WARNINGS=1

# === DEPENDÊNCIAS DO VSCODE ===
RUN apt-get update && apt-get install -y \
    gnome-keyring \
    libsecret-1-0 libsecret-1-dev

# === INSTALA TURBOVNC ===
RUN curl -L -o /tmp/turbovnc.deb https://github.com/TurboVNC/turbovnc/releases/download/3.1.1/turbovnc_3.1.1_amd64.deb && \
    dpkg -i /tmp/turbovnc.deb || apt-get install -fy && \
    rm /tmp/turbovnc.deb

ENV PATH="/opt/TurboVNC/bin:${PATH}"

# === INSTALA VSCODE ===
RUN curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor -o /etc/apt/trusted.gpg.d/microsoft.gpg && \
    echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list && \
    apt update && apt install -y code

# === CONFIGURA VNC ===
RUN mkdir -p /home/${USERNAME}/.vnc && \
    echo "senha-segura" | /opt/TurboVNC/bin/vncpasswd -f > /home/${USERNAME}/.vnc/passwd && \
    chmod 600 /home/${USERNAME}/.vnc/passwd && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.vnc

# === COPIA SCRIPTS ===
COPY ../scripts/*.sh /home/${USERNAME}/
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/*.sh && \
    chmod +x /home/${USERNAME}/*.sh

# === CONFIGURA ROS NO BASHRC ===
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /projeto1_PosDoc/init.bash" >> /home/$USERNAME/.bashrc

# === COPIA O WORKSPACE ===
COPY catkin_ws /home/$USERNAME/catkin_ws

RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc && \
    sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/catkin_ws

# === SCRIPT: ABRE TUTORIAL + RVIZ ===
RUN mkdir -p /home/$USERNAME/Desktop && \
    echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /home/ubuntu/catkin_ws/devel/setup.bash\n\
RES=$(xdpyinfo | grep dimensions | awk "{print \$2}")\n\
WIDTH=$(echo $RES | cut -d "x" -f1)\n\
HEIGHT=$(echo $RES | cut -d "x" -f2)\n\
HALF_WIDTH=$(($WIDTH / 2))\n\
xdg-open "https://moveit.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html" &\n\
sleep 3\n\
wmctrl -r :ACTIVE: -e 0,0,0,$HALF_WIDTH,$HEIGHT\n\
xfce4-terminal --title="RViz Tutorial" --geometry=100x30 --hold -e "bash -c \"roslaunch panda_moveit_config demo.launch rviz_tutorial:=true\"" &\n\
sleep 5\n\
wmctrl -r RViz -e 0,$HALF_WIDTH,0,$HALF_WIDTH,$HEIGHT\n' \
    > /home/$USERNAME/start_tutorial_and_rviz.sh && \
    chmod +x /home/$USERNAME/start_tutorial_and_rviz.sh && \
    chown $USERNAME:$USERNAME /home/$USERNAME/start_tutorial_and_rviz.sh

# === ATALHOS NA DESKTOP ===
RUN echo '[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=Init ROS\n\
Comment=Inicializa ambiente ROS\n\
Exec=/home/ubuntu/init_ros.sh\n\
Icon=utilities-terminal\n\
Terminal=true\n\
Categories=Development;' \
> /home/ubuntu/Desktop/init_ros.desktop && \
chmod +x /home/ubuntu/Desktop/init_ros.desktop && \
chown ubuntu:ubuntu /home/ubuntu/Desktop/init_ros.desktop

RUN echo '[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=Tutorial + RViz\n\
Comment=Abre navegador com o tutorial e o RViz lado a lado\n\
Exec=/home/ubuntu/start_tutorial_and_rviz.sh\n\
Icon=applications-internet\n\
Terminal=false\n\
Categories=Development;' \
> /home/ubuntu/Desktop/tutorial_and_rviz.desktop && \
chmod +x /home/ubuntu/Desktop/tutorial_and_rviz.desktop && \
chown ubuntu:ubuntu /home/ubuntu/Desktop/tutorial_and_rviz.desktop

# === WALLPAPER ===
COPY ../images/wallpapers/ /usr/share/backgrounds/labvir/
RUN chmod 644 /usr/share/backgrounds/labvir/*

USER $USERNAME
RUN mkdir -p /home/$USERNAME/.config/autostart && \
    echo "[Desktop Entry]\n\
Type=Application\n\
Name=Set Wallpaper\n\
Exec=/home/ubuntu/set-wallpaper.sh\n\
Terminal=false\n\
X-GNOME-Autostart-enabled=true" \
> /home/$USERNAME/.config/autostart/set-wallpaper.desktop && \
chmod +x /home/$USERNAME/.config/autostart/set-wallpaper.desktop

# === INSTALA ROS-APT-SOURCE (MANTÉM AS CHAVES ATUALIZADAS) ===
USER root
RUN apt update && apt install -y ros-apt-source

# === PORTAS E COMANDO PADRÃO ===
EXPOSE 5901 6080
USER $USERNAME
CMD ["/home/ubuntu/start-vnc.sh"]
