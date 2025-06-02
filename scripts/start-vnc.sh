#!/bin/bash

# Configurações de ambiente
export USER=${USER:-ubuntu}
export HOME=/home/$USER
export DISPLAY=:1

# Mata instâncias antigas e limpa arquivos temporários
vncserver -kill $DISPLAY >/dev/null 2>&1 || true
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1 $HOME/.vnc/*

# Cria xstartup para o XFCE
mkdir -p "$HOME/.vnc"
cat <<EOF > "$HOME/.vnc/xstartup.turbovnc"
#!/bin/sh
autocutsel -fork
unset SESSION_MANAGER
exec startxfce4
EOF
chmod +x "$HOME/.vnc/xstartup.turbovnc"

# Detecta resolução atual (pode variar dependendo do setup)
CURRENT_RESOLUTION=$(xrandr | grep '*' | awk '{print $1}' | head -n1)

# Fallback para uma resolução padrão caso xrandr não retorne nada
if [ -z "$CURRENT_RESOLUTION" ]; then
    CURRENT_RESOLUTION="1920x900"
fi

WIDTH=$(echo $CURRENT_RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $CURRENT_RESOLUTION | cut -d'x' -f2)

echo "Resolução detectada: ${WIDTH}x${HEIGHT}"



# ⚠️ Removido o Xvfb — desnecessário com TurboVNC
# Xvfb $DISPLAY -screen 0 1920x900x24 +extension GLX +render -noreset >/dev/null 2>&1 &

echo "Iniciando TurboVNC no display $DISPLAY..."
vncserver $DISPLAY -geometry ${WIDTH}x${HEIGHT} -depth 24 \
    -xstartup "$HOME/.vnc/xstartup.turbovnc" \
    -securitytypes TLSNone \
    -nohttpd \
    -noxstartup

# Aguarda inicialização
sleep 3

# Inicia o websockify se não estiver em execução
if ! pgrep -f "websockify.*6080" > /dev/null; then
    echo "Iniciando websockify em :6080..."
    websockify --web /usr/share/novnc 6080 localhost:5901 >/dev/null 2>&1 &
else
    echo "websockify já está em execução. Pulando."
fi

# Após o websockify
echo "Checando se DISPLAY está ativo..."
export DISPLAY=:1
xdpyinfo >/dev/null 2>&1 && echo "DISPLAY ok!" || echo "Falha ao acessar DISPLAY"

# Aguarda até que o XFCE esteja realmente pronto (aumente o tempo se necessário)
timeout 20s bash -c 'while ! xfdesktop --version >/dev/null 2>&1; do sleep 1; done'

# Ou verifique por processos específicos do XFCE
until pgrep -f xfce4-session >/dev/null; do
    sleep 1
done

# Só então execute o script do wallpaper
"$DIR/set-wallpaper.sh"

# Inicia aplicativos (se necessário)
bash "$HOME/start-apps.sh"

# Mantém o container ativo
tail -f /dev/null
