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

# Detecta resolução atual (ou usa fallback)
CURRENT_RESOLUTION=$(xrandr | grep '*' | awk '{print $1}' | head -n1)
if [ -z "$CURRENT_RESOLUTION" ]; then
    CURRENT_RESOLUTION="1920x900"
fi
WIDTH=$(echo $CURRENT_RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $CURRENT_RESOLUTION | cut -d'x' -f2)

echo "Resolução detectada: ${WIDTH}x${HEIGHT}"

# Inicia Xvfb (necessário em ambientes headless)
echo "Iniciando Xvfb no display $DISPLAY..."
Xvfb $DISPLAY -screen 0 ${WIDTH}x${HEIGHT}x24 +extension GLX +render -noreset >/dev/null 2>&1 &

sleep 2

# Inicia o VNC
echo "Iniciando TurboVNC no display $DISPLAY..."
vncserver $DISPLAY -geometry ${WIDTH}x${HEIGHT} -depth 24 \
    -xstartup "$HOME/.vnc/xstartup.turbovnc" \
    -securitytypes TLSNone \
    -noxstartup

# Verifica se o VNC subiu corretamente
if ss -tuln | grep ":5901" >/dev/null; then
    echo "✅ VNC ativo na porta 5901."
else
    echo "❌ VNC NÃO está rodando na porta 5901. Abortando."
    exit 1
fi

# Inicia o websockify se não estiver rodando
if ! pgrep -f "websockify.*6080" > /dev/null; then
    echo "Iniciando websockify em :6080..."
    websockify --web /usr/share/novnc 6080 localhost:5901 >/dev/null 2>&1 &
else
    echo "websockify já está rodando. Pulando."
fi

# Checa se o DISPLAY está ativo
export DISPLAY=:1
xdpyinfo >/dev/null 2>&1 && echo "DISPLAY ok!" || echo "Falha ao acessar DISPLAY"

# Aguarda XFCE subir
timeout 20s bash -c 'while ! pgrep -f xfce4-session >/dev/null; do sleep 1; done'

echo "XFCE carregado com sucesso."

# Executa script de wallpaper (opcional)
"$DIR/set-wallpaper.sh" 2>/dev/null || true

# Executa apps se houver
bash "$HOME/start-apps.sh" 2>/dev/null || true

# Mantém o container ativo
tail -f /dev/null
