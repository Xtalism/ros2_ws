#!/usr/bin/env bash
set -e

# Detecta versión de Python que usa ROS2
PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")

# Ruta al venv
VENV_DIR="$HOME/ros2_venv"

echo ">>> Creando venv con Python $PYVER en $VENV_DIR"
python3 -m venv --system-site-packages "$VENV_DIR"

# Activar e instalar pip actualizado
source "$VENV_DIR/bin/activate"
pip install --upgrade pip

echo ">>> Venv creado. Para activarlo:"
echo "source $VENV_DIR/bin/activate"

echo "En el bashrc o lo que sea que utilices, añade: \
if [ -d "$HOME/ros2_venv" ]; then \
    source $HOME/ros2_venv/bin/activate \
fi"