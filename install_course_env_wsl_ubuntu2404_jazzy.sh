#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="${SUDO_USER:-${USER}}"
TARGET_HOME="$(eval echo ~"${TARGET_USER}")"
AI_VENV="/opt/ai-venv"
ROS_DISTRO_NAME="jazzy"
UBUNTU_CODENAME_EXPECTED="noble"
USE_SWAP="${USE_SWAP:-0}"
SWAP_SIZE_GB="${SWAP_SIZE_GB:-8}"
INSTALL_VSCODE="${INSTALL_VSCODE:-0}"

log() {
  echo
  echo "============================================================"
  echo "$1"
  echo "============================================================"
}

warn() {
  echo "[AVISO] $1"
}

require_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "Este script debe ejecutarse con sudo o como root."
    echo "Ejemplo: sudo bash install_course_env_wsl_ubuntu2404_jazzy.sh"
    exit 1
  fi
}

check_ubuntu_version() {
  local ubuntu_codename
  ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"

  if [[ "${ubuntu_codename}" != "${UBUNTU_CODENAME_EXPECTED}" ]]; then
    echo "Este instalador está preparado para Ubuntu 24.04 LTS (noble)."
    echo "Sistema detectado: ${ubuntu_codename}"
    exit 1
  fi
}

check_wsl() {
  if ! grep -qiE '(microsoft|wsl)' /proc/version 2>/dev/null; then
    warn "No parece que esté ejecutándose dentro de WSL. El script puede seguir funcionando, pero está pensado para WSL2 + WSLg."
  fi
}

create_swap_if_needed() {
  if [[ "${USE_SWAP}" != "1" ]]; then
    echo "Se omite la creación de swap porque USE_SWAP=${USE_SWAP}."
    return 0
  fi

  if swapon --show | grep -q .; then
    echo "Swap ya configurada. Se mantiene sin cambios."
    return 0
  fi

  log "Creando swap de ${SWAP_SIZE_GB} GB"
  fallocate -l "${SWAP_SIZE_GB}G" /swapfile || dd if=/dev/zero of=/swapfile bs=1M count=$((SWAP_SIZE_GB * 1024))
  chmod 600 /swapfile
  mkswap /swapfile
  swapon /swapfile
  grep -q '^/swapfile ' /etc/fstab || echo '/swapfile none swap sw 0 0' >> /etc/fstab
}

install_base_packages() {
  log "Actualizando sistema e instalando paquetes base"
  apt-get update
  apt-get upgrade -y
  apt-get install -y software-properties-common curl gnupg2 lsb-release ca-certificates wget
  add-apt-repository -y universe
  apt-get update

  local packages=(
    net-tools iproute2 iputils-ping traceroute iptables nano git unzip zip
    build-essential cmake pkg-config
    python3-pip python3-venv python3-dev
    libopencv-dev python3-opencv ffmpeg libsm6 libxext6 libgl1 libegl1 mesa-utils
    x11-apps dbus-x11 xdg-utils
    terminator
  )

  apt-get install -y "${packages[@]}"
}

install_ros2() {
  log "Instalando ROS 2 ${ROS_DISTRO_NAME^} y paquetes del curso"
  local ros_apt_source_version
  local ubuntu_codename

  ros_apt_source_version="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"

  curl -L -o /tmp/ros2-apt-source.deb     "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb

  apt-get update

  apt-get install -y     python3-argcomplete     python3-rosdep     python3-vcstool     python3-colcon-common-extensions     ros-dev-tools

  apt-get install -y     ros-${ROS_DISTRO_NAME}-desktop     ros-${ROS_DISTRO_NAME}-ros-gz     ros-${ROS_DISTRO_NAME}-ros-gz-sim     ros-${ROS_DISTRO_NAME}-ros-gz-bridge     ros-${ROS_DISTRO_NAME}-ros-gz-interfaces     ros-${ROS_DISTRO_NAME}-navigation2     ros-${ROS_DISTRO_NAME}-nav2-bringup     ros-${ROS_DISTRO_NAME}-moveit     ros-${ROS_DISTRO_NAME}-rmw-cyclonedds-cpp

  rosdep init 2>/dev/null || true
  sudo -u "${TARGET_USER}" bash -lc 'rosdep update' || true
}

install_python_ai_stack() {
  log "Creando entorno virtual Python e instalando librerías del curso"
  rm -rf "${AI_VENV}"
  python3 -m venv "${AI_VENV}"

  source "${AI_VENV}/bin/activate"
  python -m pip install --upgrade pip setuptools wheel
  python -m pip install --no-cache-dir "sympy>=1.12"
  python -m pip install --no-cache-dir     torch torchvision torchaudio     --index-url https://download.pytorch.org/whl/cpu
  python -m pip install --no-cache-dir     transformers     huggingface_hub     langchain     langchain-community     opencv-python     ultralytics
  deactivate
}

install_vscode_linux() {
  if [[ "${INSTALL_VSCODE}" != "1" ]]; then
    echo "Se omite la instalación de VS Code en Ubuntu porque INSTALL_VSCODE=${INSTALL_VSCODE}."
    return 0
  fi

  log "Instalando Visual Studio Code en Ubuntu"
  mkdir -p /etc/apt/keyrings
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /etc/apt/keyrings/packages.microsoft.gpg
  chmod go+r /etc/apt/keyrings/packages.microsoft.gpg

  cat > /etc/apt/sources.list.d/vscode.sources <<'EOS'
Types: deb
URIs: https://packages.microsoft.com/repos/code
Suites: stable
Components: main
Architectures: amd64 arm64 armhf
Signed-By: /etc/apt/keyrings/packages.microsoft.gpg
EOS

  apt-get update
  apt-get install -y apt-transport-https code
}

configure_user_shell() {
  log "Configurando shell del usuario ${TARGET_USER}"
  local bashrc="${TARGET_HOME}/.bashrc"
  touch "${bashrc}"

  python3 - <<PY
from pathlib import Path
bashrc = Path(${bashrc@Q})
block = r"""
# >>> curso ros2 jazzy wsl >>>
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-0}

ai_on_impl() {
  bash --noprofile --norc -i <<'EOS'
source ~/.bashrc
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
source /opt/ai-venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export QT_X11_NO_MITSHM=1
export DISPLAY=\${DISPLAY:-:0}
export WAYLAND_DISPLAY=\${WAYLAND_DISPLAY:-wayland-0}
export XDG_RUNTIME_DIR=\${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}
PS1='(ai-env) '\"\${PS1}\"
echo 'Entorno del curso activado. Usa ai-off para salir.'
EOS
}

alias ai-on='ai_on_impl'
alias ai-off='exit'
alias gazebo-on='source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_LOCALHOST_ONLY=1 && export QT_X11_NO_MITSHM=1 && gz sim -v 4'
alias gz-course='gazebo-on'
alias code-course='code .'
alias term-course='terminator'
alias term2='terminator'
alias term3='terminator'
# <<< curso ros2 jazzy wsl <<<
"""
content = bashrc.read_text() if bashrc.exists() else ''
start = '# >>> curso ros2 jazzy wsl >>>'
end = '# <<< curso ros2 jazzy wsl <<<'
if start in content and end in content:
    prefix = content.split(start)[0].rstrip()
    suffix = content.split(end, 1)[1].lstrip('\n')
    new_content = (prefix + '\n\n' + block.strip() + '\n\n' + suffix).strip() + '\n'
else:
    new_content = content.rstrip() + '\n\n' + block.strip() + '\n'
bashrc.write_text(new_content)
PY

  chown "${TARGET_USER}:${TARGET_USER}" "${bashrc}"
}

create_helper_scripts() {
  log "Creando utilidades de arranque"

  cat > /usr/local/bin/gazebo-course <<EOF2
#!/usr/bin/env bash
set -e
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export QT_X11_NO_MITSHM=1
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
exec gz sim -v 4 "\$@"
EOF2
  chmod +x /usr/local/bin/gazebo-course

  cat > /usr/local/bin/code-course <<'EOF2'
#!/usr/bin/env bash
set -e
if command -v code >/dev/null 2>&1; then
  exec code .
else
  echo "No se encuentra el comando 'code'."
  echo "Opciones:"
  echo "  1) Instalar VS Code en Windows + extensión WSL."
  echo "  2) Reejecutar este script con: sudo INSTALL_VSCODE=1 bash ./install_course_env_wsl_ubuntu2404_jazzy.sh"
  exit 1
fi
EOF2
  chmod +x /usr/local/bin/code-course

  cat > /usr/local/bin/term-course <<'EOF2'
#!/usr/bin/env bash
set -e
exec terminator
EOF2
  chmod +x /usr/local/bin/term-course
}

create_verification_script() {
  log "Creando script de verificación"
  cat > /usr/local/bin/verify_ros2_wsl_env.sh <<EOF2
#!/usr/bin/env bash
set -eo pipefail

dpkg -s ros-${ROS_DISTRO_NAME}-desktop ros-${ROS_DISTRO_NAME}-navigation2 ros-${ROS_DISTRO_NAME}-nav2-bringup ros-${ROS_DISTRO_NAME}-moveit terminator >/dev/null

set +u
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
set -u
ros2 --help >/dev/null
gz sim --help >/dev/null

source /opt/ai-venv/bin/activate
python - <<'PY'
import sympy, torch, transformers, huggingface_hub, langchain, langchain_community, cv2
from ultralytics import YOLO
print('sympy', sympy.__version__)
print('torch', torch.__version__)
print('transformers', transformers.__version__)
print('huggingface_hub', huggingface_hub.__version__)
print('langchain', langchain.__version__)
print('opencv', cv2.__version__)
YOLO('yolov8n.pt')
print('ultralytics ok')
PY

echo 'Verificación completada correctamente.'
EOF2
  chmod +x /usr/local/bin/verify_ros2_wsl_env.sh
}

cleanup() {
  log "Limpiando cachés de APT"
  apt-get clean
  rm -rf /var/lib/apt/lists/*
}

final_notes() {
  log "Instalación completada"
  echo "1) Cierra completamente la terminal WSL y vuelve a abrirla."
  echo "2) Ejecuta: source ~/.bashrc"
  echo "3) Entra al entorno con: ai-on"
  echo "4) Sal del entorno con: ai-off"
  echo "5) Arranca Gazebo con: gazebo-on"
  echo "6) Abre VS Code con: code-course"
  echo "7) Abre Terminator con: term-course"
  echo "8) Verifica el entorno con: sudo /usr/local/bin/verify_ros2_wsl_env.sh"
}

main() {
  require_root
  check_ubuntu_version
  check_wsl
  create_swap_if_needed
  install_base_packages
  install_ros2
  install_python_ai_stack
  install_vscode_linux
  configure_user_shell
  create_helper_scripts
  create_verification_script
  cleanup
  final_notes
}

main "$@"
