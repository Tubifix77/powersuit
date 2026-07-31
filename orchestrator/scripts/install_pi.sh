#!/usr/bin/env bash
# Provision a Raspberry Pi 5 (Ubuntu 24.04 arm64) as Node 8.
#
# Run from a checkout on the Pi:  sudo bash orchestrator/scripts/install_pi.sh
#
# Real-time notes (not automated here, because they are board- and
# kernel-specific): the bridge's transport thread asks for SCHED_FIFO and logs a
# warning if it is refused, so either run the unit as root or grant
# CAP_SYS_NICE. For deterministic 1 kHz SPI service, use a PREEMPT_RT kernel and
# isolate a core for the bridge (isolcpus= on the kernel command line).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$REPO/orchestrator/ros2_ws"
DEPS="${DEPS:-/opt/powersuit/deps}"

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root (sudo)" >&2
    exit 1
fi

echo "== apt dependencies"
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    curl gnupg lsb-release ca-certificates \
    python3-pip python3-venv python3-vcstool \
    libgpiod-dev gpiod

if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "== adding the ROS 2 apt repository"
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
    apt-get update
fi

echo "== ROS 2 Jazzy"
apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-dev-tools

echo "== powersuit protocol library"
pip3 install --break-system-packages -e "$REPO/common/python" websockets

source /opt/ros/jazzy/setup.bash
rosdep init 2>/dev/null || true
rosdep update --rosdistro jazzy

echo "== vendored dependencies (micro-ROS agent)"
mkdir -p "$DEPS/src"
vcs import "$DEPS/src" < "$WS/deps.repos" --skip-existing
rosdep install --from-paths "$DEPS/src" --ignore-src -y --rosdistro jazzy || true
colcon build --base-paths "$DEPS/src" --build-base "$DEPS/build" \
             --install-base "$DEPS/install" --merge-install \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$DEPS/install/setup.bash"

echo "== orchestrator workspace"
rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy || true
cd "$WS"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "== runtime state + service"
install -d -m 0755 /var/lib/powersuit          # e-stop counter, black-box dumps
install -d -m 0755 /etc/powersuit
if [ ! -f /etc/powersuit/env ]; then
    cat > /etc/powersuit/env <<'ENVEOF'
# Bearer token for the Node 9 link. Keep this file 0600.
POWERSUIT_LINK_TOKEN=change-me
ENVEOF
    chmod 0600 /etc/powersuit/env
    echo "   wrote /etc/powersuit/env — set POWERSUIT_LINK_TOKEN before going live"
fi

sed -e "s|@REPO@|$REPO|g" -e "s|@DEPS@|$DEPS|g" \
    "$REPO/orchestrator/systemd/powersuit.service" > /etc/systemd/system/powersuit.service
systemctl daemon-reload

echo
echo "done. enable at boot with:  systemctl enable --now powersuit"
echo "SPI must be on: add dtparam=spi=on to /boot/firmware/config.txt and reboot."
