#!/usr/bin/env bash
# Build and test the Node 8 workspace inside a ros:jazzy container.
#
# Used verbatim by CI and by tools/verify.sh. The repo may be mounted read-only,
# so every writable path is a separate volume:
#   REPO  source tree (may be :ro)
#   DEPS  vendored dependency workspace (micro-ROS agent et al.)
#   OUT   build/install/test output for the project workspace
#
# SKIP_DEPS=1 builds only the project workspace against whatever is already in
# DEPS — useful when iterating, since the agent superbuild is slow.
# NOTE: no `set -u`. ROS's setup.bash dereferences unbound variables
# (AMENT_TRACE_SETUP_FILES and friends), so sourcing it under `set -u` aborts
# the script before anything is built.
set -eo pipefail

REPO="${REPO:-/repo}"
DEPS="${DEPS:-/deps}"
OUT="${OUT:-/out}"
WS="$REPO/orchestrator/ros2_ws"

echo "== ros_ci: repo=$REPO deps=$DEPS out=$OUT"
mkdir -p "$DEPS/src" "$OUT"

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# libgpiod is optional at compile time (spidev_transport guards the include),
# but installing it here means CI actually exercises the real DATA_READY path.
apt-get install -y -qq --no-install-recommends \
    python3-pip python3-vcstool libgpiod-dev >/dev/null

# powersuit_proto is shared with the firmware and cloud tiers. Copy it out of
# the (possibly read-only) repo mount first: an editable install needs to write
# egg-info back into the source tree, which fails under :ro.
rm -rf "$DEPS/pyproto"
cp -r "$REPO/common/python" "$DEPS/pyproto"
pip3 install --break-system-packages --quiet "$DEPS/pyproto" websockets pytest

source /opt/ros/jazzy/setup.bash
rosdep update --rosdistro jazzy >/dev/null 2>&1 || true

if [ "${SKIP_DEPS:-0}" != "1" ]; then
    echo "== importing vendored dependencies"
    vcs import "$DEPS/src" < "$WS/deps.repos" --skip-existing
    rosdep install --from-paths "$DEPS/src" --ignore-src -y --rosdistro jazzy >/dev/null || true
    echo "== building dependency workspace"
    colcon build \
        --base-paths "$DEPS/src" \
        --build-base "$DEPS/build" \
        --install-base "$DEPS/install" \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [ -f "$DEPS/install/setup.bash" ]; then
    source "$DEPS/install/setup.bash"
fi

rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy >/dev/null || true

echo "== building project workspace"
colcon build \
    --base-paths "$WS/src" \
    --build-base "$OUT/build" \
    --install-base "$OUT/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "== testing project workspace"
colcon test \
    --base-paths "$WS/src" \
    --build-base "$OUT/build" \
    --install-base "$OUT/install" \
    --test-result-base "$OUT/test_results" \
    --event-handlers console_direct+ || true

colcon test-result --test-result-base "$OUT/test_results" --verbose
echo "== ros_ci: done"
