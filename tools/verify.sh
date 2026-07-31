#!/usr/bin/env bash
# Verify every tier of the suit and print a compact summary.
#
# Full logs go to .verify-logs/<tier>.log — nothing verbose reaches stdout, which
# is the point: build output is enormous and only the failures matter.
#
#   bash tools/verify.sh            # everything
#   bash tools/verify.sh py c       # only the named tiers
#   bash tools/verify.sh --failures # re-print the error lines from the last run
#
# Tiers: py (native pytest), c (host C suites), fw (4 firmware builds),
#        ros (colcon build+test), cloud (3.12 container parity).
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO="$PWD"
LOGS="$REPO/.verify-logs"
mkdir -p "$LOGS"

PY="$REPO/.venv/Scripts/python"
[ -x "$PY" ] || PY="$REPO/.venv/bin/python"
[ -x "$PY" ] || PY="python"

DOCKER_REPO="$(echo "$REPO" | sed -e 's|^/\([a-z]\)/|\1:/|')"
IDF_IMAGE="espressif/idf:v5.5.5"
ROS_IMAGE="ros:jazzy"

declare -a NAMES=() RESULTS=() DETAILS=()

if [ "${1:-}" = "--failures" ]; then
    for f in "$LOGS"/*.log; do
        [ -e "$f" ] || continue
        hits=$(grep -nE "error:|FAILED|failed|Error [0-9]" "$f" | head -15)
        if [ -n "$hits" ]; then
            echo "=== $(basename "$f" .log)"
            echo "$hits"
            echo
        fi
    done
    exit 0
fi

WANT=("$@")
want() {
    [ ${#WANT[@]} -eq 0 ] && return 0
    for w in "${WANT[@]}"; do [ "$w" = "$1" ] && return 0; done
    return 1
}

record() {   # name  exit_code  detail
    NAMES+=("$1"); DETAILS+=("$3")
    if [ "$2" -eq 0 ]; then RESULTS+=("PASS"); else RESULTS+=("FAIL"); fi
    printf '  %-28s %s\n' "$1" "$([ "$2" -eq 0 ] && echo ok || echo FAILED)"
}

dock() { MSYS_NO_PATHCONV=1 docker "$@"; }

echo "powersuit verification -> $LOGS"

# ---- native python -----------------------------------------------------------
if want py; then
    echo "python:"
    for suite in common/python/tests cloud/tests tools/suit_sim/tests \
                 orchestrator/ros2_ws/src; do
        [ -d "$suite" ] || continue
        name="pytest $(basename "$(dirname "$suite")")/$(basename "$suite")"
        log="$LOGS/py-$(echo "$suite" | tr '/' '-').log"
        "$PY" -m pytest "$suite" -q >"$log" 2>&1
        rc=$?
        # pytest exit 5 = no tests collected; treat as skip, not failure
        [ $rc -eq 5 ] && continue
        record "$name" $rc "$(tail -1 "$log")"
    done
fi

# ---- pure C host suites ------------------------------------------------------
if want c; then
    echo "c host tests:"
    log="$LOGS/c-host.log"
    dock run --rm -v "$DOCKER_REPO:/repo" -w /repo "$ROS_IMAGE" bash -c \
        "cmake -S firmware/tests/host -B /tmp/host && cmake --build /tmp/host -j && \
         ctest --test-dir /tmp/host --output-on-failure" >"$log" 2>&1
    record "ctest firmware/tests/host" $? "$(grep -E '% tests passed' "$log" | tail -1)"
fi

# ---- firmware ----------------------------------------------------------------
if want fw; then
    echo "firmware:"
    dock volume create ps_fw_build >/dev/null 2>&1
    dock volume create ps_ccache   >/dev/null 2>&1
    for app in node_limb node_flight node_helmet node_chest_hub; do
        [ -f "firmware/apps/$app/CMakeLists.txt" ] || { printf '  %-28s %s\n' "$app" "absent"; continue; }
        log="$LOGS/fw-$app.log"
        dock run --rm -v "$DOCKER_REPO:/ws" -v ps_fw_build:/builds -v ps_ccache:/root/.ccache \
            -w "/ws/firmware/apps/$app" -e IDF_CCACHE_ENABLE=1 "$IDF_IMAGE" bash -c \
            "git config --global --add safe.directory '*'; idf.py -B /builds/$app build" >"$log" 2>&1
        record "idf build $app" $? "$(grep -E 'binary size' "$log" | tail -1)"
    done
fi

# ---- ros 2 workspace ---------------------------------------------------------
if want ros; then
    echo "ros 2:"
    if [ -f tools/ros_ci.sh ]; then
        dock volume create ps_ros_deps >/dev/null 2>&1
        dock volume create ps_ros_out  >/dev/null 2>&1
        log="$LOGS/ros.log"
        dock run --rm -v "$DOCKER_REPO:/repo:ro" -v ps_ros_deps:/deps -v ps_ros_out:/out \
            -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST "$ROS_IMAGE" \
            bash /repo/tools/ros_ci.sh >"$log" 2>&1
        record "colcon build+test" $? "$(grep -E 'Summary:' "$log" | tail -1)"
    else
        printf '  %-28s %s\n' "colcon build+test" "absent (tools/ros_ci.sh)"
    fi
fi

# ---- cloud container ---------------------------------------------------------
if want cloud; then
    echo "cloud image:"
    if [ -f cloud/Dockerfile ]; then
        log="$LOGS/cloud-docker.log"
        dock build --target test -f cloud/Dockerfile . >"$log" 2>&1
        record "docker build (py3.12 tests)" $? "$(tail -3 "$log" | head -1)"
    else
        printf '  %-28s %s\n' "docker build" "absent (cloud/Dockerfile)"
    fi
fi

# ---- summary -----------------------------------------------------------------
echo
echo "================ summary ================"
fails=0
for i in "${!NAMES[@]}"; do
    printf '%-4s %-30s %s\n' "${RESULTS[$i]}" "${NAMES[$i]}" "${DETAILS[$i]}"
    [ "${RESULTS[$i]}" = "FAIL" ] && fails=$((fails + 1))
done
echo "========================================="
if [ "$fails" -eq 0 ]; then
    echo "all green (${#NAMES[@]} checks)"
else
    echo "$fails of ${#NAMES[@]} failed — 'bash tools/verify.sh --failures' for the error lines"
fi
exit $((fails > 0))
