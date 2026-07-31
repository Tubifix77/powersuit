#!/usr/bin/env bash
# Static analysis over the C we actually wrote.
#
# Scoped deliberately: `managed_components/` and `third_party/` hold vendored
# code (led_strip, esp_lvgl_port, micro-ROS) whose findings are not ours to fix
# and which would drown the signal. Findings here are actionable by definition.
#
#   bash tools/cppcheck.sh          # report
#   bash tools/cppcheck.sh --strict # non-zero exit on any finding (CI)
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
EXIT_CODE=0
[ "${1:-}" = "--strict" ] && EXIT_CODE=1

if ! command -v cppcheck >/dev/null 2>&1; then
    echo "cppcheck not installed; running it in a container instead"
    # Git Bash reports /d/... which Docker cannot mount; rewrite to d:/... and
    # stop MSYS from mangling the container-side path.
    host_path="$(echo "$PWD" | sed -e 's|^/\([a-z]\)/|\1:/|')"
    exec env MSYS_NO_PATHCONV=1 docker run --rm -v "$host_path:/repo" -w /repo ros:jazzy \
        bash -c "apt-get update -qq >/dev/null 2>&1 && \
                 apt-get install -y -qq cppcheck >/dev/null 2>&1 && \
                 bash tools/cppcheck.sh $*"
fi

cppcheck \
    --enable=warning,style,performance,portability \
    --inline-suppr --std=c99 --language=c \
    --suppress=missingInclude --suppress=missingIncludeSystem \
    --suppress=unusedFunction --suppress=unmatchedSuppression \
    --suppress=unknownMacro \
    -i firmware/apps/node_chest_hub/managed_components \
    -i firmware/apps/node_helmet/managed_components \
    -i firmware/third_party \
    --error-exitcode="$EXIT_CODE" \
    --quiet \
    -D__GNUC__=13 \
    -DCONFIG_PS_LIMB_NODE_ID=1 \
    -DPS_UROS_ENABLED=0 \
    -I common/c/include \
    -I firmware/components/ps_can/include \
    -I firmware/components/ps_safety/include \
    -I firmware/components/ps_ctl/include \
    -I firmware/components/ps_audio/include \
    -I firmware/components/ps_router/include \
    -I firmware/components/ps_spibridge/include \
    -I firmware/components/ps_uros/include \
    common/c/src firmware/components firmware/apps

echo "cppcheck: clean"
