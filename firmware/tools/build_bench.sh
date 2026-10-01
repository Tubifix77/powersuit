#!/usr/bin/env bash
# Build both node_bench roles, each from a fresh sdkconfig, and prove the role
# actually landed in each image.
#
#   bash firmware/tools/build_bench.sh
#
# Output: firmware/apps/node_bench/build_limb/ and build_orch/, on the bind mount
# so the host can flash them (docs/bringup.md A.4d). The full build log goes to
# .verify-logs/bench_build.log; only the verdict is printed.
#
# Why a script: ESP-IDF treats an existing sdkconfig as authoritative and reads
# sdkconfig.defaults only for symbols it lacks, so rebuilding into a used
# directory silently produces the previous role's image while reporting success
# (bringup.md A.4b). Deleting the build directory every time makes that
# impossible; ccache keeps it cheap.
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPO="$PWD"
APP=firmware/apps/node_bench
LOG_DIR="$REPO/.verify-logs"
LOG="$LOG_DIR/bench_build.log"
IDF_IMAGE="espressif/idf:v5.5.5"
mkdir -p "$LOG_DIR"

docker info >/dev/null 2>&1 || { echo "docker is not running — start Docker Desktop first" >&2; exit 1; }

host_path() { echo "$1" | sed -e 's|^/\([a-z]\)/|\1:/|'; }

rm -rf "$APP/build_limb" "$APP/build_orch"

echo "== building node_bench: limb, then orchestrator (log: .verify-logs/bench_build.log)"
set +e
MSYS_NO_PATHCONV=1 docker run --rm \
    -v "$(host_path "$REPO"):/ws" -v ps_ccache:/root/.ccache \
    -e IDF_CCACHE_ENABLE=1 -w "/ws/$APP" "$IDF_IMAGE" bash -c '
        set -e
        git config --global --add safe.directory "*" 2>/dev/null
        echo "### role: limb"
        idf.py -B build_limb -D SDKCONFIG=build_limb/sdkconfig build
        echo "### role: orch"
        idf.py -B build_orch -D SDKCONFIG=build_orch/sdkconfig \
            -D SDKCONFIG_DEFAULTS="../../sdkconfig.defaults.common;sdkconfig.defaults;sdkconfig.defaults.orch" \
            build' >"$LOG" 2>&1
rc=$?
set -e

if [ $rc -ne 0 ]; then
    echo "BUILD FAILED (exit $rc). Errors:" >&2
    grep -E "error:|CMake Error|FAILED:" "$LOG" | sort -u | head -20 >&2
    exit 1
fi

# The build succeeding proves nothing about the role; read it back.
fail=0
check() {  # role, sdkconfig, expected-line
    if grep -qx "$3" "$APP/$2"; then
        echo "  ok   $1: $3"
    else
        echo "  FAIL $1: expected '$3' in $2" >&2; fail=1
    fi
}
check limb build_limb/sdkconfig "# CONFIG_PS_BENCH_IS_ORCH is not set"
check orch build_orch/sdkconfig "CONFIG_PS_BENCH_IS_ORCH=y"
for r in limb orch; do
    for f in flasher_args.json node_bench.elf node_bench.bin; do
        [ -f "$APP/build_$r/$f" ] || { echo "  FAIL $r: missing $f" >&2; fail=1; }
    done
done
if cmp -s "$APP/build_limb/node_bench.bin" "$APP/build_orch/node_bench.bin"; then
    echo "  FAIL the two app images are byte-identical — the overlay did not apply" >&2; fail=1
else
    echo "  ok   app images differ"
fi
[ $fail -eq 0 ] || exit 1

echo "node_bench: both roles built and verified"
echo "flash: powershell -File firmware/tools/flash_bench.ps1 -Role <limb|orch> -Port COMx"
