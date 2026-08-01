#!/usr/bin/env bash
# Build and run the on-target self-test under QEMU (esp32s3).
#
#   bash firmware/tools/qemu_test.sh            # build + run
#   bash firmware/tools/qemu_test.sh --run-only # reuse the existing image
#
# What this adds over firmware/tests/host: the same logic compiled for Xtensa
# LX7 and executed under real FreeRTOS — packed-struct access, the single-
# precision FPU, dual-core scheduling, actual stack consumption, and the safety
# watchdog measured against real elapsed time.
#
# QEMU models the CPU, RAM, flash, UART and timers. It does NOT model TWAI,
# MCPWM, I2S, the SPI slave or the ADC, which is why this is a dedicated image
# rather than one of the node applications.
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPO="$PWD"
OUT="$REPO/.qemu"
IDF_IMAGE="espressif/idf:v5.5.5"
TIMEOUT_S="${QEMU_TIMEOUT_S:-120}"

# Espressif's QEMU fork ships with the IDF tools; fall back to PATH.
QEMU="$(ls -d "$HOME"/.espressif/tools/qemu-xtensa/*/qemu/bin/qemu-system-xtensa* 2>/dev/null | head -1)"
[ -n "$QEMU" ] || QEMU="$(command -v qemu-system-xtensa || true)"
if [ -z "$QEMU" ]; then
    echo "qemu-system-xtensa not found." >&2
    echo "Install with: python \$IDF_PATH/tools/idf_tools.py install qemu-xtensa" >&2
    exit 1
fi

host_path() { echo "$1" | sed -e 's|^/\([a-z]\)/|\1:/|'; }

if [ "${1:-}" != "--run-only" ]; then
    mkdir -p "$OUT"
    echo "== building self-test image (esp32s3)"
    MSYS_NO_PATHCONV=1 docker run --rm \
        -v "$(host_path "$REPO"):/ws" -v ps_ccache:/root/.ccache \
        -e IDF_CCACHE_ENABLE=1 -e IDF_TARGET=esp32s3 \
        -w /ws/firmware/apps/node_qemu_test "$IDF_IMAGE" bash -c '
            git config --global --add safe.directory "*" 2>/dev/null
            idf.py -B /ws/.qemu/build build >/ws/.qemu/build.log 2>&1 || {
                grep -E "error:|CMake Error" /ws/.qemu/build.log | sort -u | head -20; exit 1; }
            cd /ws/.qemu/build
            # QEMU wants one contiguous flash image of a power-of-two size.
            python -m esptool --chip esp32s3 merge_bin --fill-flash-size 4MB \
                -o /ws/.qemu/flash.bin @flash_args >/dev/null 2>&1'
fi

echo "== running under QEMU ($(basename "$QEMU"))"
set +e
timeout "$TIMEOUT_S" "$QEMU" -nographic -machine esp32s3 -m 4M \
    -drive file="$OUT/flash.bin",if=mtd,format=raw \
    -serial mon:stdio 2>&1 | tr -d '\r' > "$OUT/run.log"
set -e

if ! grep -q QEMU_SELFTEST_DONE "$OUT/run.log"; then
    echo "self-test did not complete within ${TIMEOUT_S}s — last output:" >&2
    tail -25 "$OUT/run.log" >&2
    exit 1
fi

grep -E "^(PASS|FAIL|INFO|QEMU_SELFTEST_SUMMARY)" "$OUT/run.log"
grep -q "QEMU_SELFTEST_DONE OK" "$OUT/run.log" || { echo "FAILURES above"; exit 1; }
echo "qemu self-test: all checks passed"
