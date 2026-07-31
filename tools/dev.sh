#!/usr/bin/env bash
# Developer conveniences that don't need Docker.
#
#   bash tools/dev.sh setup     create .venv and install every package editable
#   bash tools/dev.sh test      run all three Python suites
#   bash tools/dev.sh vectors   regenerate the cross-language test vectors
#   bash tools/dev.sh sim       run the simulated suit for a real bridge to attach
#   bash tools/dev.sh clean     drop caches and build leftovers
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

PY=".venv/Scripts/python"     # Git Bash on Windows
[ -x "$PY" ] || PY=".venv/bin/python"

need_venv() {
    if [ ! -x "$PY" ]; then
        echo "no .venv — run: bash tools/dev.sh setup" >&2
        exit 1
    fi
}

case "${1:-help}" in
setup)
    python -m venv .venv
    [ -x "$PY" ] || PY=".venv/bin/python"
    "$PY" -m pip install -q -U pip
    "$PY" -m pip install -e ./common/python -e "./cloud[dev]" -e ./tools/suit_sim
    echo "ready. next: bash tools/dev.sh test"
    ;;
test)
    need_venv
    # One suite per invocation: each ships its own conftest.py, and pytest's
    # import mode lets one shadow the other if they share a run.
    fail=0
    for suite in common/python/tests cloud/tests tools/suit_sim/tests; do
        printf '%-26s ' "$suite"
        "$PY" -m pytest "$suite" -q 2>&1 | tail -1 || fail=1
    done
    exit $fail
    ;;
vectors)
    need_venv
    "$PY" common/python/tests/gen_vectors.py
    echo "regenerated — commit both files or CI's vectors-fresh job will fail"
    ;;
sim)
    need_venv
    shift
    exec "$PY" -m suit_sim.serve "$@"
    ;;
clean)
    find . -type d \( -name __pycache__ -o -name .pytest_cache -o -name .ruff_cache \) \
        -not -path "./.venv/*" -prune -exec rm -rf {} + 2>/dev/null || true
    rm -rf .verify-logs
    echo "cleaned (build/ install/ log/ and .venv left alone)"
    ;;
*)
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
    ;;
esac
