#!/usr/bin/env bash
# Clone/refresh the vendored firmware dependencies listed in firmware/third_party.pins.
# Prefers exact SHAs from third_party.lock when present (reproducible builds);
# writes/updates the lock after resolving. Runs on the host or inside the IDF image.
set -euo pipefail

FW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP_DIR="$FW_DIR/third_party"
PINS="$FW_DIR/third_party.pins"
LOCK="$FW_DIR/third_party.lock"

mkdir -p "$TP_DIR"

locked_sha() {
    local name="$1"
    [ -f "$LOCK" ] || return 1
    awk -F'|' -v n="$name" '$1 == n { print $2; found=1 } END { exit !found }' "$LOCK"
}

tmp_lock="$(mktemp)"
trap 'rm -f "$tmp_lock"' EXIT

while IFS='|' read -r name url ref; do
    case "$name" in ''|\#*) continue ;; esac
    dest="$TP_DIR/$name"
    want="$(locked_sha "$name" || echo "$ref")"

    if [ ! -d "$dest/.git" ]; then
        echo "[fetch_deps] cloning $name @ $want"
        git clone --recurse-submodules "$url" "$dest"
    fi
    git -C "$dest" fetch --tags origin "$ref" >/dev/null 2>&1 || git -C "$dest" fetch origin
    git -C "$dest" checkout --quiet "$want"
    git -C "$dest" submodule update --init --recursive --quiet
    sha="$(git -C "$dest" rev-parse HEAD)"
    echo "$name|$sha" >> "$tmp_lock"
    echo "[fetch_deps] $name @ $sha"
done < "$PINS"

mv "$tmp_lock" "$LOCK"
trap - EXIT
echo "[fetch_deps] lock written to ${LOCK#"$FW_DIR/"}"
