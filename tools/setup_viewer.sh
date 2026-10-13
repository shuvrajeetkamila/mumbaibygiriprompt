#!/usr/bin/env bash
# =============================================================================
#  MUMBAI 1:1 EXACT — viewer setup
# -----------------------------------------------------------------------------
#  The viewer is a single-file three.js application.  three.js is vendored under
#  viewer/vendor/ (decision D-017: the preview iframe has no network access, so
#  a CDN reference would render a blank page).  This script verifies that the
#  vendored files are present and, when npm is available, can refresh them from
#  the pinned version in package.json.
#
#  Usage:  bash tools/setup_viewer.sh [--refresh] [--serve]
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="$ROOT/viewer/vendor"
REFRESH=0
SERVE=0
for arg in "$@"; do
    case "$arg" in
        --refresh) REFRESH=1 ;;
        --serve)   SERVE=1 ;;
        *) echo "usage: $0 [--refresh] [--serve]" >&2; exit 2 ;;
    esac
done

THREE_VERSION="$(node -e "process.stdout.write(require('$ROOT/package.json').dependencies.three.replace(/[^0-9.]/g,''))" 2>/dev/null || echo '0.169.0')"

if [[ "$REFRESH" == "1" ]]; then
    if ! command -v npm >/dev/null 2>&1; then
        echo "npm is not installed; keeping the vendored copy" >&2
    elif ! curl -fsS -o /dev/null --max-time 10 https://registry.npmjs.org/three 2>/dev/null; then
        echo "registry.npmjs.org unreachable; keeping the vendored copy" >&2
    else
        echo "== refreshing three.js $THREE_VERSION from npm =="
        ( cd "$ROOT" && npm install --no-audit --no-fund --silent three@"$THREE_VERSION" )
        mkdir -p "$VENDOR"
        cp "$ROOT/node_modules/three/build/three.module.js" "$VENDOR/three.module.js"
        cp "$ROOT/node_modules/three/examples/jsm/controls/OrbitControls.js" "$VENDOR/OrbitControls.js"
    fi
fi

echo "== viewer =="
for f in three.module.js OrbitControls.js; do
    if [[ ! -s "$VENDOR/$f" ]]; then
        echo "  missing: viewer/vendor/$f" >&2
        echo "  run: npm install && bash tools/setup_viewer.sh --refresh" >&2
        exit 1
    fi
    printf '  %-20s %8d B\n' "$f" "$(stat -c%s "$VENDOR/$f")"
done
if [[ -s "$ROOT/viewer/data/mumbai.bin" ]]; then
    printf '  %-20s %8d B (published payload)\n' "data/mumbai.bin" "$(stat -c%s "$ROOT/viewer/data/mumbai.bin")"
else
    echo "  no payload yet — run: make world"
fi

if [[ "$SERVE" == "1" ]]; then
    echo "== serving http://0.0.0.0:8000/ (ctrl-c to stop) =="
    exec python3 -m http.server 8000 --bind 0.0.0.0 --directory "$ROOT/viewer"
fi
echo
echo "serve it with:  make serve      (python3 -m http.server 8000 --directory viewer)"
