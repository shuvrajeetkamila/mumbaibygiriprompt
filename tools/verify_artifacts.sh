#!/usr/bin/env bash
# =============================================================================
#  MUMBAI 1:1 EXACT — tools/verify_artifacts.sh
# -----------------------------------------------------------------------------
#  Artifact integrity check for the one binary this repository ships: the viewer
#  payload under viewer/data/.  The build is byte-reproducible (two runs of the
#  pipeline produce identical hashes), so viewer/data/SHA256SUMS pins the exact
#  bytes and this script re-verifies them.
#
#  It exists because a binary can be corrupted *outside* the build and still look
#  plausible: a UTF-8 round trip replaces every byte sequence that is not valid
#  UTF-8 with U+FFFD, so mumbai.bin grew from 8,359,144 B to 11,611,201 B with
#  1,635,488 replacement sequences while mumbai.json still described the original
#  layout (the viewer would have rendered garbage, and the file "existed").
#
#      bash tools/verify_artifacts.sh [viewerDir]
# =============================================================================
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="${1:-$ROOT/viewer/data}"

fail=0
note() { printf '  %-46s %s\n' "$1" "$2"; }

echo "== payload artifacts =="
for f in mumbai.bin mumbai.json atlas_basecolor.png; do
    if [[ ! -s "$DIR/$f" ]]; then
        note "$f" "MISSING — run: make world"
        fail=1
    else
        note "$f" "$(stat -c%s "$DIR/$f") B"
    fi
done
[[ "$fail" == "1" ]] && exit 1

echo "== checksums =="
if [[ ! -s "$DIR/SHA256SUMS" ]]; then
    note "SHA256SUMS" "MISSING — run: make world"
    exit 1
fi
if (cd "$DIR" && sha256sum -c --status SHA256SUMS); then
    (cd "$DIR" && sha256sum SHA256SUMS >/dev/null 2>&1 || true)
    awk '{ printf "  %-46s %s\n", $2, substr($1, 1, 16) "…" }' "$DIR/SHA256SUMS"
else
    echo "  checksum mismatch:"
    (cd "$DIR" && sha256sum -c SHA256SUMS 2>&1 | sed 's/^/    /')
    echo "  the payload on disk is not the payload this repository published."
    echo "  a corrupted artifact is almost always recoverable by rebuilding:"
    echo "      make world && make all-checks"
    fail=1
fi

if [[ "$fail" == "0" ]]; then echo "  artifacts: OK"; else echo "  artifacts: FAILED"; fi
exit "$fail"
