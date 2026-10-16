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

echo "== the shipped manifest describes the world, not the run =="
# The manifest's sha256 is the identity of the world: it names the published
# archive, and a client refuses a release whose manifest is not the one it was
# built for.  So the manifest may not contain anything that changes between two
# builds of the same data — and until D-051 it did: every stage's wall-clock
# duration was written into `quality.counters` *and* into every cell's QAReport
# section, which made two runs of the same pipeline differ in 13 counters and in
# 80 bytes of each cell.  This is the guard.
if [[ -s "$ROOT/data/processed/world_manifest.json" ]]; then
    python3 - "$ROOT/data/processed/world_manifest.json" <<'PYTHON'
import json, sys
manifest = json.load(open(sys.argv[1]))
counters = manifest.get("quality", {}).get("counters", [])
# `seconds`, and `secon`/`second` too: QaCounter::name is 32 bytes, so a long
# counter name arrives truncated (`stage.buildRailAndLanduse.secon`) and a guard that
# only looked for the whole word would let exactly the longest ones through.
timings = [c for c in counters if "seconds" in c["name"] or c["name"].endswith("secon")]
if timings:
    print("  manifest: FAILED — %d run timings are inside the world's identity:" % len(timings))
    for c in timings[:5]:
        print("    %s = %s" % (c["name"], c["value"]))
    sys.exit(1)
print("  %-46s %d counters, none of them a duration" % ("world_manifest.json", len(counters)))
PYTHON
    [[ $? == 0 ]] || fail=1
else
    note "world_manifest.json" "not built yet — skipped"
fi

if [[ "$fail" == "0" ]]; then echo "  artifacts: OK"; else echo "  artifacts: FAILED"; fi
exit "$fail"
