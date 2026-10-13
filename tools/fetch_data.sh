#!/usr/bin/env bash
# =============================================================================
#  MUMBAI 1:1 — Reproducible source-data fetch
# -----------------------------------------------------------------------------
#  Downloads every raw geospatial input the pipeline needs into data/raw/.
#  All sources are public and free; see DOCS/01_DATA_SOURCES.md for provenance,
#  licence terms and sha256 verification values.
#
#  Large raw datasets are deliberately NOT committed to git (see .gitignore).
#  This script is the reproducible substitute: run it, then `make world`.
#
#  Usage:  ./tools/fetch_data.sh [--verify-only]
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/data/raw"
mkdir -p "$DEST"

VERIFY_ONLY=0
[[ "${1:-}" == "--verify-only" ]] && VERIFY_ONLY=1

# Fetch a file from a GitHub repo via the API (raw.githubusercontent is not
# always reachable from restricted sandboxes; api.github.com is).
gh_raw() { # repo_ref_path_out
  local repo="$1" ref="$2" path="$3" out="$4"
  if [[ -f "$DEST/$out" ]]; then
    echo "  [cached] $out"
    return 0
  fi
  echo "  [fetch]  $repo :: $path"
  gh api -H "Accept: application/vnd.github.raw" \
     "/repos/$repo/contents/$path?ref=$ref" > "$DEST/$out"
}

echo "== MUMBAI 1:1 data fetch =="
echo "destination: $DEST"

if [[ $VERIFY_ONLY -eq 0 ]]; then
  # 1. OpenStreetMap extract — Mumbai Metropolitan Region (roads, buildings,
  #    coastline, railways, landuse, waterways). 35 MB XML, ODbL 1.0.
  gh_raw valmikkpatel/wrangle-osm-data master sample.osm mumbai_region.osm

  # 2. BMC administrative ward polygons — DataMeet. (CC-BY / ODbL per upstream)
  gh_raw datameet/Municipal_Spatial_Data master "Mumbai/BMC_Wards.geojson" bmc_wards.geojson

  # 3. Slum cluster polygons — DataMeet (informal-settlement zoning layer).
  gh_raw datameet/Municipal_Spatial_Data master "Mumbai/slumClusters.geojson" slum_clusters.geojson

  # 4. Slum Rehabilitation Authority 2017 polygons — jkkersey/Mumbai-Mapping-Data (CC-BY).
  gh_raw jkkersey/Mumbai-Mapping-Data main "Mumbai_SRA_2017.geojson" mumbai_sra_2017.geojson
fi

echo "== verifying sha256 =="
cd "$DEST"
if [[ -f SHA256SUMS ]]; then
  sha256sum -c SHA256SUMS || { echo "!! checksum mismatch"; exit 1; }
else
  echo "  (no SHA256SUMS present — generating)"
  sha256sum * > SHA256SUMS
fi

echo "== done. $(du -sh "$DEST" | cut -f1) of raw source data in data/raw/ =="
