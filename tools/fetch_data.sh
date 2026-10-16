#!/usr/bin/env bash
# =============================================================================
#  MUMBAI 1:1 — Reproducible source-data fetch
# -----------------------------------------------------------------------------
#  Downloads every raw geospatial input the pipeline needs into data/raw/.
#  All sources are public and free; see DOCS/01_DATA_SOURCES.md for provenance
#  and licence terms, and data/sources.sha256 for the identity of each file.
#
#  Large raw datasets are deliberately NOT committed to git (see .gitignore).
#  This script is the reproducible substitute: run it, then `make world`.
#
#  TWO DOORS, AND WHY.  `raw.githubusercontent.com` is the plain way to read a
#  file out of a public repository and it needs no credentials at all; it is not
#  reachable from every build environment, so `gh api` (which is reachable from
#  more of them) is kept as the fallback.  The order matters: the credential-free
#  path is tried first, so a runner with no token — or an expired one — still gets
#  its data.  (Both were missing at once in the first CI runs of the client: the
#  workflow's fetch step had no GH_TOKEN and the script had no other door.)
#
#  EVERY DOWNLOAD IS VERIFIED.  data/sources.sha256 is committed, so a file that
#  does not match it fails the fetch instead of quietly building a different city.
#  That is also what `--verify-only` does, without touching the network.
#
#  Usage:  ./tools/fetch_data.sh [--verify-only]
# =============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/data/raw"
SUMS="$ROOT/data/sources.sha256"
mkdir -p "$DEST"

VERIFY_ONLY=0
[[ "${1:-}" == "--verify-only" ]] && VERIFY_ONLY=1

[[ -f "$SUMS" ]] || { echo "!! $SUMS is missing: that file is the contract"; exit 1; }

# sha256, byte count, name — the three fields of each non-comment line.
sums_rows() { awk '!/^#/ && NF >= 3 { print $1, $2, $3 }' "$SUMS"; }

verify_all() {
  local rc=0 sha bytes name got size
  echo "== verifying sha256 against data/sources.sha256 =="
  while read -r sha bytes name; do
    [[ -z "${name:-}" ]] && continue
    if [[ ! -f "$DEST/$name" ]]; then
      echo "  !! missing: $name"
      rc=1
      continue
    fi
    got="$(sha256sum "$DEST/$name" | cut -d' ' -f1)"
    size="$(stat -c%s "$DEST/$name")"
    if [[ "$got" != "$sha" ]]; then
      echo "  !! sha256 mismatch: $name"
      echo "       expected $sha"
      echo "       got      $got"
      rc=1
    elif [[ "$size" != "$bytes" ]]; then
      echo "  !! size mismatch: $name (expected $bytes bytes, got $size)"
      rc=1
    else
      echo "  $name: OK"
    fi
  done < <(sums_rows)
  return $rc
}

fetch_one() { # repo ref path out
  local repo="$1" ref="$2" path="$3" out="$4"
  local dest="$DEST/$out"
  if [[ -f "$dest" ]]; then
    echo "  [cached] $out"
    return 0
  fi
  echo "  [fetch]  $repo :: $path"

  local url="https://raw.githubusercontent.com/$repo/$ref/$path"
  if command -v curl >/dev/null 2>&1 &&
     curl -fsSL --retry 2 --max-time 180 "$url" -o "$dest.part" 2>/dev/null &&
     [[ -s "$dest.part" ]]; then
    mv "$dest.part" "$dest"
    echo "           via raw.githubusercontent.com"
    return 0
  fi
  rm -f "$dest.part"

  if command -v gh >/dev/null 2>&1; then
    if gh api -H "Accept: application/vnd.github.raw" \
         "/repos/$repo/contents/$path?ref=$ref" > "$dest.part" 2>/dev/null && [[ -s "$dest.part" ]]; then
      mv "$dest.part" "$dest"
      echo "           via gh api (GH_TOKEN/GITHUB_TOKEN)"
      return 0
    fi
    rm -f "$dest.part"
    echo "           gh api refused: is GH_TOKEN set for this step?"
  fi

  echo "  !! could not fetch $out from $repo"
  echo "     tried: $url"
  return 1
}

echo "== MUMBAI 1:1 data fetch =="
echo "destination: $DEST"

if [[ $VERIFY_ONLY -eq 0 ]]; then
  # 1. OpenStreetMap extract — Mumbai Metropolitan Region (roads, buildings,
  #    coastline, railways, landuse, waterways). 35 MB XML, ODbL 1.0.
  fetch_one valmikkpatel/wrangle-osm-data master sample.osm mumbai_region.osm

  # 2. BMC administrative ward polygons — DataMeet. (CC-BY / ODbL per upstream)
  fetch_one datameet/Municipal_Spatial_Data master "Mumbai/BMC_Wards.geojson" bmc_wards.geojson

  # 3. Slum cluster polygons — DataMeet (informal-settlement zoning layer).
  fetch_one datameet/Municipal_Spatial_Data master "Mumbai/slumClusters.geojson" slum_clusters.geojson

  # 4. Slum Rehabilitation Authority 2017 polygons — jkkersey/Mumbai-Mapping-Data (CC-BY).
  fetch_one jkkersey/Mumbai-Mapping-Data main "Mumbai_SRA_2017.geojson" mumbai_sra_2017.geojson

  # A convenience for anyone holding data/raw without the repository's list.  The
  # verification above is against data/sources.sha256, never against this file.
  ( cd "$DEST" && sha256sum mumbai_region.osm bmc_wards.geojson slum_clusters.geojson \
      mumbai_sra_2017.geojson > SHA256SUMS )
fi

verify_all || { echo "!! source data does not match data/sources.sha256"; exit 1; }

echo "== done. $(du -sh "$DEST" | cut -f1) of raw source data in data/raw/ =="
