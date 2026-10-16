#!/usr/bin/env python3
# =============================================================================
#  MUMBAI 1:1 EXACT — tools/make_starter_world.py
#
#  PACKS A SLICE OF THE CITY INTO THE APK.
#
#  The full world is 1.3 GB and 924 cells over 968 km², and it is a download: no
#  store package should carry it and no phone should be asked to install it.  But
#  a client that ships with *no* world is a client that shows a black screen on
#  first run — which is exactly what the first APK did, on a real device, and the
#  reason this tool exists.
#
#  So the APK carries a starter slice: South Bombay, the part of the city the
#  acceptance shot and the landmark list live in.  It is copied out of the APK into
#  the app's own storage on first run (the client does that, a few megabytes per
#  frame, reporting progress on its panel), and the rest of the city follows as a
#  download later.
#
#  WHAT GOES IN, AND WHY NOT EVERYTHING
#
#    * Every `.mumbai_chunk` cell in the region — the geometry, the collision, the
#      nav graph, the material tables and the hero assets are all *inside* the
#      chunk files, so a cell is self-describing and nothing else is needed.
#    * `world_manifest.json`, rewritten to describe only these cells.  A subset
#      manifest is a real manifest: the client's reader takes it unchanged, and a
#      cell that is absent is simply a cell that has not been downloaded.
#    * Optionally the albedo texture array.  The three material arrays are ~100 MB
#      together, so the default ships only albedo and lets the renderer generate
#      its normal and ORM arrays; a flat generated normal map is *no* normal map,
#      which is honest, and the albedo is what makes the city look like Mumbai
#      rather than like a grey model.  `--textures none|albedo|all` chooses.
#
#  The copied files are listed in `INDEX.txt` with a version token on its first
#  line.  That is what the client installs from, and the token is what tells it
#  that a new APK has a new slice to install.
#
#      python3 tools/make_starter_world.py \
#          --manifest data/processed/world_manifest.json \
#          --out android/app/src/main/assets/world \
#          --region south-bombay --textures albedo
# =============================================================================
import argparse
import hashlib
import json
import os
import shutil
import sys

# The regions are world-space rectangles in metres, and they are named for what a
# person would call them rather than for what the grid says.
REGIONS = {
    # Colaba, the Gateway, the Taj, CSMT, Marine Drive, Fort, Churchgate.
    # World anchors: Gateway (-6878.8, 5693.2), Taj (-7036.8, 5649.0),
    # CSMT (-6793.8, 7663.5), Marine Drive (-8162.9, 8018.2).
    "south-bombay": (-9216.0, -4096.0, 4096.0, 9216.0),
    # The Sea Link and the Bandra–Worli approaches, north-west of South Bombay.
    "sea-link": (-9216.0, -6144.0, 15360.0, 19456.0),
    # Everything: for a device with room and a cable, not for an APK.
    "all": (-1.0e9, 1.0e9, -1.0e9, 1.0e9),
}


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Pack a starter world into the APK's assets")
    parser.add_argument("--manifest", default="data/processed/world_manifest.json")
    parser.add_argument("--out", default="android/app/src/main/assets/world")
    parser.add_argument("--region", default="south-bombay", choices=sorted(REGIONS))
    parser.add_argument("--textures", default="albedo", choices=["none", "albedo", "all"],
                        help="which material arrays to ship (the app generates what is missing)")
    parser.add_argument("--max-mb", type=float, default=64.0,
                        help="budget for the cells; the farthest are dropped if it is exceeded")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args(argv)


def cell_rect(cell, macro):
    x0 = cell["x"] * macro
    z0 = cell["z"] * macro
    return x0, x0 + macro, z0, z0 + macro


def main(argv):
    args = parse_args(argv)
    macro = 1024.0

    with open(args.manifest, "r") as f:
        world = json.load(f)
    extent = world.get("extent", {})
    macro = float(extent.get("macroCellSize", 1024.0))
    data_dir = os.path.dirname(os.path.abspath(args.manifest))

    # A cell is in the region when its square overlaps the rectangle: a cell half
    # in and half out still has geometry a person will see from inside the region.
    rx0, rx1, rz0, rz1 = REGIONS[args.region]
    chosen = []
    for cell in world.get("cells", []):
        x0, x1, z0, z1 = cell_rect(cell, macro)
        if x1 <= rx0 or x0 >= rx1 or z1 <= rz0 or z0 >= rz1:
            continue
        chosen.append(cell)

    if not chosen:
        print(f"error: the region '{args.region}' contains no cells in {args.manifest}")
        return 2

    # A budget, not a wish: the dropped cells are the farthest from the region's
    # centre, so a smaller slice is the same city with its edges trimmed rather
    # than a random collection of tiles.
    cx = 0.5 * (rx0 + rx1) if abs(rx0) < 1e8 else 0.0
    cz = 0.5 * (rz0 + rz1) if abs(rz0) < 1e8 else 0.0
    chosen.sort(key=lambda c: ((c["x"] * macro + macro * 0.5 - cx) ** 2 +
                               (c["z"] * macro + macro * 0.5 - cz) ** 2, c["file"]))
    budget = args.max_mb * 1024 * 1024
    kept, dropped, total = [], [], 0
    for cell in chosen:
        size = int(cell.get("bytes", 0))
        if total + size > budget and kept:
            dropped.append(cell)
            continue
        kept.append(cell)
        total += size

    if dropped:
        print(f"note: {len(dropped)} cell(s) dropped to stay inside {args.max_mb:.0f} MiB "
              f"({len(kept)} kept, {total / 1048576.0:.1f} MiB)")

    # --- copy the cells ------------------------------------------------------
    out_cells = os.path.join(args.out, "cells")
    os.makedirs(out_cells, exist_ok=True)
    files = []
    copied_bytes = 0
    for cell in kept:
        src = os.path.join(data_dir, "cells", cell["file"])
        if not os.path.exists(src):
            print(f"error: {src} is in the manifest and not on disk; run `make world`")
            return 2
        shutil.copyfile(src, os.path.join(out_cells, cell["file"]))
        files.append("cells/" + cell["file"])
        copied_bytes += os.path.getsize(src)

    # --- the manifest, describing only what is here --------------------------
    subset = dict(world)
    subset["cells"] = sorted(kept, key=lambda c: c["file"])
    subset["subset"] = {
        "tool": "tools/make_starter_world.py",
        "region": args.region,
        "cells": len(kept),
        "cellsInWorld": len(world.get("cells", [])),
        "bytes": copied_bytes,
        "note": "the starter slice that ships inside the APK; the full world is a download",
    }
    with open(os.path.join(args.out, "world_manifest.json"), "w") as f:
        json.dump(subset, f, indent=1, sort_keys=True)
    files.append("world_manifest.json")

    # --- optionally the material arrays --------------------------------------
    textures_dir = os.path.join(data_dir, "textures")
    shipped_textures = []
    wanted = {"none": [], "albedo": ["materials_basecolor.mumbaitex"],
              "all": ["materials_basecolor.mumbaitex", "materials_normal.mumbaitex",
                      "materials_orm.mumbaitex"]}[args.textures]
    for name in wanted:
        src = os.path.join(textures_dir, name)
        if not os.path.exists(src):
            print(f"warning: {src} is missing; the renderer will generate that array")
            continue
        os.makedirs(os.path.join(args.out, "textures"), exist_ok=True)
        shutil.copyfile(src, os.path.join(args.out, "textures", name))
        files.append("textures/" + name)
        shipped_textures.append(name)

    # --- the index the client installs from ----------------------------------
    # The version token is a hash of what is being installed, so a build with a
    # different slice refreshes the phone's copy and a build with the same slice
    # does not touch it.
    digest = hashlib.sha256()
    for rel in sorted(files):
        path = os.path.join(args.out, rel)
        digest.update(rel.encode())
        digest.update(str(os.path.getsize(path)).encode())
        with open(path, "rb") as f:
            digest.update(hashlib.sha256(f.read()).digest())
    version = f"# mumbai-starter-world {digest.hexdigest()[:16]} region={args.region} files={len(files)}"

    index_path = os.path.join(args.out, "INDEX.txt")
    with open(index_path, "w") as f:
        f.write(version + "\n")
        for rel in sorted(files):
            f.write(rel + "\n")

    total_bytes = sum(os.path.getsize(os.path.join(args.out, rel)) for rel in files)
    if not args.quiet:
        print(f"starter world: {len(kept)} cells, {len(shipped_textures)} texture array(s)")
        print(f"  region        : {args.region}  {int(rx1 - rx0)} x {int(rz1 - rz0)} m")
        print(f"  cells         : {copied_bytes / 1048576.0:.1f} MiB of chunks")
        for name in shipped_textures:
            size = os.path.getsize(os.path.join(args.out, "textures", name))
            print(f"  {name:14s}: {size / 1048576.0:.1f} MiB")
        print(f"  total in APK  : {total_bytes / 1048576.0:.1f} MiB ({len(files)} files)")
        print(f"  version       : {version[2:]}")
        print(f"  out           : {args.out}")
    # The workflow reads these two lines; they are the report.
    print(f"STARTER_WORLD_MB={total_bytes / 1048576.0:.1f}")
    print(f"STARTER_WORLD_CELLS={len(kept)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
