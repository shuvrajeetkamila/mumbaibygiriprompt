#!/usr/bin/env python3
"""Pack the whole city for the phone.

WHAT THIS MAKES, AND WHY IT IS SHAPED THIS WAY.

`make world` writes `data/processed/`: 924 cells, two texture arrays and the world
manifest, 1.4 GB of mmap-able tiles.  That is what the pipeline produces and what
the desktop harness and the viewer read.  It is not what the phone downloads:

  * 1.4 GB is not a download anybody finishes on mobile data, so the payload has
    to be compressed — the pipeline's own output compresses to about an eighth
    (D-049 measured it);

  * a *compressed tar* cannot be resumed.  The client streams the archive and
    remembers the byte after a completed file, because that is the only offset at
    which its parser can start again; inside a deflate stream that offset does not
    exist, and a resume would mean inflating from zero every time;

so the members are compressed and the container is not.  Every file boundary in
the archive stays a resume point, the payload travels small, and what lands on the
phone is the tree the manifest describes.

`world_manifest.json` is deliberately NOT compressed: its sha256 is the identity of
the world, it is what the checksum list and the client's build both name, and 358 KB
of plain JSON is not worth a second way to hash it.

THE CHECKSUM LIST IS THE CONTRACT.  `sha256sums-<manifest sha256:16>.txt` lists
every file in the archive — by its *archive* name, so `cells/x.mumbai_chunk.gz` —
and it is the list the client verifies while it extracts.  The engine's downloader
refuses any member that is not in it, and the sink verifies each member's sha256
before it is allowed to exist under its final name (D-048).

DETERMINISTIC BY CONSTRUCTION.  gzip's `mtime`, tar's `mtime`/uid/gid/mode and the
member order are all fixed here rather than inherited from the filesystem, so
packing the same world twice gives the same bytes.  That is not required — the
world's identity is its manifest's hash, not the container's — but it means a
re-run can be diffed against the previous asset instead of merely re-uploaded.

USAGE
    python3 tools/pack_world.py --processed data/processed --out release --verify

    --verify re-reads the archive it just wrote, inflates every member, and checks
    the payload against the file the pipeline produced.  It costs a few seconds and
    it is the difference between "the tar was written" and "the tar can be read".
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
import sys
import tarfile
import time

# The compression level the release step uses.  Measured against the pipeline's own
# output: level 6 gets most of the ratio at a fraction of level 9's time, and this
# runs on every publish (D-049 has the numbers).
GZIP_LEVEL = 6

# Which parts of `data/processed` travel, and how.  `compress` is the whole of the
# per-member decision: chunks and texture arrays are gzipped, the manifest is not.
TREES = (
    ("cells", True),
    ("textures", True),
)


def sha256_of_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(1 << 20)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def file_bytes(path: str) -> bytes:
    with open(path, "rb") as handle:
        return handle.read()


def gzip_bytes(payload: bytes) -> bytes:
    """gzip, with no timestamp and no name, so the same input gives the same bytes."""
    buffer = io.BytesIO()
    with gzip.GzipFile(fileobj=buffer, mode="wb", compresslevel=GZIP_LEVEL, mtime=0) as pump:
        pump.write(payload)
    return buffer.getvalue()


def collect(processed: str) -> list[tuple[str, str, bool]]:
    """Every file that travels, as (name in the archive, path on disk, compressed)."""
    members: list[tuple[str, str, bool]] = []
    for tree, compress in TREES:
        root = os.path.join(processed, tree)
        if not os.path.isdir(root):
            continue
        for base, _dirs, files in os.walk(root):
            for name in sorted(files):
                path = os.path.join(base, name)
                relative = os.path.relpath(path, processed).replace(os.sep, "/")
                members.append((relative, path, compress))
    members.append(("world_manifest.json", os.path.join(processed, "world_manifest.json"), False))
    members.sort(key=lambda member: member[0])
    return members


def pack(processed: str, out_dir: str, verify: bool) -> int:
    manifest_path = os.path.join(processed, "world_manifest.json")
    if not os.path.isfile(manifest_path):
        print(f"::error::no world manifest at {manifest_path}; run `make world` first")
        return 1
    manifest_sha = sha256_of_file(manifest_path)
    sha16 = manifest_sha[:16]
    members = collect(processed)
    if len(members) < 2:
        print(f"::error::only {len(members)} files under {processed}")
        return 1

    os.makedirs(out_dir, exist_ok=True)
    archive_name = f"mumbai-world-{sha16}.tar"
    archive_path = os.path.join(out_dir, archive_name)
    sums_name = f"sha256sums-{sha16}.txt"
    sums_path = os.path.join(out_dir, sums_name)

    staged: dict[str, bytes] = {}
    listed: list[tuple[str, str]] = []          # (archive name, sha256 of the stored bytes)
    uncompressed_total = 0
    stored_total = 0
    started = time.time()
    for name, path, compress in members:
        payload = file_bytes(path)
        uncompressed_total += len(payload)
        stored = gzip_bytes(payload) if compress else payload
        stored_total += len(stored)
        member_name = name + ".gz" if compress else name
        staged[member_name] = stored
        listed.append((member_name, hashlib.sha256(stored).hexdigest()))
        if stored_total % (64 << 20) < len(stored):
            print(f"  ... {stored_total / 1048576.0:8.1f} MiB packed of "
                  f"{uncompressed_total / 1048576.0:8.1f} MiB read", flush=True)

    # The tar: ustar, fixed metadata, members in name order.  `tar -cf` would write
    # the same layout, but it would also stamp mtimes and uids, and this has to be
    # byte-reproducible to be worth publishing twice.
    with tarfile.open(archive_path, "w", format=tarfile.USTAR_FORMAT) as archive:
        for member_name, _sha in listed:
            info = tarfile.TarInfo(member_name)
            info.size = len(staged[member_name])
            info.mtime = 0
            info.mode = 0o644
            info.uid = 0
            info.gid = 0
            info.uname = ""
            info.gname = ""
            archive.addfile(info, io.BytesIO(staged[member_name]))

    # The checksum list, in sha256sum's own format, sorted the way `sha256sum` sorts:
    # the client parses it with the same code path the tests cover (parseChecksumList).
    with open(sums_path, "w", encoding="utf-8") as handle:
        for member_name, sha in sorted(listed):
            handle.write(f"{sha}  {member_name}\n")

    archive_bytes = os.path.getsize(archive_path)
    print(f"::notice::world manifest sha256 (16): {sha16}")
    print(f"::notice::packed {len(listed)} files: {uncompressed_total} bytes read "
          f"-> {stored_total} bytes stored -> {archive_bytes} byte tar "
          f"({100.0 * archive_bytes / max(uncompressed_total, 1):.1f} %), "
          f"{time.time() - started:.1f} s")
    print(f"::notice::the asset to publish: {archive_name} ({archive_bytes} bytes), "
          f"{sums_name} ({os.path.getsize(sums_path)} bytes)")

    if verify:
        code = verify_archive(archive_path, processed, listed)
        if code != 0:
            return code
        print("::notice::verified: every member inflates back to the pipeline's own file")
    return 0


def verify_archive(archive_path: str, processed: str, listed: list[tuple[str, str]]) -> int:
    """Re-reads the archive and checks it against the tree it came from."""
    originals = {}
    for name, path, _compress in collect(processed):
        originals[name] = path
    seen = 0
    with tarfile.open(archive_path, "r") as archive:
        names = archive.getnames()
        expected_names = [name for name, _sha in listed]
        if names != expected_names:
            print(f"::error::the archive's members are not the ones packed "
                  f"({len(names)} vs {len(expected_names)})")
            return 1
        for member in archive:
            if not member.isfile():
                print(f"::error::the archive carries a non-file member: {member.name}")
                return 1
            stored = archive.extractfile(member).read()
            sha = hashlib.sha256(stored).hexdigest()
            if sha != dict(listed)[member.name]:
                print(f"::error::{member.name} does not hash to what was packed")
                return 1
            source = member.name[:-3] if member.name.endswith(".gz") else member.name
            payload = gzip.decompress(stored) if member.name.endswith(".gz") else stored
            if len(payload) != os.path.getsize(originals[source]):
                print(f"::error::{member.name} inflates to {len(payload)} bytes, the "
                      f"pipeline wrote {os.path.getsize(originals[source])}")
                return 1
            if hashlib.sha256(payload).hexdigest() != sha256_of_file(originals[source]):
                print(f"::error::{member.name} inflates to different bytes than "
                      f"{source}")
                return 1
            seen += 1
    if seen != len(listed):
        print(f"::error::the archive was read back as {seen} members, not {len(listed)}")
        return 1
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Pack the published world archive.")
    parser.add_argument("--processed", default="data/processed")
    parser.add_argument("--out", default="release")
    parser.add_argument("--verify", action="store_true",
                        help="re-read the archive and check it against the pipeline's files")
    args = parser.parse_args(argv)
    return pack(args.processed, args.out, args.verify)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
