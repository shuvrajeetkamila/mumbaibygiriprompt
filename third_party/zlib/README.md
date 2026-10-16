# zlib 1.3.1 — the inflate half, vendored

## What is here, and why it is here

The city travels to the phone as one **uncompressed tar** whose **members are
gzipped**.  Resume needs a stream whose offsets a client can come back to, and a
compressed tar has no such offsets; a transfer needs compression, and an
uncompressed tar has none.  Compressing each member and leaving the container
plain gives both — so the client must be able to inflate a member, and that
inflater is this code.

`engine/core/gzip.h` wraps it.  Nothing else in the engine includes `zlib.h`.

## What is *not* here

Only the inflate side: `adler32.c`, `crc32.c`, `inflate.c`, `inffast.c`,
`inftrees.c`, `zutil.c` and the headers they need.  `deflate.c`, `trees.c`, the
`gz*` file API and the test programs are not vendored: this project never
compresses, it only uncompresses what its own release step produced.

## Built with `Z_PREFIX`

`-DZ_PREFIX` renames every exported symbol to `z_*` (`z_inflate`,
`z_inflateInit2_`, ...).  Android's libc already ships a zlib, and a library that
exports `inflate` from `libmumbai.so` could be bound to by anything in the process
that asks for `inflate` — including the platform's own zlib.  Renaming makes that
impossible by construction.  **Both this source and `engine/core/gzip.cpp` must be
compiled with `-DZ_PREFIX`**; a build that sets it on only one side fails at link
time, which is the failure this comment wants.

## Provenance

Upstream `madler/zlib`, tag `v1.3.1`, fetched through
`https://codeload.github.com/madler/zlib/tar.gz/refs/tags/v1.3.1`
(sha256 of that tarball:
`17e88863f3600672ab49182f217281b6fc4d3c762bde361935e436a95214d05c`).
Licence: zlib's own, see `LICENSE`, which permits this use and requires that
altered source be marked as such.  The only alterations here are the removal of
files and the build flags described above; no source file was edited.
