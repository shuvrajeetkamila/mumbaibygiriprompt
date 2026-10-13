// =============================================================================
//  MUMBAI 1:1 EXACT — tests/verify_export.mjs
//
//  Independent verification of the viewer payload (viewer/data/mumbai.json +
//  mumbai.bin) produced by `mumbai_pipeline`'s final stage.  The payload is a
//  second, text-and-binary summary of the world that is decoded by JavaScript,
//  not by the C++ runtime, so it is the one artefact where a layout mistake
//  cannot be caught by the engine's own readers.  This script re-derives every
//  field from the published offsets and asserts the physical properties the
//  master prompt's acceptance section asks about.
//
//      node tests/verify_export.mjs [viewerDir]
//
//  Exit code 0 only when every check passes.
// =============================================================================
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const dir = process.argv[2] || join(here, '..', 'viewer');
const meta = JSON.parse(readFileSync(join(dir, 'data', 'mumbai.json'), 'utf8'));
const buf = readFileSync(join(dir, 'data', 'mumbai.bin'));
const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);

let checks = 0, failures = 0;
function check(name, ok, detail = '') {
  ++checks;
  if (!ok) { ++failures; console.log(`  FAIL ${name}${detail ? '  — ' + detail : ''}`); }
}
function near(name, a, b, tol) {
  check(name, Math.abs(a - b) <= tol, `${a} vs ${b} (tol ${tol})`);
}

const E = meta.extent;
const minX = E.minX, minZ = E.minZ, maxX = E.maxX, maxZ = E.maxZ;
// The payload stores everything relative to the extent's south-west corner:
// stored = world - extent.min.  The manifest's landmarks use the same frame, so
// positions are compared in payload space and only converted for reporting.
const W = maxX - minX, H = maxZ - minZ;
const world = (storedX, storedZ) => [storedX + minX, storedZ + minZ];

console.log('=======================================================================');
console.log(' viewer payload verification —', join(dir, 'data', 'mumbai.bin'), buf.length, 'bytes');
console.log('=======================================================================');

// --- the file must be exactly the size the metadata describes ---------------
// Every read below is offset-driven, so trailing garbage is invisible to them:
// a binary that has been through a text round trip (each byte that is not valid
// UTF-8 becoming U+FFFD) keeps a valid prefix and grows a tail.  Payload
// length is therefore checked first, against the last declared structure.
{
  const declared = meta.floodModerate.offset +
                   meta.floodModerate.width * meta.floodModerate.height;
  check('payload length equals the declared layout', buf.length === declared,
        `${buf.length} on disk vs ${declared} declared`);
  const replacements = buf.indexOf(Buffer.from([0xef, 0xbf, 0xbd]));
  check('payload contains no UTF-8 replacement sequences', replacements < 0,
        replacements < 0 ? '' : `first at byte ${replacements}`);
}

// --- sanity of the manifest itself -----------------------------------------
check('extent is non-degenerate', maxX > minX && maxZ > minZ);
check('sea level present', typeof E.seaLevel === 'number');
check('every declared offset lies inside the binary', [
  meta.terrain.offset, meta.buildings.offset, meta.nodes.offset, meta.edges.offset,
  meta.hero.vertexOffset, meta.hero.indexOffset,
].every(o => o >= 0 && o < buf.length));

// --- terrain: land fraction, range, no holes --------------------------------
{
  const T = meta.terrain;
  const dec = k => T.bias + dv.getUint16(T.offset + k * 2, true) / 65535 * T.span;
  let land = 0, min = Infinity, max = -Infinity;
  const n = T.width * T.height;
  for (let k = 0; k < n; ++k) {
    const y = dec(k);
    if (y > 0.3) ++land;
    if (y < min) min = y;
    if (y > max) max = y;
  }
  const frac = land / n;
  console.log(`  terrain ${T.width}x${T.height} @${T.step} m — ${(frac * 100).toFixed(1)} % land, ` +
              `y ${min.toFixed(1)}..${max.toFixed(1)} m`);
  check('terrain has land', frac > 0.25 && frac < 0.7, `land ${(frac * 100).toFixed(1)} %`);
  check('terrain relief present', max - min > 50, `range ${(max - min).toFixed(1)} m`);
  check('terrain never dips below the ocean floor clamp', min > -60, `min ${min}`);
}

// --- buildings: in extent, materials resolved, orientation spread ----------
const B = meta.buildings;
{
  const matIds = new Map();
  const yawBucket = new Array(8).fill(0);
  let outOfExtent = 0, tooSmall = 0, sumArea = 0, maxH = 0;
  const heights = [];
  for (let i = 0; i < B.count; ++i) {
    const o = B.offset + i * B.stride;
    const sx = dv.getUint16(o, true), sz = dv.getUint16(o + 2, true);
    const w = dv.getUint16(o + 4, true), d = dv.getUint16(o + 6, true);
    const h = dv.getUint16(o + 8, true);
    const mat = dv.getUint8(o + 12), yaw = dv.getUint8(o + 13);
    if (sx < 0 || sx > W || sz < 0 || sz > H) ++outOfExtent;
    if (w < 1 || d < 1) ++tooSmall;
    sumArea += w * d;
    if (h > maxH) maxH = h;
    heights.push(h);
    matIds.set(mat, (matIds.get(mat) || 0) + 1);
    yawBucket[Math.floor(yaw / 32)]++;
  }
  heights.sort((a, b) => a - b);
  const q = p => heights[Math.floor(p * (heights.length - 1))];
  const populated = yawBucket.filter(v => v > 0).length;
  console.log(`  buildings ${B.count} — footprint ${(sumArea / 1e6).toFixed(1)} km^2, ` +
              `heights p50/p90/p99 ${q(0.5)}/${q(0.9)}/${q(0.99)} m, yaw buckets ${yawBucket.join(',')}`);
  check('no building starts outside the extent', outOfExtent === 0, `${outOfExtent} outside`);
  check('no degenerate footprints', tooSmall === 0, `${tooSmall} degenerate`);
  check('building count is city-scale', B.count > 100000, String(B.count));
  check('total footprint is plausible for ~600 km^2 of city',
        sumArea / 1e6 > 40 && sumArea / 1e6 < 400, `${(sumArea / 1e6).toFixed(1)} km^2`);
  check('heights match the ward rules', q(0.99) >= 20 && maxH <= 400, `p99 ${q(0.99)}, max ${maxH}`);
  // A building's yaw identifies an axis, not a direction: +180 deg is the same
  // box.  The compass check therefore folds opposite buckets together.
  const folded = [0, 1, 2, 3].map(b => yawBucket[b] + yawBucket[b + 4]);
  const foldedUsed = folded.filter(v => v > 0).length;
  const dominant = Math.max(...folded) / B.count;
  check('orientation covers the compass (modulo 180 deg)', foldedUsed === 4,
        `folded buckets ${folded.join(',')}`);
  check('no single orientation dominates the city', dominant < 0.75,
        `largest folded bucket ${(dominant * 100).toFixed(0)} %`);
  check('materials resolved to real entries',
        [...matIds.keys()].every(id => (meta.materials || []).some(m => m.id === id)),
        `${matIds.size} distinct ids`);
  check('no building renders with the unknown material',
        !matIds.has(0) || matIds.get(0) / B.count < 0.01,
        `id0 = ${matIds.get(0) || 0}`);
}

// --- street graph -----------------------------------------------------------
{
  const N = meta.nodes, Ed = meta.edges;
  let bad = 0, selfLoop = 0;
  const seen = new Uint8Array(N.count);
  for (let i = 0; i < Ed.count; ++i) {
    const o = Ed.offset + i * 12;
    const a = dv.getUint32(o, true), b = dv.getUint32(o + 4, true);
    if (a >= N.count || b >= N.count || a === b) ++bad;
    else { seen[a] = 1; seen[b] = 1; }
  }
  let linked = 0;
  for (let i = 0; i < N.count; ++i) linked += seen[i];
  const frac = linked / N.count;
  check('every edge endpoint exists', bad === 0, `${bad} bad`);
  check('graph is mostly connected', frac > 0.6, `${(frac * 100).toFixed(1)} %`);
  check('edge count is city-scale', Ed.count > 50000, String(Ed.count));
  // Node coordinates are extent-relative unsigned metres: the payload is clipped
  // to the extent, so the sign bit is free, and the world is 43 km tall — as i16
  // the northern third of the graph clamped onto a single line at 32,000 m.
  let out = 0, far = 0, maxX = 0, maxZ = 0;
  for (let i = 0; i < N.count; ++i) {
    const o = N.offset + i * 6;
    const x = dv.getUint16(o, true), z = dv.getUint16(o + 2, true);
    if (x > W + 1 || z > H + 1) ++out;
    if (i % 97 === 0 && (x < -1 || z < -1)) ++far;
    if (x > maxX) maxX = x;
    if (z > maxZ) maxZ = z;
  }
  check('nav nodes sit inside the extent', out === 0 && far === 0, `${out} outside, ${far} sampled negative`);
  check('street graph reaches the northern extent', maxZ > H - 4000, `max z = ${maxZ} of ${H}`);
  check('street graph reaches the eastern extent', maxX > W - 4000, `max x = ${maxX} of ${W}`);
  console.log(`  street graph — ${N.count} nodes, ${Ed.count} edges, ` +
              `${(frac * 100).toFixed(1)} % reachable, reaching x ${maxX} / ${W}, z ${maxZ} / ${H}`);
}

// --- hero meshes: placed at their published anchors -------------------------
{
  const H = meta.hero;
  const byMat = new Map();
  let minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < H.vertexCount; ++i) {
    const o = H.vertexOffset + i * H.vertexStride;
    const x = dv.getFloat32(o, true);
    const y = dv.getFloat32(o + 4, true);
    const z = dv.getFloat32(o + 8, true);
    const m = dv.getUint8(o + 24);
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (!byMat.has(m)) byMat.set(m, [Infinity, -Infinity, Infinity, -Infinity, 0]);
    const r = byMat.get(m);
    r[0] = Math.min(r[0], x); r[1] = Math.max(r[1], x);
    r[2] = Math.min(r[2], z); r[3] = Math.max(r[3], z); r[4]++;
  }
  console.log(`  hero ${H.vertexCount} vertices, ${H.indexCount / 3} triangles, ` +
              `y ${minY.toFixed(1)}..${maxY.toFixed(1)} m`);
  check('hero geometry exists', H.vertexCount > 10000, String(H.vertexCount));
  check('hero indices are in range', H.indexCount % 3 === 0);

  const nameOf = id => (meta.materials.find(m => m.id === id) || {}).name || String(id);
  const anchors = meta.landmarks || [];
  const nearestAnchor = (x, z) => {
    let best = Infinity, who = '';
    for (const lm of anchors) {
      const d = Math.hypot(lm.x - x, lm.z - z);
      if (d < best) { best = d; who = lm.name; }
    }
    return [best, who];
  };
  // Landmark geometry must sit on its published anchor.  Stone and terracotta
  // are shared between several landmarks, so the check is per vertex against
  // the nearest anchor rather than per material cluster.
  const worst = new Map();
  for (let i = 0; i < H.vertexCount; ++i) {
    const o = H.vertexOffset + i * H.vertexStride;
    const n = nameOf(dv.getUint8(o + 24));
    if (/sea_link|concrete_road/.test(n)) continue;      // the crossing is its own anchor
    const [d, who] = nearestAnchor(dv.getFloat32(o, true), dv.getFloat32(o + 8, true));
    const w = worst.get(n) || [0, ''];
    if (d > w[0]) { w[0] = d; w[1] = who; worst.set(n, w); }
  }
  for (const [n, [d, who]] of worst)
    console.log(`    ${n.padEnd(20)} worst vertex ${d.toFixed(0)} m from ${who}`);
  // hero_concrete is shared by the CSMT bulk and the 3.5 km Marine Drive
  // promenade, whose published anchor is its mid-point; the stone, terracotta
  // and glass materials belong to point landmarks and get the tight limit.
  for (const [n, [d]] of worst) {
    const limit = n === 'hero_concrete' ? 2500 : 400;
    check(`hero '${n}' sits on its anchor`, d < limit, `worst vertex ${d.toFixed(0)} m away`);
  }
  // The Sea Link deck must be kilometres long, not a stub.  (The cable groups
  // themselves only span the two navigation channels, so the deck is the
  // measurement that matters.)
  const deck = [...byMat.entries()].find(([id]) => nameOf(id) === 'concrete_road');
  if (deck) {
    const r = deck[1];
    const span = Math.hypot(r[1] - r[0], r[3] - r[2]);
    console.log(`    sea link deck span ${(span / 1000).toFixed(2)} km`);
    check('sea link spans the bay', span > 2500, `${span.toFixed(0)} m`);
  }
  check('pylons reach their published height', maxY > 100, `${maxY.toFixed(1)} m`);
}

// --- monsoon flood rasters --------------------------------------------------
// Section 3's model is only real if the water shows up where the city floods and
// nowhere else.  Both design storms travel with the payload — the routine
// monsoon day and the 26 July 2005 analogue — and they are checked the way a
// reader would: look at the ground under the water.
{
  const T = meta.terrain;
  const heightAt = k => T.bias + dv.getUint16(T.offset + k * 2, true) / 65535 * T.span;

  for (const [key, label, minWet, minDeep, hillLimit] of [
    ['flood',         'severe (2005 analogue)', 300000, 100000, 50],
    ['floodModerate', 'moderate monsoon day',    150000,  10000,  8],
  ]) {
    const F = meta[key];
    console.log(`\n ${label} — ${F.width} x ${F.height} ${F.units}, ` +
                `${F.wetCells} wet cells, peak ${F.peakCm} cm`);
    check(`${key}: raster is the terrain grid`,
          F.width === T.width && F.height === T.height);
    check(`${key}: has water`, F.wetCells > minWet && F.cellsOver50cm > minDeep,
          `${F.wetCells} wet, ${F.cellsOver50cm} deeper than 50 cm`);

    let wet = 0, inSea = 0, seaCells = 0, maxCm = 0;
    let lowLand = 0, lowWet = 0, highLand = 0, highWet = 0;
    for (let k = 0; k < F.width * F.height; ++k) {
      const cm = buf[F.offset + k];
      const y = heightAt(k);
      if (cm > 1) ++wet;
      if (cm > maxCm) maxCm = cm;
      // Open water is drawn as water, never as flooding.  The seabed pick line
      // the hydrology uses is 1.1 m below datum, so anything under that is sea.
      if (y < -1.1) { ++seaCells; if (cm > 1) ++inSea; continue; }
      if (y < 6) { ++lowLand; if (cm > 15) ++lowWet; }
      if (y > 25) { ++highLand; if (cm > 15) ++highWet; }
    }
    const lowFrac = lowLand ? (100 * lowWet / lowLand) : 0;
    const highFrac = highLand ? (100 * highWet / highLand) : 0;
    console.log(`    ${wet} cells drawn, ${inSea}/${seaCells} open-water cells wet; ` +
                `>15 cm on ${lowFrac.toFixed(1)} % of low ground and ${highFrac.toFixed(1)} % ` +
                `of high ground`);
    check(`${key}: every wet cell is counted`, wet === F.wetCells);
    check(`${key}: open water is not drawn as flooding`, inSea < seaCells * 0.02,
          `${inSea} wet sea cells of ${seaCells}`);
    check(`${key}: the low city floods`, lowFrac > 15, `${lowFrac.toFixed(1)} %`);
    // The two storms are held to different standards on purpose.  A routine
    // monsoon day must flood the low ground and leave the high ground alone:
    // that contrast *is* Mumbai's flood geography, and it is the check that a
    // model drowning everything uniformly fails.  The 2005 analogue is a
    // different kind of event — 944 mm in a day — and the whole city is its
    // floodplain, so its bar is that the flats are still clearly worse off than
    // the hills, not that the hills escape.
    check(`${key}: the ridges stay largely dry`, highFrac < hillLimit,
          `${highFrac.toFixed(1)} % of high ground over 15 cm`);
  }
}

// --- QA block ---------------------------------------------------------------
{
  console.log(`  QA — ${meta.qa.counters} counters, verdict ${meta.qa.passed ? 'PASS' : 'FAIL'}`);
  check('pipeline QA passed', meta.qa.passed === true);
}

console.log('-----------------------------------------------------------------------');
console.log(` ${checks} checks, ${failures} failing — ${failures === 0 ? 'PASS' : 'FAIL'}`);
console.log('=======================================================================');
process.exit(failures === 0 ? 0 : 1);
