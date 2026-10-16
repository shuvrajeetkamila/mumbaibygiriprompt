// =============================================================================
//  MUMBAI 1:1 EXACT — tests/verify_viewer_orientation.mjs
//
//  The viewer must show Mumbai the right way round.  This asks three.js itself,
//  rather than trusting the comment in the middle of index.html, because the
//  failure mode is invisible: a mirrored city has every landmark in its correct
//  position relative to every other landmark, and the only way to notice is to
//  know which side of Marine Drive the sea is on.
//
//  The chain being verified:
//    * the world's axes are x = east, y = up, z = north (left-handed);
//    * three.js is right-handed, so its camera's right vector points WEST when
//      facing north;
//    * index.html therefore mirrors the projection's x scale, which puts the
//      Arabian Sea back on the left where it belongs.
//
//  Run with: node tests/verify_viewer_orientation.mjs
// =============================================================================
import * as THREE from '../viewer/vendor/three.module.js';

let failures = 0;
let checks = 0;
const check = (cond, what) => {
  ++checks;
  if (!cond) { console.log(`  FAIL ${what}`); ++failures; } else { console.log(`  ok   ${what}`); }
};

// A camera at Marine Drive, 1.7 m up, facing due north.
const cam = new THREE.PerspectiveCamera(60, 16 / 9, 0.25, 6000);
cam.position.set(0, 1.7, 0);
cam.lookAt(0, 1.7, 100);
cam.updateMatrixWorld(true);

const project = (x, y, z) => new THREE.Vector3(x, y, z)
  .applyMatrix4(cam.matrixWorldInverse)
  .applyMatrix4(cam.projectionMatrix);

// --- 1. three.js's own camera basis, before any fix --------------------------
const e = cam.matrixWorldInverse.elements;
const right = new THREE.Vector3(e[0], e[4], e[8]);
check(right.x < -0.99, `three.js camera right vector points west (${right.x.toFixed(3)}) — `
  + 'so an uncorrected three.js render of this world is mirrored');

const seaBefore = project(-200, 1.7, 300);
check(seaBefore.x > 0.5, 'uncorrected: the sea (200 m west) lands on the RIGHT of the frame');

// --- 2. the mirror, exactly as index.html applies it -------------------------
const mirror = (camera) => {
  camera.projectionMatrix.elements[0] *= -1;
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
};
mirror(cam);

const sea = project(-200, 1.7, 300);
const city = project(200, 1.7, 300);
const ahead = project(0, 1.7, 1000);
check(sea.x < -0.5, `mirrored: the sea (west) is on the LEFT (ndc.x ${sea.x.toFixed(3)})`);
check(city.x > 0.5, `mirrored: the city (east) is on the RIGHT (ndc.x ${city.x.toFixed(3)})`);
check(Math.abs(ahead.x) < 1e-6 && Math.abs(ahead.y) < 1e-6,
  'the heading is still dead ahead after mirroring (a mirror must not move the centre)');
// A mirror flips x, so the depth mapping and the vertical axis are untouched.
//
// The sign of ndc.y here is the opposite of the engine's for the same geometry, and
// both are correct: WebGL's clip space has +Y up the framebuffer, Vulkan's has it
// down, so engine/render/camera.cpp's projection flips Y (m[5] = -f) and three.js
// does not.  A reader comparing the two renderers will find this difference; it is
// not a bug in either, and "fixing" one to match the other puts the sky underground.
const high = project(0, 400, 300);
check(high.y > 0.1, `the sky stays up in WebGL's convention (ndc.y ${high.y.toFixed(3)}, positive = up)`);
const near = project(0, 1.7, 0.25);
check(Math.abs(near.z - 0) < 1e-3 || Math.abs(near.z - (-1)) < 1e-3 || near.z < 0,
  'the near plane is untouched by the x mirror');

// --- 3. the mirror is a pure reflection, not a rotation ---------------------
// A rotation would change the winding of the whole world and could be undone by
// culling; a reflection cannot.  index.html therefore switches every material to
// DoubleSide, and this asserts the reason is real.
const a = project(0, 1.7, 100), b = project(100, 1.7, 0), c = project(0, 1.7, -100);
const cross2 = (p, q, r) => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
const mirroredWinding = cross2(a, b, c);
cam.projectionMatrix.elements[0] *= -1;              // undo
const plainWinding = cross2(project(0, 1.7, 100), project(100, 1.7, 0), project(0, 1.7, -100));
mirror(cam);                                          // redo
check(Math.sign(mirroredWinding) !== Math.sign(plainWinding),
  'the mirror reverses screen winding, so back-face culling would hide half the city '
  + '(every material is DoubleSide for this reason)');

// --- 4. the viewer still applies the mirror everywhere it must ---------------
// `updateProjectionMatrix()` rebuilds the projection unmirrored, so every call
// site has to be followed by the mirror.  The resize path lost it once (D-046),
// and that is the version of this bug a person actually sees: the city flips the
// first time the phone is turned.
{
  const fs = await import('node:fs');
  const source = fs.readFileSync(new URL('../viewer/index.html', import.meta.url), 'utf8');
  const mirrors = source.match(/applyMirror\(\)/g) || [];
  check(mirrors.length >= 2,
    `index.html applies the mirror at startup and on resize (${mirrors.length} call sites)`);

  const lines = source.split('\n');
  let unguarded = 0;
  lines.forEach((line, i) => {
    if (!line.includes('updateProjectionMatrix()')) return;
    if (!lines.slice(i, i + 3).join('\n').includes('applyMirror()')) ++unguarded;
  });
  check(unguarded === 0, `${unguarded} updateProjectionMatrix() call(s) are not followed by applyMirror()`);

  check(/uncullForMirror/.test(source) && /THREE\.DoubleSide/.test(source),
    'index.html unculls the mirror (DoubleSide on every material), which section 3 proves is required');
}

console.log('-----------------------------------------------------------------------');
if (failures) {
  console.log(` ${checks} checks, ${failures} failures — FAIL`);
  process.exit(1);
}
console.log(` ${checks} checks, 0 failures — PASS`);
