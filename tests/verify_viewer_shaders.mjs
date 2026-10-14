// =============================================================================
//  MUMBAI 1:1 EXACT — tests/verify_viewer_shaders.mjs
//
//  The browser preview applies the procedural material atlas by patching
//  three.js's own MeshStandardMaterial shader with string substitution.  That is
//  the one part of the viewer that cannot be checked from C++ and cannot be run
//  here (no GPU, no browser), and its failure mode is a silently missing
//  texture: `String.replace` on a renamed chunk returns the shader unchanged.
//
//  So this script performs the substitutions the viewer performs, against the
//  vendored three.js, and holds the result:
//
//    * the chunks the injection anchors on still exist, and
//    * `color_fragment` still runs before `normal_fragment_begin`, and
//    * the injected GLSL declares every varying it uses, uses no view-space
//      normal, and keeps the shader's brace balance unchanged, and
//    * the attributes the geometry supplies (aTile / aScale / aMean) are the
//      ones the shader reads.
//
//      node tests/verify_viewer_shaders.mjs
//
//  Exit code 0 only when every check passes.
// =============================================================================
import * as THREE from '../viewer/vendor/three.module.js';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, '..', 'viewer', 'index.html'), 'utf8');

let checks = 0, failures = 0;
function check(name, ok, detail = '') {
  ++checks;
  if (!ok) { ++failures; console.log(`  FAIL ${name}${detail ? '  — ' + detail : ''}`); }
  else console.log(`  ok   ${name}`);
}

// --- the pieces of the viewer's patch, read out of the viewer itself ---------
// Read rather than duplicated: a copy of the injection would keep passing after
// the viewer had drifted away from it.
function extract(pattern, what) {
  const m = html.match(pattern);
  if (!m) { check(`viewer still contains ${what}`, false); return null; }
  return m[1];
}
const ATLAS = extract(/const ATLAS_GLSL = `([\s\S]*?)`;/, 'the atlas GLSL block');
const commonInjection = extract(/\.replace\('#include <common>', `([\s\S]*?)`\)/, 'the vertex <common> injection');
const beginInjection = extract(/\.replace\('#include <begin_vertex>', `([\s\S]*?)`\);/, 'the <begin_vertex> injection');
const colorInjection = extract(/\.replace\('#include <color_fragment>', `([\s\S]*?)`\);/, 'the <color_fragment> injection');
const fragmentCommon = extract(/\.replace\('#include <common>', '#include <common>\\n' \+ ATLAS_GLSL\)/, 'the fragment <common> injection');
if (!ATLAS || !commonInjection || !beginInjection || !colorInjection || fragmentCommon === null) {
  console.log(`\n ${checks} checks, ${failures} failing — FAIL`);
  process.exit(1);
}

const vs = THREE.ShaderLib.physical.vertexShader;
const fs = THREE.ShaderLib.physical.fragmentShader;

// --- anchors ----------------------------------------------------------------
check('vertex shader still has <common>', vs.includes('#include <common>'));
check('vertex shader still has <begin_vertex>', vs.includes('#include <begin_vertex>'));
check('fragment shader still has <common>', fs.includes('#include <common>'));
check('fragment shader still has <color_fragment>', fs.includes('#include <color_fragment>'));
check('fragment <color_fragment> still runs before <normal_fragment_begin>',
      fs.indexOf('<color_fragment>') < fs.indexOf('<normal_fragment_begin>'));

// --- resolve the template literals exactly as the shader compiler will -------
// The only interpolation in these snippets is a JS branch that has been removed
// in favour of GLSL `#ifdef USE_INSTANCING`; assert that, so a reintroduced
// `${...}` cannot silently produce invalid GLSL.
for (const [name, text] of [['<common>', commonInjection], ['<begin_vertex>', beginInjection],
                            ['<color_fragment>', colorInjection]]) {
  check(`${name} injection has no unresolved interpolation`, !text.includes('${'));
}
check('the projection plane comes from the object-space normal',
      beginInjection.includes('vec3 aN = normal;'));
check('the fragment path never touches the view-space normal',
      !colorInjection.includes('vNormal') && !ATLAS.includes('vNormal'));
check('the sampler is divided by the published tile mean',
      colorInjection.includes('max(vMean, vec3(0.03))'));
check('the atlas sample is inset by half a texel',
      ATLAS.includes('0.5 / (uPixels * uGrid)'));
check('instancing is handled in GLSL, not by a JS branch',
      beginInjection.includes('#ifdef USE_INSTANCING') &&
      beginInjection.includes('mat3(instanceMatrix)'));

// --- assemble the shaders the way three.js will ------------------------------
function finalize(src) {
  return src.replace(/#include <([\w_]+)>/g, (m, n) => {
    if (!(n in THREE.ShaderChunk)) { check(`chunk ${n} exists`, false); return m; }
    return THREE.ShaderChunk[n];
  });
}
const vPatched = vs.replace('#include <common>', '#include <common>\n' + commonInjection)
                   .replace('#include <begin_vertex>', beginInjection);
const fPatched = fs.replace('#include <common>', '#include <common>\n' + ATLAS)
                   .replace('#include <color_fragment>', colorInjection);
const vFinal = finalize(vPatched);
const fFinal = finalize(fPatched);
const balance = (s) => (s.match(/\{/g) || []).length - (s.match(/\}/g) || []).length;

// three.js ships `#if`-guarded shader text whose raw brace count is not zero, so
// the check is that the patch does not *change* the balance, not that it is 0.
const vBase = balance(finalize(vs)), fBase = balance(finalize(fs));
check('vertex patch keeps the brace balance', balance(vFinal) === vBase,
      `${balance(vFinal)} vs baseline ${vBase}`);
check('fragment patch keeps the brace balance', balance(fFinal) === fBase,
      `${balance(fFinal)} vs baseline ${fBase}`);

// --- declarations and plumbing ----------------------------------------------
for (const v of ['vTile', 'vScale', 'vMean', 'vAtlasUv']) {
  const decl = new RegExp(`varying (float|vec[23]) ${v};`);
  check(`${v} is declared on both sides`,
        decl.test(vFinal) && decl.test(fFinal));
}
check('the atlas helper is defined once in the fragment shader',
      (fFinal.match(/vec3 atlasSample\(/g) || []).length === 1);
for (const a of ['aTile', 'aScale', 'aMean']) {
  check(`attribute ${a} is declared and read`,
        vFinal.includes(`attribute`) && new RegExp(`attribute (float|vec3) ${a};`).test(vFinal));
}
for (const a of ['aTile', 'aScale', 'aMean']) {
  check(`viewer geometry supplies ${a}`, new RegExp(`'${a}'`).test(html));
}
check('the atlas uniforms are declared', ['uAtlas', 'uGrid', 'uPixels']
      .every((u) => ATLAS.includes(`uniform`) && new RegExp(`uniform \\w+ ${u};`).test(ATLAS)));
check('the patch is actually applied to all three world materials',
      (html.match(/patchAtlas\(/g) || []).length >= 4);   // 1 definition + 3 call sites

// --- the data the shader reads must exist in the payload ---------------------
const meta = JSON.parse(readFileSync(join(here, '..', 'viewer', 'data', 'mumbai.json'), 'utf8'));
check('the payload describes the atlas', !!meta.atlas && meta.atlas.count > 0);
const textured = (meta.materials || []).filter((m) => m.tex >= 0);
check('textured materials publish tex, tileMetres and tileMean',
      textured.length > 0 && textured.every((m) => m.tileMetres > 0 &&
        m.tileMeanR > 0 && m.tileMeanG > 0 && m.tileMeanB > 0));
check('untextured materials are marked -1',
      (meta.materials || []).filter((m) => m.tex === undefined).length === 0 &&
      (meta.materials || []).some((m) => m.tex === -1));

console.log('-----------------------------------------------------------------------');
console.log(` ${checks} checks, ${failures} failing — ${failures === 0 ? 'PASS' : 'FAIL'}`);
console.log('=======================================================================');
process.exit(failures === 0 ? 0 : 1);
