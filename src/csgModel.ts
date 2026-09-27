/**
 * Kernel-side `.csg` geometry builder — walks `csgImport.ts`'s AST with LIVE
 * OCCT handles and returns one `TopoDS_Shape` (a compound, possibly empty).
 * Called from `loadBRep`/`exportBRep`'s `"csg"` branches; the `.csg` document's
 * base shape is OPAQUE (like a STEP import — user edits layer on top via the
 * sidecar, the parsed structure is not itself an edit history). See
 * `csgImport.ts` for why lowering to `EditOp`s was rejected (index
 * simulation fragility, shear, `addPolyhedron`-needs-an-icon).
 *
 * Every OCCT call shape below was probed live before writing (see the
 * session's throwaway `csg-probe*.cjs` scripts, not committed):
 * - empty compound + `applyEditsBRep`-free direct build: `TopoDS_Compound` +
 *   `BRep_Builder.MakeCompound` + per-node solids, union volume exact.
 * - primitives: `BRepPrimAPI_MakeBox_3(pnt, pnt)` (same overload booleans
 *   use), `MakeSphere_5`, `MakeCylinder_3(ax2, r, h)`, `MakeCone_3`,
 *   `MakeTorus_5` — the exact suffixes `occtOperations.ts` uses.
 * - booleans: `BRepAlgoAPI_{Fuse,Cut,Common}_3(a, b).Shape()` + `IsDone()`
 *   gate; operands compounded first (the `booleanSolids` skeleton).
 * - transforms: EVERYTHING (rigid, scale, mirror, shear) through ONE path —
 *   `gp_GTrsf_1` + `SetValue(r, c, v)` (1-based 3×4) +
 *   `BRepBuilderAPI_GTransform_2(shape, g, true)` — verified for
 *   near-identity (float noise `2.22045e-16`), scale-×2 (volume doubles
 *   exactly), and mirror (volume stays positive). No rigid/non-rigid split,
 *   no Euler decomposition into op-shaped pieces.
 * - polyhedron: per-face `MakeWire_1` + `MakeEdge_3` + `MakeFace_15(wire,
 *   true)`, then `BRepBuilderAPI_Sewing(1e-6, true, true, true, false)` +
 *   `Perform(Handle_Message_ProgressIndicator_1)` + `NbFreeEdges() == 0`
 *   gate (the promotion pipeline's own closure check) + `MakeSolid_3` +
 *   orientation fix (below). Verified: square pyramid sews to 0 free edges.
 * - N-gon prism (faceted cylinders): wire + face + `MakePrism_1`, volume
 *   exact vs analytic `(N/2)r²sin(2π/N)h` — the `addPrism` recipe.
 * - hull: `convexHull3d` (pure TS, no OCCT — OCCT ships no convex hull) over
 *   the children's tessellated vertices, then the SAME facet→solid path
 *   `polyhedron` uses (`solidFromFacets`).
 *
 * Memory discipline: every created handle is pushed into `cleanup` (the
 * `meshExtract.ts`/`occtOperations.ts` convention); the RETURNED shape is
 * also in `cleanup` — its lifetime belongs to the caller, exactly like
 * `applyEditsBRep`'s contract.
 *
 * ## Do NOT merge a facet solid's coplanar triangles (measured, rejected)
 *
 * `solidFromFacets` emits one face per triangle even though adjacent triangles
 * are frequently exactly coplanar — a *tessellated* cylinder's side is made of
 * planar strips, and a `hull()` emits raw triangles. Merging each coplanar group
 * into one polygon is a geometry-preserving reduction (same vertices, same
 * planar region, same winding) and it was implemented and measured: **build
 * 19.2s → 5.5s** on the moulded enclosure, face count 2745 → 584, sewing
 * 4.5s → 1.0s, and the booleans 9.3s → 3.0s, because the BOP then intersects
 * far fewer faces.
 *
 * It is NOT here, because it changes what the BOOLEANS see and **this OCCT
 * build then fails to glue a coplanar face-to-face contact**: on the enclosure
 * the tray came back 5.451mm³ lighter (2 × 2.725523mm³ tabs detached) with two
 * extra zero-volume 2-face sheets, i.e. 14 solids where the design has 10. The
 * total volume stayed exact, so nothing below catches it — only the solid
 * decomposition does. Ruled out by measurement, in this order:
 *
 * - **Tolerance.** Tightening the merge tolerances to exact-only (`1e-12`
 *   normal, `diagonal×1e-12` offset) changed nothing at all — the groups being
 *   merged really are exactly coplanar.
 * - **Where the unification happens.** Unifying the *solid* after sewing
 *   instead of the facets before it reproduces the same 14 solids.
 * - **OCCT's own remedies.** `SetFuzzyValue` is **unbound** in this build, and
 *   the bound `SetGlue` modes are worse, not better: `GlueShift` left six
 *   sub-1mm³ slivers and a 11417mm³ tray, `GlueFull` collapsed the model to
 *   7772mm³ over 21 solids.
 * - **Repairing afterwards.** Re-`Fuse_3`ing the detached tab into the tray
 *   reports success and returns the same two solids (`done=true solids=2`), so
 *   the pieces cannot be re-glued once they are built this way.
 *
 * The one variant that *is* correct — leaving the merged groups that lie on the
 * solid's own bounding-box planes triangulated — costs the whole win (16.9s),
 * precisely because those planes are the cylinder caps that make the merge worth
 * doing. `npm run probe -- scripts/probe/examples/csg-solid-inventory.ts` is the
 * per-solid inventory used to establish all of the above; run it on any model
 * before touching this code.
 */

import type { CsgNode } from "./csgImport";
import { resolveSegments } from "./csgImport";
import { buildFlatFace } from "./occtOperations";
import { convexHull3d } from "./convexHull";
import { tessellateShape } from "./meshExtract";

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Oc = any;
// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Shape = any;
type Cleanup = Array<{ delete(): void }>;

const SEW_TOL = 1e-6;
/** Last row a `gp_GTrsf` can represent — anything else is projective. */
const AFFINE_LAST_ROW: [number, number, number, number] = [0, 0, 0, 1];

function keep<T extends { delete(): void }>(cleanup: Cleanup, h: T): T {
  cleanup.push(h);
  return h;
}

/** Frees an OCCT handle NOW instead of at the end of the build, and drops it
 * from `cleanup` so it is not freed twice.
 *
 * The `.csg` walk builds far more intermediates than the rest of this codebase:
 * a `difference` with N subtrahends performs N cuts and a `union` of N operands
 * N-1 fuses, and every one of those results used to stay alive until the whole
 * tree finished. Measured on `S4i_Pico_IB_V210_Mouldable_SnapFit.scad`: the full
 * enclosure aborted with `wasmTable.get(...) is not a function` at 2m46s until
 * each superseded accumulator (and each spent subtrahend) was released as soon
 * as it was replaced. Only handles this module OWNS may be released — never one
 * that is still an element of the operand list, since a failed combine returns
 * that list intact.
 *
 * **This is load-bearing, not an optimization — re-verified by A/B.** Holding
 * every performed BOP alive (the `occtOperations.ts` recipe, which is safe
 * there because an edit op list performs a handful of booleans rather than
 * hundreds) makes the same enclosure fail outright with
 * `OCCT crashed (abort(undefined))`. So the algorithm handle is deleted as soon
 * as its result is in hand, and each superseded accumulator and spent
 * subtrahend is released here. `algo.Shape()` returns a reference-counted copy
 * of the result, which is what makes both safe. */
function release(cleanup: Cleanup, h: unknown): void {
  const i = cleanup.indexOf(h as { delete(): void });
  if (i >= 0) cleanup.splice(i, 1);
  try {
    (h as { delete(): void }).delete();
  } catch {
    /* already freed with its owner */
  }
}

function num(v: unknown): number | undefined {
  return typeof v === "number" && Number.isFinite(v) ? v : undefined;
}

function asVec3(v: unknown): [number, number, number] | null {
  if (!Array.isArray(v) || v.length !== 3) return null;
  const [a, b, c] = v as unknown[];
  if (typeof a !== "number" || typeof b !== "number" || typeof c !== "number") return null;
  if (![a, b, c].every(Number.isFinite)) return null;
  return [a, b, c];
}

function pnt(oc: Oc, p: [number, number, number], cleanup: Cleanup): unknown {
  return keep(cleanup, new oc.gp_Pnt_3(p[0], p[1], p[2]));
}

/** Signed volume of a shape (in the source's units³) — SIGNED, because
 * `orientPositive` below depends on the sign to detect a reversed solid.
 * `NaN` when the integration produced no usable number.
 *
 * **Memoized by handle identity**, which is safe because an OCCT shape is
 * immutable: nothing in this file mutates a shape in place, it always builds a
 * new one. The same wrapper object is genuinely re-measured several times per
 * boolean without this — `orientPositive` measures a hull's solid, then
 * `unionSound` measures it again as an operand, then `cutSound` does the same
 * for the minuend. Measured on the moulded enclosure: 47
 * `VolumeProperties2` calls costing 2.0s of a 19.2s build, for ~15 distinct
 * shapes. */
const volumeByShape = new WeakMap<object, number>();

function shapeVolume(oc: Oc, shape: Shape, cleanup: Cleanup): number {
  void cleanup;
  const cached = volumeByShape.get(shape);
  if (cached !== undefined) return cached;
  const props = new oc.GProp_GProps_1();
  try {
    oc.BRepGProp.VolumeProperties2(shape, props, 1e-3, false, false);
    const v = props.Mass() as number;
    if (Number.isFinite(v)) volumeByShape.set(shape, v);
    return Number.isFinite(v) ? v : NaN;
  } finally {
    props.delete();
  }
}

/** Did a union silently lose geometry? `IsDone()` is NOT sufficient.
 *
 * Measured on `S4i_Pico_IB_V210_Mouldable_SnapFit.scad`: the bottom tray's snap
 * bosses meet the tray wall on the SHARED PLANE y = 16.0, and fusing them onto
 * the tray returns a solid of 504.19 mm³ where OpenSCAD's own render of the same
 * subtree is 11435.56 — while `IsDone()` reports true, so nothing upstream
 * noticed. Same hazard `rib()` documented for a coplanar touch generally.
 *
 * A union can never be smaller than its largest operand, so that bound is a
 * cheap and reliable detector (and it stays valid when the operands overlap,
 * where the true union is smaller than the sum). */
function unionLostGeometry(oc: Oc, a: Shape, b: Shape, result: Shape, cleanup: Cleanup): boolean {
  const va = Math.abs(shapeVolume(oc, a, cleanup));
  const vb = Math.abs(shapeVolume(oc, b, cleanup));
  const vr = Math.abs(shapeVolume(oc, result, cleanup));
  if (!Number.isFinite(vr)) return true;
  const floor = Math.max(va, vb);
  if (!Number.isFinite(floor)) return false;
  return vr < floor * (1 - 1e-6) - 1e-9;
}

/** Canonical "positive" orientation — the same rule `occtOperations.ts`'s
 * (private) `orientPositiveVolume` enforces for thin features: a reversed
 * solid reports NEGATIVE volume into every consumer, so flip it. Probed:
 * a CCW-wound pyramid sews to volume −266.67 without this. */
function orientPositive(oc: Oc, solid: Shape, cleanup: Cleanup): Shape | null {
  const v = shapeVolume(oc, solid, cleanup);
  if (!Number.isFinite(v) || v < 1e-9) return null;
  return v < 0 ? keep(cleanup, solid.Reversed()) : solid;
}

function compoundOf(oc: Oc, shapes: Shape[], cleanup: Cleanup): Shape {
  const comp = keep(cleanup, new oc.TopoDS_Compound());
  const builder = keep(cleanup, new oc.BRep_Builder());
  builder.MakeCompound(comp);
  for (const s of shapes) {
    if (s == null) continue;
    if (typeof s.IsNull === "function" && s.IsNull()) continue;
    builder.Add(comp, s);
  }
  return comp;
}

function emptyCompound(oc: Oc, cleanup: Cleanup): Shape {
  const comp = keep(cleanup, new oc.TopoDS_Compound());
  keep(cleanup, new oc.BRep_Builder()).MakeCompound(comp);
  return comp;
}

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

function buildBox(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void): Shape | null {
  const s = node.params["size"] ?? node.params["#0"];
  let size: [number, number, number] | null = null;
  if (typeof s === "number") size = [s, s, s];
  else if (Array.isArray(s) && s.length === 3) size = asVec3(s as unknown);
  if (!size || size.some((x) => !(x > 0))) { warn(`cube with non-positive/unparseable size — skipping`); return null; }
  const center = node.params["center"] === true;
  const c: [number, number, number] = center ? [0, 0, 0] : [size[0] / 2, size[1] / 2, size[2] / 2];
  const mk = keep(cleanup, new oc.BRepPrimAPI_MakeBox_3(
    pnt(oc, [c[0] - size[0] / 2, c[1] - size[1] / 2, c[2] - size[2] / 2], cleanup),
    pnt(oc, [c[0] + size[0] / 2, c[1] + size[1] / 2, c[2] + size[2] / 2], cleanup),
  ));
  return mk.Shape();
}

function buildSphere(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void, useMaxFN: number): Shape | null {
  const p = node.params;
  let r = num(p["r"]) ?? (num(p["d"]) !== undefined ? (num(p["d"]) as number) / 2 : undefined);
  if (!(r !== undefined && r > 0)) { warn(`sphere with unparseable radius — skipping`); return null; }
  const n = resolveSegments(r, num(p["$fn"]), num(p["$fa"]), num(p["$fs"]));
  if (n <= useMaxFN) {
    warn(`sphere(r=${r}, $fn=${num(p["$fn"]) ?? 0}) is a ${n}-segment faceted solid in OpenSCAD — importing analytic (chord error ${(r * (1 - Math.cos(Math.PI / n))).toPrecision(3)}mm)`);
  }
  const mk = keep(cleanup, new oc.BRepPrimAPI_MakeSphere_5(pnt(oc, [0, 0, 0], cleanup), r));
  return mk.Shape();
}

function buildCylinder(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void, useMaxFN: number): Shape | null {
  const p = node.params;
  const h = num(p["h"]) ?? num(p["height"]);
  if (!(h !== undefined && h > 0)) { warn(`cylinder with unparseable height — skipping`); return null; }
  let r1 = num(p["r1"]) ?? (num(p["d1"]) !== undefined ? (num(p["d1"]) as number) / 2 : undefined);
  let r2 = num(p["r2"]) ?? (num(p["d2"]) !== undefined ? (num(p["d2"]) as number) / 2 : undefined);
  const r = num(p["r"]) ?? (num(p["d"]) !== undefined ? (num(p["d"]) as number) / 2 : undefined);
  if (r !== undefined) { r1 = r; r2 = r; }
  if (!(r1 !== undefined && r2 !== undefined && r1 >= 0 && r2 >= 0 && (r1 > 0 || r2 > 0))) {
    warn(`cylinder with unparseable radii — skipping`);
    return null;
  }
  const center = p["center"] === true;
  const base: [number, number, number] = center ? [0, 0, -h / 2] : [0, 0, 0];
  const ax2 = keep(cleanup, new oc.gp_Ax2_3(pnt(oc, base, cleanup), keep(cleanup, new oc.gp_Dir_4(0, 0, 1))));
  const rMax = Math.max(r1, r2);
  const n = resolveSegments(rMax, num(p["$fn"]), num(p["$fa"]), num(p["$fs"]));
  if (r1 === r2 && n <= useMaxFN) {
    // Faithful N-gon prism. OpenSCAD starts its first polygon vertex at +X;
    // so does this loop — orientation matches by construction.
    return buildNgonPrism(oc, base, r1, n, h, cleanup);
  }
  if (r1 === r2) {
    warn(`cylinder(r=${r1}, $fn=${num(p["$fn"]) ?? 0} → ${n} segments) above useMaxFN=${useMaxFN} — importing analytic (chord error ${(r1 * (1 - Math.cos(Math.PI / n))).toPrecision(3)}mm)`);
    return keep(cleanup, new oc.BRepPrimAPI_MakeCylinder_3(ax2, r1, h)).Shape();
  }
  if (n <= useMaxFN) warn(`tapered cylinder(r1=${r1}, r2=${r2}, ${n} segments) is faceted in OpenSCAD — importing analytic frustum`);
  return keep(cleanup, new oc.BRepPrimAPI_MakeCone_3(ax2, r1, r2, h)).Shape();
}

/** Regular N-gon prism, base at `base`, axis +Z — the `addPrism` recipe
 * (`occtOperations.ts`), restricted to the axis OpenSCAD cylinders use. */
function buildNgonPrism(
  oc: Oc, base: [number, number, number], radius: number, sides: number, height: number, cleanup: Cleanup,
): Shape | null {
  const pts: Array<[number, number, number]> = [];
  for (let i = 0; i < sides; i++) {
    const a = (2 * Math.PI * i) / sides;
    pts.push([base[0] + radius * Math.cos(a), base[1] + radius * Math.sin(a), base[2]]);
  }
  const wireMk = keep(cleanup, new oc.BRepBuilderAPI_MakeWire_1());
  for (let i = 0; i < pts.length; i++) {
    const e = keep(cleanup, new oc.BRepBuilderAPI_MakeEdge_3(pnt(oc, pts[i], cleanup), pnt(oc, pts[(i + 1) % pts.length], cleanup)));
    wireMk.Add_1(e.Edge());
  }
  if (!wireMk.IsDone()) return null;
  const faceMk = keep(cleanup, new oc.BRepBuilderAPI_MakeFace_15(wireMk.Wire(), true));
  const face = faceMk.Face();
  const vec = keep(cleanup, new oc.gp_Vec_4(0, 0, height));
  return keep(cleanup, new oc.BRepPrimAPI_MakePrism_1(face, vec, false, true)).Shape();
}

// ---------------------------------------------------------------------------
// 2D profiles — linear_extrude / rotate_extrude children (the node-coverage
// extension of the OpenSCAD support feature). A 2D node is only ever built through here, never as a
// standalone solid: 2D has no volume, so a bare square/circle/polygon under
// a union (or at top level) stays skipped, same as before.
// ---------------------------------------------------------------------------

/** 2D point pairs for a polygon/square/circle node, or null (warning already
 * issued). `paths=` (holes) is a v1 refusal: importing the outer contour
 * alone would be a confidently-wrong disk where OpenSCAD cuts a washer. */
function profilePoints2D(node: CsgNode, warn: (m: string) => void, useMaxFN: number): Array<[number, number]> | null {
  const p = node.params;
  if (node.name === "polygon") {
    if ("paths" in p) { warn(`polygon() with paths= (holes) is a later phase — skipping`); return null; }
    const raw = p["points"];
    if (!Array.isArray(raw) || raw.length < 6 || raw.length % 2 !== 0) {
      warn(`polygon() with <3 points — skipping`);
      return null;
    }
    if (raw.some((x) => typeof x !== "number" || !Number.isFinite(x))) {
      warn(`polygon() with non-numeric points — skipping`);
      return null;
    }
    const pts: Array<[number, number]> = [];
    for (let i = 0; i + 1 < raw.length; i += 2) pts.push([raw[i] as number, raw[i + 1] as number]);
    return pts;
  }
  if (node.name === "square") {
    const s = p["size"];
    let w: number | undefined;
    let h: number | undefined;
    if (typeof s === "number") { w = s; h = s; }
    else if (Array.isArray(s) && s.length === 2) { w = num(s[0]); h = num(s[1]); }
    if (!(w !== undefined && h !== undefined && w > 0 && h > 0)) {
      warn(`square() with non-positive/unparseable size — skipping`);
      return null;
    }
    const centered = p["center"] === true;
    const x0 = centered ? -w / 2 : 0;
    const y0 = centered ? -h / 2 : 0;
    return [[x0, y0], [x0 + w, y0], [x0 + w, y0 + h], [x0, y0 + h]];
  }
  if (node.name === "circle") {
    const r = num(p["r"]) ?? (num(p["d"]) !== undefined ? (num(p["d"]) as number) / 2 : undefined);
    if (!(r !== undefined && r > 0)) { warn(`circle() with non-positive/unparseable radius — skipping`); return null; }
    // Always the N-gon OpenSCAD actually draws (never an analytic face whose
    // wall would deviate from the authored outline everywhere): first vertex
    // at +X, the `buildCylinder` convention above.
    const n = resolveSegments(r, num(p["$fn"]), num(p["$fa"]), num(p["$fs"]));
    warn(`circle(r=${r}, $fn=${num(p["$fn"]) ?? 0} → ${n} segments) is faceted in OpenSCAD — importing N-gon (chord error ${(r * (1 - Math.cos(Math.PI / n))).toPrecision(3)}mm)`);
    const pts: Array<[number, number]> = [];
    for (let i = 0; i < n; i++) {
      const a = (2 * Math.PI * i) / n;
      pts.push([r * Math.cos(a), r * Math.sin(a)]);
    }
    return pts;
  }
  return null;
}

/** The single 2D child of an extrude: transparent containers (`group`,
 * `color` — the same transparency `evalShapes` gives them) unwrap one level;
 * anything else (a 2D boolean union, `offset`, a second profile) is a v1
 * refusal — fusing 2D faces would need an unprobed face-boolean. */
function extrudeChild(node: CsgNode, op: string, warn: (m: string) => void): CsgNode | null {
  if (node.children.length !== 1) {
    warn(`${op}() with ${node.children.length} children (exactly one 2D profile in v1) — skipping`);
    return null;
  }
  let child = node.children[0];
  while (child.name === "group" || child.name === "color") {
    if (child.children.length !== 1) break;
    child = child.children[0];
  }
  if (child.name !== "polygon" && child.name !== "square" && child.name !== "circle") {
    warn(`${op}() of ${child.name}() (only polygon/square/circle profiles in v1) — skipping`);
    return null;
  }
  return child;
}

/** `true` unless `scale` is present and differs from the identity (a number
 * `!== 1`, or a vector with any component `!== 1`): real `.csg` files always
 * carry the default `scale = 1`, which must not trip the later-phase refusal. */
function isIdentityScale(v: unknown): boolean {
  if (v === undefined || v === null) return true;
  if (typeof v === "number") return v === 1;
  if (Array.isArray(v)) return v.length > 0 && v.every((x) => x === 1);
  return false;
}

function buildLinearExtrude(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void, useMaxFN: number): Shape | null {
  const p = node.params;
  const child = extrudeChild(node, "linear_extrude", warn);
  if (!child) return null;
  const h = num(p["height"]) ?? num(p["h"]);
  if (!(h !== undefined && h > 0)) { warn(`linear_extrude() with non-positive/unparseable height — skipping`); return null; }
  if (num(p["twist"])) { warn(`linear_extrude() with twist=${num(p["twist"])} (a later phase) — skipping`); return null; }
  if (!isIdentityScale(p["scale"])) { warn(`linear_extrude() with non-identity scale (a later phase) — skipping`); return null; }
  const pts2d = profilePoints2D(child, warn, useMaxFN);
  if (!pts2d) return null;
  // slices/convexity are preview-only (OpenSCAD ignores them at render too).
  const face = buildFlatFace(oc, pts2d.map(([x, y]): [number, number, number] => [x, y, 0]), cleanup);
  if (!face || face.IsNull()) { warn(`linear_extrude() profile failed to face — skipping`); return null; }
  const vec = keep(cleanup, new oc.gp_Vec_4(0, 0, h));
  let solid: Shape;
  try {
    solid = keep(cleanup, keep(cleanup, new oc.BRepPrimAPI_MakePrism_1(face, vec, false, true)).Shape());
  } catch (e) {
    warn(`linear_extrude() build threw (${e instanceof Error ? e.message : String(e)}) — skipping`);
    return null;
  }
  if (p["center"] === true) {
    const moved = applyMatrix(oc, [solid], translateMatrix([0, 0, -h / 2]), cleanup, warn);
    if (!moved) return null;
    solid = moved[0];
  }
  const oriented = orientPositive(oc, solid, cleanup);
  if (!oriented) { warn(`linear_extrude() collapsed to zero volume — skipping`); return null; }
  return oriented;
}

function buildRotateExtrude(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void, useMaxFN: number): Shape | null {
  const p = node.params;
  const child = extrudeChild(node, "rotate_extrude", warn);
  if (!child) return null;
  const angleDeg = num(p["angle"]) ?? 360;
  if (!Number.isFinite(angleDeg) || angleDeg === 0) {
    warn(`rotate_extrude() with non-finite/zero angle — skipping`);
    return null;
  }
  const pts2d = profilePoints2D(child, warn, useMaxFN);
  if (!pts2d) return null;
  // OpenSCAD reads profile (x, y) as (radius, height): the profile plane must
  // CONTAIN the revolve axis, so (x, y) maps to (x, 0, y) revolved about Z —
  // revolving the raw XY face about Z would spin it in place (volume 0,
  // verified live). Partial angles sweep CCW from +X, matching OpenSCAD.
  const face = buildFlatFace(oc, pts2d.map(([x, y]): [number, number, number] => [x, 0, y]), cleanup);
  if (!face || face.IsNull()) { warn(`rotate_extrude() profile failed to face — skipping`); return null; }
  const ax = keep(cleanup, new oc.gp_Ax1_2(pnt(oc, [0, 0, 0], cleanup), keep(cleanup, new oc.gp_Dir_4(0, 0, 1))));
  let solid: Shape;
  try {
    solid = keep(cleanup, keep(cleanup, new oc.BRepPrimAPI_MakeRevol_1(face, ax, (angleDeg * Math.PI) / 180, false)).Shape());
  } catch (e) {
    warn(`rotate_extrude() build threw (${e instanceof Error ? e.message : String(e)}) — skipping`);
    return null;
  }
  const oriented = orientPositive(oc, solid, cleanup);
  if (!oriented) { warn(`rotate_extrude() collapsed to zero volume — skipping`); return null; }
  return oriented;
}

function buildPolyhedron(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void): Shape | null {
  const rawPts = node.params["points"];
  if (!Array.isArray(rawPts) || (rawPts as unknown[]).length < 12) {
    warn(`polyhedron with <4 points — skipping`);
    return null;
  }
  const flat = rawPts as unknown[];
  if (flat.some((x) => typeof x !== "number" || !Number.isFinite(x))) {
    warn(`polyhedron with non-numeric points — skipping`);
    return null;
  }
  const points: Array<[number, number, number]> = [];
  for (let i = 0; i + 2 < flat.length; i += 3) points.push([flat[i] as number, flat[i + 1] as number, flat[i + 2] as number]);
  const faces = node.faces;
  if (!faces || faces.length < 4) {
    warn(`polyhedron with <4 faces (or unparseable faces=) — skipping`);
    return null;
  }
  return solidFromFacets(oc, points, faces, cleanup, warn, "polyhedron");
}

/**
 * `hull()` — the convex hull of every child's TESSELLATED vertices.
 *
 * Not a transform, so it cannot be passed through: `hull()` REPLACES its
 * children. Skipping it (the previous behaviour) drops the geometry entirely —
 * the moulded enclosure that motivated this fix built to volume 0.001 against
 * OpenSCAD's own 32109.77, because 23 hulls vanished and the top-level
 * `difference()` then had no minuend left to cut.
 *
 * Why tessellated vertices rather than B-rep vertices: a `cylinder` has only
 * two seam vertices, so hulling those yields a degenerate sliver instead of a
 * prism. This is also exactly OpenSCAD's own semantics — it hulls the faceted
 * polyset, not an analytic surface — so the tessellation is agreement, not an
 * approximation. `tessellateShape` is the same 0.1 mm display tessellator the
 * viewer uses; the existing `useMaxFN` faceting dial still decides whether the
 * child itself was built analytic or as a real N-gon prism.
 *
 * Scope, stated plainly: a 2-D `hull()` (children spanning no volume, e.g. two
 * circles) is NOT imported. The points are coplanar, `convexHull3d` returns
 * null, and the subtree is skipped with a warning — OpenSCAD's 2-D hull is a
 * distinct operation this importer has no representation for, and emitting a
 * flat face where a solid belongs would be confidently-wrong geometry.
 */
function buildHull(
  oc: Oc,
  node: CsgNode,
  cleanup: Cleanup,
  warn: (m: string) => void,
  useMaxFN: number,
): Shape | null {
  const kids = node.children.flatMap((c) => tryShapes(oc, c, cleanup, warn, useMaxFN));
  if (kids.length === 0) {
    warn(`hull() — every child was skipped, nothing to hull`);
    return null;
  }

  const points: Array<[number, number, number]> = [];
  let unmeshable = 0;
  for (const s of kids) {
    try {
      for (const buf of tessellateShape(oc, s)) {
        const pos = buf.positions;
        for (let i = 0; i + 2 < pos.length; i += 3) points.push([pos[i], pos[i + 1], pos[i + 2]]);
      }
    } catch (e) {
      // Skip just this child: a hull of the remaining points is still a far
      // better answer than dropping the whole hull (which is how the moulded
      // enclosure lost its geometry).
      unmeshable++;
      warn(`hull() child tessellation threw (${e instanceof Error ? e.message : String(e)}) — leaving that child out`);
    }
  }
  if (unmeshable > 0 && points.length < 4) {
    warn(`hull() — ${unmeshable} child(ren) could not be meshed and nothing else remained — skipping`);
    return null;
  }
  if (points.length < 4) {
    warn(`hull() — fewer than 4 points across its children — skipping`);
    return null;
  }

  const hull = convexHull3d(points);
  if (!hull) {
    warn(`hull() — children do not span a volume (collinear/coplanar points; a 2-D hull is not imported) — skipping`);
    return null;
  }

  const solid = solidFromFacets(oc, hull.points, hull.facets.map((f) => [...f]), cleanup, warn, "hull");
  if (!solid) return null;
  warn(`hull() — hulled ${points.length} tessellated point(s) into ${hull.facets.length} facet(s)`);
  return solid;
}

/**
 * Sews a triangulated facet set into a solid — the shared tail of
 * `polyhedron` (facets from the `.csg` text) and `hull` (facets from
 * `convexHull.ts`): per-face `MakeWire_1` + `MakeEdge_3` +
 * `MakeFace_15(wire, true)`, then `BRepBuilderAPI_Sewing(SEW_TOL, ...)` + a
 * `NbFreeEdges() == 0` closure gate (the promotion pipeline's own check) +
 * `MakeSolid_3` + the orientation fix.
 *
 * One face per triangle, deliberately: a coplanar-triangle merge was implemented
 * and measured (3.2× on the build), but it changes what the BOOLEANS see and
 * this OCCT build then fails to glue a coplanar face-to-face contact — see the
 * module's own "Do NOT merge the facets" note.
 *
 * `label` names the caller in every warning, so a skipped subtree still says
 * which construct was responsible.
 */
function solidFromFacets(
  oc: Oc,
  points: Array<[number, number, number]>,
  faces: number[][],
  cleanup: Cleanup,
  warn: (m: string) => void,
  label: string,
): Shape | null {
  try {
    const gpPts = points.map((q) => pnt(oc, q, cleanup));
    const brepFaces: Shape[] = [];
    for (const f of faces) {
      if (f.length < 3 || f.some((x) => !(Number.isInteger(x) && x >= 0 && x < points.length))) {
        warn(`${label} with out-of-range face index — skipping`);
        return null;
      }
      // Fan-triangulate N-gons. One face per triangle, deliberately — see
      // `solidFromFacets`' doc comment for the measured reason the obvious
      // "merge each coplanar group into one polygon" optimisation is NOT here.
      for (let i = 1; i + 1 < f.length; i++) {
        const wireMk = keep(cleanup, new oc.BRepBuilderAPI_MakeWire_1());
        for (const [a, b] of [[f[0], f[i]], [f[i], f[i + 1]], [f[i + 1], f[0]]] as Array<[number, number]>) {
          const e = keep(cleanup, new oc.BRepBuilderAPI_MakeEdge_3(gpPts[a], gpPts[b]));
          wireMk.Add_1(e.Edge());
        }
        if (!wireMk.IsDone()) { warn(`${label} face failed to wire — skipping`); return null; }
        brepFaces.push(keep(cleanup, new oc.BRepBuilderAPI_MakeFace_15(wireMk.Wire(), true)).Face());
      }
    }

    const sew = keep(cleanup, new oc.BRepBuilderAPI_Sewing(SEW_TOL, true, true, true, false));
    for (const f of brepFaces) sew.Add(f);
    sew.Perform(keep(cleanup, new oc.Handle_Message_ProgressIndicator_1()));
    if (sew.NbFreeEdges() > 0) {
      warn(`${label} did not close (${sew.NbFreeEdges()} free edges) — skipping`);
      return null;
    }
    const exp = keep(cleanup, new oc.TopExp_Explorer_2(sew.SewedShape(), oc.TopAbs_ShapeEnum.TopAbs_SHELL, oc.TopAbs_ShapeEnum.TopAbs_SHAPE));
    if (!exp.More()) { warn(`${label} produced no shell — skipping`); return null; }
    const solid = keep(cleanup, new oc.BRepBuilderAPI_MakeSolid_3(oc.TopoDS.Shell_1(exp.Current()))).Solid();
    const oriented = orientPositive(oc, solid, cleanup);
    if (!oriented) { warn(`${label} collapsed to zero volume — skipping`); return null; }
    return oriented;
  } catch (e) {
    warn(`${label} build threw (${e instanceof Error ? e.message : String(e)}) — skipping`);
    return null;
  }
}

// ---------------------------------------------------------------------------
// Transforms — one uniform gp_GTrsf path for every node kind
// ---------------------------------------------------------------------------

function translateMatrix(v: [number, number, number]): number[] {
  return [1, 0, 0, v[0], 0, 1, 0, v[1], 0, 0, 1, v[2], 0, 0, 0, 1];
}

function scaleMatrix(v: [number, number, number]): number[] {
  return [v[0], 0, 0, 0, 0, v[1], 0, 0, 0, 0, v[2], 0, 0, 0, 0, 1];
}

function mirrorMatrix(n: [number, number, number]): number[] {
  const l = Math.hypot(n[0], n[1], n[2]);
  const nx = n[0] / l, ny = n[1] / l, nz = n[2] / l;
  return [
    1 - 2 * nx * nx, -2 * nx * ny, -2 * nx * nz, 0,
    -2 * ny * nx, 1 - 2 * ny * ny, -2 * ny * nz, 0,
    -2 * nz * nx, -2 * nz * ny, 1 - 2 * nz * nz, 0,
    0, 0, 0, 1,
  ];
}

/** OpenSCAD `rotate(a=[x,y,z])` is extrinsic X→Y→Z (R = Rz·Ry·Rx); `a=deg,
 * v=` is axis-angle (Rodrigues). Returns row-major 4×4 or null. */
function rotateMatrix(p: Record<string, unknown>): number[] | null {
  const a = p["a"] ?? p["#0"];
  const v = asVec3(p["v"]);
  if (typeof a === "number" && !v) {
    // rotate(deg) — about Z by OpenSCAD convention.
    return rotateMatrix({ a, v: [0, 0, 1] });
  }
  if (typeof a === "number" && v) {
    const l = Math.hypot(v[0], v[1], v[2]);
    if (!(l > 0)) return null;
    const [x, y, z] = [v[0] / l, v[1] / l, v[2] / l];
    const t = (a * Math.PI) / 180;
    const c = Math.cos(t), s = Math.sin(t), C = 1 - c;
    return [
      x * x * C + c, x * y * C - z * s, x * z * C + y * s, 0,
      y * x * C + z * s, y * y * C + c, y * z * C - x * s, 0,
      z * x * C - y * s, z * y * C + x * s, z * z * C + c, 0,
      0, 0, 0, 1,
    ];
  }
  if (Array.isArray(a) && a.length === 3) {
    const av = asVec3(a as unknown);
    if (!av) return null;
    const [ex, ey, ez] = [(av[0] * Math.PI) / 180, (av[1] * Math.PI) / 180, (av[2] * Math.PI) / 180];
    const Rx = [1, 0, 0, 0, 0, Math.cos(ex), -Math.sin(ex), 0, 0, Math.sin(ex), Math.cos(ex), 0, 0, 0, 0, 1];
    const Ry = [Math.cos(ey), 0, Math.sin(ey), 0, 0, 1, 0, 0, -Math.sin(ey), 0, Math.cos(ey), 0, 0, 0, 0, 1];
    const Rz = [Math.cos(ez), -Math.sin(ez), 0, 0, Math.sin(ez), Math.cos(ez), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    return matMul(Rz, matMul(Ry, Rx));
  }
  return null;
}

function matMul(A: number[], B: number[]): number[] {
  const out = new Array(16).fill(0);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) for (let k = 0; k < 4; k++) out[r * 4 + c] += A[r * 4 + k] * B[k * 4 + c];
  return out;
}

function nodeMatrix(node: CsgNode, warn: (m: string) => void): number[] | null {
  const p = node.params as Record<string, unknown>;
  switch (node.name) {
    case "multmatrix": {
      const m = (p["m"] ?? p["#0"]) as unknown;
      if (!Array.isArray(m) || m.length !== 16 || !(m as unknown[]).every((x) => typeof x === "number" && Number.isFinite(x))) {
        warn(`multmatrix with unparseable m= — dropping children`);
        return null;
      }
      return m as number[];
    }
    case "translate": {
      const v = asVec3(p["v"] ?? p["#0"]);
      if (!v) { warn(`translate with unparseable v — dropping children`); return null; }
      return translateMatrix(v);
    }
    case "scale": {
      const raw = (p["v"] ?? p["#0"]) as unknown;
      const v = typeof raw === "number" ? [raw, raw, raw] as [number, number, number] : asVec3(raw);
      if (!v) { warn(`scale with unparseable v — dropping children`); return null; }
      return scaleMatrix(v);
    }
    case "mirror": {
      const v = asVec3(p["v"] ?? p["#0"] ?? p["vec"]);
      if (!v || Math.hypot(v[0], v[1], v[2]) <= 0) { warn(`mirror with unparseable v — dropping children`); return null; }
      return mirrorMatrix(v);
    }
    case "rotate": {
      const m = rotateMatrix(p);
      if (!m) { warn(`rotate with unparseable params — importing children unrotated`); return null; }
      return m;
    }
    default:
      return null;
  }
}

/** `A = s·R` with `R` orthogonal? Returns `s`, or `null` when the 3×3 needs a
 * general `gp_GTrsf`.
 *
 * **The check has to live here, in JS: this build's OCCT no longer validates
 * it.** `gp_Trsf.SetValues` is documented to raise
 * `Standard_ConstructionError` for a non-similarity, but probed live it
 * ACCEPTED `diag(1, 2, 3)` without a word — this WASM ships no C++ exception
 * runtime (the same gap that surfaces an OCCT throw as
 * `___cxa_can_catch is not defined` elsewhere in this file), so the guard is
 * silently absent and a shear routed here would be stored as a wrong
 * transformation, not rejected. A genuinely non-uniform matrix misses on row
 * norms by a factor of two or more, so a relative `1e-9` test cannot
 * misclassify one.
 *
 * Why the split earns its keep — one cylinder (r=2, h=10) under a rigid
 * rotate + translate, which is what every `multmatrix` in a real `.scad` is:
 *
 * | path | volume | surface type |
 * | --- | --- | --- |
 * | `BRepBuilderAPI_GTransform_2` | 125.658588 | 6 = BSplineSurface |
 * | `BRepBuilderAPI_Transform_2` | 125.663706 | 1 = Cylinder |
 *
 * The exact value is πr²h = 125.663706, so the general path also APPROXIMATES
 * analytic surfaces (this repo's own ±4e-5 translated-revolve note) — and, far
 * more expensively downstream, it hands the next boolean a BSpline patch where
 * an analytic cylinder would have been. Uniform scale (×2.5 → 1963.495408,
 * exact) and a reflection (volume unchanged, type preserved) ride the same
 * path; only shear and non-uniform scale fall through to `gp_GTrsf`. */
function similarityScale(m: number[]): number | null {
  const rows: Array<[number, number, number]> = [
    [m[0], m[1], m[2]],
    [m[4], m[5], m[6]],
    [m[8], m[9], m[10]],
  ];
  const dot = (a: [number, number, number], b: [number, number, number]): number =>
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const norms = rows.map((r) => dot(r, r));
  if (!norms.every((n) => Number.isFinite(n) && n > 1e-18)) return null;
  const mean = (norms[0] + norms[1] + norms[2]) / 3;
  const tol = mean * 1e-9;
  if (norms.some((n) => Math.abs(n - mean) > tol)) return null;
  if (Math.abs(dot(rows[0], rows[1])) > tol) return null;
  if (Math.abs(dot(rows[0], rows[2])) > tol) return null;
  if (Math.abs(dot(rows[1], rows[2])) > tol) return null;
  return Math.sqrt(mean);
}

/** Applies a row-major 4×4 to every shape. The last row must be affine;
 * shear needs NO special path (`gp_GTrsf` is general 3×4 — probed).
 *
 * Rigid, uniform-scale and mirrored matrices take the `gp_Trsf` route instead
 * (`similarityScale`); they are the overwhelming majority of `.csg` transforms
 * and are the only ones that keep a cylinder analytic. */
function applyMatrix(oc: Oc, shapes: Shape[], m: number[], cleanup: Cleanup, warn: (m: string) => void): Shape[] | null {
  for (let i = 0; i < 4; i++) {
    if (Math.abs(m[12 + i] - AFFINE_LAST_ROW[i]) > 1e-9) {
      warn(`transform with projective last row — dropping children (affine only)`);
      return null;
    }
  }
  const uniform = similarityScale(m);
  const out: Shape[] = [];
  for (const s of shapes) {
    try {
      if (uniform !== null) {
        const t = keep(cleanup, new oc.gp_Trsf_1());
        t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
        out.push(keep(cleanup, new oc.BRepBuilderAPI_Transform_2(s, t, true)).Shape());
        continue;
      }
      const g = keep(cleanup, new oc.gp_GTrsf_1());
      for (let r = 1; r <= 3; r++) for (let c = 1; c <= 4; c++) g.SetValue(r, c, m[(r - 1) * 4 + (c - 1)]);
      out.push(keep(cleanup, new oc.BRepBuilderAPI_GTransform_2(s, g, true)).Shape());
    } catch (e) {
      warn(`transform build threw (${e instanceof Error ? e.message : String(e)}) — dropping child`);
      return null;
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Tree walk — returns live shapes (caller owns nothing; all in cleanup)
// ---------------------------------------------------------------------------

const SKIP_SUBTREE_MSGS: Record<string, string> = {
  minkowski: "minkowski() has no OCCT equivalent — skipping",
  text: "text() carries a font name, not outlines (the reference WASM ships with no font support) — skipping",
  import: "import() references an external file — skipping",
  surface: "surface() references an external heightmap file — skipping",
  square: "standalone 2D square() is not imported — only as a linear_extrude/rotate_extrude child — skipping",
  circle: "standalone 2D circle() is not imported — only as a linear_extrude/rotate_extrude child — skipping",
  polygon: "standalone 2D polygon() is not imported — only as a linear_extrude/rotate_extrude child — skipping",
  offset: "2D offset() is not imported in v1 — skipping",
  projection: "projection() cuts to 2D — skipping",
  resize: "resize() is not imported in v1 — skipping",
};

function booleanOf(oc: Oc, kind: "union" | "subtract" | "intersect", a: Shape, b: Shape, cleanup: Cleanup): Shape | null {
  const Ctor =
    kind === "union" ? oc.BRepAlgoAPI_Fuse_3
    : kind === "subtract" ? oc.BRepAlgoAPI_Cut_3
    : oc.BRepAlgoAPI_Common_3;
  // The algorithm object is freed as soon as the result is in hand, NOT kept
  // alive in `cleanup` until the end of the build. A performed BOP retains its
  // argument lists and full interference history, and a `.csg` tree performs
  // dozens of them, so holding them all is what exhausted the Emscripten heap
  // (measured: the enclosure aborted with `wasmTable.get(...) is not a
  // function`). `Shape()` returns a reference-counted copy, so the result
  // outlives its algorithm — which is also what makes it safe for callers to
  // release an operand once its boolean has been consumed.
  const algo = new Ctor(a, b);
  try {
    if (!algo.IsDone()) return null;
    const r = algo.Shape();
    cleanup.push(r);
    if (typeof r.IsNull === "function" && r.IsNull()) return null;
    return r;
  } finally {
    algo.delete();
  }
}

/** `TopTools_ListOfShape` — the only OCCT list type this codebase has verified
 * (the `addVolumeFromSurfaces` recipe). */
type ShapeList = { Append_1(s: Shape): void; delete(): void };

function shapeListOf(oc: Oc, shapes: Shape[], cleanup: Cleanup): ShapeList {
  const list = keep(cleanup, new oc.TopTools_ListOfShape_1()) as ShapeList;
  for (const s of shapes) list.Append_1(s);
  return list;
}

function isNullShape(s: Shape): boolean {
  return typeof s?.IsNull === "function" && s.IsNull() === true;
}

/**
 * ONE `BRepAlgoAPI` for a whole operand list instead of a pairwise fold — the
 * single biggest cost in a real `.scad` import.
 *
 * Profiled on `S4i_Pico_IB_V210_Mouldable_SnapFit.scad` with the committed probe
 * harness (`npm run scad:check -- … --profile`): of a 280 s build, 17 pairwise
 * `Fuse_3` calls cost **90.7 s** and 28 `Cut_3` calls **157.3 s** — 88 % of the
 * import, at ~5.4 s per call, because every step re-processes an accumulator
 * that has already grown to the size of the finished part. One call over the
 * whole list pays the interference computation once.
 *
 * The working form is the BOPAlgo one: `Fuse_1` + `SetArguments([first])` +
 * `SetTools(rest)` + `Build()`. It must NOT be `BRepAlgoAPI_BuilderAlgo_1`
 * (OCCT's "general fuse"), which looks like the natural choice and is a trap:
 * on four operands that touch or overlap it returns the right VOLUME but leaves
 * them as **5 solids / 6 shells / 36 faces** instead of the pairwise fold's
 * **1 solid / 1 shell / 18 faces** — the same shape, unglued. Measured on the
 * enclosure that unglued form built in 21.8s against 280s (and with the right
 * volume, 32047.4 against 32064.8) but then made `tessellateByGroup` never
 * finish at all, where the pairwise result meshes in 1.5s: 31 solids / 4733
 * faces of coincident geometry, handed to the mesher. The BOPAlgo form instead
 * reproduces the fold's topology exactly (1/1/18, volume 2500.000, valid) in
 * ONE call, and probe #4 measures it at 63ms against the fold's 163ms on that
 * same four-operand case.
 *
 * `BRepAlgoAPI_Fuse_1` + `SetArguments(all)` with no tools is NOT usable either
 * — an empty tools list gives `IsDone() === false`, and `Shape()` without
 * `Build()` returns empty. `BOPAlgo_BOP`/`BOPAlgo_Builder` are unbound in this
 * build.
 *
 * `intersection` deliberately keeps the pairwise fold: no multi-argument
 * `Common` form was probed, and the cost is in the union and the difference.
 */
function multiUnion(oc: Oc, shapes: Shape[], cleanup: Cleanup): Shape | null {
  const algo = new oc.BRepAlgoAPI_Fuse_1();
  const lists: ShapeList[] = [];
  try {
    const args = shapeListOf(oc, [shapes[0]], cleanup);
    const toolList = shapeListOf(oc, shapes.slice(1), cleanup);
    lists.push(args, toolList);
    algo.SetArguments(args);
    algo.SetTools(toolList);
    algo.Build();
    if (!algo.IsDone()) return null;
    const r = algo.Shape();
    cleanup.push(r);
    return isNullShape(r) ? null : r;
  } finally {
    algo.delete();
    // `Build` copies the lists into the algorithm's own structures, so the lists
    // themselves are spent. The OPERANDS are deliberately left in `cleanup`:
    // they are small beside the result, and `release`'s own rule forbids freeing
    // a shape that any caller-supplied list might still hold.
    for (const l of lists) release(cleanup, l);
  }
}

/**
 * ONE cut with every subtrahend supplied as a separate TOOL, which is the form
 * OCCT is designed around — `Cut_1` + `SetArguments([minuend])` +
 * `SetTools(tools)` + `Build()`.
 *
 * Probed on a 40×20×5 plate minus three boxes (analytic 3865): **89 ms →
 * 3865.000 exact**, against 145 ms → 3865.000 pairwise.
 *
 * The pairwise loop's own comment records why the tools must NOT be
 * pre-compounded (an 18-tool compound returned 134.89 mm³ where the true
 * difference is 13001.15). That is about handing OCCT ONE compound, whose
 * members then interfere with each other *as a single argument*. A tool LIST is
 * the multi-operand form the class exists to accept, so this uses it — with
 * `cutSound`'s volume bound and the pairwise loop as the fallback, and with the
 * whole-model result verified against the known-good pairwise build before this
 * shipped.
 */
function multiCut(oc: Oc, minuend: Shape, tools: Shape[], cleanup: Cleanup): Shape | null {
  const algo = new oc.BRepAlgoAPI_Cut_1();
  const lists: ShapeList[] = [];
  try {
    const args = shapeListOf(oc, [minuend], cleanup);
    const toolList = shapeListOf(oc, tools, cleanup);
    lists.push(args, toolList);
    algo.SetArguments(args);
    algo.SetTools(toolList);
    algo.Build();
    if (!algo.IsDone()) return null;
    const r = algo.Shape();
    cleanup.push(r);
    return isNullShape(r) ? null : r;
  } finally {
    algo.delete();
    for (const l of lists) release(cleanup, l);
  }
}

/** A union can be neither smaller than its largest operand nor larger than the
 * sum of the operands. Both bounds are cheap (the operands are small beside the
 * result) and they catch the two ways a boolean silently lies: dropping
 * material — `unionLostGeometry`'s own measured case — and keeping material
 * that should have merged away. */
function unionSound(oc: Oc, operands: Shape[], result: Shape, cleanup: Cleanup): boolean {
  const vr = Math.abs(shapeVolume(oc, result, cleanup));
  if (!Number.isFinite(vr)) return false;
  let max = 0;
  let sum = 0;
  let seen = false;
  for (const s of operands) {
    const v = Math.abs(shapeVolume(oc, s, cleanup));
    if (!Number.isFinite(v)) continue;
    seen = true;
    if (v > max) max = v;
    sum += v;
  }
  if (!seen) return true;
  if (vr < max * (1 - 1e-6) - 1e-9) return false;
  return vr <= sum * (1 + 1e-6) + 1e-9;
}

/** A cut may only REMOVE material: `0 ≤ V(result) ≤ V(minuend)`. */
function cutSound(oc: Oc, minuend: Shape, result: Shape, cleanup: Cleanup): boolean {
  const vm = Math.abs(shapeVolume(oc, minuend, cleanup));
  const vr = Math.abs(shapeVolume(oc, result, cleanup));
  if (!Number.isFinite(vm) || !Number.isFinite(vr)) return false;
  return vr <= vm * (1 + 1e-6) + 1e-9;
}

function countFaces(oc: Oc, shape: Shape): number {
  const exp = new oc.TopExp_Explorer_2(
    shape,
    oc.TopAbs_ShapeEnum.TopAbs_FACE,
    oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
  );
  try {
    let n = 0;
    for (; exp.More(); exp.Next()) n++;
    return n;
  } finally {
    exp.delete();
  }
}

/**
 * Merge same-domain faces on a boolean's RESULT (see `replaced`) — the one
 * face-count reduction that survived measurement, because it never touches the
 * operands of the boolean that produced it.
 *
 * A boolean fragmentizes its operands: the enclosure's `union()`/`difference()`
 * leave faces where the geometry needs far fewer, because every coplanar plane
 * the BOP touched stays its own face. Everything downstream pays for that count
 * — the next boolean, `BRepMesh`, the edge extraction, the number of
 * `THREE.Mesh` objects in the webview, and the STEP export. Measured on the
 * enclosure with
 * `npm run probe -- scripts/probe/examples/csg-same-domain.ts`:
 *
 * | | faces | volume |
 * | --- | --- | --- |
 * | as built | 2745 | 32047.3837 |
 * | `ShapeUpgrade_UnifySameDomain_2(s, true, true, false)` | **552** | 32047.3836 |
 *
 * — a 5× reduction at a measured **−3e-9 relative** volume change (the pass
 * merges within the shape's own tolerance), and a `Cut_3` against that result
 * runs in **1221ms instead of 3475ms** (2.85×). The pass itself costs 1395ms on
 * that 2745-face shape. On the finished document the same pass leaves **1312**
 * faces (the tray 1085 → 187), which is where the display-side win comes from:
 * tessellation 1.31s → 0.72s, edge extraction 290ms → 167ms.
 *
 * Placement is load-bearing and was measured both ways: applying it to every
 * boolean result as it is produced is **18.7s** on the enclosure, while applying
 * it once to the finished document is **21.4s** — the unified intermediate is
 * what makes the NEXT boolean cheap, because a BOP's cost tracks its operands'
 * face count.
 *
 * That is also the hazard, and it is the same one the rejected facet merge hit
 * (see the module note): a unified shape IS handed to later booleans here, and
 * "a unified operand" is exactly the condition this build fails on for a
 * coplanar face-to-face contact. Two things make it acceptable rather than
 * reckless — it is guarded (`IsNull`, the volume bound below, `try`/`catch`, and
 * it only ever runs above `UNIFY_MIN_FACES`), and it is measured on the very
 * model that exposed the facet merge: with this pass on and that one off the
 * enclosure still comes back as its design's 10 solids at 32047.3837mm³. If a
 * contact ever does fail, the first thing to try is raising `UNIFY_MIN_FACES`
 * (the finished-document variant costs 2.5s of build time and is otherwise
 * identical).
 *
 * The three flags are `(unifyEdges, unifyFaces, concatBSplines)` — the
 * combination the roadmap's own probe verified; B-spline concatenation is left
 * off because these shapes carry none.
 *
 * Three guards, all cheap: the volume bound below (same 1e-6 relative form
 * `unionSound`/`cutSound` use, so a pass that silently dropped or added material
 * is rejected and the un-merged result is kept), `IsNull`, and a `try`/`catch` —
 * the pass is a courtesy, never a correctness dependency.
 *
 * **Only above `UNIFY_MIN_FACES`.** Below that the pass costs more than it saves
 * and it would churn the face decomposition of small models for no gain —
 * bracket.csg's 30-face result is exactly that case (39ms to merge 30 faces into
 * 26), and `npm run mcp:smoke` pins that count.
 */
const UNIFY_MIN_FACES = 256;

function unifyFragmented(
  oc: Oc,
  shape: Shape,
  cleanup: Cleanup,
  warn: (m: string) => void,
  label: string,
): Shape {
  let faces = 0;
  try {
    faces = countFaces(oc, shape);
  } catch {
    return shape;
  }
  if (faces < UNIFY_MIN_FACES) return shape;
  const before = shapeVolume(oc, shape, cleanup);
  let merged: Shape | null = null;
  try {
    const up = new oc.ShapeUpgrade_UnifySameDomain_2(shape, true, true, false);
    try {
      up.Build();
      merged = up.Shape();
    } finally {
      up.delete();
    }
  } catch (e) {
    warn(`${label} — same-domain cleanup threw (${e instanceof Error ? e.message : String(e)}) — keeping the un-merged result`);
    return shape;
  }
  if (!merged || isNullShape(merged)) return shape;
  cleanup.push(merged);
  const after = shapeVolume(oc, merged, cleanup);
  if (!Number.isFinite(before) || !Number.isFinite(after) || Math.abs(after - before) > Math.abs(before) * 1e-6 + 1e-9) {
    warn(
      `${label} — same-domain cleanup changed the volume (${before} → ${after}) — keeping the un-merged result`,
    );
    return shape;
  }
  return merged;
}

/** Swaps in `unifyFragmented`'s result while keeping the "did WE build this
 * handle, and may we free it" bookkeeping straight — a merged shape is a new
 * handle, so the un-merged one is only released when this walk owned it (never
 * when it came from a child, or from the caller-supplied operand list). */
function replaced(
  oc: Oc,
  acc: Shape,
  accOwned: boolean,
  cleanup: Cleanup,
  warn: (m: string) => void,
  label: string,
): Shape {
  const merged = unifyFragmented(oc, acc, cleanup, warn, label);
  if (merged === acc) return acc;
  if (accOwned) release(cleanup, acc);
  return merged;
}

/**
 * Evaluates one child subtree, containing a fault to that child alone.
 *
 * A `.csg` walk is deep and many operands wide, so a single bad operand must not
 * cost the whole model. That is a real, measured failure here rather than a
 * theoretical one: this OCCT build surfaces a C++ exception raised inside a
 * boolean as a plain JS `ReferenceError` (`___cxa_can_catch is not defined` —
 * the Emscripten exception runtime is absent), and before this containment one
 * such throw unwound all the way to `buildCsgShape`'s per-root `catch`, which
 * replaced the ENTIRE part with an empty compound. Measured on the moulded
 * enclosure: the bottom tray's eighth cut threw, `props.Mass()` came back `0`,
 * and an 11.5 cm³ part vanished from the render — leaving a view showing only
 * six loose fragments. The module survives the throw (verified: the same
 * process went on to construct, measure and tessellate afterwards), so skipping
 * just the offending subtree is both safe and strictly better.
 */
function tryShapes(
  oc: Oc, child: CsgNode, cleanup: Cleanup, warn: (m: string) => void, useMaxFN: number,
): Shape[] {
  try {
    return evalShapes(oc, child, cleanup, warn, useMaxFN);
  } catch (e) {
    warn(`${child.name}() subtree threw (${e instanceof Error ? e.message : String(e)}) — skipping`);
    return [];
  }
}

function evalShapes(oc: Oc, node: CsgNode, cleanup: Cleanup, warn: (m: string) => void, useMaxFN: number): Shape[] {
  if (node.modifier === "*") { warn(`disabled (*${node.name}) subtree — skipping`); return []; }
  if (node.modifier === "!") warn(`show-only (!${node.name}) treated as transparent — siblings are NOT hidden on import`);

  const name = node.name;
  if (name === "group" || name === "color" || name === "render") {
    // transparent containers (color carries no geometry; render is a no-op)
    return node.children.flatMap((c) => tryShapes(oc, c, cleanup, warn, useMaxFN));
  }
  if (name === "union" || name === "intersection") {
    // Note: the .csg verb is `intersection`, the boolean kind is `intersect`.
    const kind = name === "union" ? "union" : "intersect";
    const all = node.children.flatMap((c) => tryShapes(oc, c, cleanup, warn, useMaxFN));
    if (all.length === 0) { warn(`${name}() — every child was skipped, nothing to combine`); return []; }
    if (all.length === 1) return all;
    // One call over the whole operand list, before falling back to the fold —
    // see `multiUnion`. `intersection` has no multi-argument counterpart.
    if (kind === "union") {
      let fast: Shape | null = null;
      try {
        fast = multiUnion(oc, all, cleanup);
      } catch (e) {
        warn(`union() — the single-call form threw (${e instanceof Error ? e.message : String(e)}) — combining pairwise`);
        fast = null;
      }
      if (fast) {
        if (unionSound(oc, all, fast, cleanup)) return [replaced(oc, fast, true, cleanup, warn, `${name}()`)];
        warn(`union() — the single-call form failed its volume bounds — combining pairwise`);
      }
    }
    let acc = all[0];
    let accOwned = false;
    let dropped = 0;
    for (const s of all.slice(1)) {
      // Guard the UNION only: its result must contain at least the larger
      // operand, so a smaller result is proof of a silent loss (see
      // `unionLostGeometry`). An intersection has no such bound.
      let r: Shape | null = null;
      let lost = false;
      try {
        r = booleanOf(oc, kind, acc, s, cleanup);
        lost = kind === "union" && r !== null && unionLostGeometry(oc, acc, s, r, cleanup);
      } catch (e) {
        warn(`${name}() — an operand threw (${e instanceof Error ? e.message : String(e)}) — leaving it out`);
        r = null;
      }
      if (!r || lost) {
        // Leave THIS operand out and keep fusing the rest. The previous
        // behaviour — bailing out with every operand uncombined — handed the
        // caller a compound of heavily overlapping solids for the next boolean
        // to consume, which is the very "members interfere with EACH OTHER"
        // hazard the `difference` fold below exists to avoid; it is also
        // measurably worse here (11771 uncombined against an oracle of 11571,
        // versus 11418 from the accumulator at the point the fuse failed).
        dropped++;
        continue;
      }
      // `acc` is superseded. Only a result WE built may be freed — `all[0]` is
      // still an element of the operand list the failure path returns intact.
      if (accOwned) release(cleanup, acc);
      acc = r;
      accOwned = true;
    }
    if (dropped > 0) {
      warn(
        `${name}() — ${dropped} of ${all.length} operand(s) could not be combined and were left out of the result`,
      );
    }
    if (!accOwned && all.length > 1) {
      // Nothing combined at all — hand the operands back rather than an
      // arbitrary single member.
      warn(`${name}() — no operand could be combined; keeping ${all.length} operand(s) uncombined`);
      return all;
    }
    return [replaced(oc, acc, accOwned, cleanup, warn, `${name}()`)];
  }
  if (name === "difference") {
    if (node.children.length === 0) { warn(`difference() with no children — skipping`); return []; }
    const first = tryShapes(oc, node.children[0], cleanup, warn, useMaxFN);
    const rest = node.children.slice(1).flatMap((c) => tryShapes(oc, c, cleanup, warn, useMaxFN));
    if (first.length === 0) {
      if (rest.length > 0) warn(`difference() — minuend was skipped, dropping ${rest.length} subtrahend solid(s)`);
      return [];
    }
    if (rest.length === 0) return first;
    let acc = first.length === 1 ? first[0] : compoundOf(oc, first, cleanup);
    let accOwned = first.length > 1;
    // One cut with every subtrahend as a separate tool, before the one-at-a-time
    // fold — see `multiCut`.
    if (rest.length > 1) {
      let fast: Shape | null = null;
      try {
        fast = multiCut(oc, acc, rest, cleanup);
      } catch (e) {
        warn(`difference() — the single-call form threw (${e instanceof Error ? e.message : String(e)}) — cutting one at a time`);
        fast = null;
      }
      if (fast) {
        if (cutSound(oc, acc, fast, cleanup)) return [replaced(oc, fast, true, cleanup, warn, "difference()")];
        warn(`difference() — the single-call form failed its volume bound — cutting one at a time`);
      }
    }
    // Cut ONE subtrahend at a time, never as a single compound.
    // `difference(a, b1..bn)` IS `a - b1 - ... - bn`, and handing OCCT one
    // compound whose members interfere with EACH OTHER makes its BOP return a
    // wrong result that still reports success — measured on the top cover: an
    // 18-tool compound cut returned 134.89 mm³ of subtrahend geometry where
    // OpenSCAD's render of the same subtree is 13001.15 (the same 18 tools
    // union correctly, so the tool set itself is sound). Cutting singly gives
    // every boolean two well-formed single-solid operands, and makes one
    // subtrahend's failure local instead of discarding the whole difference.
    let failed = 0;
    for (const tool of rest) {
      // One cut throwing must not cost the cuts that already succeeded.
      let r: Shape | null = null;
      try {
        r = booleanOf(oc, "subtract", acc, tool, cleanup);
      } catch (e) {
        warn(
          `difference() — a cut threw (${e instanceof Error ? e.message : String(e)}) — leaving that subtrahend uncut`,
        );
        r = null;
      }
      if (!r) { failed++; continue; }
      // The tool has been applied and is never read again; the old accumulator
      // is superseded. Releasing both keeps the live-shape count flat across
      // what is otherwise N cuts on an ever-growing result.
      release(cleanup, tool);
      if (accOwned) release(cleanup, acc);
      acc = r;
      accOwned = true;
    }
    if (failed > 0) {
      warn(
        `difference() — ${failed} of ${rest.length} cut(s) did not complete; those subtrahends were left uncut`,
      );
    }
    return [replaced(oc, acc, accOwned, cleanup, warn, "difference()")];
  }
  if (name === "multmatrix" || name === "translate" || name === "scale" || name === "mirror" || name === "rotate") {
    const m = nodeMatrix(node, warn);
    const kids = node.children.flatMap((c) => evalShapes(oc, c, cleanup, warn, useMaxFN));
    if (kids.length === 0) return [];
    if (!m) {
      // nodeMatrix already warned. rotate() with bad params keeps children
      // unrotated (documented); every other failure drops them (documented).
      if (name === "rotate") return kids;
      return [];
    }
    return applyMatrix(oc, kids, m, cleanup, warn) ?? [];
  }
  if (name === "cube") return single(oc, buildBox(oc, node, cleanup, warn));
  if (name === "sphere") return single(oc, buildSphere(oc, node, cleanup, warn, useMaxFN));
  if (name === "cylinder") return single(oc, buildCylinder(oc, node, cleanup, warn, useMaxFN));
  if (name === "polyhedron") return single(oc, buildPolyhedron(oc, node, cleanup, warn));
  if (name === "hull") return single(oc, buildHull(oc, node, cleanup, warn, useMaxFN));
  if (name === "linear_extrude") return single(oc, buildLinearExtrude(oc, node, cleanup, warn, useMaxFN));
  if (name === "rotate_extrude") return single(oc, buildRotateExtrude(oc, node, cleanup, warn, useMaxFN));

  const skip = SKIP_SUBTREE_MSGS[name];
  if (skip) { warn(`${name}(): ${skip}`); return []; }
  warn(`unknown statement ${name}() — skipping`);
  return [];
}

function single(_oc: Oc, s: Shape | null): Shape[] {
  void _oc;
  return s ? [s] : [];
}

/**
 * Builds the `.csg` base shape: one compound of every root's solids (or a
 * single solid, or an empty compound when nothing built — the blank-model
 * empty-shape guard downstream handles that, same as an empty `.brep`).
 * Parser-level warnings pass through; kernel-level ones append. Never
 * throws on bad geometry (only on a dead kernel) — an empty result with
 * warnings beats a failed open.
 */
export function buildCsgShape(
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  oc: any,
  roots: CsgNode[],
  parserWarnings: string[],
  useMaxFN: number,
  cleanup: Cleanup,
  warnings: string[],
): Shape {
  warnings.push(...parserWarnings);
  const warn = (m: string): void => { warnings.push(m); };
  const all: Shape[] = [];
  for (const r of roots) {
    try {
      all.push(...evalShapes(oc, r, cleanup, warn, useMaxFN));
    } catch (e) {
      warn(`${r.name}() subtree threw (${e instanceof Error ? e.message : String(e)}) — skipping`);
    }
  }
  if (all.length === 0) return emptyCompound(oc, cleanup);
  if (all.length === 1) return all[0];
  return compoundOf(oc, all, cleanup);
}
