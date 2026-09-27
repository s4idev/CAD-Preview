/**
 * 3-D convex hull — the missing half of OpenSCAD's `hull()`.
 *
 * `hull()` is not a transform: it REPLACES its children with their convex
 * hull, so a `.csg` importer cannot pass the children through instead (the
 * geometry would be confidently wrong, not merely approximate). OCCT ships no
 * convex-hull algorithm, so the hull is computed here, in pure TypeScript, and
 * `csgModel.ts` turns the resulting facets into a real `TopoDS_Solid` through
 * the same sew-and-solidify path `polyhedron` already uses.
 *
 * Pure by design (no OCCT/THREE/DOM/`vscode`) so it unit-tests headlessly —
 * the convention every other geometry module in this repo follows.
 *
 * Algorithm: incremental (a.k.a. beneath-beyond) hull, seeded from a
 * tetrahedron of four well-separated points and grown one point at a time.
 * Chosen over quickhull because the state it maintains is a plain face list
 * with no recursion and no horizon-walk bookkeeping beyond a single edge
 * count, and because its worst case on the point counts this actually sees
 * (`hull()` over a handful of tessellated primitives — hundreds to a few
 * thousand points) is irrelevant next to the OCCT sewing that follows.
 *
 * Correctness decisions:
 * - **Orientation is decided by ONE interior point**, never by edge order. A
 *   fixed point inside the seed tetrahedron stays inside the hull for the
 *   whole construction, so "outward" is just `dot(normal, faceCentre −
 *   interior) > 0`. That is what makes the horizon's winding irrelevant: a
 *   new face is built in whichever order the horizon edge was recorded and
 *   then flipped if needed. Getting this wrong would produce a self-consistent
 *   *inward* solid — which still sews closed, and is why the caller's
 *   `orientPositive` volume check is a backstop rather than the primary guard.
 * - **Tolerances are relative to the point set's own bbox diagonal**, never an
 *   absolute constant: `hull()` inputs range from a sub-millimetre moulding
 *   detail to a metre-scale frame, and this codebase's tessellated coordinates
 *   are `Float32` (precision tracks coordinate MAGNITUDE, not model size — the
 *   same finding the hidden-line and primitive-fitting work recorded).
 * - **Degenerate input returns `null`, never a guess.** Fewer than four
 *   points, all-coincident, collinear and coplanar sets are all reported as
 *   "no 3-D hull" for the caller to warn about and skip. OpenSCAD's own
 *   `hull()` over 2-D children is a 2-D hull, which this module deliberately
 *   does not attempt — see `csgModel.ts`'s caller for the stated gap.
 */

/** A 3-D point, matching this repo's `Vec3` tuple convention. */
export type Vec3 = [number, number, number];

export interface ConvexHull {
  /** Hull vertices only — interior input points are dropped. */
  points: Vec3[];
  /** Outward-oriented triangles, indices into `points`. */
  facets: Array<[number, number, number]>;
}

export interface ConvexHullOptions {
  /**
   * Tolerance as a FRACTION of the point set's bbox diagonal (default `1e-9`).
   * Point dedup, the seed-tetrahedron separation tests and face visibility all
   * use it, so the whole module scales with the input.
   */
  toleranceFrac?: number;
}

interface Face {
  a: number;
  b: number;
  c: number;
  /** Unit normal. */
  nx: number;
  ny: number;
  nz: number;
}

const DEFAULT_TOLERANCE_FRAC = 1e-9;

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function length(a: Vec3): number {
  return Math.sqrt(dot(a, a));
}

/**
 * Builds a face with a unit normal, or `null` when the three points are
 * (near-)collinear — a zero-area triangle cannot be oriented, and sewing one
 * into a shell would leave a free edge.
 */
function makeFace(points: readonly Vec3[], a: number, b: number, c: number, tol: number): Face | null {
  const n = cross(sub(points[b], points[a]), sub(points[c], points[a]));
  const len = length(n);
  // Compare the cross product's magnitude against the triangle's own longest
  // edge, so the test is scale-free: |n| has units of area, and a degenerate
  // sliver is degenerate regardless of how large the coordinates are.
  const longest = Math.max(
    length(sub(points[b], points[a])),
    length(sub(points[c], points[b])),
    length(sub(points[a], points[c])),
  );
  if (len === 0 || len <= tol * longest) return null;
  return { a, b, c, nx: n[0] / len, ny: n[1] / len, nz: n[2] / len };
}

/** Signed distance from `p` to the face's plane (positive on the normal side). */
function signedDistance(p: Vec3, face: Face, points: readonly Vec3[]): number {
  const v = sub(p, points[face.a]);
  return v[0] * face.nx + v[1] * face.ny + v[2] * face.nz;
}

/**
 * Flips a face's winding so its normal points AWAY from `interior`.
 *
 * The interior point sits on the NEGATIVE side of an outward-facing face, so a
 * POSITIVE signed distance is what means "currently wound inward" — getting
 * this backwards inverts the whole hull (and, because the growth step only
 * expands toward points it is strictly in front of, silently freezes it at the
 * seed tetrahedron).
 */
function orientOutward(face: Face, interior: Vec3, points: readonly Vec3[]): Face {
  if (signedDistance(interior, face, points) > 0) {
    return { a: face.a, b: face.c, c: face.b, nx: -face.nx, ny: -face.ny, nz: -face.nz };
  }
  return face;
}

/**
 * The convex hull of `input`, or `null` when the points do not span a volume.
 */
export function convexHull3d(input: readonly Vec3[], options: ConvexHullOptions = {}): ConvexHull | null {
  const frac = options.toleranceFrac ?? DEFAULT_TOLERANCE_FRAC;

  // 1. Finite points only. A NaN from a bad transform would otherwise poison
  //    every comparison downstream (`NaN > tol` is false), silently producing
  //    an empty hull instead of a diagnosable one.
  const finite = input.filter((p) => Number.isFinite(p[0]) && Number.isFinite(p[1]) && Number.isFinite(p[2]));
  if (finite.length < 4) return null;

  // 2. Scale from the bbox diagonal — every tolerance below is relative to it.
  const min: Vec3 = [Infinity, Infinity, Infinity];
  const max: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (const p of finite) {
    for (let k = 0; k < 3; k++) {
      if (p[k] < min[k]) min[k] = p[k];
      if (p[k] > max[k]) max[k] = p[k];
    }
  }
  const diag = length(sub(max, min));
  if (!(diag > 0)) return null;
  const tol = diag * frac;

  // 3. Deduplicate, else a repeated point drifts the seed search (the "farthest
  //    from" probes below would keep selecting the same duplicate) and inflates
  //    the face list with zero-area triangles.
  const seen = new Set<string>();
  const pts: Vec3[] = [];
  const q = Math.max(tol, Number.MIN_VALUE);
  for (const p of finite) {
    const key = `${Math.round(p[0] / q)},${Math.round(p[1] / q)},${Math.round(p[2] / q)}`;
    if (seen.has(key)) continue;
    seen.add(key);
    pts.push([p[0], p[1], p[2]]);
  }
  if (pts.length < 4) return null;

  // 4. Seed tetrahedron: four well-separated points. Each step maximises the
  //    distance to the feature built from the previous ones, which is the
  //    standard way to avoid picking a degenerate (collinear/coplanar) seed.
  const i0 = 0;
  let i1 = -1;
  let best = tol;
  for (let i = 1; i < pts.length; i++) {
    const d = length(sub(pts[i], pts[i0]));
    if (d > best) { best = d; i1 = i; }
  }
  if (i1 < 0) return null; // every point coincides with pts[0]

  const lineDir = sub(pts[i1], pts[i0]);
  let i2 = -1;
  best = tol;
  for (let i = 0; i < pts.length; i++) {
    if (i === i0 || i === i1) continue;
    const d = length(cross(sub(pts[i], pts[i0]), lineDir)) / length(lineDir);
    if (d > best) { best = d; i2 = i; }
  }
  if (i2 < 0) return null; // collinear

  const planeNormal = cross(sub(pts[i1], pts[i0]), sub(pts[i2], pts[i0]));
  const planeLen = length(planeNormal);
  let i3 = -1;
  best = tol;
  for (let i = 0; i < pts.length; i++) {
    if (i === i0 || i === i1 || i === i2) continue;
    const d = Math.abs(dot(sub(pts[i], pts[i0]), planeNormal)) / planeLen;
    if (d > best) { best = d; i3 = i; }
  }
  if (i3 < 0) return null; // coplanar

  // 5. A point strictly inside the seed tetrahedron stays inside the growing
  //    hull forever, so it can orient every face built from here on.
  const interior: Vec3 = [
    (pts[i0][0] + pts[i1][0] + pts[i2][0] + pts[i3][0]) / 4,
    (pts[i0][1] + pts[i1][1] + pts[i2][1] + pts[i3][1]) / 4,
    (pts[i0][2] + pts[i1][2] + pts[i2][2] + pts[i3][2]) / 4,
  ];

  // 6. The four seed faces, each oriented outward.
  let faces: Face[] = [];
  for (const [a, b, c] of [
    [i0, i1, i2],
    [i0, i1, i3],
    [i0, i2, i3],
    [i1, i2, i3],
  ] as Array<[number, number, number]>) {
    const f = makeFace(pts, a, b, c, tol);
    if (f) faces.push(orientOutward(f, interior, pts));
  }
  if (faces.length < 4) return null; // shouldn't happen after the seed tests

  const isSeed = new Set([i0, i1, i2, i3]);

  // 7. Grow with the remaining points.
  for (let pi = 0; pi < pts.length; pi++) {
    if (isSeed.has(pi)) continue;
    const p = pts[pi];

    // Visible faces: those the point is strictly in front of.
    const visible: boolean[] = faces.map((f) => signedDistance(p, f, pts) > tol);
    if (!visible.some(Boolean)) continue; // inside or on the hull so far

    // Horizon: edges belonging to exactly ONE visible face. `Map` keyed by the
    // UNDIRECTED pair, so this survives whichever way the visible faces happen
    // to wind — the new faces are oriented by the interior point anyway.
    const edgeCount = new Map<string, [number, number]>();
    const counts = new Map<string, number>();
    for (let fi = 0; fi < faces.length; fi++) {
      if (!visible[fi]) continue;
      const f = faces[fi];
      for (const [a, b] of [
        [f.a, f.b],
        [f.b, f.c],
        [f.c, f.a],
      ] as Array<[number, number]>) {
        const key = a < b ? `${a}_${b}` : `${b}_${a}`;
        counts.set(key, (counts.get(key) ?? 0) + 1);
        edgeCount.set(key, [a, b]);
      }
    }

    const next: Face[] = faces.filter((_, fi) => !visible[fi]);
    for (const [key, count] of counts) {
      if (count !== 1) continue; // shared by two visible faces — interior
      const [a, b] = edgeCount.get(key)!;
      const f = makeFace(pts, a, b, pi, tol);
      if (f) next.push(orientOutward(f, interior, pts));
    }
    // A horizon that closed nothing means the point lay exactly on an edge;
    // keep the previous hull rather than emitting an open shell.
    if (next.length >= 4) faces = next;
  }

  // 8. Compact to the vertices actually used, so the caller allocates one OCCT
  //    point handle per real hull vertex and not one per interior input point.
  const used = new Map<number, number>();
  const outPoints: Vec3[] = [];
  const facets: Array<[number, number, number]> = [];
  for (const f of faces) {
    const remap = (i: number): number => {
      let r = used.get(i);
      if (r === undefined) {
        r = outPoints.length;
        used.set(i, r);
        outPoints.push(pts[i]);
      }
      return r;
    };
    facets.push([remap(f.a), remap(f.b), remap(f.c)]);
  }
  if (outPoints.length < 4 || facets.length < 4) return null;
  return { points: outPoints, facets };
}
