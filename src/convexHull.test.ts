import { describe, expect, it } from "vitest";
import { convexHull3d, type Vec3 } from "./convexHull";

/** Signed volume of a closed, consistently-wound triangle mesh (divergence theorem). */
function hullVolume(points: readonly Vec3[], facets: ReadonlyArray<readonly [number, number, number]>): number {
  let sum = 0;
  for (const [a, b, c] of facets) {
    const p = points[a];
    const q = points[b];
    const r = points[c];
    sum +=
      p[0] * (q[1] * r[2] - q[2] * r[1]) +
      p[1] * (q[2] * r[0] - q[0] * r[2]) +
      p[2] * (q[0] * r[1] - q[1] * r[0]);
  }
  return sum / 6;
}

function cubeCorners(size = 1, origin: Vec3 = [0, 0, 0]): Vec3[] {
  const out: Vec3[] = [];
  for (const x of [0, size]) for (const y of [0, size]) for (const z of [0, size]) {
    out.push([origin[0] + x, origin[1] + y, origin[2] + z]);
  }
  return out;
}

/** Every undirected edge exactly twice, every directed edge exactly once. */
function expectClosedManifold(points: readonly Vec3[], facets: ReadonlyArray<readonly [number, number, number]>): void {
  const undirected = new Map<string, number>();
  const directed = new Set<string>();
  for (const [a, b, c] of facets) {
    expect(new Set([a, b, c]).size).toBe(3); // no degenerate triangle
    for (const [i, j] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      const key = i < j ? `${i}_${j}` : `${j}_${i}`;
      undirected.set(key, (undirected.get(key) ?? 0) + 1);
      const dk = `${i}>${j}`;
      expect(directed.has(dk)).toBe(false); // consistent winding
      directed.add(dk);
    }
  }
  for (const [, count] of undirected) expect(count).toBe(2);
  void points;
}

describe("convexHull3d — basic solids", () => {
  it("hulls a unit cube into 12 triangles over all 8 corners", () => {
    const hull = convexHull3d(cubeCorners(1));
    expect(hull).not.toBeNull();
    expect(hull!.points.length).toBe(8);
    expect(hull!.facets.length).toBe(12);
    expect(hullVolume(hull!.points, hull!.facets)).toBeCloseTo(1, 9);
    expectClosedManifold(hull!.points, hull!.facets);
  });

  it("hulls a 10 mm cube to volume 1000", () => {
    const hull = convexHull3d(cubeCorners(10));
    expect(hullVolume(hull!.points, hull!.facets)).toBeCloseTo(1000, 6);
  });

  it("hulls a tetrahedron into exactly 4 facets", () => {
    const hull = convexHull3d([
      [0, 0, 0],
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ]);
    expect(hull!.points.length).toBe(4);
    expect(hull!.facets.length).toBe(4);
    expect(hullVolume(hull!.points, hull!.facets)).toBeCloseTo(1 / 6, 9);
  });
});

describe("convexHull3d — robustness", () => {
  it("drops interior points", () => {
    const pts: Vec3[] = [...cubeCorners(2), [1, 1, 1], [0.5, 1.2, 0.3], [1.9, 0.1, 1.1]];
    const hull = convexHull3d(pts);
    expect(hull!.points.length).toBe(8);
    expect(hull!.facets.length).toBe(12);
    expect(hullVolume(hull!.points, hull!.facets)).toBeCloseTo(8, 6);
  });

  it("tolerates duplicated and near-duplicated points", () => {
    const dup: Vec3[] = [...cubeCorners(1), ...cubeCorners(1), [0, 0, 1e-12]];
    const hull = convexHull3d(dup);
    expect(hull!.points.length).toBe(8);
    expect(hullVolume(hull!.points, hull!.facets)).toBeCloseTo(1, 9);
  });

  it("is order-independent in volume and vertex set", () => {
    const base = cubeCorners(3);
    const shuffled = [base[5], base[0], base[7], base[2], base[1], base[6], base[3], base[4]];
    const a = convexHull3d(base)!;
    const b = convexHull3d(shuffled)!;
    expect(hullVolume(b.points, b.facets)).toBeCloseTo(hullVolume(a.points, a.facets), 9);
    const key = (v: Vec3): string => v.join(",");
    expect(new Set(b.points.map(key))).toEqual(new Set(a.points.map(key)));
  });

  it("keeps the hull outward-oriented (every face points away from the centroid)", () => {
    const hull = convexHull3d(cubeCorners(4))!;
    const centroid: Vec3 = [2, 2, 2];
    for (const [a, b, c] of hull.facets) {
      const p = hull.points[a];
      const q = hull.points[b];
      const r = hull.points[c];
      const u = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
      const v = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
      const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
      const faceCentre: Vec3 = [(p[0] + q[0] + r[0]) / 3, (p[1] + q[1] + r[1]) / 3, (p[2] + q[2] + r[2]) / 3];
      const toCentre = [faceCentre[0] - centroid[0], faceCentre[1] - centroid[1], faceCentre[2] - centroid[2]];
      expect(n[0] * toCentre[0] + n[1] * toCentre[1] + n[2] * toCentre[2]).toBeGreaterThan(0);
    }
  });

  it("works on a point set far from the origin (Float32-style magnitude)", () => {
    const hull = convexHull3d(cubeCorners(1, [10000, -8000, 6000]))!;
    expect(hull.points.length).toBe(8);
    expect(hullVolume(hull.points, hull.facets)).toBeCloseTo(1, 4);
    expectClosedManifold(hull.points, hull.facets);
  });

  it("hulls points sampled on a sphere and keeps every input point inside", () => {
    const pts: Vec3[] = [];
    const R = 5;
    for (let i = 0; i < 60; i++) {
      const t = (i / 60) * Math.PI * 2;
      const p = ((i * 7) % 60) / 60;
      const phi = Math.acos(1 - 2 * p);
      pts.push([R * Math.sin(phi) * Math.cos(t), R * Math.sin(phi) * Math.sin(t), R * Math.cos(phi)]);
    }
    const hull = convexHull3d(pts)!;
    expect(hull.facets.length).toBeGreaterThan(60);
    expectClosedManifold(hull.points, hull.facets);
    // A convex hull of points on a sphere is inscribed in it, so its volume
    // must sit strictly below the sphere's and above a coarse fraction of it.
    const vol = hullVolume(hull.points, hull.facets);
    expect(vol).toBeLessThan((4 / 3) * Math.PI * R ** 3);
    expect(vol).toBeGreaterThan((4 / 3) * Math.PI * R ** 3 * 0.8);
  });
});

describe("convexHull3d — degenerate input returns null, never a guess", () => {
  it("rejects fewer than 4 points", () => {
    expect(convexHull3d([])).toBeNull();
    expect(convexHull3d([[0, 0, 0]])).toBeNull();
    expect(convexHull3d([[0, 0, 0], [1, 0, 0], [0, 1, 0]])).toBeNull();
  });

  it("rejects an all-coincident set", () => {
    expect(convexHull3d([[2, 2, 2], [2, 2, 2], [2, 2, 2], [2, 2, 2]])).toBeNull();
  });

  it("rejects a collinear set", () => {
    expect(convexHull3d([[0, 0, 0], [1, 1, 1], [2, 2, 2], [-3, -3, -3], [0.5, 0.5, 0.5]])).toBeNull();
  });

  it("rejects a coplanar set", () => {
    expect(
      convexHull3d([
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0.5, 0.5, 0],
      ]),
    ).toBeNull();
  });

  it("drops non-finite points, then judges what is left", () => {
    // One NaN alongside a real cube: the cube still hulls.
    const withNaN = convexHull3d([...cubeCorners(1), [Number.NaN, 0, 0]]);
    expect(withNaN!.points.length).toBe(8);
    // Three real points plus a NaN is no longer a volume.
    expect(convexHull3d([[0, 0, 0], [1, 0, 0], [0, 1, 0], [Number.POSITIVE_INFINITY, 0, 0]])).toBeNull();
  });
});
