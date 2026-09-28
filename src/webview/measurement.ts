/**
 * Pure measurement math: distance, polyline length, angle, and circle radius
 * over plain `[x,y,z]` tuples / flat coordinate arrays — no DOM, no Three.js
 * types, so this unit-tests headless (same convention as `picking.ts`/
 * `selection.ts`). Client-side triangulated-approximation precision is a
 * deliberate scope boundary: entity-to-entity distance/angle work from the
 * already-tessellated positions transmitted to the webview (tied to the
 * existing 0.1 tessellation deflection tolerance, `meshExtract.ts`), never an
 * exact `BRepExtrema_DistShapeShape` host round trip.
 */

import type { ExactMeasureResult } from "../entityFacts";

export type Vec3 = [number, number, number];

export function pointDistance(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** Total length of a polyline given as a flat `[x0,y0,z0, x1,y1,z1, …]` array. */
export function polylineLength(points: ArrayLike<number>): number {
  let total = 0;
  for (let i = 0; i + 5 < points.length; i += 3) {
    total += Math.hypot(points[i + 3] - points[i], points[i + 4] - points[i + 1], points[i + 5] - points[i + 2]);
  }
  return total;
}

/** Angle between two direction vectors, in degrees (matches every other angle field in this codebase). */
export function angleBetweenVectors(a: Vec3, b: Vec3): number {
  const la = Math.hypot(a[0], a[1], a[2]);
  const lb = Math.hypot(b[0], b[1], b[2]);
  if (la === 0 || lb === 0) return NaN;
  const dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cos = Math.min(1, Math.max(-1, dot / (la * lb)));
  return (Math.acos(cos) * 180) / Math.PI;
}

/**
 * Circumradius of the circle through three points, or `null` if they're
 * (near-)collinear (no well-defined circle). Used to estimate a circular/arc
 * edge's radius from three samples of its already-transmitted polyline.
 */
export function circleRadiusFromArcPoints(p0: Vec3, p1: Vec3, p2: Vec3): number | null {
  const a = pointDistance(p1, p2);
  const b = pointDistance(p0, p2);
  const c = pointDistance(p0, p1);

  const v1: Vec3 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
  const v2: Vec3 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
  const cross: Vec3 = [
    v1[1] * v2[2] - v1[2] * v2[1],
    v1[2] * v2[0] - v1[0] * v2[2],
    v1[0] * v2[1] - v1[1] * v2[0],
  ];
  const twiceArea = Math.hypot(cross[0], cross[1], cross[2]);
  if (twiceArea < 1e-9) return null;
  return (a * b * c) / (2 * twiceArea);
}

/**
 * One line's worth of presentation for an exact measurement: the readout's own
 * `D_exact = …` shape, split into its parts so the interesting decisions are
 * pure and testable rather than buried in a DOM handler.
 *
 * `extras` carries the additive facts `measureExact` publishes beside the
 * primary value — the perpendicular plane-to-plane gap, the bbox-centre
 * distance, the infinite-axis separation, the normal angle, and (for the angle
 * kind) the orientation-independent line angle. Each is shown ONLY when it
 * tells the reader something the primary value does not: an extra that formats
 * to the same string as the value is dropped, which is what keeps a plain
 * parallel-face measurement from reading `12.5 mm · parallel 12.5 mm`.
 *
 * The angle kind's `lineAngleDeg` uses the same rule, so it appears exactly
 * when the raw direction angle is obtuse (e.g. 180° for two parallel faces
 * whose stored normals happen to be antiparallel, where `0°` is the reading a
 * user wants) and stays hidden when the raw value is already the intuitive one.
 * Unit prefixes stay out of here: the caller passes the formatters, since
 * display units are webview state.
 */
export function exactReadout(
  result: ExactMeasureResult,
  formatLength: (mm: number) => string,
  formatDegrees: (deg: number) => string
): { label: string; value: string; extras: string[] } {
  const label =
    result.kind === "distance"
      ? "D"
      : result.kind === "edgeLength"
        ? "L"
        : result.kind === "radius"
          ? "R"
          : "A";
  const primary = result.kind === "angle" ? formatDegrees(result.value) : formatLength(result.value);

  const candidates: string[] = [];
  if (result.kind === "distance") {
    if (result.primary === "parallel" && result.parallelDistance !== undefined) {
      candidates.push(`parallel ${formatLength(result.parallelDistance)}`);
    }
    if (result.centreDistance !== undefined) candidates.push(`centre ${formatLength(result.centreDistance)}`);
    if (result.axisDistance !== undefined) candidates.push(`axis ${formatLength(result.axisDistance)}`);
    if (result.angleDeg !== undefined) candidates.push(`angle ${formatDegrees(result.angleDeg)}`);
  } else if (result.kind === "angle" && result.lineAngleDeg !== undefined) {
    candidates.push(`line ${formatDegrees(result.lineAngleDeg)}`);
  }

  // Drop an extra that merely repeats the primary value (`endsWith` covers the
  // prefixed forms: "parallel 12.5 mm" ends with the value "12.5 mm").
  return { label, value: primary, extras: candidates.filter((c) => !c.endsWith(primary)) };
}
