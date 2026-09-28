import { describe, it, expect } from "vitest";
import { pointDistance, polylineLength, angleBetweenVectors, circleRadiusFromArcPoints, exactReadout } from "./measurement";

describe("pointDistance", () => {
  it("computes Euclidean distance", () => {
    expect(pointDistance([0, 0, 0], [3, 4, 0])).toBe(5);
    expect(pointDistance([1, 1, 1], [1, 1, 1])).toBe(0);
  });
});

describe("polylineLength", () => {
  it("sums segment lengths along a flat coordinate array", () => {
    // (0,0,0) -> (3,0,0) -> (3,4,0): 3 + 4 = 7
    expect(polylineLength([0, 0, 0, 3, 0, 0, 3, 4, 0])).toBe(7);
  });

  it("returns 0 for a single point or empty array", () => {
    expect(polylineLength([1, 2, 3])).toBe(0);
    expect(polylineLength([])).toBe(0);
  });
});

describe("angleBetweenVectors", () => {
  it("returns 90 for perpendicular vectors", () => {
    expect(angleBetweenVectors([1, 0, 0], [0, 1, 0])).toBeCloseTo(90, 6);
  });

  it("returns 0 for parallel vectors", () => {
    expect(angleBetweenVectors([2, 0, 0], [5, 0, 0])).toBeCloseTo(0, 6);
  });

  it("returns 180 for opposite vectors", () => {
    expect(angleBetweenVectors([1, 0, 0], [-1, 0, 0])).toBeCloseTo(180, 6);
  });

  it("returns NaN for a zero-length vector", () => {
    expect(angleBetweenVectors([0, 0, 0], [1, 0, 0])).toBeNaN();
  });
});

describe("exactReadout", () => {
  const mm = (v: number): string => `${Number(v.toPrecision(5))} mm`;
  const deg = (v: number): string => `${Number(v.toPrecision(5))}°`;

  it("labels each kind and formats lengths through the length formatter", () => {
    expect(exactReadout({ kind: "distance", value: 12.5 }, mm, deg)).toEqual({
      label: "D",
      value: "12.5 mm",
      extras: [],
    });
    expect(exactReadout({ kind: "edgeLength", value: 40 }, mm, deg).label).toBe("L");
    expect(exactReadout({ kind: "radius", value: 3 }, mm, deg)).toEqual({
      label: "R",
      value: "3 mm",
      extras: [],
    });
  });

  it("formats an angle in degrees via the degree formatter, never as a length", () => {
    const out = exactReadout({ kind: "angle", value: 90, lineAngleDeg: 90 }, mm, deg);
    expect(out).toEqual({ label: "A", value: "90°", extras: [] });
  });

  it("shows the line angle when the raw direction angle is obtuse — the case it exists for", () => {
    // Two parallel faces whose stored normals happen to be antiparallel: raw 180,
    // line 0. This is the whole reason `lineAngleDeg` ships (probe:
    // scripts/probe/scratch/exact-angle.ts measured this on a real box).
    const out = exactReadout({ kind: "angle", value: 180, lineAngleDeg: 0 }, mm, deg);
    expect(out.value).toBe("180°");
    expect(out.extras).toEqual(["line 0°"]);
  });

  it("drops an extra that only repeats the primary value", () => {
    // `primary: "parallel"` on a pair whose plane gap equals the minimum
    // distance must not render as `12.5 mm · parallel 12.5 mm`.
    const out = exactReadout(
      { kind: "distance", value: 12.5, primary: "parallel", parallelDistance: 12.5, centreDistance: 30 },
      mm,
      deg
    );
    expect(out.extras).toEqual(["centre 30 mm"]);
  });

  it("names every distance extra it actually has, in reading order", () => {
    const out = exactReadout(
      {
        kind: "distance",
        value: 12.5,
        primary: "parallel",
        parallelDistance: 12.6,
        centreDistance: 30.25,
        axisDistance: 25,
        angleDeg: 0.001,
      },
      mm,
      deg
    );
    expect(out.extras).toEqual(["parallel 12.6 mm", "centre 30.25 mm", "axis 25 mm", "angle 0.001°"]);
  });

  it("never invents an extras line for the single-entity kinds", () => {
    expect(exactReadout({ kind: "radius", value: 3, centreDistance: 99 }, mm, deg).extras).toEqual([]);
    expect(exactReadout({ kind: "edgeLength", value: 5 }, mm, deg).extras).toEqual([]);
  });
});

describe("circleRadiusFromArcPoints", () => {
  it("computes the radius of a known circle in the XY plane", () => {
    // Circle of radius 5 centered at origin: (5,0,0), (0,5,0), (-5,0,0)
    const r = circleRadiusFromArcPoints([5, 0, 0], [0, 5, 0], [-5, 0, 0]);
    expect(r).toBeCloseTo(5, 6);
  });

  it("returns null for (near-)collinear points", () => {
    expect(circleRadiusFromArcPoints([0, 0, 0], [1, 0, 0], [2, 0, 0])).toBeNull();
  });
});
