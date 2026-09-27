/**
 * Scratch probe: the `hull()` path's `BRepBuilderAPI_Sewing.Perform` is 4.4s of
 * the enclosure's 19.2s build (23 calls, ~190ms each, ~140 triangles apiece).
 * Two questions:
 *
 *  1. Is the 5-arg ctor's FIRST flag (`sewing`) worth its time? Every hull facet
 *     is built from shared `gp_Pnt` objects, so the edges are already exactly
 *     coincident and a tolerance search has nothing to find.
 *  2. **Does the closure gate still work with it off?** `solidFromFacets` rejects
 *     a hull when `NbFreeEdges() > 0`; if `sewing=false` also disabled free-edge
 *     detection, an open facet set would be accepted as a solid — a silent
 *     wrong-geometry bug, which is exactly what that gate exists to prevent.
 *
 * So: the same facet set closed AND deliberately opened (two triangles removed),
 * under both flag settings, checking `NbFreeEdges`, the sewn face count, and the
 * solid's volume (the closed case must match the analytic torus).
 *
 *   npm run probe -- scripts/probe/examples/csg-shell-build.ts
 */
import * as fs from "node:fs";

import { getOcct, wrapOcctFault } from "../../../src/occtService";

type Cleanup = Array<{ delete(): void }>;

/** Synchronous, unbuffered: a WASM abort is followed by a JS throw, and
 * `process.exit` truncates pending async pipe writes — which silently lost
 * every line of a first version of this probe. */
function say(line: string): void {
  fs.writeSync(1, `${line}\n`);
}

/** Torus as an indexed triangle list: closed, manifold, every edge shared by
 * exactly two triangles, and — unlike a UV sphere, the first attempt, which
 * aborted the module — no degenerate pole triangles. `MakeEdge_3` between two
 * coincident points raises a raw OCCT `Standard_ConstructionError`, surfaced in
 * this build as a bare numeric pointer. */
function torusMesh(rings: number, sectors: number): { points: number[][]; tris: number[][] } {
  const R = 1;
  const r = 0.35;
  const points: number[][] = [];
  for (let i = 0; i < rings; i++) {
    const v = (2 * Math.PI * i) / rings;
    for (let j = 0; j < sectors; j++) {
      const u = (2 * Math.PI * j) / sectors;
      points.push([
        (R + r * Math.cos(v)) * Math.cos(u),
        (R + r * Math.cos(v)) * Math.sin(u),
        r * Math.sin(v),
      ]);
    }
  }
  const at = (i: number, j: number): number => (i % rings) * sectors + (j % sectors);
  const tris: number[][] = [];
  for (let i = 0; i < rings; i++) {
    for (let j = 0; j < sectors; j++) {
      const a = at(i, j), b = at(i, j + 1), c = at(i + 1, j + 1), d = at(i + 1, j);
      tris.push([a, b, c], [a, c, d]);
    }
  }
  return { points, tris };
}

async function main(): Promise<void> {
  const oc: any = await getOcct(process.cwd());
  const { points, tris } = torusMesh(8, 16);
  const cleanup: Cleanup = [];
  const keep = <T extends { delete(): void }>(h: T): T => (cleanup.push(h), h);

  try {
    const gp = points.map((p) => keep(new oc.gp_Pnt_3(p[0], p[1], p[2])));

    /** Today's construction: a fresh edge/wire/face per triangle. */
    const buildFaces = (list: number[][]): any[] =>
      list.map((t) => {
        const w = keep(new oc.BRepBuilderAPI_MakeWire_1());
        for (let k = 0; k < 3; k++) {
          w.Add_1(keep(new oc.BRepBuilderAPI_MakeEdge_3(gp[t[k]], gp[t[(k + 1) % 3]])).Edge());
        }
        return keep(new oc.BRepBuilderAPI_MakeFace_15(w.Wire(), true)).Face();
      });

    const closedFaces = buildFaces(tris);
    const openFaces = buildFaces(tris.slice(0, tris.length - 2));

    for (const [label, faces] of [
      ["closed", closedFaces],
      ["OPEN (2 triangles removed)", openFaces],
    ] as Array<[string, any[]]>) {
      for (const sewing of [true, false]) {
        const t0 = performance.now();
        const s = keep(new oc.BRepBuilderAPI_Sewing(1e-6, sewing, true, true, false));
        for (const f of faces) s.Add(f);
        s.Perform(keep(new oc.Handle_Message_ProgressIndicator_1()));
        const ms = performance.now() - t0;
        const free = s.NbFreeEdges();
        const sewed = s.SewedShape();
        const exp = keep(
          new oc.TopExp_Explorer_2(sewed, oc.TopAbs_ShapeEnum.TopAbs_FACE, oc.TopAbs_ShapeEnum.TopAbs_SHAPE),
        );
        let n = 0;
        for (; exp.More(); exp.Next()) n++;
        let vol = "";
        if (free === 0) {
          const shellExp = keep(
            new oc.TopExp_Explorer_2(sewed, oc.TopAbs_ShapeEnum.TopAbs_SHELL, oc.TopAbs_ShapeEnum.TopAbs_SHAPE),
          );
          if (shellExp.More()) {
            const solid = keep(
              new oc.BRepBuilderAPI_MakeSolid_3(oc.TopoDS.Shell_1(shellExp.Current())),
            ).Solid();
            cleanup.push(solid);
            const props = new oc.GProp_GProps_1();
            oc.BRepGProp.VolumeProperties2(solid, props, 1e-3, false, false);
            vol = ` volume=${props.Mass().toFixed(6)}`;
            props.delete();
          }
        }
        say(
          `  ${label.padEnd(28)} sewing=${String(sewing).padEnd(5)} ${ms.toFixed(1).padStart(7)}ms  ` +
            `freeEdges=${String(free).padStart(3)}  faces=${String(n).padStart(4)}${vol}`,
        );
      }
    }
    say(`  analytic torus volume = ${(2 * Math.PI * Math.PI * 1 * 0.35 * 0.35).toFixed(6)}`);
  } catch (e) {
    throw wrapOcctFault(e);
  } finally {
    for (const h of cleanup.reverse()) {
      try {
        h.delete();
      } catch {
        /* ignore */
      }
    }
  }
}

void main().catch((e) => {
  say(`FAILED: ${e instanceof Error ? e.message : String(e)}`);
  process.exitCode = 1;
});
