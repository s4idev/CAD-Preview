/**
 * Scratch probe: does `ShapeUpgrade_UnifySameDomain` buy anything on a real
 * `.csg` build, and what does it cost?
 *
 * The enclosure's profile (`npm run scad:check -- out.csg --profile`) leaves a
 * 10-solid, **2745-face** shape whose faces are mostly hull facets, and 10.1s
 * of the 19.2s build sits in three `BRepAlgoAPI_BooleanOperation.Build` calls
 * that consume those thousands of faces. So: measure (a) how many faces unify
 * merges at the end, (b) its own cost, (c) whether the volume/topology survive.
 *
 *   npm run probe -- scripts/probe/examples/csg-same-domain.ts /path/out.csg
 */
import * as fs from "node:fs";

import { getOcct, readShape, wrapOcctFault } from "../../../src/occtService";

type Cleanup = Array<{ delete(): void }>;

function countTopo(oc: any, shape: any, kind: string, cleanup: Cleanup): number {
  const exp = new oc.TopExp_Explorer_2(
    shape,
    oc.TopAbs_ShapeEnum[kind],
    oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
  );
  cleanup.push(exp);
  let n = 0;
  for (; exp.More(); exp.Next()) n++;
  return n;
}

function volume(oc: any, shape: any): number {
  const props = new oc.GProp_GProps_1();
  try {
    oc.BRepGProp.VolumeProperties2(shape, props, 1e-3, false, false);
    return props.Mass();
  } finally {
    props.delete();
  }
}

async function main(): Promise<void> {
  const path = process.argv[2];
  if (!path) throw new Error("usage: unify.ts <model.csg>");
  const text = fs.readFileSync(path, "utf8");

  const oc = await getOcct(process.cwd());
  const cleanup: Cleanup = [];
  try {
    oc.FS.writeFile("/m.csg", new TextEncoder().encode(text));
    const warnings: string[] = [];
    const t0 = performance.now();
    const shape = readShape(oc, "/m.csg", "csg", cleanup, warnings);
    const buildMs = performance.now() - t0;

    const faces = countTopo(oc, shape, "TopAbs_FACE", cleanup);
    const solids = countTopo(oc, shape, "TopAbs_SOLID", cleanup);
    const v0 = volume(oc, shape);
    console.log(`build      ${buildMs.toFixed(0)}ms  faces=${faces} solids=${solids} volume=${v0.toFixed(4)} warnings=${warnings.length}`);

    // --- unify -----------------------------------------------------------------
    const t1 = performance.now();
    let unified: any = null;
    let err = "";
    try {
      const up = new oc.ShapeUpgrade_UnifySameDomain_2(shape, true, true, false);
      try {
        up.Build();
        unified = up.Shape();
        if (unified) cleanup.push(unified);
      } finally {
        up.delete();
      }
    } catch (e) {
      err = e instanceof Error ? e.message : String(e);
    }
    const unifyMs = performance.now() - t1;
    if (!unified || (typeof unified.IsNull === "function" && unified.IsNull())) {
      console.log(`unify      ${unifyMs.toFixed(0)}ms  FAILED${err ? ` — ${err}` : " — no shape"}`);
    } else {
      const v1 = volume(oc, unified);
      console.log(
        `unify      ${unifyMs.toFixed(0)}ms  faces=${countTopo(oc, unified, "TopAbs_FACE", cleanup)} ` +
          `solids=${countTopo(oc, unified, "TopAbs_SOLID", cleanup)} volume=${v1.toFixed(4)} ` +
          `dV=${(((v1 - v0) / v0) * 100).toFixed(6)}%`,
      );
      // Is a unified operand actually cheaper to boolean against?
      const tool = new oc.BRepPrimAPI_MakeBox_3(
        new oc.gp_Pnt_3(-500, -500, -500),
        new oc.gp_Pnt_3(500, 500, -400),
      );
      cleanup.push(tool);
      const t2 = performance.now();
      const cut = new oc.BRepAlgoAPI_Cut_3(shape, tool.Shape());
      cut.Build();
      const cutMs = performance.now() - t2;
      cleanup.push(cut.Shape());
      cut.delete();
      const t3 = performance.now();
      const cut2 = new oc.BRepAlgoAPI_Cut_3(unified, tool.Shape());
      cut2.Build();
      const cut2Ms = performance.now() - t3;
      cleanup.push(cut2.Shape());
      cut2.delete();
      console.log(`cut(plain) ${cutMs.toFixed(0)}ms    cut(unified) ${cut2Ms.toFixed(0)}ms`);
    }
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
  console.error(`FAILED: ${e instanceof Error ? e.message : String(e)}`);
  process.exit(1);
});
