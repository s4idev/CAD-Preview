/**
 * WHICH bodies does a `.csg` build actually produce, by volume and bounding box?
 *
 * This is the probe that settles whether an import produced *the design's* solid
 * decomposition, and it exists because the two obvious signals do not: a
 * silently-unglued model keeps its exact total volume AND passes
 * `BRepCheck_Analyzer`. Measured with it, the coplanar facet merge (since
 * removed) took the moulded enclosure from 10 solids to 14 — its tray 5.451mm³
 * light, two 2.725523mm³ tabs detached at `y = 16.0…16.55` and two zero-volume
 * 2-face sheets lying in the contact plane `y = 16.000` — while every volume and
 * validity check stayed green.
 *
 * The group sizes alone (e.g. 193/11/2/11/2/30/30/30/30/221/6/6/6/6) cannot say
 * whether the 2-face members are slivers, whether a body split, or whether two
 * bodies merely stopped being glued; the "tiny" line and the bboxes separate
 * those cases.
 *
 *   npm run probe -- scripts/probe/examples/csg-solid-inventory.ts /path/out.csg
 */
import * as fs from "node:fs";

import { getOcct, readShape, wrapOcctFault } from "../../../src/occtService";

type Cleanup = Array<{ delete(): void }>;

function say(line: string): void {
  fs.writeSync(1, `${line}\n`);
}

async function main(): Promise<void> {
  const modelPath = process.argv[2];
  if (!modelPath) throw new Error("usage: csg-solid-inventory.ts <model.csg>");
  const text = fs.readFileSync(modelPath, "utf8");
  const oc: any = await getOcct(process.cwd());
  const cleanup: Cleanup = [];
  try {
    oc.FS.writeFile("/m.csg", new TextEncoder().encode(text));
    const warnings: string[] = [];
    const t0 = performance.now();
    const shape = readShape(oc, "/m.csg", "csg", cleanup, warnings);
    const buildMs = performance.now() - t0;

    const solids: any[] = [];
    const exp = new oc.TopExp_Explorer_2(
      shape,
      oc.TopAbs_ShapeEnum.TopAbs_SOLID,
      oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
    );
    cleanup.push(exp);
    for (; exp.More(); exp.Next()) solids.push(oc.TopoDS.Solid_1(exp.Current()));

    const rows = solids.map((solid, i) => {
      const props = new oc.GProp_GProps_1();
      oc.BRepGProp.VolumeProperties2(solid, props, 1e-3, false, false);
      const volume = props.Mass();
      props.delete();
      const box = new oc.Bnd_Box_1();
      oc.BRepBndLib.Add(solid, box, false);
      const a = box.CornerMin();
      const b = box.CornerMax();
      const bbox = [a.X(), a.Y(), a.Z(), b.X(), b.Y(), b.Z()].map((v: number) => v.toFixed(3));
      box.delete();
      const faces = count(oc, solid, "TopAbs_FACE");
      const shells = count(oc, solid, "TopAbs_SHELL");
      return { i, volume, faces, shells, bbox };
    });

    rows.sort((x, y) => y.volume - x.volume);
    say(`build ${buildMs.toFixed(0)}ms  ${solids.length} solid(s)  warnings=${warnings.length}`);
    for (const w of warnings) say(`  ! ${w}`);
    for (const r of rows) {
      say(
        `  solid-${String(r.i).padStart(2)} vol=${r.volume.toFixed(6).padStart(14)}  ` +
          `faces=${String(r.faces).padStart(4)} shells=${r.shells}  bbox=[${r.bbox.join(", ")}]`,
      );
    }
    const tiny = rows.filter((r) => Math.abs(r.volume) < 1);
    say(`  tiny (<1mm³) solids: ${tiny.length}${tiny.length ? ` — ${tiny.map((t) => t.volume.toFixed(6)).join(", ")}` : ""}`);
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

function count(oc: any, shape: any, kind: string): number {
  const exp = new oc.TopExp_Explorer_2(shape, oc.TopAbs_ShapeEnum[kind], oc.TopAbs_ShapeEnum.TopAbs_SHAPE);
  try {
    let n = 0;
    for (; exp.More(); exp.Next()) n++;
    return n;
  } finally {
    exp.delete();
  }
}

void main().catch((e) => {
  say(`FAILED: ${e instanceof Error ? e.message : String(e)}`);
  process.exitCode = 1;
});
