/**
 * How should a facet set become a solid? **RESOLVED: FAILED** — this file is the
 * evidence, and the conclusion is recorded in `doc/roadmap.md`'s "Kernel-blocked"
 * Non-goals group (the hand-assembled-shell entry) and in `CLAUDE.md`'s `.csg`
 * build-cost section.
 *
 * `solidFromFacets` sews: one `MakeEdge`+`MakeWire`+`MakeFace` per triangle, then
 * `BRepBuilderAPI_Sewing.Perform` to discover, by tolerance search, the shared
 * edges that were never shared. On the moulded enclosure that is **4.4s of an
 * 18.9s build** (23 `Sewing.Perform` calls over 3510 triangles) plus 2.3s of
 * `MakeFace`. Both callers (`polyhedron`, `hull`) hand it vertices that are
 * ALREADY shared by index, so the search has nothing to find — which is why
 * "build a shell from shared edges and skip sewing" looked like the obvious win.
 *
 * It does not work, for two independently measured reasons:
 *
 *  1. **No hand-assembled shell is possible in this build.** The only shell
 *     mutators bound are `BRep_Builder.MakeShell` and the 2-arg `Add`, and
 *     `Add(shell, face)` DEEP-COPIES the face's subshape tree: three edges handed
 *     to two faces, then added to a shell, give **6** edges, not 3. `MakeWire.Add`
 *     and `MakeFace_15` both keep the TShape (`IsSame`); the copy happens at
 *     `Add`. There is no `UpdateShell` and no 3-arg `Add`. So the shell reports
 *     every edge as free and `BRepCheck_Analyzer` calls it invalid — `Sewing` is
 *     not merely the tolerance search here, it is the only assembly that glues a
 *     facet set into a valid shell.
 *  2. **Sharing the edges but KEEPING the sewing is a null result on real
 *     geometry.** `Sewing` rebuilds its own edges, so it cannot exploit the
 *     sharing: on the enclosure `MakeEdge` drops 10530 → 5265 calls (exactly the
 *     predicted halving) while sewing goes 4.4s → 4.3s against a build that stays
 *     ~19s (volume 32047.3836, 10 solids, 94 warnings — identical). Edge creation
 *     is 0.1s of that 18.9s; there was never anything there to win.
 *
 * So `solidFromFacets` keeps per-triangle edges and sewing. The remaining cost is
 * per-FACE work (`MakeFace` × 3510) and sewing's own reconstruction, which only a
 * smaller face count could address — and that is the coplanar facet merge, which
 * was measured and rejected for changing the solid decomposition (see
 * `csgModel.ts`'s module doc).
 *
 * Also kept here, because it is a trap rather than a win: **the 5-arg
 * `BRepBuilderAPI_Sewing` ctor's FIRST flag (`sewing`) must stay `true`.** With it
 * off, `Perform` is ~2.4× faster and `NbFreeEdges()` still answers correctly (an
 * opened facet set still reports its free edges) — but `SewedShape()` comes back
 * EMPTY, so all the speed buys is a silently missing solid.
 *
 *   npm run probe -- scripts/probe/examples/csg-shell-build.ts [model.csg]
 */
import * as fs from "node:fs";

import { getOcct, readShape, wrapOcctFault } from "../../../src/occtService";

type Cleanup = Array<{ delete(): void }>;

/** Synchronous, unbuffered: a WASM abort is followed by a JS throw, and
 * `process.exit` truncates pending async pipe writes — which silently lost every
 * line of a first version of this probe. */
function say(line: string): void {
  fs.writeSync(1, `${line}\n`);
}

/** Torus as an indexed triangle list: closed, manifold, every edge shared by
 * exactly two triangles, and — unlike a UV sphere, which aborts the module —
 * no degenerate pole triangles. `MakeEdge_3` between two coincident points raises
 * a raw OCCT `Standard_ConstructionError`, surfaced here as a bare numeric
 * pointer. */
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

interface Built {
  faces: number;
  edges: number;
  volume: number;
  valid: boolean;
  freeEdges: number;
  ms: number;
  note: string;
}

async function main(): Promise<void> {
  const oc: any = await getOcct(process.cwd());
  const cleanup: Cleanup = [];
  const keep = <T extends { delete(): void }>(h: T): T => (cleanup.push(h), h);
  const { points, tris } = torusMesh(8, 16);
  const OPEN = tris.slice(0, tris.length - 2);

  const count = (shape: any, kind: string): number => {
    const exp = keep(
      new oc.TopExp_Explorer_2(shape, oc.TopAbs_ShapeEnum[kind], oc.TopAbs_ShapeEnum.TopAbs_SHAPE),
    );
    let n = 0;
    for (; exp.More(); exp.Next()) n++;
    return n;
  };
  const volumeOf = (shape: any): number => {
    const p = new oc.GProp_GProps_1();
    oc.BRepGProp.VolumeProperties2(shape, p, 1e-3, false, false);
    const v = p.Mass();
    p.delete();
    return v;
  };
  const isValid = (shape: any): boolean => keep(new oc.BRepCheck_Analyzer(shape, true)).IsValid_2();
  /** The sewing-independent closure check: `ShapeAnalysis_Shell` counts edges
   * bounding exactly one face. */
  const freeEdgesOf = (shape: any): number => {
    const sa = keep(new oc.ShapeAnalysis_Shell());
    sa.LoadShells(shape);
    sa.CheckOrientedShells(shape, true, false);
    return sa.HasFreeEdges() ? count(sa.FreeEdges(), "TopAbs_EDGE") : 0;
  };

  try {
    const gp = points.map((p) => keep(new oc.gp_Pnt_3(p[0], p[1], p[2])));

    /** Today's construction: a fresh edge/wire/face per triangle. */
    const buildFacesFresh = (list: number[][]): any[] =>
      list.map((t) => {
        const w = keep(new oc.BRepBuilderAPI_MakeWire_1());
        for (let k = 0; k < 3; k++) {
          w.Add_1(keep(new oc.BRepBuilderAPI_MakeEdge_3(gp[t[k]], gp[t[(k + 1) % 3]])).Edge());
        }
        return keep(new oc.BRepBuilderAPI_MakeFace_15(w.Wire(), true)).Face();
      });

    /** The same faces, but one `TopoDS_Edge` per vertex pair, reused (as-is or
     * reversed) by both neighbours. `.Reversed()` + a `TopoDS.Edge_1` cast is this
     * codebase's orientation idiom (`TopoDS.Wire_1(wire.Reversed())` in
     * `occtOperations.ts`); `Oriented(...)` needs `TopAbs_Orientation`, and
     * passing the wrong enum surfaces only as an opaque `reading 'value'`. */
    const buildFacesShared = (
      list: number[][],
    ): { faces: any[]; edges: Map<string, any>; wiresDone: number; wiresFailed: number } => {
      const edges = new Map<string, { edge: any; forward: [number, number] }>();
      let wiresDone = 0;
      let wiresFailed = 0;
      const faces: any[] = [];
      for (const t of list) {
        const w = keep(new oc.BRepBuilderAPI_MakeWire_1());
        for (let k = 0; k < 3; k++) {
          const u = t[k];
          const v = t[(k + 1) % 3];
          const key = u < v ? `${u}_${v}` : `${v}_${u}`;
          let entry = edges.get(key);
          if (!entry) {
            entry = {
              edge: keep(new oc.BRepBuilderAPI_MakeEdge_3(gp[u], gp[v])).Edge(),
              forward: [u, v],
            };
            edges.set(key, entry);
          }
          const sameDirection = entry.forward[0] === u && entry.forward[1] === v;
          w.Add_1(sameDirection ? entry.edge : keep(oc.TopoDS.Edge_1(entry.edge.Reversed())));
        }
        if (!w.IsDone()) {
          wiresFailed++;
          continue;
        }
        wiresDone++;
        faces.push(keep(new oc.BRepBuilderAPI_MakeFace_15(w.Wire(), true)).Face());
      }
      return { faces, edges, wiresDone, wiresFailed };
    };

    /** Sew the faces into a solid — the assembly `solidFromFacets` uses. */
    const sewnSolidOf = (faces: any[], t0: number): Built => {
      const sew = keep(new oc.BRepBuilderAPI_Sewing(1e-6, true, true, true, false));
      for (const f of faces) sew.Add(f);
      sew.Perform(keep(new oc.Handle_Message_ProgressIndicator_1()));
      const ms = performance.now() - t0;
      const free = sew.NbFreeEdges();
      const exp = keep(
        new oc.TopExp_Explorer_2(
          sew.SewedShape(),
          oc.TopAbs_ShapeEnum.TopAbs_SHELL,
          oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
        ),
      );
      if (!exp.More()) {
        return { faces: 0, edges: 0, volume: NaN, valid: false, freeEdges: free, ms, note: "no shell" };
      }
      const solid = keep(new oc.BRepBuilderAPI_MakeSolid_3(oc.TopoDS.Shell_1(exp.Current()))).Solid();
      return {
        faces: count(solid, "TopAbs_FACE"),
        edges: count(solid, "TopAbs_EDGE"),
        volume: volumeOf(solid),
        valid: isValid(solid),
        freeEdges: free,
        ms,
        note: "build+sew",
      };
    };

    /** A: today — fresh edges, sewing. */
    const pathA = (list: number[][]): Built => {
      const t0 = performance.now();
      return sewnSolidOf(buildFacesFresh(list), t0);
    };
    /** C: shared edges, sewing still doing the assembly. */
    const pathC = (list: number[][]): Built => {
      const t0 = performance.now();
      const { faces, edges, wiresDone, wiresFailed } = buildFacesShared(list);
      const built = sewnSolidOf(faces, t0);
      return { ...built, note: `map ${edges.size} edge(s), wires ok=${wiresDone}/failed=${wiresFailed}` };
    };
    /** B: shared edges + a shell built by hand, no sewing. */
    const pathB = (list: number[][]): Built => {
      const t0 = performance.now();
      const { faces, edges, wiresDone, wiresFailed } = buildFacesShared(list);
      const shell = keep(new oc.TopoDS_Shell());
      const builder = keep(new oc.BRep_Builder());
      builder.MakeShell(shell);
      for (const f of faces) builder.Add(shell, f);
      const freeEdges = freeEdgesOf(shell);
      const solid = keep(new oc.BRepBuilderAPI_MakeSolid_3(shell)).Solid();
      return {
        faces: count(solid, "TopAbs_FACE"),
        edges: count(solid, "TopAbs_EDGE"),
        volume: volumeOf(solid),
        valid: isValid(solid),
        freeEdges,
        ms: performance.now() - t0,
        note: `map ${edges.size} edge(s), wires ok=${wiresDone}/failed=${wiresFailed}, shell freeEdges=${freeEdges}`,
      };
    };

    const analytic = 2 * Math.PI * Math.PI * 1 * 0.35 * 0.35;
    const show = (label: string, b: Built): void => {
      say(
        `  ${label.padEnd(22)} ${b.ms.toFixed(0).padStart(6)}ms  faces=${String(b.faces).padStart(4)} ` +
          `edges=${String(b.edges).padStart(4)} volume=${
            Number.isFinite(b.volume) ? b.volume.toFixed(6) : "n/a"
          } valid=${b.valid} freeEdges=${b.freeEdges}`,
      );
      say(`    ${b.note}`);
    };

    say(`torus ${points.length} points / ${tris.length} triangles, analytic volume ${analytic.toFixed(6)}`);
    say(`  CLOSED set — A and C must agree; B is expected to FAIL (no valid hand-assembled shell):`);
    show("A fresh edges + sew", pathA(tris));
    show("B shared edges shell", pathB(tris));
    show("C shared edges + sew", pathC(tris));
    say(`  OPEN set (2 triangles removed) — all three must reject it:`);
    show("A fresh edges + sew", pathA(OPEN));
    show("B shared edges shell", pathB(OPEN));
    show("C shared edges + sew", pathC(OPEN));

    // ---- the `sewing` ctor flag: a trap, not a win -------------------------
    say(`\n  ctor flag \`sewing\` (the 5-arg ctor's FIRST flag) — must stay true:`);
    for (const [label, list] of [
      ["closed", tris],
      ["OPEN", OPEN],
    ] as Array<[string, number[][]]>) {
      for (const sewing of [true, false]) {
        const t0 = performance.now();
        const s = keep(new oc.BRepBuilderAPI_Sewing(1e-6, sewing, true, true, false));
        for (const f of buildFacesFresh(list)) s.Add(f);
        s.Perform(keep(new oc.Handle_Message_ProgressIndicator_1()));
        const ms = performance.now() - t0;
        const exp = keep(
          new oc.TopExp_Explorer_2(
            s.SewedShape(),
            oc.TopAbs_ShapeEnum.TopAbs_FACE,
            oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
          ),
        );
        let n = 0;
        for (; exp.More(); exp.Next()) n++;
        say(
          `    ${label.padEnd(7)} sewing=${String(sewing).padEnd(5)} ${ms.toFixed(1).padStart(7)}ms  ` +
            `freeEdges=${String(s.NbFreeEdges()).padStart(3)}  sewn faces=${String(n).padStart(4)}`,
        );
      }
    }

    // ---- where does the sharing get lost? ---------------------------------
    // Three edges handed to two opposite-winding triangles, then assembled into a
    // shell: 3 edges = the reuse survived; 6 = something copied them. Measured
    // after each stage so the culprit is named.
    say(`\n  sharing diagnostic (3 edges, two opposite faces, one shell):`);
    const e = [0, 1, 2].map((i) =>
      keep(new oc.BRepBuilderAPI_MakeEdge_3(gp[i], gp[(i + 1) % 3])).Edge(),
    );
    const makeFaceOf = (list: any[]): any => {
      const w = keep(new oc.BRepBuilderAPI_MakeWire_1());
      for (const edge of list) w.Add_1(edge);
      return keep(new oc.BRepBuilderAPI_MakeFace_15(w.Wire(), true)).Face();
    };
    const f1 = makeFaceOf(e);
    const f2 = makeFaceOf([e[2], e[1], e[0]]);
    const firstEdge = (shape: any): any =>
      keep(
        new oc.TopExp_Explorer_2(
          shape,
          oc.TopAbs_ShapeEnum.TopAbs_EDGE,
          oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
        ),
      ).Current();
    say(`    MakeWire.Add + MakeFace_15 keep the TShape: ${e.some((x) => x.IsSame(firstEdge(f1)))}`);
    const shell2 = keep(new oc.TopoDS_Shell());
    const builder2 = keep(new oc.BRep_Builder());
    builder2.MakeShell(shell2);
    builder2.Add(shell2, f1);
    say(`    after BRep_Builder.Add(shell, f1): shell edges=${count(shell2, "TopAbs_EDGE")}`);
    builder2.Add(shell2, f2);
    say(
      `    after a second Add: shell edges=${count(shell2, "TopAbs_EDGE")} ` +
        `(3 = reuse survived, 6 = Add copied the face)`,
    );
    const mutators = (() => {
      const found = new Set<string>();
      let proto = Object.getPrototypeOf(keep(new oc.BRep_Builder()));
      while (proto && proto !== Object.prototype) {
        for (const n of Object.getOwnPropertyNames(proto)) {
          if (/^(Add|UpdateShell|MakeShell)/.test(n)) found.add(n);
        }
        proto = Object.getPrototypeOf(proto);
      }
      return [...found].sort();
    })();
    say(`    shell mutators bound at all: ${mutators.join(", ")}`);

    // ---- the real model, for context --------------------------------------
    const modelPath = process.argv[2];
    if (modelPath) {
      say(`\n${modelPath}`);
      oc.FS.writeFile("/m.csg", new TextEncoder().encode(fs.readFileSync(modelPath, "utf8")));
      const warnings: string[] = [];
      const t0 = performance.now();
      const shape = readShape(oc, "/m.csg", "csg", cleanup, warnings);
      say(
        `  shipped build: ${(performance.now() - t0).toFixed(0)}ms, ` +
          `${count(shape, "TopAbs_SOLID")} solids, ${count(shape, "TopAbs_FACE")} faces, ` +
          `${warnings.length} warning(s), volume ${volumeOf(shape).toFixed(4)}`,
      );
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
  say(`FAILED: ${e instanceof Error ? e.message : String(e)}`);
  process.exitCode = 1;
});
