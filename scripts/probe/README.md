# Probe harness

Runs a TypeScript file against the real OCCT, Gmsh, meshio++ and fTetWild WASM
kernels, so a probe can establish a fact about a binding without writing a
build script first.

```sh
npm run probe -- scripts/probe/examples/bull-counts.ts          # builds first
node scripts/probe/run.mjs --no-build path/to/probe.ts [args…]   # reuse dist/
```

`run.mjs` bundles the entry with `scripts/nodeBundleConfig.mjs`, the same
Node/CJS recipe `esbuild.mjs` uses for the shipped bundles. It then runs the
bundle with the repo root as `cwd`, so a probe passes `process.cwd()` as
`extensionPath`. Without `--no-build` it runs `esbuild.mjs` first, so
`dist/*.wasm` exist and match the sources. Stack traces point at the `.ts`
lines (inline source maps). `kernelVersions()` from `src/kernelVersions.ts`
reports the installed package versions.

The environment is passed through unchanged, so the harness also runs under
the Flatpak recipe in `doc/development.md`
(`ELECTRON_RUN_AS_NODE=1 …/code scripts/probe/run.mjs …`).

## Starting a probe

Copy `examples/bull-counts.ts` into `scratch/`, which is git-ignored. The
example shows the three things every probe needs:

- Push every OCCT handle onto a `cleanup` list and `.delete()` it in reverse
  order in `finally`.
- Keep MEMFS paths to 10 characters or fewer. Longer paths silently corrupt
  STEP writes in this build.
- Throw a caught error through `wrapOcctFault`, which resets a corrupt kernel.
  The Gmsh equivalents are `getGmsh`/`resetGmsh` in `src/gmshService.ts`.

Commit a probe under `examples/` only when it is worth rerunning, for example
when it pins a fact a later dependency bump could change.

## Protocol

Every probe write-up records:

1. The installed artifact versions (print `kernelVersions()`).
2. The fixture path.
3. The exact call shapes that worked: overload suffix (`_1`, `_2`, …) and
   argument count. OCCT has no `.d.ts`, so signatures are found by listing a
   prototype and trying suffixes. Record the ones that failed too, and how
   they failed.
4. The output facts, checked against an analytic or independently measured
   value where one exists. A method that accepts its arguments and changes
   nothing counts as a failed probe.
5. Cleanup behaviour, and a kernel reset after any deliberate abort.
6. Wall-clock timing on the largest fixture that fits.

## Load + render a model locally (SCAD/CSG)

`examples/scad-load.ts` — `npm run scad:check -- <model.scad|model.csg>` — is the
committed probe for "the viewer hangs (or is just slow) on my model": it runs the
shipped `.scad` → `.csg` conversion, prints a construct histogram, builds the base
shape with a **live per-construct trace**, then times tessellation / edges /
vertices separately. No `.vsix` repackage, no extension reload.

```sh
npm run scad:check -- /path/model.scad --timeout 120   # bounded, prints the trace tail
npm run scad:check -- /path/model.scad --worker        # the viewer's own path
npm run scad:check -- /path/model.scad --render        # PNGs (render_snapshot's engine)
npm run scad:check -- /path/model.csg --only-hull 0    # list the hull blocks
npm run scad:check -- /path/model.csg --only-hull 9    # build ONLY hull #9
npm run scad:check -- /path/model.csg --profile        # ms + calls by OCCT entry point
```

The trace is the point. `readShape`'s `"csg"` branch pushes a warning per
completed construct (`hull() — hulled 472 point(s) into 244 facet(s)`), so the
harness wraps that array and echoes each entry the moment it is pushed, stamped
with `+ms` and RSS — **to stdout and to a trace file**. A hang therefore names
the construct it hung in: the last line printed is the last thing that finished.
`--timeout <sec>` re-executes the harness as a child and kills it on expiry
(a synchronous WASM call cannot be interrupted in-process, and stdout through a
pipe loses its tail on `SIGTERM`, which is why the file matters), printing the
trace tail. `--worker` goes through the real `createKernelClient` and
`dist/kernel-worker.js`, so it reproduces the viewer's
`… did not respond within …ms` message rather than approximating it.
`--only-hull <n>` builds just the nth `hull() { … }` block: the trace alone
cannot say whether a long gap was spent *inside* a hull or in the boolean right
after it, and rebuilding one hull alone settles it in seconds. The run ends with
the five widest spans, so nobody has to eyeball the stamps.

`--profile` goes one level deeper, to the **OCCT call**: it wraps every
`BRepAlgoAPI_*` / `BRepBuilderAPI_*` / `BRepPrimAPI_*` / `BRepOffsetAPI_*` /
`BRepMesh_*` / `GeomAPI_*` / `GC_*` / `ShapeFix_*` constructor plus the two-phase
`Build`/`Perform` methods (where a deferred algorithm really runs) and the
`BRepGProp` integration statics, then prints the top entry points by total time
and call count, alongside the built shape's volume and topology. A span says
*when*; this says *what the kernel was doing*. It is how a 280s enclosure build
was attributed to 28 `Cut_3` calls (157.3s) and 17 `Fuse_3` calls (90.7s) rather
than to the 23 `hull()` blocks it looked like — see `CLAUDE.md` for that
finding and the fix that followed from it (`multiUnion`/`multiCut` plus the
analytic-preserving transform path: 280s → 20s), and for the second pass that
took the same model to **5.5s of build / 6.7s end to end** (same-domain cleanup
after each boolean, plus coplanar hull facets merged before sewing).

Known cost, stated rather than hidden: `--render` re-loads the model inside
`renderSnapshot` (there is no plumbing to hand it the shape already built), so it
doubles the wall clock on a slow model.

Two more committed probes belong to the same loop, both about where a `.csg`
build spends its time:

- `examples/csg-solid-inventory.ts <model.csg>` — the per-solid inventory:
  volume, face count, shell count and bounding box for every solid, plus a count
  of bodies under 1mm³. This is the probe that decides whether a `.csg` build
  produced *the design's* solid decomposition: total volume and `BRepCheck`
  validity both survived the coplanar facet merge, while the enclosure silently
  came back as 14 solids (two detached 2.7mm³ tabs and two zero-volume 2-face
  sheets) instead of 10. Run it on any model before changing how facets,
  operands or booleans are built.
- `examples/csg-same-domain.ts <model.csg>` — builds one model and reports what
  `ShapeUpgrade_UnifySameDomain_2` does to it (face count, volume delta, its own
  cost) plus a `Cut_3` against the merged and unmerged results, which is how the
  2.85× boolean speedup it buys was measured. This one PASSED and ships (as
  `unifyFragmented`, applied to boolean results).
- `examples/csg-shell-build.ts` — a self-contained microbenchmark of the two ways
  to turn a facet set into a closed shell: per-triangle faces + `Sewing.Perform`
  (today's path) against one shared `TopoDS_Edge` per vertex pair + `TopoDS_Shell`
  (2.9×, but its closure check does not yet agree — see roadmap 5.2). It also
  times the `sewing=false` ctor flag, which is 2.45× faster and returns an EMPTY
  `SewedShape()`, so it must not be used.

## Where results go

- **Pass:** the item's "If admitted" phases move into the roadmap's Tier 1
  with firm estimates, and the verified call shapes go into `CLAUDE.md`.
- **Fail:** the item moves to the roadmap's Non-goals (Kernel-blocked for a
  dead binding, Rejected scope for a product judgement), with the calls that
  failed and what would change our mind.
- **Partial:** the item stays in the roadmap, narrowed to the part that
  survived, and the failed part is recorded under Non-goals.

The write-up belongs in those documents. The scratch script can be thrown
away.
