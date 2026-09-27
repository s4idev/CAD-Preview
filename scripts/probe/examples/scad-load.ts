/**
 * SCAD/CSG load + render harness — the fast local loop for "the viewer hangs
 * (or is just slow) on my model", with no `.vsix` repackage and no extension
 * reload. It runs the SHIPPED code paths in this process (or in the shipped
 * kernel worker, with `--worker`), so what it measures is what the extension
 * does — only the packaging is skipped.
 *
 *   npm run scad:check -- /path/model.scad                     # diagnose
 *   npm run scad:check -- /path/model.scad --timeout 120       # + watchdog
 *   npm run scad:check -- /path/model.scad --render            # + PNGs
 *   npm run scad:check -- /path/out.csg                        # reuse a .csg
 *   npm run scad:check -- /path/model.scad --worker --timeout 30
 *
 * Why each stage is timed and the build reports LIVE progress:
 *
 * 1. `.scad` → `.csg` (`scadService.convertScadToCsg`, the shipped conversion).
 * 2. `parseCsg` (pure) + a construct histogram (`hull×23`, `cylinder×71`, …),
 *    so a slow model is identifiable before any WASM runs.
 * 3. Base-shape build (`readShape`'s `"csg"` branch → `csgModel.buildCsgShape`).
 *    **This is where the enclosure hangs.** The walk pushes a warning per
 *    completed construct (e.g. `hull() — hulled 12345 point(s) into N
 *    facet(s)`), so every warning is echoed AS IT HAPPENS — to stdout and to
 *    a trace file, with a `+ms` stamp and RSS. A hang therefore names the
 *    construct it hung in: the last line printed is the last thing that
 *    finished. (A synchronous WASM call cannot be interrupted from JS, so a
 *    progress trace is the only in-process attribution available — and the
 *    trace file survives the watchdog's `SIGTERM`, which stdout-through-a-pipe
 *    does not.)
 * 4. Tessellation / edges / vertices, timed separately — a model can build
 *    fine and still be unusable if `BRepMesh_IncrementalMesh_2` is the slow
 *    part, and those are two different problems with two different fixes.
 * 5. `--render` writes PNGs through the shipped headless renderer
 *    (`renderService.renderSnapshot`, the `render_snapshot` MCP tool's own
 *    engine), which re-runs the load internally — stated, not hidden, because
 *    it doubles the wall clock on a slow model.
 * 6. `--worker` reproduces the panel's exact path: the real
 *    `createKernelClient` forking `dist/kernel-worker.js`, with the watchdog
 *    timeout as `--timeout` (default 60s). Use it to confirm a hang is in the
 *    kernel rather than in the IPC/queue layer, and to see the panel's own
 *    message.
 *
 * `--timeout <sec>` re-executes this harness as a child and kills it on
 * expiry, so a 5-minute hang costs `--timeout` seconds instead of 5 minutes.
 * The child's trace file is read back and its tail printed on the kill.
 *
 * Probe-harness conventions apply: MEMFS paths stay ≤10 characters, every
 * OCCT handle is pushed onto `cleanup` and deleted in reverse in `finally`,
 * and a caught error goes through `wrapOcctFault` (which resets a corrupt
 * kernel). See `scripts/probe/README.md`.
 */
import { spawnSync } from "node:child_process";
import * as fs from "node:fs";
import * as os from "node:os";
import * as path from "node:path";

import { parseCsg, type CsgNode } from "../../../src/csgImport";
import { kernelVersions } from "../../../src/kernelVersions";
import { extractEdges, extractVertices, tessellateByGroup } from "../../../src/meshExtract";
import { getOcct, readShape, wrapOcctFault } from "../../../src/occtService";
import { convertScadToCsg, resolveOpenscadBinary, ScadUnavailableError } from "../../../src/scadService";
import { TESSELLATION_PRESETS, type TessellationQuality } from "../../../src/tessellationQuality";
import { DEFAULT_VIEWS, isRenderAvailable, renderSnapshot } from "../../../src/renderService";

const QUALITIES: TessellationQuality[] = ["draft", "standard", "fine"];
/** Cap on printed per-group rows — a model with hundreds of solids must not bury the timing lines. */
const MAX_GROUP_ROWS = 20;
/** A span this long is reported in the in-process summary at the end of a run. */
const SLOW_SPAN_MIN_MS = 1000;

interface Options {
  model?: string;
  csg?: string;
  openscad?: string;
  quality: TessellationQuality;
  timeoutSec: number;
  render: boolean;
  noLoad: boolean;
  profile: boolean;
  onlyHull?: number;
  out: string;
  trace: string;
  worker: boolean;
  child: boolean;
}

const USAGE = `usage: npm run scad:check -- <model.scad|model.csg> [options]

  --csg <path>        use an existing .csg instead of running openscad
  --openscad <path>   explicit openscad binary (else $OPENSCAD_BINARY, else PATH)
  --quality <p>       draft | standard | fine   (default standard)
  --timeout <sec>     watchdog: kill the run after N seconds (default 0 = off)
  --worker            run through the real kernel worker (the panel's path)
  --render            also render PNGs (needs Playwright's Chromium)
  --no-load           skip the in-process build (use with --render: the renderer
                      re-loads internally, so this avoids paying for it twice)
  --profile           report where the kernel time went, by OCCT entry point
  --only-hull <n>     build ONLY the nth "hull() { … }" block (0 = list them)
  --out <dir>         where --render writes PNGs (default ./scad-render)
  --trace <path>      progress log (default <tmpdir>/cad-preview-scad-trace.log)
  --no-build          accepted and ignored (it belongs to scripts/probe/run.mjs)
  --child             internal: this process is the watchdog's child`;

function parseArgs(argv: string[]): Options {
  const o: Options = {
    quality: "standard",
    timeoutSec: 0,
    render: false,
    noLoad: false,
    profile: false,
    out: path.join(process.cwd(), "scad-render"),
    trace: path.join(os.tmpdir(), "cad-preview-scad-trace.log"),
    worker: false,
    child: false,
  };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = (): string => {
      const v = argv[++i];
      if (v === undefined) throw new Error(`${a} needs a value`);
      return v;
    };
    if (a === "--csg") o.csg = next();
    else if (a === "--openscad") o.openscad = next();
    else if (a === "--out") o.out = next();
    else if (a === "--trace") o.trace = next();
    else if (a === "--timeout") o.timeoutSec = Number(next());
    else if (a === "--quality") {
      const q = next() as TessellationQuality;
      if (!QUALITIES.includes(q)) throw new Error(`--quality must be one of ${QUALITIES.join(" | ")}`);
      o.quality = q;
    } else if (a === "--worker") o.worker = true;
    else if (a === "--render") o.render = true;
    else if (a === "--no-load") o.noLoad = true;
    else if (a === "--profile") o.profile = true;
    else if (a === "--only-hull") o.onlyHull = Number(next());
    // `run.mjs --no-build`: npm appends `--`-args AFTER the entry path, so
    // `npm run scad:check -- --no-build <model>` hands this flag to the probe
    // rather than to the runner. Tolerated, not rejected — it says nothing
    // about what the probe itself should do.
    else if (a === "--no-build") continue;
    else if (a === "--child") o.child = true;
    else if (a === "--help" || a === "-h") {
      console.log(USAGE);
      process.exit(0);
    } else if (a.startsWith("-")) throw new Error(`unknown flag ${a}`);
    else if (o.model === undefined) o.model = a;
    else throw new Error(`unexpected extra argument ${a}`);
  }
  if (o.model === undefined && o.csg === undefined) throw new Error("a model path (or --csg) is required");
  if (!Number.isFinite(o.timeoutSec) || o.timeoutSec < 0) throw new Error("--timeout must be a non-negative number");
  return o;
}

/** The `--timeout` half of the harness: re-run THIS bundle as a child and kill
 * it on expiry, then report where it got to. A synchronous WASM hang cannot be
 * interrupted from inside the same process (a JS timer never gets to run), so
 * an out-of-process kill is the only way to bound a hang at all. */
function watchdogReexec(opts: Options): void {
  const entry = process.argv[1];
  const childArgs = [...process.argv.slice(2), "--child"];
  console.log(`[watchdog] will kill this run after ${opts.timeoutSec}s (Ctrl-C is always available)`);
  const res = spawnSync(process.execPath, ["--enable-source-maps", entry, ...childArgs], {
    cwd: process.cwd(),
    stdio: "inherit",
    env: process.env,
    timeout: opts.timeoutSec * 1000,
  });
  const killed = res.signal === "SIGTERM" || (res.error as NodeJS.ErrnoException | undefined)?.code === "ETIMEDOUT";
  if (!killed) {
    process.exit(res.status ?? 1);
  }
  console.error(`\n[watchdog] killed after ${opts.timeoutSec}s — the run did not finish.`);
  const spans = spansFromTraceFile(opts.trace, 3);
  if (spans.length > 0) {
    console.error(`[watchdog] slowest spans (parsed from the trace file, so they survive the kill):`);
    for (const [i, s] of spans.entries()) {
      console.error(`  #${i + 1} ${(s.ms / 1000).toFixed(1)}s — after "${clip(s.after)}" → "${clip(s.before)}"`);
    }
  }
  console.error(`[watchdog] trace tail (the LAST line is the last construct that completed):`);
  for (const line of traceTail(opts.trace, 12)) console.error(`  ${line}`);
  console.error(`[watchdog] full trace: ${opts.trace}`);
  process.exit(1);
}

const TRACE_LINE = /^\[\+\s*(\d+)ms[^\]]*\]\s*(.*)$/;

function clip(message: string): string {
  return message.length > 72 ? `${message.slice(0, 69)}…` : message;
}

/** The slowest spans of a trace file — the same "widest gap between stamps"
 * view {@link Trace.slowest} gives in-process, recomputed from the file so it
 * still works on a run the watchdog had to kill (the child never gets to
 * print it). */
function spansFromTraceFile(file: string, n: number): Array<{ ms: number; after: string; before: string }> {
  const stamps = traceTail(file, Number.MAX_SAFE_INTEGER)
    .map((line) => TRACE_LINE.exec(line))
    .filter((m): m is RegExpExecArray => m !== null)
    .map((m) => ({ at: Number(m[1]), message: m[2] }));
  const spans: Array<{ ms: number; after: string; before: string }> = [];
  for (let i = 1; i < stamps.length; i++) {
    spans.push({ ms: stamps[i].at - stamps[i - 1].at, after: stamps[i - 1].message, before: stamps[i].message });
  }
  return spans.sort((a, b) => b.ms - a.ms).slice(0, n);
}

function traceTail(file: string, lines: number): string[] {
  try {
    return fs
      .readFileSync(file, "utf8")
      .split("\n")
      .filter((l) => l !== "")
      .slice(-lines);
  } catch {
    return [];
  }
}

function main(): void {
  let opts: Options;
  try {
    opts = parseArgs(process.argv.slice(2));
  } catch (err) {
    console.error(`${(err as Error).message}\n\n${USAGE}`);
    process.exit(2);
    return;
  }
  if (opts.timeoutSec > 0 && !opts.child) {
    watchdogReexec(opts);
    return;
  }
  void run(opts).catch((err) => {
    console.error(`\nFAILED: ${err instanceof Error ? err.message : String(err)}`);
    console.error(`trace: ${opts.trace}`);
    process.exit(1);
  });
}

void main();

// --- trace ------------------------------------------------------------------

interface Trace {
  (message: string): void;
  path: string;
  /** The trace's slowest spans, longest first — the automatic version of
   * "read the +ms stamps and find the gap", which is the whole diagnosis for a
   * model that builds but takes minutes. */
  slowest(n: number): Array<{ ms: number; after: string; before: string }>;
}

/** Stdout AND an append-only file, stamped with elapsed ms and RSS. The file
 * is the load-bearing half: stdout is a pipe under `npm run`, so its writes
 * are asynchronous and a `SIGTERM` from the watchdog can drop the tail. */
function makeTrace(file: string, t0: number): Trace {
  fs.writeFileSync(file, "");
  const entries: Array<{ at: number; message: string }> = [];
  const trace = ((message: string): void => {
    const at = Date.now() - t0;
    entries.push({ at, message });
    const line = `[+${String(at).padStart(7)}ms rss=${Math.round(process.memoryUsage().rss / 1048576)}MB] ${message}`;
    console.log(line);
    fs.appendFileSync(file, line + "\n");
  }) as Trace;
  trace.path = file;
  trace.slowest = (n: number) => {
    const spans: Array<{ ms: number; after: string; before: string }> = [];
    for (let i = 1; i < entries.length; i++) {
      spans.push({ ms: entries[i].at - entries[i - 1].at, after: entries[i - 1].message, before: entries[i].message });
    }
    return spans.sort((a, b) => b.ms - a.ms).slice(0, n);
  };
  return trace;
}

/** The `warnings` array `readShape` pushes into, wrapped so every entry is
 * traced the moment it is pushed. Gives a live construct-by-construct progress
 * log with zero changes to the shipped walk (the array is its documented
 * output channel). */
function liveWarnings(trace: Trace): string[] {
  const sink: string[] = [];
  return new Proxy(sink, {
    get(target, prop, receiver) {
      if (prop === "push") {
        return (...items: string[]) => {
          for (const item of items) trace(`csg: ${item}`);
          return Array.prototype.push.apply(target, items);
        };
      }
      return Reflect.get(target, prop, receiver);
    },
  }) as string[];
}

// --- .csg introspection -----------------------------------------------------

interface ProfileEntry {
  calls: number;
  ms: number;
}

/**
 * Wraps the OCCT module's own entry points so the harness can report where the
 * kernel time went — no product code involved, and it works for any model.
 *
 * Only constructor-performs-the-work calls are measured (which is how
 * `BRepAlgoAPI_Fuse_3` / `BRepBuilderAPI_GTransform_2` / `BRepMesh_..._2` all
 * behave in this codebase) plus the three `BRepGProp` integrators, which are
 * static methods rather than constructors. Wrapping preserves `new` semantics
 * by returning the original object from a plain function, so `instanceof`-free
 * call sites are unaffected; a name that is not a function is skipped, and a
 * prototype whose methods cannot be patched simply stays uninstrumented.
 */
function instrumentOcct(oc: Record<string, unknown>, profile: Map<string, ProfileEntry>, warn: (m: string) => void): void {
  const bump = (label: string, ms: number): void => {
    const e = profile.get(label) ?? { calls: 0, ms: 0 };
    e.calls++;
    e.ms += ms;
    profile.set(label, e);
  };
  const time = <T>(label: string, fn: () => T): T => {
    const t0 = performance.now();
    try {
      return fn();
    } finally {
      bump(label, performance.now() - t0);
    }
  };

  // Static integrators (`BRepGProp` is a holder object, not a class).
  const gprop = oc.BRepGProp as Record<string, unknown> | undefined;
  for (const name of ["VolumeProperties2", "SurfaceProperties2", "LinearProperties"]) {
    const orig = gprop?.[name] as ((...a: unknown[]) => unknown) | undefined;
    if (typeof orig !== "function") continue;
    gprop![name] = function (...args: unknown[]): unknown {
      // VolumeProperties2/SurfaceProperties2 take the deflection as arg 2.
      const eps = name === "LinearProperties" ? undefined : args[2];
      const label = eps === undefined ? name : `${name}(eps=${String(eps)})`;
      return time(label, () => orig.apply(this, args));
    };
  }

  // Every constructor whose construction IS the operation. `BRepMesh_*_2` is
  // the mesher, and the deferred BOP form (`Fuse_1()` + `SetArguments` +
  // `Build()`) does its work in `Build`, patched below — so both are included
  // and their constructors simply read ~0ms.
  //
  // Each underlying FUNCTION is wrapped exactly once: OCCT's aliases share
  // prototypes (`BRepAlgoAPI_Cut_1` and `BRepAlgoAPI_Fuse_1` inherit one
  // `Build`), so wrapping per holder installs a wrapper over a wrapper and the
  // report claims >100% of the wall clock — which is how this was caught. A
  // function shared by several classes is labelled by its FAMILY
  // (`BRepAlgoAPI.Build`), because naming it after whichever alias happened to
  // be seen first would attribute a fuse's time to `Common`.
  const CLASS_RE = /^(BRepAlgoAPI_|BRepBuilderAPI_|BRepPrimAPI_|BRepOffsetAPI_|BRepMesh_|BRepFeat_|GeomAPI_|GC_|ShapeFix_)/;
  const FAMILY_RE = /^([A-Za-z]+API)_/;
  const ctorWrappers = new Map<unknown, unknown>();
  const records: Array<{ label: string; method: string; owners: string[]; fn: (...a: unknown[]) => unknown }> = [];
  const methodRecords = new Map<unknown, (typeof records)[number]>();
  const stripArity = (key: string): string => key.replace(/_\d+$/, "");
  const familyOf = (kernelKey: string): string => FAMILY_RE.exec(kernelKey)?.[1] ?? stripArity(kernelKey);

  for (const key of Object.keys(oc)) {
    if (!CLASS_RE.test(key)) continue;
    const orig = oc[key] as { prototype?: Record<string, unknown> } | undefined;
    if (typeof orig !== "function") continue;
    let wrapped = ctorWrappers.get(orig) as { prototype?: Record<string, unknown> } | undefined;
    if (!wrapped) {
      wrapped = function (this: unknown, ...args: unknown[]): unknown {
        return time(stripArity(key), () => new (orig as unknown as new (...a: unknown[]) => unknown)(...args));
      } as unknown as { prototype?: Record<string, unknown> };
      wrapped.prototype = orig.prototype;
      ctorWrappers.set(orig, wrapped);
    }
    oc[key] = wrapped;

    // The two-phase methods, which is where a deferred algorithm actually runs:
    // `Build()` for the multi-operand booleans (the form `csgModel.ts` now uses,
    // whose ctor does nothing), `Perform` for the sewing and the mesher. Without
    // these the profiler reports a large build as fully attributed while the
    // single biggest call reads `0.0s` — the harness must measure the form it
    // recommends, not only the one-shot constructors.
    const proto = (wrapped as { prototype?: Record<string, unknown> }).prototype;
    if (!proto) continue;
    for (const method of ["Build", "Perform"] as const) {
      const fn = proto[method] as ((...a: unknown[]) => unknown) | undefined;
      if (typeof fn !== "function") continue;
      const existing = methodRecords.get(fn);
      if (existing) {
        existing.owners.push(key);
        continue;
      }
      const rec = { label: `${stripArity(key)}.${method}`, method, owners: [key], fn };
      methodRecords.set(fn, rec);
      records.push(rec);
      const wrapper = function (this: unknown, ...args: unknown[]): unknown {
        return time(rec.label, () => rec.fn.apply(this, args));
      };
      try {
        proto[method] = wrapper;
      } catch (e) {
        warn(`profiler: could not instrument ${rec.label} (${e instanceof Error ? e.message : String(e)})`);
      }
    }
  }

  // A method shared by more than one class reads as its family rather than as
  // whichever alias was visited first (see the comment above). Applied after
  // the loop, because an owner list is only complete once every class has been
  // walked — and through the record, so the label the wrapper reads at call
  // time is the final one.
  for (const rec of records) {
    if (rec.owners.length > 1) rec.label = `${familyOf(rec.owners[0])}.${rec.method}`;
  }
}

/** Prints the profile, biggest total first — the attribution a span summary
 * cannot give (it says *when*, this says *what*). */
function reportProfile(profile: Map<string, ProfileEntry>, trace: Trace, totalMs: number): void {
  const rows = [...profile.entries()].sort((a, b) => b[1].ms - a[1].ms);
  if (rows.length === 0) return;
  const sum = rows.reduce((n, [, e]) => n + e.ms, 0);
  trace(`profile: kernel calls by entry point (${(sum / 1000).toFixed(1)}s of ${(totalMs / 1000).toFixed(1)}s wall clock):`);
  for (const [label, e] of rows.slice(0, 14)) {
    trace(`profile: ${(e.ms / 1000).toFixed(1).padStart(8)}s  ${String(e.calls).padStart(6)}×  ${label}`);
  }
}

function countConstructs(roots: CsgNode[]): Map<string, number> {
  const counts = new Map<string, number>();
  const walk = (node: CsgNode): void => {
    counts.set(node.name, (counts.get(node.name) ?? 0) + 1);
    for (const child of node.children) walk(child);
  };
  for (const root of roots) walk(root);
  return counts;
}

// --- the run ----------------------------------------------------------------

/** Blanks out `//` and `/* *\/` comments, PRESERVING length, so offsets found
 * in the masked copy index the original text exactly.
 *
 * Load-bearing, not tidiness: OpenSCAD's own `.csg` output is full of comments
 * (this repo's `examples/OpenSCAD/mixed.csg` documents itself with "exercises
 * the hull() path"), and an unmasked scan matches the `hull()` inside a comment
 * first — then brace-matches from there, which for a file-leading comment
 * swallows the whole model and silently "isolates" everything. A `//` inside a
 * string literal (an `import("//host/share/…")`) would be mis-masked; OpenSCAD
 * writes no such thing and the failure would be a skip, not a wrong number. */
function maskComments(text: string): string {
  const out = text.split("");
  let i = 0;
  while (i < text.length) {
    if (text[i] === "/" && text[i + 1] === "/") {
      while (i < text.length && text[i] !== "\n") out[i++] = " ";
    } else if (text[i] === "/" && text[i + 1] === "*") {
      out[i++] = " ";
      out[i++] = " ";
      while (i < text.length && !(text[i] === "*" && text[i + 1] === "/")) out[i++] = " ";
      if (i < text.length) {
        out[i++] = " ";
        out[i++] = " ";
      }
    } else i++;
  }
  return out.join("");
}

/** The `.csg`'s `hull() { … }` blocks, located by brace matching from the
 * block's opening brace (comments masked out, above).
 *
 * This exists because the construct trace cannot, on its own, tell "stalled
 * INSIDE hull #9" from "stalled in the boolean immediately after it" — the last
 * line printed is the last *child* of whichever construct hung. Rebuilding one
 * hull alone is the decisive experiment, and it costs seconds. */
function hullBlocks(text: string): Array<{ start: number; end: number }> {
  const masked = maskComments(text);
  const out: Array<{ start: number; end: number }> = [];
  const re = /hull\s*\(/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(masked)) !== null) {
    const brace = masked.indexOf("{", re.lastIndex);
    if (brace < 0) continue;
    let depth = 0;
    for (let i = brace; i < masked.length; i++) {
      const c = masked[i];
      if (c === "{") depth++;
      else if (c === "}") {
        depth--;
        if (depth === 0) {
          out.push({ start: m.index, end: i + 1 });
          re.lastIndex = i + 1;
          break;
        }
      }
    }
  }
  return out;
}

/** `--only-hull`: `0` lists the blocks (and exits), `n` returns a synthetic
 * `.csg` holding only that hull, wrapped in a `group()` so it goes through the
 * exact same parse-and-build path. */
function isolateHull(csgBytes: Uint8Array, n: number, trace: Trace): Uint8Array {
  const text = Buffer.from(csgBytes).toString("utf8");
  const blocks = hullBlocks(text);
  if (n === 0) {
    for (const [i, b] of blocks.entries()) {
      const head = text.slice(b.start, b.start + 160).split("\n")[0] ?? "";
      trace(`hull #${i + 1} — offset ${b.start}, ${b.end - b.start} chars: ${head.trim()}`);
    }
    trace(`hulls: ${blocks.length} total — rerun with --only-hull <n> to build just one`);
    process.exit(0);
  }
  const block = blocks[n - 1];
  if (!block) throw new Error(`--only-hull ${n}: this .csg has ${blocks.length} hull block(s)`);
  trace(`only-hull: building hull #${n} of ${blocks.length} alone (${block.end - block.start} chars) — omit --only-hull for the whole model`);
  return Buffer.from(`group() {\n${text.slice(block.start, block.end)}\n}`, "utf8");
}

async function run(opts: Options): Promise<void> {
  const t0 = Date.now();
  const trace = makeTrace(opts.trace, t0);
  // `run.mjs` sets cwd to the repo root, where dist/*.wasm lives.
  const extensionPath = process.cwd();

  trace(`harness: node ${process.versions.node}${opts.child ? " (watchdog child)" : ""}, mode=${opts.worker ? "worker" : "in-process"}`);
  trace(`kernels: ${JSON.stringify(kernelVersions())}`);

  let csgBytes = await resolveCsg(opts, trace);
  if (opts.onlyHull !== undefined) csgBytes = isolateHull(csgBytes, opts.onlyHull, trace);
  announceCsg(csgBytes, trace);

  // Instrument BEFORE any kernel work, so both the build below and the
  // renderer's own internal (re-)load are covered. `getOcct` is a memoized
  // module singleton, so the render path sees the same wrapped object.
  const profile = new Map<string, ProfileEntry>();
  if (opts.profile) {
    instrumentOcct((await getOcct(extensionPath)) as Record<string, unknown>, profile, trace);
    trace(`profile: instrumented OCCT entry points`);
  }

  if (opts.worker) {
    await runViaWorker(opts, csgBytes, extensionPath, trace);
  } else {
    if (!opts.noLoad) await runInProcess(opts, csgBytes, extensionPath, trace);
    else if (!opts.render) throw new Error("--no-load needs --render (or --worker): nothing left to do");
    if (opts.render) await runRender(opts, csgBytes, extensionPath, trace);
  }
  trace(`done — ${Date.now() - t0}ms total${opts.worker ? "" : ` (${opts.render ? "load + render" : "load only"})`}`);
  if (opts.profile) reportProfile(profile, trace, Date.now() - t0);
  reportSlowestSpans(trace);
}

/** Prints the widest gaps in the trace. On a model that eventually builds but
 * takes minutes, this names the construct that cost the time without anyone
 * having to eyeball the stamps — and on a model that *hangs*, it is the last
 * thing printed before the watchdog fires. */
function reportSlowestSpans(trace: Trace): void {
  const spans = trace.slowest(5).filter((s) => s.ms >= SLOW_SPAN_MIN_MS);
  if (spans.length === 0) return;
  trace(`slowest: ${spans.length} span(s) over ${SLOW_SPAN_MIN_MS / 1000}s:`);
  for (const [i, s] of spans.entries()) {
    trace(`slowest #${i + 1}: ${(s.ms / 1000).toFixed(1)}s — after "${clip(s.after)}" → "${clip(s.before)}"`);
  }
}

/** Step 1: an existing `.csg` (positional or `--csg`) is used as-is; a `.scad`
 * goes through the SHIPPED conversion. The conversion result is also written
 * to a temp cache and its path printed, so iterating on the *load* half of a
 * slow model skips openscad entirely on the next run. */
async function resolveCsg(opts: Options, trace: Trace): Promise<Uint8Array> {
  const source = opts.csg ?? opts.model!;
  const isCsg = path.extname(source).toLowerCase() === ".csg";
  if (isCsg || opts.csg !== undefined) {
    const bytes = fs.readFileSync(source);
    trace(`csg: read ${bytes.length} bytes from ${source} (openscad not run)`);
    return bytes;
  }
  const binary = resolveOpenscadBinary(opts.openscad);
  trace(`scad: converting with "${binary}" (${path.basename(source)}) …`);
  const t = Date.now();
  try {
    const { csgBytes, warnings } = await convertScadToCsg(source, { binary: opts.openscad });
    trace(`scad: converted in ${Date.now() - t}ms → ${csgBytes.length} bytes of .csg`);
    for (const w of warnings) trace(`scad: ${w}`);
    const cacheDir = path.join(os.tmpdir(), "cad-preview-scad-cache");
    fs.mkdirSync(cacheDir, { recursive: true });
    const cached = path.join(cacheDir, `${path.basename(source, path.extname(source))}.csg`);
    fs.writeFileSync(cached, csgBytes);
    trace(`scad: cached → ${cached}  (reuse this run's load half with: --csg ${cached})`);
    return csgBytes;
  } catch (err) {
    if (err instanceof ScadUnavailableError) {
      throw new Error(`${err.reason}\n(pass --csg <path> to a pre-converted .csg to skip openscad entirely)`);
    }
    throw err;
  }
}

/** Step 2: pure parse + construct histogram. `parseCsg`'s own warnings are
 * re-emitted by the build below (it forwards them into the warnings array),
 * so they are not repeated here. */
function announceCsg(bytes: Uint8Array, trace: Trace): void {
  const text = Buffer.from(bytes).toString("utf8");
  const parsed = parseCsg(text);
  const counts = [...countConstructs(parsed.roots)].sort((a, b) => b[1] - a[1]);
  trace(`csg: ${parsed.roots.length} root statement(s), useMaxFN=${parsed.useMaxFN}, ${parsed.warnings.length} parse warning(s)`);
  trace(`csg: constructs — ${counts.map(([name, n]) => `${name}×${n}`).join(", ")}`);
}

/** Whole-shape volume + topology counts for the summary line.
 *
 * These are the regression numbers for a change to the `.csg` walk: a perf fix
 * must not move the volume or the face count. They are deliberately raw OCCT
 * rather than `massProperties.ts`, so nothing sits between the kernel's output
 * and the number printed — the same "measure the real thing" rule the rest of
 * this harness follows. */
function describeShape(
  oc: Awaited<ReturnType<typeof getOcct>>,
  shape: ReturnType<typeof readShape>,
  cleanup: Array<{ delete(): void }>,
): string {
  try {
    const props = new oc.GProp_GProps_1();
    cleanup.push(props);
    oc.BRepGProp.VolumeProperties2(shape, props, 1e-3, false, false);
    const volume = props.Mass() as number;
    props.delete();
    cleanup.splice(cleanup.indexOf(props), 1);

    const count = (target: number): number => {
      const ex = new oc.TopExp_Explorer_2(
        shape,
        target as Parameters<typeof oc.TopExp_Explorer_2>[1],
        oc.TopAbs_ShapeEnum.TopAbs_SHAPE,
      );
      let n = 0;
      while (ex.More()) {
        n++;
        ex.Next();
      }
      ex.delete();
      return n;
    };
    return `, volume=${Number.isFinite(volume) ? volume.toFixed(4) : String(volume)}mm³, solid(s)=${count(
      oc.TopAbs_ShapeEnum.TopAbs_SOLID,
    )}, shell(s)=${count(oc.TopAbs_ShapeEnum.TopAbs_SHELL)}, topoFace(s)=${count(oc.TopAbs_ShapeEnum.TopAbs_FACE)}`;
  } catch (e) {
    return `, volume=<unavailable: ${e instanceof Error ? e.message : String(e)}>`;
  }
}

/** Steps 3–4: build the base shape (live-traced), then tessellate. */
async function runInProcess(opts: Options, csgBytes: Uint8Array, extensionPath: string, trace: Trace): Promise<void> {
  const tGet = Date.now();
  const oc = await getOcct(extensionPath);
  trace(`occt: kernel ready in ${Date.now() - tGet}ms`);

  // MEMFS paths stay ≤10 characters — 11+ silently corrupts writes in this build.
  const memPath = "/s.csg";
  oc.FS.writeFile(memPath, csgBytes);
  const cleanup: Array<{ delete(): void }> = [];
  try {
    trace(`build: walking the .csg tree (the hang candidate — the next line names what finished) …`);
    const tBuild = Date.now();
    const warnings = liveWarnings(trace);
    const shape = readShape(oc, memPath, "csg", cleanup, warnings);
    const buildMs = Date.now() - tBuild;
    // Report the topology BEFORE the display steps: those are the ones that can
    // stall on a shape the build itself produced, and this line is what names
    // how big the shape they were handed actually is (a general fuse of
    // touching solids splits faces at every contact, so the face/edge counts
    // are the numbers that explain a slow tessellate or extraction).
    trace(`build: base shape in ${buildMs}ms, ${warnings.length} warning(s)${describeShape(oc, shape, cleanup)}`);

    trace(`tessellate: meshing the shape (the next line names the step that finished) …`);
    const tMesh = Date.now();
    const groups = tessellateByGroup(oc, shape, TESSELLATION_PRESETS[opts.quality]);
    const meshMs = Date.now() - tMesh;
    trace(`tessellate: done in ${meshMs}ms`);
    const tEdges = Date.now();
    const edges = extractEdges(oc, shape);
    const edgesMs = Date.now() - tEdges;
    trace(`edges: done in ${edgesMs}ms (${edges.length})`);
    const tPoints = Date.now();
    const points = extractVertices(oc, shape);
    const pointsMs = Date.now() - tPoints;
    trace(`points: done in ${pointsMs}ms (${points.length})`);

    const faces = groups.reduce((n, g) => n + g.faceCount, 0);
    const triangles = groups.reduce((n, g) => n + g.faces.reduce((m, f) => m + f.buffers.indices.length / 3, 0), 0);
    trace(`summary: build=${buildMs}ms  tessellate(${opts.quality})=${meshMs}ms  edges=${edgesMs}ms  points=${pointsMs}ms`);
    trace(
      `summary: ${groups.length} group(s), ${faces} face(s), ${triangles} triangle(s), ${edges.length} edge(s), ${points.length} point(s)`
    );
    const shown = groups.slice(0, MAX_GROUP_ROWS);
    for (const g of shown) trace(`summary: group ${g.id} — ${g.faceCount} face(s)`);
    if (groups.length > shown.length) trace(`summary: … ${groups.length - shown.length} more group(s)`);
  } catch (err) {
    // An abort leaves the Emscripten instance permanently corrupt;
    // wrapOcctFault recognizes one and calls resetOcct() itself.
    throw wrapOcctFault(err);
  } finally {
    for (let i = cleanup.length - 1; i >= 0; i--) {
      try {
        cleanup[i].delete();
      } catch {
        /* already freed with its owner */
      }
    }
    try {
      oc.FS.unlink(memPath);
    } catch {
      /* not written */
    }
  }
}

/** Step 5: the shipped headless renderer, writing PNGs for eyeballing. It
 * re-loads the model internally (there is no plumbing to hand it the shape we
 * just built), so this doubles the wall clock on a slow model — stated in the
 * trace, not hidden. */
async function runRender(opts: Options, csgBytes: Uint8Array, extensionPath: string, trace: Trace): Promise<void> {
  const available = await isRenderAvailable();
  if (!available.available) {
    trace(`render: skipped — ${available.reason}`);
    return;
  }
  trace(`render: re-runs the load internally, then renders ${DEFAULT_VIEWS.length} view(s) …`);
  const t = Date.now();
  const result = await renderSnapshot(extensionPath, csgBytes, "csg", [], {});
  if (!result.supported || !result.images) {
    trace(`render: unsupported — ${result.reason ?? "no images returned"}`);
    return;
  }
  fs.mkdirSync(opts.out, { recursive: true });
  const source = opts.model ?? opts.csg ?? "model";
  const stem = path.basename(source, path.extname(source));
  for (const image of result.images) {
    const file = path.join(opts.out, `${stem}-${image.label}.png`);
    fs.writeFileSync(file, Buffer.from(image.dataBase64, "base64"));
    trace(`render: wrote ${file}`);
  }
  trace(`render: ${result.images.length} image(s) in ${Date.now() - t}ms → ${opts.out}`);
}

/** Step 6: the panel's exact path — the real kernel client forking
 * `dist/kernel-worker.js`, so this reproduces the reported
 * `loadBRepCachedForDocument did not respond within …ms` message rather than
 * approximating it. The timeout is `--timeout` (default 60s) so a hang is
 * bounded; a failure here after a clean in-process build means the IPC/queue
 * layer, not the kernel. */
async function runViaWorker(opts: Options, csgBytes: Uint8Array, extensionPath: string, trace: Trace): Promise<void> {
  const timeoutMs = (opts.timeoutSec > 0 ? opts.timeoutSec : 60) * 1000;
  const { createKernelClient } = await import("../../../src/kernelClient");
  const client = createKernelClient(extensionPath, { timeoutMs });
  const documentKey = "scad-check";
  trace(`worker: calling loadBRepCachedForDocument (kernel watchdog ${Math.round(timeoutMs / 1000)}s) …`);
  const t = Date.now();
  try {
    const result = await client.loadBRepCachedForDocument(documentKey, extensionPath, csgBytes, "csg", []);
    const faces = result.groups.reduce((n, g) => n + g.faceCount, 0);
    trace(`worker: loaded in ${Date.now() - t}ms — ${result.groups.length} group(s), ${faces} face(s), ${result.edges.length} edge(s)`);
    if (result.warnings?.length) trace(`worker: ${result.warnings.length} warning(s), first: ${result.warnings[0]}`);
    // The worker is shared and stateful; release this document's cache.
    await client.disposeBRepCacheForDocument(documentKey).catch(() => undefined);
  } catch (err) {
    trace(`worker: FAILED after ${Date.now() - t}ms — ${err instanceof Error ? err.message : String(err)}`);
    process.exitCode = 1;
  } finally {
    // The forked child's open IPC channel keeps THIS process's event loop
    // alive, so without this the harness prints "done" and then sits there
    // until the watchdog kills it — which reads exactly like the hang being
    // investigated (observed on the first run of this mode). `cancelCurrent()`
    // with nothing running recycles the idle child.
    client.cancelCurrent();
  }
}
