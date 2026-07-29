# Rust Optimization Plan

## Goal

Improve runtime beyond the translated C++ structure, accepting non-1:1 Rust
implementation changes only when they preserve public behavior and strict raw
parity for every benchmarked example in the default benchmark set.

This plan is about speed and memory layout. It is not a precision redesign; the
known same-scanline intersection tie limitation remains documented and out of
scope.

## Current Baseline

The arena migration removed most small hot-path allocations:

- `IntersectNode` is inline in a reusable vector.
- `Join` and `ghost_joins` are inline vectors.
- `OutRec` ownership is centralized.
- `OutPt` uses chunk allocation.

Latest ten-iteration benchmark notes:

- Rust is faster than C++ on most default parity cases.
- Remaining weak spots are `offset_open_round`, `union_dense`, and cases near
  parity with C++ such as `open_paths_clip`.
- Prior profiling still showed time in `build_intersect_list`,
  `process_edges_at_top_of_scanbeam`, `append_polygon`, `join_points`,
  `do_simple_polygons`, and clipping cleanup after offsetting.

## Ground Rules

- Keep `bash bench/run_benchmarks.sh 10` parity-clean after every retained change.
- Do not add a benchmarked example to the default set unless Rust and upstream C++
  match on the runner's parity fields.
- If a case is useful but cannot retain raw parity because of known Clipper tie
  behavior, keep it outside the default benchmark set and label it diagnostic.
- Run `cargo fmt`, `cargo clippy --all-targets -- -D warnings`, and `cargo test`
  before comparing benchmark results.
- Use current benchmark timings as noisy guidance, not proof from a single run.
- Prefer changes that improve cache locality or remove repeated work without
  changing geometric semantics.
- Do not chase exact behavior for `jittered_sliver_union`; it is tie-sensitive and
  excluded from strict benchmark parity.

## Suspected Optimization Items

### 1. Release Profile Tuning

Add benchmark-oriented release settings in `clipper-rust/Cargo.toml`:

```toml
[profile.release]
lto = "thin"
codegen-units = 1
```

Why: hot code is function-heavy and pointer-heavy. Thin LTO may help inlining
across modules with very low implementation risk.

Risk: slower release builds. Runtime parity risk should be low.

Validation: compare `bench/run_benchmarks.sh 10` before and after.

### 2. Reuse `Clipper` Instances Inside Benchmarks

The benchmark currently measures both setup and execution. For application-style
throughput, add a second benchmark mode that reuses allocated `Clipper` and
`ClipperOffset` capacity across iterations where the public API permits it.

Why: after arenas, capacity reuse may be a real Rust advantage, but current
per-iteration constructors hide it.

Risk: benchmark-only change. Keep existing fresh-instance benchmark as the default
for C++ comparability.

Validation: add a separate `reuse` CSV mode rather than replacing current numbers.

### 3. Reserve Known Output Capacities

Use cheap input-derived estimates to reserve vector capacity:

- `intersect_list.reserve(...)` based on active edge count for dense scanbeams.
- `joins.reserve(...)` and `ghost_joins.reserve(...)` from horizontal edge counts.
- `solution.reserve(...)` in benchmark and public wrappers where the expected
  result count is available.
- `OutPtArena` block count preallocation for offset-heavy cases.

Why: arena blocks helped, but vector growth still appears in profiles and RSS
measurements.

Risk: over-reserving can increase RSS. Use conservative estimates and keep RSS in
the benchmark table.

Validation: compare both elapsed time and RSS.

### 4. Optimize `build_intersect_list`

Current structure mirrors C++: copy AEL into SEL, repeatedly bubble-sort by
top-`X`, and emit intersection nodes when adjacent edges swap.

Possible non-1:1 improvements:

- Count active edges once and use a temporary contiguous `Vec<*mut TEdge>` for the
  scanbeam ordering.
- Use insertion-sort over the contiguous vector for mostly sorted AEL data.
- Emit intersection nodes from adjacent swaps, then write the final SEL links only
  once if later code still needs them.

Why: profiles for open paths and offset cleanup are dominated by
`build_intersect_list`.

Risk: high. Intersection ordering is subtle and tied to parity. Implement behind a
feature flag or alternate function first, then compare every benchmark case.

Validation: default benchmarks, plus explicit `jittered_sliver_union_N` diagnostic
runs to watch tie-sensitive behavior.

### 5. Optimize `process_edges_at_top_of_scanbeam`

Suspected issues:

- Repeated `next_in_lml`/horizontal checks through raw pointers.
- Repeated calls to `top_x` and slope predicates.
- Branch-heavy edge promotion logic.

Possible changes:

- Cache `top_x` for active edges per scanbeam.
- Split horizontal and non-horizontal promotion into smaller specialized loops.
- Reuse a temporary vector of edges finishing at `top_y`.

Why: this is consistently hot in profiles, especially open paths and offset
cleanup.

Risk: medium-high. This code mutates AEL and SEL state heavily.

Validation: focus on `open_paths_clip`, `offset_open_round`, and
`large_coord_xor`.

### 6. Optimize `append_polygon` And `join_points`

These remain raw linked-ring surgery and are hot in `touching_rect_grid` and
strict-simple workloads.

Possible changes:

- Add lightweight ring metadata for first/last points and point count.
- Avoid repeated low-level ring walks when the same `OutRec` is used repeatedly.
- Split horizontal join cases into clearer specialized helpers.
- Consider storing output rings as arena indices after raw-pointer behavior is
  fully covered by tests.

Why: arena allocation gave large wins here, so locality/metadata improvements are
likely still useful.

Risk: high for topology. Keep changes very small and benchmark after each one.

Validation: `touching_rect_grid`, `strict_simple_stars`, `polytree_closed_nested`,
and C++ conformance tests.

### 7. Optimize `do_simple_polygons`

Current behavior scans linked rings looking for repeated vertices and then splits
rings.

Possible changes:

- For each output ring, build a temporary `HashMap<IntPoint, Vec<OutPtHandle>>`
  or sorted vector of repeated points to avoid quadratic scans on large rings.
- Use this only when the ring point count exceeds a threshold; keep the current
  loop for tiny rings.

Why: `do_simple_polygons` is hot in `strict_simple_stars`.

Risk: medium-high. Splitting order can affect hole assignment and output order.

Validation: strict-simple tests plus `strict_simple_stars` benchmark.

### 8. Offset-Specific Fast Paths

`offset_open_round` is the remaining clear slower case.

Possible changes:

- Precompute round step sin/cos tables for repeated `DoRound` calls with the same
  `steps`.
- Reserve `dest_poly` capacity from source length and join type.
- Avoid cloning `dest_poly` into `dest_polys` by using `mem::take` and reusing a
  spare vector.
- Investigate whether the cleanup `Clipper` call after offsetting dominates more
  than round generation; optimize the cleanup path first if so.

Why: `offset_open_round` remains slower than C++ after arena migration.

Risk: low to medium for reserve/table changes; medium for vector ownership changes.

Validation: `offset_open_round`, `offset_stars`, offset unit tests, C++
conformance test.

### 9. Convert Raw Pointers To Index Handles Selectively

Do not do this globally first. Pick one structure only if profiling says pointer
traversal remains a bottleneck after simpler changes.

Best candidates:

- `OutPt` ring links: high potential, high topology risk.
- AEL/SEL `TEdge` links: high potential, very high sweep-line risk.
- `OutRec` references: lower speed potential, useful for safety.

Why: index handles can improve locality and make ownership safer, but they are the
largest departure from the translation.

Risk: high. This should be a later phase with additional regression fixtures.

## Suggested Execution Order

1. Add release profile tuning and measure.
2. Add offset fast-path reserves/tables and measure `offset_open_round`.
3. Add conservative vector reserves in `Clipper` hot paths.
4. Prototype a contiguous-vector `build_intersect_list` behind a feature flag.
5. Optimize `do_simple_polygons` only if `strict_simple_stars` remains important.
6. Consider index handles only after the smaller changes plateau.

## Required Evidence For Completion

For each retained optimization:

- Every benchmarked example in the default benchmark set has strict parity with
  upstream C++.
- The README benchmark table is updated with the new run.
- The optimization plan records what changed and whether it helped.
- Any changed hot path has focused tests or existing tests that directly cover it.

## Execution Notes

Status after the first non-1:1 optimization pass:

- Retained: `OutPtArena::alloc` now uses unchecked block and slot access after
  the local block-growth invariant is established. This removes bounds checks
  from the output-point arena allocation path without changing pointer identity
  or topology behavior.
- Retained: `process_intersect_list` and `fixup_intersection_order` use
  unchecked `intersect_list` access inside loops whose bounds are captured before
  iteration. `swap` preserves vector length, so the invariant is local and
  auditable.
- Retained: `build_intersect_list` reserves `intersect_list` capacity from the
  active edge count before emitting intersections. This is conservative and does
  not alter intersection order.
- Retained: `ClipperOffset::do_offset` reuses the existing `src_poly` allocation
  with `clear` plus `extend_from_slice` instead of replacing it with a cloned
  vector for every node.
- Evaluated and rejected: Thin LTO plus `codegen-units = 1` increased release
  build time and did not clearly improve the benchmarked slower cases.
- Evaluated and rejected: moving `dest_poly` into `dest_polys` with `mem::take`
  removed one clone but lost scratch capacity reuse and increased offset-heavy
  RSS.
- Evaluated and rejected: broad `dest_poly` reserve estimates were too noisy and
  could increase RSS without reliable speed improvement.

Validation run:

- `cargo fmt --manifest-path clipper-rust/Cargo.toml --check`
- `cargo clippy --manifest-path clipper-rust/Cargo.toml --all-targets -- -D warnings`
- `cargo test --manifest-path clipper-rust/Cargo.toml`
- `bash bench/run_benchmarks.sh 10`

The benchmark run passed strict parity for every default benchmark case. Current
remaining slower Rust cases are `nested_holes`, `open_paths_clip`, and
`offset_open_round` by benchmark-internal elapsed time, while Rust still uses
less RSS in every default case.

Profiling notes:

- `offset_open_round` spends almost all time in the cleanup `Clipper` pass after
  offset generation. `do_offset` itself was below 1% in the sampled run.
- `open_paths_clip` and `offset_open_round` are both dominated by
  `build_intersect_list`, `process_edges_at_top_of_scanbeam`, and
  `fixup_intersection_order`.
- A larger rewrite of `build_intersect_list` remains the best suspected speed
  item, but it is topology-risky because equal-scanline intersection order is a
  known source of geometry differences. Keep it behind an alternate path and
  require strict benchmark parity before retaining it.

## Build Intersect List Rewrite Pass

Status: retained.

Change:

- `build_intersect_list` now copies active edges into a reusable contiguous vector
  and simulates the same adjacent bubble-swap passes over that vector.
- It still emits `IntersectNode` entries in the same adjacent-swap order as the
  linked-SEL implementation.
- It no longer builds or mutates SEL links during the initial intersection-list
  construction. `fixup_intersection_order` already rebuilds SEL from AEL before
  it needs adjacency checks, and the single-intersection path does not need SEL.

Why it helped:

- The old implementation did pointer-heavy linked-list swaps just to discover the
  intersection list.
- The new implementation keeps the ordering work in cache-friendly vector slots
  while preserving the original swap-emission semantics.

Validation run:

- `cargo fmt --manifest-path clipper-rust/Cargo.toml --check`
- `cargo clippy --manifest-path clipper-rust/Cargo.toml --all-targets -- -D warnings`
- `cargo test --manifest-path clipper-rust/Cargo.toml`
- `bash bench/run_benchmarks.sh 10`

The benchmark run passed strict parity for every default benchmark case. The
previously slower `open_paths_clip`, `nested_holes`, and `offset_open_round`
cases were faster than C++ in this local run.
