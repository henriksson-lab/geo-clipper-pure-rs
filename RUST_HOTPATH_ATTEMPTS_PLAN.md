# Rust Hotpath Attempts Plan

## Goal

Attempt the next three optimization areas without changing public behavior or the
default benchmark parity contract:

1. `process_edges_at_top_of_scanbeam`
2. `fixup_intersection_order`
3. `set_winding_count` / `insert_edge_into_ael`

Each attempt should be a small, reviewable pass. Keep a change only if it passes
tests and strict parity benchmarks. Revert or discard any change that is noisy,
unclear, or shifts geometry ordering.

## Ground Rules

- Preserve strict raw parity for every default benchmark case in
  `bash bench/run_benchmarks.sh 10`.
- Do not add diagnostic tie-sensitive cases to the default benchmark set.
- Prefer local, auditable changes over broad rewrites.
- Unsafe is allowed where the invariant is local and removes a measured hot-path
  bounds check.
- Do not change AEL, SEL, maxima, or output-ring mutation order unless the pass is
  explicitly isolated and parity-clean.
- After each retained attempt, commit the pass separately.

Required validation for each retained attempt:

```bash
cargo fmt --manifest-path clipper-rust/Cargo.toml --check
cargo clippy --manifest-path clipper-rust/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path clipper-rust/Cargo.toml
bash bench/run_benchmarks.sh 10
```

Record the resulting benchmark table in `README.md` and note the outcome here.

## Baseline

The current baseline is commit `9d2bd4c Optimize intersection list construction`.

That pass made `build_intersect_list` vector-backed while retaining default
benchmark parity. Remaining likely hot areas from prior profiling are
`process_edges_at_top_of_scanbeam`, `fixup_intersection_order`,
`set_winding_count`, and `insert_edge_into_ael`.

Before starting attempt 1, capture or reuse a fresh profile for:

```bash
perf record -o .tmp/perf-open-paths.data --call-graph dwarf \
  clipper-rust/target/release/bench open_paths_clip 50
perf record -o .tmp/perf-offset-open.data --call-graph dwarf \
  clipper-rust/target/release/bench offset_open_round 30
```

Use profile data as guidance only; strict benchmark parity is the gate.

## Attempt 1: `process_edges_at_top_of_scanbeam`

### Hypothesis

This routine is hot because it repeatedly follows raw edge links, checks
horizontal/non-horizontal state, and mutates AEL while promoting edges at the top
of each scanbeam.

### Candidate Changes

- Cache repeated raw-pointer reads inside the loop:
  - `next_in_lml`
  - `out_idx >= 0`
  - `wind_delta == 0`
  - `top` / `bot` points used repeatedly in the same branch
- Split clearly terminal maxima handling from ordinary edge promotion where it
  can be done without changing branch order.
- Avoid repeated `is_horizontal(&*next_in_lml)` calls when `next_in_lml` is
  already known and unchanged within a branch.
- Use local `unsafe` unchecked access only if a fixed-bounds vector loop appears
  in the profile. This routine is mostly pointer-based, so do not force it.

### Risks

- High risk of subtle geometry changes if AEL mutation order changes.
- Open path clipping is especially sensitive to promotion timing.

### Focus Cases

- `open_paths_clip`
- `offset_open_round`
- `large_coord_xor`
- `polytree_closed_nested`

### Keep Criteria

- Strict default benchmark parity passes.
- No local test regression.
- At least one focus case improves without a broad RSS regression.

## Attempt 2: `fixup_intersection_order`

### Hypothesis

After `build_intersect_list` was rewritten, `fixup_intersection_order` remains a
visible cost. The current code sorts all intersections by descending `Y` and then
repairs adjacency by scanning forward.

### Candidate Changes

- Add a specialized path for very small lists, such as lengths 2 through 16,
  using insertion sort by descending `Y`.
- Keep the same unstable equal-`Y` behavior as closely as possible. Do not add a
  tie-breaker unless it is a separate diagnostic experiment, because equal-`Y`
  ordering is known to affect geometry.
- Avoid repeated indexing in the adjacency repair loop with local unchecked
  access where the vector length is fixed.
- Measure whether sort time is actually significant after the builder rewrite;
  if it is below noise, record as evaluated and skip.

### Risks

- Equal-`Y` ordering can change output geometry.
- A different small-list sorting algorithm may not match `sort_unstable_by` tie
  order for degenerate cases.

### Focus Cases

- `open_paths_clip`
- `offset_open_round`
- `strict_simple_stars`
- Diagnostic only: `jittered_sliver_union_N`

### Keep Criteria

- Strict default benchmark parity passes.
- Diagnostic tie-sensitive behavior is documented if it changes.
- Focus-case improvement is measurable enough to justify the extra branch.

## Attempt 3: `set_winding_count` / `insert_edge_into_ael`

### Hypothesis

These routines are pointer-walk heavy during local-minima insertion. They may
benefit from branch simplification and reducing repeated raw-pointer reads.

### Candidate Changes

- In `insert_edge_into_ael`, cache `next_in_ael` during the insertion search so
  the loop does not reload the same link and dereference pattern multiple times.
- In `set_winding_count`, identify repeated fill-rule and poly-type checks that
  can be hoisted into locals.
- Split common fill-type cases only if it removes repeated branches without
  duplicating large logic.
- Keep insertion ordering exactly equivalent to `e2_inserts_before_e1`.

### Risks

- Winding-count mistakes can preserve simple tests but break holes, XOR, and open
  path interactions.
- AEL insertion order is core sweep-line state; do not change comparisons or tie
  handling.

### Focus Cases

- `open_paths_clip`
- `intersection_grid`
- `difference_holes`
- `nested_holes`
- `large_coord_xor`

### Keep Criteria

- Strict default benchmark parity passes.
- C++ conformance test passes.
- No worse result on hole-heavy cases unless another focus case clearly improves
  and the tradeoff is documented.

## Completion Checklist

- Attempt 1 retained or documented as rejected.
- Attempt 2 retained or documented as rejected.
- Attempt 3 retained or documented as rejected.
- `README.md` benchmark table updated for every retained pass.
- Each retained pass committed separately.

## Attempt 1 Outcome

Status: retained.

Change:

- Cached `top` and `next_in_lml` state in the first
  `process_edges_at_top_of_scanbeam` AEL pass.
- Reused the cached `next_in_lml` pointer for the intermediate-horizontal check.
- Replaced the second pass's `is_intermediate` helper call with the equivalent
  local `top.y == top_y && next_in_lml != null` check.

Why retained:

- The change preserves branch order and AEL mutation order.
- It passed the full validation gate and strict benchmark parity.
- The final benchmark run improved the `open_paths_clip`, `offset_open_round`,
  `large_coord_xor`, and `polytree_closed_nested` focus cases versus the prior
  README table, though local timing remains noisy.

Validation run:

- `cargo fmt --manifest-path clipper-rust/Cargo.toml --check`
- `cargo clippy --manifest-path clipper-rust/Cargo.toml --all-targets -- -D warnings`
- `cargo test --manifest-path clipper-rust/Cargo.toml`
- `bash bench/run_benchmarks.sh 10`

## Attempt 2 Outcome

Status: retained.

Change:

- Added a small-list insertion sort path for `fixup_intersection_order` when the
  intersection list has at most 16 nodes.
- Larger lists still use `sort_unstable_by` with the same descending-`Y`
  comparator as before.

Why retained:

- The default benchmark suite passed strict parity.
- The focus cases improved versus the prior README table in this local run.
- The change is local to sorting by `Y`; it does not alter adjacency repair or
  SEL swap order after sorting.

Diagnostic note:

- `jittered_sliver_union_56` remains outside the default parity set. In the
  diagnostic run, Rust reported `paths=1076 points=5780 area_abs=554810.000
  checksum=230362069`, while C++ reported `paths=1076 points=5763
  area_abs=554834.000 checksum=229692153`. This remains consistent with the
  known equal-scanline tie limitation and is not a retained default benchmark.

Validation run:

- `cargo fmt --manifest-path clipper-rust/Cargo.toml --check`
- `cargo clippy --manifest-path clipper-rust/Cargo.toml --all-targets -- -D warnings`
- `cargo test --manifest-path clipper-rust/Cargo.toml`
- `bash bench/run_benchmarks.sh 10`

## Attempt 3 Outcome

Status: retained.

Change:

- `insert_edge_into_ael` now caches the searched `next_in_ael` link while walking
  to the insertion point, avoiding repeated loads of the same link.
- `set_winding_count` now hoists `edge.poly_typ` and `edge.wind_delta` into local
  values. These fields are read many times and are not changed by the routine.

Why retained:

- The change preserves `e2_inserts_before_e1` ordering and winding formulas.
- The default benchmark suite passed strict parity.
- Focused insertion and winding tests passed.
- Benchmark impact was small and noisy; this is retained as a local cleanup with
  no observed parity or RSS regression, not as a major speedup.

Validation run:

- `cargo fmt --manifest-path clipper-rust/Cargo.toml --check`
- `cargo clippy --manifest-path clipper-rust/Cargo.toml --all-targets -- -D warnings`
- `cargo test --manifest-path clipper-rust/Cargo.toml`
- `bash bench/run_benchmarks.sh 10`
