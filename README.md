# clipper-rs

Rust port of the C++ Clipper 6.4.2 polygon clipping library.

The current crate lives in `clipper-rust/`. It ports the C++ library code only; demos,
language bindings, and optional deprecated APIs are out of scope for now.

## Port Status

The first pass prioritized auditability over idiomatic ownership design. The Rust code
keeps the C++ algorithm structure close enough to compare function-by-function, while
using Rust names and safe public entry points where feasible.

Current compatibility choices:

- Coordinate type: `i64`
- C++ `use_int32`: disabled
- C++ `use_xyz`: disabled
- C++ `use_lines`: enabled
- C++ `use_deprecated`: disabled
- Internal linked structures currently use raw pointers

The next cleanup phase can replace raw-pointer internals with arenas or index handles
once conformance coverage is stronger.

## Crate

```bash
cd clipper-rust
cargo test
```

Main modules:

- `types`: Clipper enums, point/path types, and internal structs
- `helpers`: free geometry helpers such as area, orientation, point-in-polygon,
  simplification, and Minkowski helpers
- `clipper_base`: translated `ClipperBase`
- `clipper`: translated boolean clipping operations
- `clipper_offset`: translated offsetting operations

Example:

```rust
use clipper_rust::clipper::Clipper;
use clipper_rust::types::{ClipType, IntPoint, PolyFillType, PolyType};

let a = vec![
    IntPoint::new(0, 0),
    IntPoint::new(10, 0),
    IntPoint::new(10, 10),
    IntPoint::new(0, 10),
];
let b = vec![
    IntPoint::new(5, 5),
    IntPoint::new(15, 5),
    IntPoint::new(15, 15),
    IntPoint::new(5, 15),
];

let mut clipper = Clipper::new();
clipper.add_path(&a, PolyType::Subject, true)?;
clipper.add_path(&b, PolyType::Subject, true)?;

let mut solution = Vec::new();
clipper.execute(ClipType::Union, &mut solution, PolyFillType::NonZero)?;
# Ok::<(), clipper_rust::ClipperError>(())
```

## Validation

Run the normal checks from the crate directory:

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test
```

The test suite includes a C++ oracle test that compiles the bundled
`Clipper/cpp/clipper.cpp` with `g++` and compares representative boolean and offset
results against the Rust implementation.

## Benchmarks

The benchmark harness compares Rust against the bundled C++ implementation for:

- `union_dense`: union of a dense grid of overlapping rectangles
- `touching_rect_grid`: many rectangles sharing exact edges and corners
- `intersection_grid`: vertical strips intersected with horizontal strips
- `difference_holes`: large rectangle minus many smaller rectangles
- `nested_holes`: mixed subject islands and clip holes
- `strict_simple_stars`: strict-simple union of many self-crossing star-like inputs
- `open_paths_clip`: open subject paths clipped through `PolyTree`
- `large_coord_xor`: XOR with coordinates large enough to exercise full-range arithmetic
- `offset_stars`: round offsetting of many closed star polygons
- `offset_open_round`: round offsetting of many open polylines
- `polytree_closed_nested`: closed-path extraction from a nested `PolyTree` result

Each case is generated deterministically in both Rust and C++. The runner reports:

- parity: matching path count, point count, absolute area, and coordinate checksum
- speed: elapsed milliseconds reported by the benchmark binaries
- RSS: peak resident set size from `/usr/bin/time`

The script exits with a non-zero status if any benchmark case fails parity against
the bundled upstream C++ code.

`jittered_sliver_union` is intentionally excluded from the default benchmark table.
That workload creates many same-scanline intersection ties. Clipper 6 sorts
intersections only by `Y`, so equal-`Y` events can change output geometry depending
on sort tie behavior. This affects both the original C++ implementation and this
Rust translation.

This port is not intended to provide perfect geometric precision. Clipper 6 is an
integer polygon clipping algorithm that rounds intersection coordinates; exact
geometry for highly ambiguous sliver cases is outside the scope of a translation.
The current Rust code favors the faster unstable intersection sort, matching the
performance-oriented behavior of the original rather than trying to repair
precision limitations.

Run all benchmark cases:

```bash
bash bench/run_benchmarks.sh
```

Run with more iterations per case:

```bash
bash bench/run_benchmarks.sh 10
```

Output is CSV:

```text
case,parity,rust_elapsed_ms,cpp_elapsed_ms,rust_wall_s,cpp_wall_s,rust_rss_kb,cpp_rss_kb,rust_paths,cpp_paths,rust_points,cpp_points,rust_area_abs,cpp_area_abs,rust_checksum,cpp_checksum
```

During the cleanup phase, a `parity=fail` row is useful evidence: it means the
case found a Rust/C++ output difference by summary metrics and should be reduced
into a focused conformance test before refactoring the relevant code.

Per-engine raw outputs are written to `clipper-rust/target/bench/`.

Latest local smoke run, one iteration per case:

| Case | Parity | Rust ms | C++ ms | Rust/C++ time | Rust RSS KB | C++ RSS KB | Rust/C++ RSS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `union_dense` | ok | 5.639 | 14.974 | 0.38x | 4800 | 6400 | 0.75x |
| `touching_rect_grid` | ok | 49.485 | 46.228 | 1.07x | 10880 | 11840 | 0.92x |
| `intersection_grid` | ok | 20.330 | 21.326 | 0.95x | 5120 | 6276 | 0.82x |
| `difference_holes` | ok | 7.470 | 8.388 | 0.89x | 3840 | 5440 | 0.71x |
| `nested_holes` | ok | 8.432 | 9.988 | 0.84x | 4160 | 5760 | 0.72x |
| `strict_simple_stars` | ok | 28.090 | 23.351 | 1.20x | 5760 | 7040 | 0.82x |
| `open_paths_clip` | ok | 13.248 | 10.204 | 1.30x | 2560 | 3840 | 0.67x |
| `large_coord_xor` | ok | 11.283 | 10.958 | 1.03x | 3840 | 5120 | 0.75x |
| `offset_stars` | ok | 99.571 | 101.007 | 0.99x | 30400 | 31680 | 0.96x |
| `offset_open_round` | ok | 33.536 | 33.664 | 1.00x | 4160 | 6080 | 0.68x |
| `polytree_closed_nested` | ok | 3.388 | 3.507 | 0.97x | 2880 | 4480 | 0.64x |

Lower ratios are better for Rust. These are smoke numbers from one local run;
use more iterations for comparison work.

## Translation Plan

The original staged plan is saved in `RUST_PORT_PLAN.md`.
