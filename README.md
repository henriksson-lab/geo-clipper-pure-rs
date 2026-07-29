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

Per-engine raw outputs are written to `clipper-rust/target/bench/`.

Latest local arena run, ten iterations per case:

| Case | Parity | Rust ms | C++ ms | Rust/C++ time | Rust RSS KB | C++ RSS KB | Rust/C++ RSS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `union_dense` | ok | 44.615 | 40.430 | 1.10x | 4800 | 6080 | 0.79x |
| `touching_rect_grid` | ok | 256.359 | 309.865 | 0.83x | 10560 | 11840 | 0.89x |
| `intersection_grid` | ok | 68.542 | 85.908 | 0.80x | 5120 | 6468 | 0.79x |
| `difference_holes` | ok | 28.965 | 32.821 | 0.88x | 3840 | 5120 | 0.75x |
| `nested_holes` | ok | 33.564 | 34.689 | 0.97x | 4160 | 5440 | 0.76x |
| `strict_simple_stars` | ok | 106.079 | 116.587 | 0.91x | 5308 | 7040 | 0.75x |
| `open_paths_clip` | ok | 48.396 | 48.603 | 1.00x | 2560 | 4160 | 0.62x |
| `large_coord_xor` | ok | 40.688 | 47.965 | 0.85x | 4160 | 5760 | 0.72x |
| `offset_stars` | ok | 625.978 | 804.843 | 0.78x | 30720 | 32016 | 0.96x |
| `offset_open_round` | ok | 194.720 | 165.326 | 1.18x | 4480 | 5760 | 0.78x |
| `polytree_closed_nested` | ok | 13.107 | 20.001 | 0.66x | 2880 | 4160 | 0.69x |

Lower ratios are better for Rust. These are local benchmark numbers; rerun on the
target machine before making performance claims.

Arena migration comparison against the prior pointer-allocation Rust baseline:

| Case | Pointer Rust ms | Arena Rust ms | Speedup | Pointer RSS KB | Arena RSS KB | RSS change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `union_dense` | 48.727 | 44.615 | 1.09x | 4800 | 4800 | 1.00x |
| `touching_rect_grid` | 386.531 | 256.359 | 1.51x | 10880 | 10560 | 0.97x |
| `intersection_grid` | 85.559 | 68.542 | 1.25x | 5260 | 5120 | 0.97x |
| `difference_holes` | 33.819 | 28.965 | 1.17x | 3840 | 3840 | 1.00x |
| `nested_holes` | 32.407 | 33.564 | 0.97x | 4160 | 4160 | 1.00x |
| `strict_simple_stars` | 136.168 | 106.079 | 1.28x | 5760 | 5308 | 0.92x |
| `open_paths_clip` | 52.222 | 48.396 | 1.08x | 2560 | 2560 | 1.00x |
| `large_coord_xor` | 44.605 | 40.688 | 1.10x | 4160 | 4160 | 1.00x |
| `offset_stars` | 717.400 | 625.978 | 1.15x | 31136 | 30720 | 0.99x |
| `offset_open_round` | 199.119 | 194.720 | 1.02x | 4480 | 4480 | 1.00x |
| `polytree_closed_nested` | 17.691 | 13.107 | 1.35x | 2880 | 2880 | 1.00x |

## Translation Plan

The original staged plan is saved in `RUST_PORT_PLAN.md`.

The arena migration plan is saved in `ARENA_MIGRATION_PLAN.md`.
