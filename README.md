# geo-clipper-pure-rs

Rust port of the C++ [Clipper](https://github.com/Geri-Borbas/Clipper) polygon clipping library. (vs 6.4.2, commit `d59f4d02e0d28fd7ee88d8b7dc93ff5f0806b7b2`.)

* 2026-07-30: Initial port. Parity on this crate is tricky, see port design notes



## Port design choices

The Clipper library does not perform precise clipping; if data contains ties, the result
will vary depending on underlying C++ library implementation (e.g. unstable sort order).
This is hard to replicate in Rust, and also not very meaningful; instead, the translation
uses a stable sort, and while it will sometimes deviate vs the original C++ code, the
output is guaranteed to be the same for future version of Rust.

Memory allocation is done in an "arena" instead of pointers, improving cache locality
and (often) speed. This also plays better with the memory model of Rust. Some hot code
algorithms have been rewritten to better accomodate this data structure.

The original code defines features based on DEFINE's. Translation is made assuming:

- Coordinate type: `i64`
- C++ `use_int32`: disabled
- C++ `use_xyz`: disabled
- C++ `use_lines`: enabled
- C++ `use_deprecated`: disabled
- Internal linked structures currently use raw pointers


## Crate

```bash
cargo test
```

The public API is exposed from the crate root. The translated implementation modules
are private so raw pointer internals and C++ support structs do not become part of
the crate interface.

Main public types and functions:

- `Clipper`, `ClipperOptions`, and `ClipperOffset`
- `IntPoint`, `IntRect`, `Path`, `Paths`, and Clipper enums. `Path` and
  `Paths` are thin owned newtypes over point and path vectors, with slice access
  through deref and `AsRef`.
- typed status/query results such as `AddPathResult`, `Orientation`, and
  `PointLocation`
- helpers such as signed `area`, `orientation`, `is_counter_clockwise`,
  `point_in_polygon`, simplification, cleaning, Minkowski operations, and
  `PolyTree` path extraction

Example:

```rust
use geo_clipper_pure_rs::{ClipType, Clipper, IntPoint, PolyFillType, PolyType};

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

let solution = clipper.execute(ClipType::Union, PolyFillType::NonZero)?;
# Ok::<(), geo_clipper_pure_rs::ClipperError>(())
```

Inputs are accepted as slices where possible, and normal execution methods return
owned `Paths` or `PolyTree` values. `_into` variants are available when callers want
to reuse output allocations. `add_path` and `add_paths` return `AddPathResult`
so callers can distinguish paths that were inserted from valid-but-degenerate
paths that were skipped.

Options use a builder-style value:

```rust
use geo_clipper_pure_rs::{Clipper, ClipperOptions};

let mut clipper = Clipper::with_options(
    ClipperOptions::new()
        .strictly_simple(true)
        .preserve_collinear(true),
);
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

Run all benchmark cases:

```bash
bash bench/run_benchmarks.sh
```

The benchmark CLI is behind the default-off `bench-bin` feature; the script enables
it when building `src/bin/bench.rs`.

Run with more iterations per case:

```bash
bash bench/run_benchmarks.sh 10
```

Output is CSV:

```text
case,parity,rust_elapsed_ms,cpp_elapsed_ms,rust_wall_s,cpp_wall_s,rust_rss_kb,cpp_rss_kb,rust_paths,cpp_paths,rust_points,cpp_points,rust_area_abs,cpp_area_abs,rust_checksum,cpp_checksum
```

Per-engine raw outputs are written to `target/bench/`.

Latest local optimized run, ten iterations per case:

| Case | Parity | Rust ms | C++ ms | Rust/C++ time | Rust RSS KB | C++ RSS KB | Rust/C++ RSS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `union_dense` | ok | 48.200 | 49.476 | 0.97x | 4800 | 6080 | 0.79x |
| `touching_rect_grid` | ok | 307.238 | 377.723 | 0.81x | 10560 | 11840 | 0.89x |
| `intersection_grid` | ok | 61.964 | 85.752 | 0.72x | 5120 | 6468 | 0.79x |
| `difference_holes` | ok | 29.599 | 33.683 | 0.88x | 3840 | 5120 | 0.75x |
| `nested_holes` | ok | 31.965 | 35.227 | 0.91x | 4160 | 5760 | 0.72x |
| `strict_simple_stars` | ok | 108.196 | 134.949 | 0.80x | 5316 | 7040 | 0.76x |
| `open_paths_clip` | ok | 48.058 | 63.586 | 0.76x | 2560 | 3840 | 0.67x |
| `large_coord_xor` | ok | 40.709 | 48.307 | 0.84x | 4160 | 5760 | 0.72x |
| `offset_stars` | ok | 588.632 | 751.390 | 0.78x | 30720 | 32024 | 0.96x |
| `offset_open_round` | ok | 166.786 | 186.774 | 0.89x | 4480 | 5760 | 0.78x |
| `polytree_closed_nested` | ok | 14.870 | 18.421 | 0.81x | 2880 | 4160 | 0.69x |

Lower ratios are better for Rust. These are local benchmark numbers; rerun on the
target machine before making performance claims.
