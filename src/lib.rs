//! Pure Rust port of the C++ [Clipper](https://github.com/Geri-Borbas/Clipper) 6.4.2
//! polygon clipping and offsetting library.
//!
//! Boolean operations (union, intersection, difference, XOR) on closed polygons and
//! open polylines are performed by [`Clipper`]; polygon and polyline offsetting is
//! performed by [`ClipperOffset`]. Coordinates are 64-bit integers ([`CInt`]), so
//! results are exact and reproducible.
//!
//! ```
//! use geo_clipper_pure_rs::{ClipType, Clipper, IntPoint, PolyFillType, PolyType};
//!
//! let a = vec![
//!     IntPoint::new(0, 0),
//!     IntPoint::new(10, 0),
//!     IntPoint::new(10, 10),
//!     IntPoint::new(0, 10),
//! ];
//! let b = vec![
//!     IntPoint::new(5, 5),
//!     IntPoint::new(15, 5),
//!     IntPoint::new(15, 15),
//!     IntPoint::new(5, 15),
//! ];
//!
//! let mut clipper = Clipper::new();
//! clipper.add_path(&a, PolyType::Subject, true)?;
//! clipper.add_path(&b, PolyType::Subject, true)?;
//!
//! let solution = clipper.execute(ClipType::Union, PolyFillType::NonZero)?;
//! assert_eq!(solution.len(), 1);
//! # Ok::<(), geo_clipper_pure_rs::ClipperError>(())
//! ```
//!
//! Inputs are accepted as slices where possible and execution methods return owned
//! [`Paths`] or [`PolyTree`] values; `_into` variants let callers reuse output
//! allocations. See the crate README for port design notes and benchmarks.

#![allow(
    dead_code,
    clippy::approx_constant,
    clippy::double_comparisons,
    clippy::excessive_precision,
    clippy::if_same_then_else,
    clippy::manual_clamp,
    clippy::manual_swap,
    clippy::missing_safety_doc,
    // Explicit borrows of raw-pointer derefs are deliberate: removing them trips the
    // rustc `dangerous_implicit_autorefs` lint.
    clippy::needless_borrow,
    clippy::needless_range_loop,
    clippy::useless_vec,
    clippy::vec_box
)]

mod clipper;
mod clipper_base;
mod clipper_offset;
mod error;
mod helpers;
mod types;

pub use clipper::{Clipper, ClipperOptions};
pub use clipper_offset::ClipperOffset;
pub use error::{ClipperError, Result};
pub use helpers::{
    area, clean_polygon, clean_polygon_into, clean_polygon_mut, clean_polygons,
    clean_polygons_into, clean_polygons_mut, closed_paths_from_poly_tree,
    closed_paths_from_poly_tree_into, is_counter_clockwise, minkowski_diff_into,
    minkowski_sum_into, minkowski_sum_paths_into, open_paths_from_poly_tree,
    open_paths_from_poly_tree_into, orientation, point_in_polygon, poly_tree_to_paths,
    poly_tree_to_paths_into, simplify_polygon, simplify_polygon_into, simplify_polygons_into,
    simplify_polygons_mut,
};
pub use types::{
    AddPathResult, CInt, CLIPPER_VERSION, ClipType, EndType, IntPoint, IntRect, JoinType,
    Orientation, Path, Paths, PointLocation, PolyFillType, PolyNode, PolyNodeChildren, PolyTree,
    PolyType,
};
