#![allow(
    dead_code,
    clippy::approx_constant,
    clippy::double_comparisons,
    clippy::excessive_precision,
    clippy::if_same_then_else,
    clippy::manual_clamp,
    clippy::manual_swap,
    clippy::missing_safety_doc,
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
    closed_paths_from_poly_tree_into, minkowski_diff_into, minkowski_sum_into,
    minkowski_sum_paths_into, open_paths_from_poly_tree, open_paths_from_poly_tree_into,
    orientation, point_in_polygon, poly_tree_to_paths, poly_tree_to_paths_into, simplify_polygon,
    simplify_polygon_into, simplify_polygons_into, simplify_polygons_mut,
};
pub use types::{
    CInt, CLIPPER_VERSION, ClipType, EndType, IntPoint, IntRect, JoinType, Path, Paths,
    PolyFillType, PolyNode, PolyNodeChildren, PolyTree, PolyType,
};
