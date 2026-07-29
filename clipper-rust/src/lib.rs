#![allow(
    clippy::approx_constant,
    clippy::double_comparisons,
    clippy::excessive_precision,
    clippy::if_same_then_else,
    clippy::manual_clamp,
    clippy::manual_swap,
    clippy::missing_safety_doc,
    clippy::needless_range_loop,
    clippy::useless_vec
)]

pub mod clipper;
pub mod clipper_base;
pub mod clipper_offset;
pub mod error;
pub mod helpers;
pub mod types;

pub use error::{ClipperError, Result};
pub use helpers::*;
pub use types::*;
