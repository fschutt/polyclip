//! `polyclip` is a library to do boolean operations on polygons
//! A and B, with the operations:
//!
//! - Union (OR): Resulting polygon contains A and B
//! - Intersection (AND): Resulting polygon(s) is contained in both A and B
//! - Exclusive-Or (XOR): Resulting polygon(s) contains A and B, except for the Intersection of A and B
//! - Subtract (A_NOT_B): Resulting polygon(s) contains A, except for the Intersection of A and B
//! - Cut (CUT): Resulting polygon(s) contains all possible intersections of A and B,
//!   however the nodes don't have to be a valid polygon

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(unused_variables)]

mod point;
mod algorithm;
mod polygon;

pub use point::{Point2D, line_intersect};

#[cfg(not(use_double_precision))]
pub type fsize = f32;
#[cfg(use_double_precision)]
pub type fsize = f64;

/// Bounding box
#[derive(Debug, Clone)]
pub struct Bbox {
    pub(crate) top: fsize,
    pub(crate) right: fsize,
    pub(crate) bottom: fsize,
    pub(crate) left: fsize,
}

