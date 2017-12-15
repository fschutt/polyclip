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
#![allow(unused_variables)]
#![allow(non_camel_case_types)]

#[cfg(not(use_double_precision))]
pub type fsize = f32;
#[cfg(use_double_precision)]
pub type fsize = f64;

mod bbox;
mod connector;
mod point;
mod sweep_event;
mod point_chain;
mod polygon;
mod segment;
mod utils;

pub use point::Point2D;
pub use polygon::Polygon;
pub use bbox::Bbox;

// TODO: Replace all (*thing.other_vec)[thing.other_idx]
// with (*thing.other_vec).get_unchecked(thing.other_idx)
