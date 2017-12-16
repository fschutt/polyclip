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

#![feature(core)]
#![feature(generic_param_attrs)]
#![feature(dropck_eyepatch)]
#![feature(nonzero)]
#![feature(unique)]
#![feature(collections_range)]
#![feature(allocator_api)]
#![feature(core_intrinsics)]
#![feature(fused)]
extern crate core;

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
mod custom_btreeset;

pub use point::{Point2D, line_intersect};
pub use polygon::{Polygon, WindingOrder, calculate_bounding_box, calculate_winding_order};
pub use bbox::Bbox;
pub use utils::{calculate_signed_area2, calculate_signed_area3};

// TODO: Replace all (*thing.other_vec)[thing.other_idx]
// with (*thing.other_vec).get_unchecked(thing.other_idx)
