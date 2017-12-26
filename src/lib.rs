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
#![warn(unused_features)]
#![allow(unused_unsafe)]

// NOTE: These features are only because the `std::collections::BTreeSet`
// does not allow to immediately construct an iterator to the last inserted element,
// which is crucial for performance. For now, this repository contains a fork of the
// `std::collections::BTreeSet`, licensed under the Apache/MIT license
// (see the main Rust repository) for details.

// Tracking RFC: https://github.com/rust-lang/rfcs/pull/1194

// However, I need this NOW and not in two years, so this should explain the fork.
// A BTreeSet is just a BTreeMap with the value set to `()`, so an implementation
// is fairly trivial.

// Collections crate for Cursor-like behaviour for RBTrees
#[macro_use]
extern crate intrusive_collections;

#[cfg(not(use_double_precision))]
pub type fsize = f32;
#[cfg(use_double_precision)]
pub type fsize = f64;

macro_rules! inner {
    ($e:expr) => ((unsafe { &*$e.inner.get() }))
}

macro_rules! inner_mut {
    ($e:expr) => ((unsafe { &mut *$e.inner.get() }))
}

macro_rules! other {
    ($e:expr) => (unsafe { &(*(*inner!($e).other).inner.get()) })
}

macro_rules! other_mut {
    ($e:expr) => (unsafe { &mut (*(*inner!($e).other).inner.get()) })
}

mod bbox;
mod connector;
mod point;
mod sweep_event;
mod point_chain;
mod polygon;
mod segment;
mod utils;

pub use point::{Point2D, line_intersect};
pub use polygon::{Polygon, WindingOrder};
pub use bbox::Bbox;
pub use utils::{calculate_signed_area2,
                calculate_signed_area3,
                calculate_bounding_box,
                calculate_winding_order};

// TODO: Replace all (*thing.other_vec)[thing.other_idx]
// with (*thing.other_vec).get_unchecked(thing.other_idx)
