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

pub use point::{Point2D, line_intersect};

#[cfg(not(use_double_precision))]
type fsize = f32;

#[cfg(use_double_precision)]
type fsize = f64;

/// Bounding box
#[derive(Debug, Clone)]
pub struct Bbox {
    pub(crate) top: fsize,
    pub(crate) right: fsize,
    pub(crate) bottom: fsize,
    pub(crate) left: fsize,
}

#[derive(Debug, Clone)]
pub struct MultiPolygon {
    pub(crate) polygons: Vec<Polygon>,
}

/// Modifying the nodes of a polygon must be done via a closure,
/// because if the points are modified, the bounding box has to be recomputed
#[derive(Debug, Clone)]
pub struct Polygon {
    pub(crate) nodes: Vec<Point2D>,
    pub(crate) bounding_box: Bbox,
}

impl MultiPolygon {

    /// Substracts a polygon from the current one
    ///
    /// If the current polygon is empty, returns None.
    pub fn subtract(&self, other: &Self) -> Option<Self> {

        if self.polygons.iter().all(|p| p.nodes.is_empty()) {
            return None;
        } else if other.polygons.iter().all(|p| p.nodes.is_empty()) {
            return Some(self.clone())
        }

        None
    }

    /// Intersect the current polygon with the other one, returning the area(s)
    /// where the two polygons intersect
    pub fn intersect(&self, other: &Self) -> Option<Self> {
        None
    }

    /// Combines the current polygon with the other one
    pub fn union(&self, b: &Self) -> Option<Self> {
        None
    }

    /// XORs the current polygon with the other one
    pub fn xor(&self, b: &Self) -> Option<Self> {
        None
    }

    /// Cuts two MultiPolygons into paths, returning the polygons
    /// including all possible intersection areas.
    ///
    /// Unioning all cuts will result
    pub fn cut(&self, b: &Self) -> Option<Vec<Polygon, >> {
        None
    }
}
