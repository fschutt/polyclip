//! Core Martinez-Rueda-Feito algorithm

use Point2D;
use ::std::cmp::Ordering;
use std::cell::UnsafeCell;

/// Wrapper struct so we can implement `Ord` on `UnsafeCell<SweepEvent>`
#[derive(Debug)]
pub(crate) struct SweepEventRef<'a> {
    pub(crate) inner: UnsafeCell<SweepEvent<'a>>
}

impl<'a> PartialEq for SweepEventRef<'a> {
    fn eq(&self, other: &SweepEventRef) -> bool {

        // The eq does not work otherwise. For some reason you cannot change
        // the lifetime in trait methods. For this to work, I'd have to use
        // &'a self and &'a SweepEventRef<'a> in the function arguments
        // but Rust does not allow this, because then the method is incompatible
        // with the trait.

        let a = unsafe { self.inner.get() };
        let b = unsafe { other.inner.get() };

        unsafe {
            (*a).p == (*b).p &&
            (*a).other as usize == (*b).other as usize &&
            (*a).polygon_type == (*b).polygon_type &&
            (*a).position_in_sweep_line == (*b).position_in_sweep_line &&
            (*a).left == (*b).left &&
            (*a).in_out == (*b).in_out &&
            (*a).is_inside == (*b).is_inside &&
            (*a).edge_type == (*b).edge_type
        }
    }
}

impl<'a> Eq for SweepEventRef<'a> { }

/// Indicates if the edge belongs to the subject or clipping polygon
#[derive(Debug, PartialEq, Copy, Clone, Eq)]
pub(crate) enum PolygonType {
    Subject,
    Clipping,
}

#[derive(Debug, PartialEq, Copy, Clone, Eq)]
pub(crate) enum EdgeType {
    Normal,
    NonContributing,
    SameTransition,
    DifferentTransition,
}

// NOTE: Rust does struct layout optimization. It is useless to use bitfields here,
// Rust creates bitfields automatically. The size of the SweepEvent is 24 bytes total
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SweepEvent<'a> {
    /// Point associated with the event
    pub p: &'a Point2D,
    /// other: Event associated to the other endpoint of the edge
    pub other: *const SweepEventRef<'a>,
    /// Polygon type
    pub polygon_type: PolygonType,
    /// Only used in "left" events. Index of the event (line segment) in the current sweep line.
    ///
    /// This value is called "poss" in the C++ code, I thought it should be more readable
    pub position_in_sweep_line: usize,
    /// Is the point the left endpoint of the edge (p or other->p)?
    pub left: bool,
    /// Inside-outside transition into the polygon
    ///
    /// Indicates if the edge determines an inside-outside
    /// transition into the polygon, to which the edge
    /// belongs, for a vertical semi-line that goes up and
    /// intersects the edge
    pub in_out: bool,
    /// Is the edge (p, other->p) inside the other polygon?
    /// inside: indicates if the edge is inside the other polygon.
    pub is_inside: bool,
    /// Used for overlapping edges
    pub edge_type: EdgeType,
}

impl<'a> PartialOrd for SweepEvent<'a> {
    fn partial_cmp(&self, other: &SweepEvent) -> Option<Ordering> {
        // Return true means that other is placed at the event queue after self
        // i.e,, self is processed by the algorithm after other
        if self.compare(other) {
            Some(Ordering::Greater)
        } else {
            Some(Ordering::Less)
        }
    }
}

impl<'a> PartialOrd for SweepEventRef<'a> {
    fn partial_cmp(&self, other: &SweepEventRef) -> Option<Ordering> {
        // Return true means that other is placed at the event queue after self
        // i.e,, self is processed by the algorithm after other
        if inner!(self).compare(inner!(other)) {
            Some(Ordering::Greater)
        } else {
            Some(Ordering::Less)
        }
    }
}

impl<'a> Ord for SweepEventRef<'a> {
    fn cmp(&self, other: &SweepEventRef) -> Ordering {
        unsafe {
            if (*self.inner.get()).compare(&(*other.inner.get())) {
                Ordering::Greater
            } else {
                Ordering::Less
            }
        }
    }
}

impl<'a> Ord for SweepEvent<'a> {
    fn cmp(&self, other: &SweepEvent) -> Ordering {
        if self.compare(other) {
            Ordering::Greater
        } else {
            Ordering::Less
        }
    }
}


impl<'a> SweepEvent<'a> {

    /// Is the line segment (p, other->p) below point x
    #[inline]
    pub fn below(&self, other: &Point2D) -> bool {
        unsafe {
            if self.left {
                ::utils::calculate_signed_area3(&self.p, unsafe { &(*(*self.other).inner.get()).p }, other) > 0.0
            } else {
                ::utils::calculate_signed_area3(unsafe { &(*(*self.other).inner.get()).p }, &self.p, other) > 0.0
            }
        }
    }

    /// Is the line segment (p, other->p) above point x
    #[inline]
    pub fn above(&self, other: &Point2D) -> bool {
        return !self.below(other);
    }

    // Return true means that self is placed at the event queue after other
    // i.e,, self is processed by the algorithm after other
    #[inline]
    pub(crate) fn compare(&self, other: &SweepEvent) -> bool {

        if self.p.x > other.p.x {
            return true;
        }

        if other.p.x > self.p.x {
            return false;
        }

        // Different points, but same x coordinate
        // The event with lower y-coordinate is processed first
        if self.p != other.p {
            return self.p.y > other.p.y;
        }

        // Same point, but one is a left endpoint and the other a right endpoint.
        // The right endpoint is processed first
        if self.left != other.left {
            return self.left;
        }

        // Same point, both events are left endpoints or both are right endpoints.
        // The event associated to the bottom segment is processed first
        return unsafe { self.above(&(*(*self.other).inner.get()).p) };
    }

    /// Function to set inside and in_out flags for a left endpoint.
    /// Mutates the current SweepEvent
    ///
    /// ### Inputs
    ///
    /// current: current SweepEvent
    /// previous: immediate predecessor of SweepEvent
    pub(crate) fn set_inside_flag_of_left_endpoint(&mut self, previous: Option<&SweepEvent>) {
        match previous {
            None => {
                // If ple is None then current is the first event in S
                // and the flags can be trivially set to false.
                self.is_inside = false;
                self.in_out = false;
            },
            Some(prev) => {
                if (self as *const _ as *const usize) == (prev as *const _ as *const usize) {
                    // both reference the same polygon
                    self.is_inside = prev.is_inside;
                    self.in_out    = !prev.in_out;
                } else {
                    self.is_inside = !prev.in_out;
                    self.in_out = !prev.is_inside;
                }
            }
        }
    }
}
