//! Core Martinez-Rueda-Feito algorithm

use Point2D;
use ::std::cmp::Ordering;

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
#[derive(PartialEq, Eq, Clone, Debug)]
pub(crate) struct SweepEvent<'a> {
    /// Point associated with the event
    pub p: &'a Point2D,
    /// other: Event associated to the other endpoint of the edge
    ///
    /// other_vec is the vec of the sweep event. The `Vec` cannot move, but
    /// the layout of the vector can grow and shrink.
    /// In order to avoid a dereference of an invalid pointer, we store a reference
    /// to the vector + and index (other_index)
    ///
    /// Rust does cannot modify a Vec if it's borrowed. This is why this is not a &'a reference -
    /// we sometimes still have to push to the end of vector. However, the index would stay the same.
    /// This is not the case when using a pointer. Maybe in the future this can be elided.
    pub other_vec: *const Vec<SweepEvent<'a>>,
    /// The index into the vector where the other value lives
    pub other_idx: usize,
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

impl<'a> SweepEvent<'a> {

    /// Is the line segment (p, other->p) below point x
    #[inline]
    pub fn below(&self, other: &Point2D) -> bool {
        if self.left {
            ::utils::calculate_signed_area3(&self.p, unsafe { &(*self.other_vec)[self.other_idx].p }, other) > 0.0
        } else {
            ::utils::calculate_signed_area3(unsafe { &(*self.other_vec)[self.other_idx].p }, &self.p, other) > 0.0
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
        let vec = unsafe { &*other.other_vec };
        return self.above(vec[other.other_idx].p);
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

impl<'a> Ord for SweepEvent<'a> {
    fn cmp(&self, other: &SweepEvent) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}
