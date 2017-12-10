//! Core Martinez-Rueda-Feito algorithm

use Point2D;
use fsize;
use ::std::cmp::Ordering;

#[derive(Debug, PartialEq, Copy, Clone, Eq)]
pub(crate) enum EventType {
    Left,
    Right,
}

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
    /// Event associated to the other endpoint of the edge
    pub other: *const SweepEvent<'a>,
    /// Polygon type
    pub pl: PolygonType,
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
            calculate_signed_area3(&self.p, unsafe { &(*self.other).p }, other) > 0.0
        } else {
            calculate_signed_area3(unsafe { &(*self.other).p }, &self.p, other) > 0.0
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
    fn compare(&self, other: &SweepEvent) -> bool {

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
        return self.above(unsafe { &(*other.other).p });
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

struct EventPoint {
    pub event_type: EventType,
}

/// Function to set inside and in_out flags for a left endpoint.
/// Mutates the current SweepEvent
///
/// ### Inputs
///
/// current: current SweepEvent
/// previous: immediate predecessor of SweepEvent
fn set_inside_flag_of_left_endpoint(current: &mut SweepEvent, previous: Option<&SweepEvent>) {
    match previous {
        None => {
            // If ple is None then current is the first event in S
            // and the flags can be trivially set to false.
            current.is_inside = false;
            current.in_out = false;
        },
        Some(prev) => {
            if (current as *const _ as *const usize) == (prev as *const _ as *const usize) {
                // both reference the same polygon
                current.is_inside = prev.is_inside;
                current.in_out    = !prev.in_out;
            } else {
                current.is_inside = !prev.in_out;
                current.in_out = !prev.is_inside;
            }
        }
    }
}

/// Signed area of the triangle (p0, p1, p2)
#[inline]
fn calculate_signed_area3(p0: &Point2D, p1: &Point2D, p2: &Point2D) -> fsize {
    (p0.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p0.y - p2.y)
}

/// Signed area of the triangle ( (0,0), p1, p2)
#[inline]
fn calculate_signed_area2(p0: &Point2D, p1: &Point2D) -> fsize {
    (-p1.x) * (p0.y - p1.y) - (-p1.y) * (p0.x - p1.x)
}

#[derive(PartialEq, Eq)]
enum Sign {
    Positive,
    Negative,
    Equal,
}

/// Sign of triangle (p1, p2, o)
/// Returns -1, 0 or +1
#[inline]
fn calculate_sign(p0: &Point2D, p1: &Point2D, o: &Point2D) -> Sign {
    let det = (p0.x - o.x) * (p1.y - o.y) - (p1.x - o.x) * (p0.y - o.y);
    if det < 0.0 {
        Sign::Negative
    } else if det > 0.0 {
        Sign::Positive
    } else {
        Sign::Equal
    }
}

#[inline]
fn is_point_in_triangle(s: &[Point2D; 2], o: &Point2D, p: &Point2D) -> bool {
    let sign_first = calculate_sign(&s[0], &s[1], p);
    (sign_first == calculate_sign(&s[1], o, p)) &&
    (sign_first == calculate_sign(o, &s[0], p))
}
