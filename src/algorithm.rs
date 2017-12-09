//! Core Martinez-Rueda-Feito algorithm

use Polygon;
use Point2D;
use fsize;
use ::std::cmp::Ordering;

/// Edge structure between exactly two points
struct Edge<'a> {
    /// The polygon this edge references
    pub polygon: &'a Polygon,
    /// Index of the point at which the point begins
    pub left_index: usize,
    /// Index of the point that ends the edge
    pub right_index: usize,
}

#[derive(PartialEq, Copy, Clone, Eq)]
pub(crate) enum EventType {
    Left,
    Right,
}

/// Indicates if the edge belongs to the subject or clipping polygon
#[derive(PartialEq, Copy, Clone, Eq)]
pub(crate) enum PolygonType {
    Subject,
    Clipping,
}

#[derive(PartialEq, Copy, Clone, Eq)]
pub(crate) enum EdgeType {
    Normal,
    NonContributing,
    SameTransition,
    DifferentTransition,
}

// NOTE: Rust does struct layout optimization. It is useless to use bitfields here,
// Rust creates bitfields automatically. The size of the SweepEvent is 24 bytes total
#[derive(PartialEq)]
pub(crate) struct SweepEvent<'a> {
    /// Point associated with the event
    pub p: &'a Point2D,
    /// Event associated to the other endpoint of the edge
    ///
    /// If this is `None`, this event has not been initialized yet
    /// (in the C++ code, this is a null pointer)
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
            calculate_signed_area3(&self.p, &(*self.other).p, other) > 0.0
        } else {
            calculate_signed_area3(&(*self.other).p, &self.p, other) > 0.0
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
        return self.above(&(*other.other).p);
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


/// Core function to calculate the intersection
fn calculate_intersect() {
    /*
    Insert the endpoints of the edges of polygons into priority queue Q

    while !Q.is_empty() {
        let event = Q.top();
        Q.pop();

        if (event.left_endpoint()) {
            // left endpoint
            let pos = S.insert(event);
            event.set_inside_other_polygon_flag(S.prev(pos));
            possible_inter(pos, S.next(pos));
            possible_inter(pos, S.prev(pos));
        } else {
            // right endpoint
            let pos = S.find(*event.other);
            let next = S.next(pos);
            let pref = S.prev(pos);
            if event.inside_other_polygon() {
                intersection.add(event.segment());
            } else {
                Union.add(event.segment());
            }
            S.erase(pos);
            possible_inter(prev, next);
        }
    }
    */
}

/*
    We must hold a set C - initially empty - of chains of
    connected edges and a set R that holds the result
    polygons.

    Every edge e that belongs to the solution must
    be processed as follows:

    - If e cannot be connected at any of the ends of any chain
    of C, then a new chain, formed by e, is added to C.

    - If e can be connected to only one chain c of C, then e is
    added to c. If the first and last edges in c are connected,
    then c holds a result polygon and it is moved to R.

    - If e can be connected to two chains c1 and c2 of C, then
    the edges of c2 and e are added to c1, and c2 is removed
    from C. If the first and last edges in c1 are connected
    then c1 is moved to R.
*/

/*
Martinez mr (subj, clip);
mr.compute (op, martinezResult);
*/

/// Subdivide the edges of the polygons at their intersection points.
fn subdivide_edges() {

}

/// Select those subdivided edges that lie inside the other
/// polygon—or that do not lie depending on the operation.
fn select_edges() { }

/// Join the edges selected in step 2 to form the result polygon.
fn join_edges() { }
