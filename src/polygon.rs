use {Point2D, Bbox, fsize};
use sweep_event::{SweepEvent, PolygonType, EdgeType};
use segment::Segment;
use std::collections::{BTreeSet, BinaryHeap};

/// Modifying the nodes of a polygon must be done via a closure,
/// because if the points are modified, the bounding box has to be recomputed
#[derive(Debug, Clone)]
pub struct Polygon {
    pub nodes: Vec<Point2D>,
    /// Is this polygon a hole?
    pub is_hole: bool,
    /// Is this polygon closed?
    pub is_closed: bool,
    /// Are the nodes of this polygon in a clockwise order?
    /// By default, this field is not calculated, due to performance reasons
    /// If you want to calculate it, call `calculate_winding(&self.nodes)`
    ///
    /// If you already know the winding order, please set it beforehand, to speed up
    /// the calculation.
    pub winding: Option<WindingOrder>,
}

/// Winding order
#[derive(Debug, Copy,Clone, PartialEq, Eq)]
pub enum WindingOrder {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum BoolOpType {
    Intersection,
    Union,
    Difference,
    Xor,
}

impl Default for Polygon {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            is_hole: false,
            is_closed: true,
            winding: None,
        }
    }
}

impl Polygon {

    pub fn subtract(&self, other: &Self)
    -> Option<Vec<Self>>
    {
        self.calculate(other, BoolOpType::Intersection)
    }

    pub fn union(&self, other: &Self)
    -> Option<Vec<Self>>
    {
        self.calculate(other, BoolOpType::Union)
    }

    pub fn difference(&self, other: &Self)
    -> Option<Vec<Self>>
    {
        self.calculate(other, BoolOpType::Difference)
    }

    pub fn xor(&self, other: &Self)
    -> Option<Vec<Self>>
    {
        self.calculate(other, BoolOpType::Xor)
    }

    /// Substracts a polygon from the current one
    ///
    /// If the current polygon is empty, returns None.
    #[inline(always)]
    fn calculate(&self, other: &Self, operation_type: BoolOpType)
    -> Option<Vec<Self>>
    {
        use self::BoolOpType::*;
        use connector::Connector;

        // Trivial result case - either self or other polygon do not exist
        // or they are lines. At the very least we need a triangle.
        if (self.nodes.len() * other.nodes.len()) == 0 {
            match operation_type {
                Difference => return Some(vec![self.clone()]),
                Intersection => return None,
                Union | Xor  => if self.nodes.is_empty() {
                    return Some(vec![other.clone()])
                } else {
                    return Some(vec![self.clone()])
                },
            }
        }

        // Trivial result case - one of the polygons is actually a line
        // Cannot subtract a polygon and a line (this may change in the future)
        if self.nodes.len() < 3 || other.nodes.len() < 3 {
            return None;
        }

        // Trivial result case - boundaries don't overlap
        // NOTE: This should not be done here, this should be done in the MultiPolygon
        // class (R* tree)
        let self_bbox = calculate_bounding_box(&self.nodes);
        let other_bbox = calculate_bounding_box(&other.nodes);

        if !self_bbox.overlaps(&other_bbox) {
            match operation_type {
                Difference => return Some(vec![self.clone()]),
                Intersection => return None,
                Union | Xor => return Some(vec![self.clone(), other.clone()])
            }
        }

        // Boolean operation is non-trivial

        // Create the sweep events
        let vec_of_sweep_events_subject = create_sweep_events(&self.nodes, PolygonType::Subject);
        let vec_of_sweep_events_clipping = create_sweep_events(&other.nodes, PolygonType::Clipping);
        // Sort the sweep events
        // Insert all the endpoints associated to the line segments into the event queue
        let mut event_queue = BinaryHeap::<SweepEvent>::with_capacity((self.nodes.len() * 2) + (other.nodes.len() * 2));

        for event in &vec_of_sweep_events_subject { event_queue.push(event.clone()); }
        for event in &vec_of_sweep_events_clipping { event_queue.push(event.clone()); }

        // -------------------------------------------------------------------- sweep events created

        let mut connector = Connector::new();
        let mut sweep_line = BTreeSet::<SweepEvent>::new();
        let mut event_holder = Vec::<SweepEvent>::new();

        let minimum_x_bbox_pt = self_bbox.right.min(other_bbox.right);

        // calculate the necessary events
        while let Some(event) = event_queue.pop() {

            // -----------------------------------------------------------------   optimization 1

            if (operation_type == Intersection && (event.p.x > minimum_x_bbox_pt)) ||
               (operation_type == Difference && (event.p.x > self_bbox.right)) {
                break;
            }

            if operation_type == Union && (event.p.x > minimum_x_bbox_pt) && !event.left {
                // add all the non-processed line segments to the result
                connector.add_segment(Segment::new(event.p, unsafe { (*event.other_vec)[event.other_idx].p }));
                while let Some(new_event) = event_queue.pop() {
                    if !new_event.left {
                        connector.add_segment(Segment::new(new_event.p, unsafe { (*new_event.other_vec)[event.other_idx].p }));
                    }
                }
                break;
            }

            // ---------------------------------------------------------------- end of optimization 1

            if event.left {
                // the current line segment must be inserted into the sweepline

            } else {
                // the current line segment must be removed into the sweepline

            }
        }

        return Some(connector.to_polygons());
    }
}

// DO NOT modify the return type, otherwise you will invalidate all internal pointers!
//
// __WARNING__: Extremely important that this is `#[inline(always)]`,
// otherwise the self-referential pointers get invalidated because of a move!
#[inline(always)]
fn create_sweep_events(nodes: &[Point2D], polygon_type: PolygonType) -> Vec<SweepEvent> {

    let vec_len = nodes.len() * 2;
    let mut new_vec = Vec::<SweepEvent>::with_capacity(vec_len);
    unsafe { new_vec.set_len(vec_len); }

    let iter1 = nodes.iter();
    let mut iter2 = nodes.iter().cycle();
    iter2.next();

    let mut cur_pt_idx = 0;
    for (cur_point, next_point) in iter1.zip(iter2) {

        let mut e1_left = true;
        let mut e2_left = true;

        if cur_point.x < next_point.x {
            e2_left = false;
        } else if cur_point.x > next_point.x {
            e1_left = false;
        } else if cur_point.y < next_point.y {
            // The line segment is vertical.
            // The bottom endpoint is the left endpoint
            e2_left = false;
        } else {
            e1_left = false;
        }

        let e1_idx = cur_pt_idx;
        let e2_idx = cur_pt_idx + 1;

        let e1 = SweepEvent {
            p: cur_point,
            other_vec: &new_vec,
            other_idx: e2_idx,
            left: e1_left,
            position_in_sweep_line: 0,
            polygon_type: polygon_type,
            in_out: false,
            is_inside: false,
            edge_type: EdgeType::Normal,
        };

        let e2 = SweepEvent {
            p: next_point,
            other_vec: &new_vec,
            other_idx: e1_idx,
            position_in_sweep_line: 0,
            left: e2_left,
            polygon_type: polygon_type,
            in_out: false,
            is_inside: false,
            edge_type: EdgeType::Normal,
        };

        new_vec[e1_idx] = e1;
        new_vec[e2_idx] = e2;

        cur_pt_idx += 2;
    }

    // assert that the vector does not have moved (in memory)
    // if it did, the internal pointer would be garbage
    assert_eq!(new_vec.len(), vec_len);
    assert_eq!(new_vec.capacity(), vec_len);

    new_vec
}

fn calculate_winding(nodes: &[Point2D]) -> WindingOrder {
    // cannot happen, since the parent function should
    // take care of early returning on invalid polygons
    assert!(nodes.len() > 2);

    let iter1 = nodes.iter();
    let mut iter2 = nodes.iter().cycle();
    iter2.next();

    // shoelace formula
    let sum: fsize = iter1.zip(iter2).map(|(p0, p1)| (p1.x - p0.x) * (p1.y + p0.y)).sum();
    match sum > 0.0 {
        true  => WindingOrder::Clockwise,
        false => WindingOrder::CounterClockwise,
    }
}

/// Calculates the bounding box of all points in the nodes
fn calculate_bounding_box(nodes: &[Point2D]) -> Bbox {

    #[cfg(not(use_double_precision))]
    let mut min_x = ::std::f32::MAX;

    #[cfg(use_double_precision)]
    let mut min_x = ::std::f64::MAX;

    let mut min_y = min_x;

    #[cfg(not(use_double_precision))]
    let mut max_x = -(::std::f32::MAX);

    #[cfg(use_double_precision)]
    let mut max_x = -(::std::f64::MAX);

    let mut max_y = max_x;

    for node in nodes {
        if node.x > max_x {
            max_x = node.x;
        }
        if node.x < min_x {
            min_x = node.x;
        }
        if node.y > max_y {
            max_y = node.y;
        }
        if node.y < min_y {
            min_y = node.y;
        }
    }

    Bbox {
        top: max_y,
        bottom: min_y,
        left: min_x,
        right: max_x,
    }
}

// event = event_vec[event_idx]
//
// __WARNING__: It is assumed that `event_vec` == `event.other_vec`, i.e. that
// event_vec only stores references to itself.
//
// After this function has run once, this is not the case anymore, some references
// will be swapped for the `event_holder`
//
// This function essentially moves events from the event_vec to the event_holder
//
// Do not call this function on the same index twice!
fn divide_segment<'a>(event_vec: &'a mut Vec<SweepEvent<'a>>,
                      event_idx: usize,
                      divide_pt: &'a Point2D,
                      event_holder: &mut Vec<SweepEvent<'a>>,
                      eq: &mut BinaryHeap<SweepEvent<'a>>)
{
    let event_clone = event_vec[event_idx].clone();
    {
        // push right event
        event_holder.push(SweepEvent {
            p: divide_pt,
            left: false,
            polygon_type: event_clone.polygon_type,
            other_vec: event_clone.other_vec,
            other_idx: event_clone.other_idx,
            in_out: false,
            position_in_sweep_line: 0,
            is_inside: false,
            edge_type: event_clone.edge_type,
        });

        // push left event
        event_holder.push(SweepEvent {
            p: divide_pt,
            left: true,
            polygon_type: event_clone.polygon_type,
            other_vec: event_clone.other_vec,
            other_idx: event_clone.other_idx,
            in_out: false,
            position_in_sweep_line: 0,
            is_inside: false,
            edge_type: unsafe { (*event_clone.other_vec)[event_clone.other_idx].edge_type },
        });
    }

    let last = event_holder.len() - 1;

    {
        let left = event_holder.get_mut(last).unwrap();

        if left.compare(unsafe { &(*event_clone.other_vec)[event_clone.other_idx] }) {
            // avoid a rounding error. The left event would be processed after the right event
            // original: (*event.other_vec)[event.other_idx].left = true
            //
            // assumes that `event.other_vec` == `event_vec`
            event_vec[event_clone.other_idx].left = true;
            left.left = false;
        }
    }

    let left  = event_holder.get(last).unwrap();
    let right = event_holder.get(last - 1).unwrap();

    // original code:

    /*
        event->other->other = left; // this probably will be invalidate when the event_holder resizes !!!
        event->other = right;       // in the Rust version, it is replaced by an index
        eq.push(left);
        eq.push(right);
    */

    let event_other_idx = event_vec[event_idx].other_idx;

    // assumes that `event.other_vec` == `event_vec`
    event_vec[event_other_idx].other_vec = event_holder;
    event_vec[event_other_idx].other_idx = last;

    event_vec[event_idx].other_vec = event_holder;
    event_vec[event_idx].other_idx = last - 1;

    eq.push(left.clone());
    eq.push(right.clone());
}

macro_rules! other {
    ($e:expr) => (unsafe { (*$e.other_vec).get_unchecked($e.other_idx)})
}

fn possible_intersection(e1: &SweepEvent, e2: &SweepEvent) {

    let result = ::point::line_intersect(&e1.p, &other!(e1).p, &e2.p, &other!(e2).p);
    let (a, b) = match result {
        Some(a) => (a.0, a.1),
        None => return,
    };


    /*
    Point ip1, ip2;  // intersection points
    int nintersections;

    if (!(nintersections = findIntersection(e1->segment (), e2->segment (), ip1, ip2)))
        return;

    if ((nintersections == 1) && ((e1->p == e2->p) || (e1->other->p == e2->other->p)))
        return; // the line segments intersect at an endpoint of both line segments

    if (nintersections == 2 && e1->pl == e2->pl) {  // the line segments overlap, but they belong to the same polygon
        std::cerr << "A polygon has overlapping edges. Sorry, but the program does not work yet with this kind of polygon\n";
        exit (1);
    }

    // The line segments associated to e1 and e2 intersect
    nint += nintersections;

    if (nintersections == 1) {
        if (e1->p != ip1 && e1->other->p != ip1)  // if ip1 is not an endpoint of the line segment associated to e1 then divide "e1"
            divideSegment (e1, ip1);
        if (e2->p != ip1 && e2->other->p != ip1)  // if ip1 is not an endpoint of the line segment associated to e2 then divide "e2"
            divideSegment (e2, ip1);
        return;
    }

    // The line segments overlap
    vector<SweepEvent *> sortedEvents;
    if (e1->p == e2->p) {
        sortedEvents.push_back (0);
    } else if (sec (e1, e2)) {
        sortedEvents.push_back (e2);
        sortedEvents.push_back (e1);
    } else {
        sortedEvents.push_back (e1);
        sortedEvents.push_back (e2);
    }
    if (e1->other->p == e2->other->p) {
        sortedEvents.push_back (0);
    } else if (sec (e1->other, e2->other)) {
        sortedEvents.push_back (e2->other);
        sortedEvents.push_back (e1->other);
    } else {
        sortedEvents.push_back (e1->other);
        sortedEvents.push_back (e2->other);
    }

    if (sortedEvents.size () == 2) { // are both line segments equal?
        e1->type = e1->other->type = NON_CONTRIBUTING;
        e2->type = e2->other->type = (e1->inOut == e2->inOut) ? SAME_TRANSITION : DIFFERENT_TRANSITION;
        return;
    }
    if (sortedEvents.size () == 3) { // the line segments share an endpoint
        sortedEvents[1]->type = sortedEvents[1]->other->type = NON_CONTRIBUTING;
        if (sortedEvents[0])         // is the right endpoint the shared point?
            sortedEvents[0]->other->type = (e1->inOut == e2->inOut) ? SAME_TRANSITION : DIFFERENT_TRANSITION;
         else                               // the shared point is the left endpoint
            sortedEvents[2]->other->type = (e1->inOut == e2->inOut) ? SAME_TRANSITION : DIFFERENT_TRANSITION;
        divideSegment (sortedEvents[0] ? sortedEvents[0] : sortedEvents[2]->other, sortedEvents[1]->p);
        return;
    }
    if (sortedEvents[0] != sortedEvents[3]->other) { // no line segment includes totally the other one
        sortedEvents[1]->type = NON_CONTRIBUTING;
        sortedEvents[2]->type = (e1->inOut == e2->inOut) ? SAME_TRANSITION : DIFFERENT_TRANSITION;
        divideSegment (sortedEvents[0], sortedEvents[1]->p);
        divideSegment (sortedEvents[1], sortedEvents[2]->p);
        return;
    }
     // one line segment includes the other one
    sortedEvents[1]->type = sortedEvents[1]->other->type = NON_CONTRIBUTING;
    divideSegment (sortedEvents[0], sortedEvents[1]->p);
    sortedEvents[3]->other->type = (e1->inOut == e2->inOut) ? SAME_TRANSITION : DIFFERENT_TRANSITION;
    divideSegment (sortedEvents[3]->other, sortedEvents[2]->p);
    */
}

/*
// Not sure what this does or if this is needed
fn number_of_intersections(u0: fsize, u1: fsize, v0: fsize, v1: fsize, w: &mut [fsize; 2]) -> u8 {
    if (u1 < v0) || (u0 > v1) {
        return 0;
    }
    if u1 > v0 {
        if u0 < v1 {
            w[0] = if u0 < v0 { v0 } else { u0 };
            w[1] = if u1 > v1 { v1 } else { u1 };
            return 2;
        } else {
            // u0 == v1
            w[0] = u0;
            return 1;
        }
    } else {
        // u1 == v0
        w[0] = u1;
        return 1;
    }
}
*/
