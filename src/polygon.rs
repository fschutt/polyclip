use sweep_event::{SweepEvent, SweepEventRef, PolygonType, EdgeType};
use segment::Segment;
use std::collections::BinaryHeap;
use custom_btreeset::set::BTreeSet;
use std::cell::UnsafeCell;
use Point2D;

/// Modifying the nodes of a polygon must be done via a closure,
/// because if the points are modified, the bounding box has to be recomputed
#[derive(Debug, Clone)]
pub struct Polygon {
    /// The points that this polygon is made of
    pub nodes: Vec<Point2D>,
    /// Is this polygon a hole?
    pub is_hole: bool,
    /// Is this polygon closed?
    pub is_closed: bool,
    /// Are the nodes of this polygon in a clockwise order?
    /// By default, this field is not calculated, due to performance reasons
    /// If you want to calculate it, call `calculate_winding_order(&self.nodes)`
    ///
    /// If you already know the winding order, please set it beforehand, to speed up
    /// the calculation.
    pub winding: Option<WindingOrder>,
}

/// Winding order of a polygon
#[derive(Debug, Copy,Clone, PartialEq, Eq)]
pub enum WindingOrder {
    Clockwise,
    CounterClockwise,
}

/// Only used for internal operations: type of boolean
/// operation to perform on the polygons
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

    /// Substracts a polygon from the current one
    ///
    /// If the current polygon is empty, returns None.
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

    // NOTE: The method should be inlined, because this will elide the `operation_type`
    // tests, which will make the whole thing faster. The function will be inlined four times,
    // one for each `BoolOpType`.
    #[inline(always)]
    fn calculate(&self, other: &Self, operation_type: BoolOpType)
    -> Option<Vec<Self>>
    {
        use self::BoolOpType::*;
        use self::EdgeType::*;
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
        let self_bbox = ::utils::calculate_bounding_box(&self.nodes);
        let other_bbox = ::utils::calculate_bounding_box(&other.nodes);

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
        let mut event_queue = BinaryHeap::<&SweepEventRef>::with_capacity((self.nodes.len() * 2) + (other.nodes.len() * 2));

        for event in &*vec_of_sweep_events_subject {
            event_queue.push(event);
        }

        for event in &*vec_of_sweep_events_clipping {
            event_queue.push(event);
        }

        // -------------------------------------------------------------------- sweep events created

        let mut connector = Connector::new();
        let mut sweep_line = BTreeSet::<&SweepEventRef>::new();
        let mut event_holder = Vec::<SweepEventRef>::new();

        let minimum_x_bbox_pt = self_bbox.right.min(other_bbox.right);

        // calculate the necessary events
        while let Some(mut event) = event_queue.pop() {

            // -----------------------------------------------------------------   optimization 1

            if (operation_type == Intersection && (inner!(event).p.x > minimum_x_bbox_pt)) ||
               (operation_type == Difference && (inner!(event).p.x > self_bbox.right)) {
                break;
            }

            if operation_type == Union && (inner!(event).p.x > minimum_x_bbox_pt) && !inner!(event).left {
                // add all the non-processed line segments to the result
                connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                while let Some(new_event) = event_queue.pop() {
                    if !inner!(new_event).left {
                        connector.add_segment(Segment::new(inner!(new_event).p, other!(new_event).p));
                    }
                }
                break;
            }

            // ---------------------------------------------------------------- end of optimization 1

            if inner!(event).left {
                // the current line segment must be inserted into the sweepline

                // NOTE: This won't work correctly. A BTreeSet cannot be indexed,
                // since it is not contigouus in memory. This should return an
                // interator instead, so that we can use .next() and the like.
                //
                // Returning a number as an index is only a placeholder and will 100%
                // crash at runtime
                let event_pos_in_sweep_line = sweep_line.insert_return_index(event);

                // Also: Note that we are assigning to event here.
                // Not sure if event should be a &mut Event
                inner_mut!(event).position_in_sweep_line = event_pos_in_sweep_line;

                let it = event_pos_in_sweep_line;
                let next = event_pos_in_sweep_line;
                let mut prev = event_pos_in_sweep_line;

                // TODO: does the sweep line get modified after this initial insert?
                // If yes, the it iterator is invalid

                let sweep_line_len = sweep_line.len();
                // make "prev" wrap around
                if prev != 0 {
                    prev -= 1;
                } else {
                    prev = sweep_line_len;
                }

                if prev == sweep_line_len {
                    // there is not a previous line segment in S?
                    inner!(event).is_inside = false;
                    inner!(event).in_out = false;
                } else if sweep_line.map.keys_mut()[prev].edge_type != EdgeType::Normal {
                    if prev == 0 {
                        inner!(event).is_inside = true; // it is not relevant to set true or false
                        inner!(event).in_out = false;
                    } else {
                        // the previous two line segments in S are overlapping line segments
                        let sli = prev;
                        sli -= 1;

                        let ptr_prev = sweep_line.map.keys_mut()[prev];
                        let ptr_sli = sweep_line.map.keys_mut()[sli];

                        if ptr_prev.polygon_type == inner!(event).polygon_type {
                            inner!(event).in_out = !ptr_prev.in_out;
                            inner!(event).is_inside = !ptr_sli.in_out;
                        } else {
                            inner!(event).in_out = !ptr_sli.in_out;
                            inner!(event).is_inside = !ptr_prev.in_out;
                        }
                    }
                } else if inner!(event).polygon_type == sweep_line.map.keys_mut()[prev].polygon_type {
                    inner!(event).is_inside = sweep_line.map.keys_mut()[prev].inside;
                    inner!(event).in_out = sweep_line.map.keys_mut()[prev].in_out;
                } else {
                    inner!(event).is_inside = sweep_line.map.keys_mut()[prev].in_out;
                    inner!(event).in_out = sweep_line.map.keys_mut()[prev].inside;
                }

                if (next + 1) != sweep_line_len {
                    possible_intersection(&mut event, &mut sweep_line.map.keys_mut()[next], &mut event_holder, &mut event_queue)
                }

                if prev != sweep_line_len {
                    possible_intersection(&mut event, &mut sweep_line.map.keys_mut()[next], &mut event_holder, &mut event_queue)
                }

            } else {
                // NOTE: In this block, there is no insertion happening!

                // the current line segment must be removed into the sweep_line
                let sli = other!(event).position_in_sweep_line;
                let mut prev = sli;
                let next = prev + 1;
                let sweep_line_len = sweep_line.len();

                // Get the next and previous line segments to "event" in sweep_line
                if prev != 0 {
                    prev -= 1;
                } else {
                    prev = sweep_line_len;
                }

                match inner!(event).edge_type {
                    Normal => {
                        match operation_type {
                            Intersection => {
                                if other!(event).is_inside {
                                    connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                                }
                            },
                            Union => {
                                if !(other!(event).is_inside) {
                                    connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                                }
                            },
                            Difference => {
                                if (inner!(event).polygon_type == PolygonType::Subject) && !(other!(event).is_inside) ||
                                   (inner!(event).polygon_type == PolygonType::Clipping && other!(event).is_inside) {
                                        connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                                }
                            },
                            Xor => {
                                connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                            }
                        }
                    },
                    SameTransition => {
                        if operation_type == Intersection || operation_type == Union {
                            connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                        }
                    },
                    DifferentTransition => {
                        if operation_type == Difference {
                            connector.add_segment(Segment::new(inner!(event).p, other!(event).p));
                        }
                    },
                    NonContributing => { },
                }

                // delete line segment associated to event from sweep_line and
                // check for intersection between the neighbors of "event" in sweep_line
                sweep_line.remove(&sli);

                if next != sweep_line_len && prev != sweep_line_len {
                    let ptr_prev = sweep_line.map.keys_mut()[prev];
                    let ptr_next = sweep_line.map.keys_mut()[next];
                    possible_intersection(ptr_prev, ptr_next, &mut event_holder, &mut event_queue);
                }
            }
        }

        connector.to_polygons()
    }
}

// DO NOT modify the return type, otherwise you will invalidate all internal pointers!
fn create_sweep_events(nodes: &[Point2D], polygon_type: PolygonType) -> Box<[SweepEventRef]> {

    let vec_len = nodes.len() * 2;
    let mut new_vec = Vec::<SweepEventRef>::with_capacity(vec_len);
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

        let e1 = SweepEventRef {
            inner: UnsafeCell::new(SweepEvent {
                p: cur_point,
                other: unsafe { ::std::mem::zeroed() },
                left: e1_left,
                position_in_sweep_line: 0,
                polygon_type: polygon_type,
                in_out: false,
                is_inside: false,
                edge_type: EdgeType::Normal,
            })
        };

        unsafe { *new_vec.get_unchecked_mut(e1_idx) = e1; }

        let e2 = SweepEventRef {
            inner: UnsafeCell::new(SweepEvent {
                p: next_point,
                other: unsafe { new_vec.get_unchecked(e1_idx) }, // new_vec does not live long enough
                position_in_sweep_line: 0,
                left: e2_left,
                polygon_type: polygon_type,
                in_out: false,
                is_inside: false,
                edge_type: EdgeType::Normal,
            })
        };

        unsafe { *new_vec.get_unchecked_mut(e2_idx) = e2; }
        unsafe { (*new_vec.get_unchecked_mut(e1_idx).inner.get()).other = new_vec.get_unchecked(e2_idx); }

        cur_pt_idx += 2;
    }

    // assert that the vector does not have moved (in memory)
    // if it did, the internal pointer would be garbage
    assert_eq!(new_vec.len(), vec_len);
    assert_eq!(new_vec.capacity(), vec_len);

    new_vec.into_boxed_slice()
}

/// NOTE: `possible_intersection` is the only function that calls `point::line_intersect`
fn possible_intersection<'a>(e1: &'a SweepEventRef<'a>, e2: &'a SweepEventRef<'a>,
                             event_holder: &'a mut Vec<SweepEventRef<'a>>,
                             eq: &'a mut BinaryHeap<&'a SweepEventRef<'a>>)
{

    // This function essentially moves events from the event_vec to the event_holder
    // Do not call this function on the same index twice!
    //
    // NOTE: `event.other` gets mutated!
    // `event_holder` and `eq` get pushed to!
    fn divide_segment<'a>(event: &'a mut SweepEvent<'a>,
                          divide_pt: &'a Point2D,
                          event_holder: &'a mut Vec<SweepEventRef<'a>>,
                          eq: &'a mut BinaryHeap<&SweepEventRef<'a>>)
    {
        {
            // push right event
            event_holder.push(SweepEventRef {
                inner: UnsafeCell::new(SweepEvent {
                    p: divide_pt,
                    left: false,
                    polygon_type: event.polygon_type,
                    other: event.other,
                    in_out: false,
                    position_in_sweep_line: 0,
                    is_inside: false,
                    edge_type: event.edge_type,
            })});

            // push left event
            event_holder.push(SweepEventRef {
                inner: UnsafeCell::new(SweepEvent {
                    p: divide_pt,
                    left: true,
                    polygon_type: event.polygon_type,
                    other: event.other,
                    in_out: false,
                    position_in_sweep_line: 0,
                    is_inside: false,
                    edge_type: event.edge_type,
            })});
        }

        let last = event_holder.len() - 1;

        {
            let left = event_holder.get_mut(last).unwrap();

            if inner!(left).compare(unsafe { &(*(*event.other).inner.get()) }) {
                unsafe { (*(*event.other).inner.get()).left = true; }
                inner_mut!(left).left = false;
            }
        }

        let left  = event_holder.get(last).unwrap();
        let right = event_holder.get(last - 1).unwrap();

        /*
            event->other->other = left;   // NOTE: this probably will be invalidated
            event->other = right;         // when the event_holder resizes !!!
            eq.push(left);
            eq.push(right);
        */

        unsafe { (*(*event.other).inner.get()).other = left };
        event.other = right;

        eq.push(left.clone());
        eq.push(right.clone());
    }

    // end of divide_segment()

    // note: this will COPY the point!
    let e1_other_p = other!(e1).p;
    let e2_other_p = other!(e2).p;

    let result = ::point::line_intersect(&inner!(e1).p, &e1_other_p, &inner!(e2).p, &e2_other_p);

    let (a, b) = match result {
        Some(a) => (a.0, a.1),
        None => return, // no intersections found
    };

    let new_b;

    match b {
        Some(new) => {
            if inner!(e1).polygon_type == inner!(e2).polygon_type {
                eprintln!("A polygon has overlapping edges. \n
                           Sorry, but the program does not work yet
                           with this kind of polygon");
                return;
            }
            new_b = new;
        },
        None => {
            if !((inner!(e1).p == inner!(e2).p) || (e1_other_p == e2_other_p)){
                if *inner!(e1).p != a && *e1_other_p != a {
                    // if a is not an endpoint of the line segment associated to e1 then divide "e1"
                    // divide_segment(e1, a);
                }

                if *inner!(e2).p != a && *e2_other_p != a {
                    // divide_segment(e2, a);
                }
            }

            // else:
            // the line segments intersect at an endpoint of both line segments
            return;
        },
    }

    // the line segments overlap
    let mut sorted_events = Vec::<Option<&mut SweepEvent>>::with_capacity(4);

    if inner!(e1).p == inner!(e2).p {
        sorted_events.push(None)
    } else if inner!(e1).compare(inner_mut!(e2)) {
        sorted_events.push(Some(inner_mut!(e2)));
        sorted_events.push(Some(inner_mut!(e1)));
    } else {
        sorted_events.push(Some(inner_mut!(e1)));
        sorted_events.push(Some(inner_mut!(e2)));
    }

    if e1_other_p == e2_other_p {

    } else if inner!(e1).compare(inner!(e2)) {
        sorted_events.push(Some(other_mut!(e2)));
        sorted_events.push(Some(other_mut!(e1)));
    } else {
        sorted_events.push(Some(other_mut!(e1)));
        sorted_events.push(Some(other_mut!(e2)));
    }


    if sorted_events.len() == 2 {
        // are both line segments equal?
        inner_mut!(e1).edge_type = EdgeType::NonContributing;
        other_mut!(e1).edge_type = EdgeType::NonContributing;
        return;
    }

    if sorted_events.len() == 3 {
        // the line segments share an endpoint
        sorted_events[1].unwrap().edge_type = EdgeType::NonContributing;
        unsafe { (*(*sorted_events[1].unwrap().other).inner.get()) }.edge_type = EdgeType::NonContributing;

        if sorted_events[0].is_some() {
            // is the right endpoint the shared point?
            sorted_events[0].unwrap().edge_type = if inner!(e1).in_out == inner!(e2).in_out {
                EdgeType::SameTransition
            } else {
                EdgeType::DifferentTransition
            };
            divide_segment(sorted_events[0].unwrap(), sorted_events[1].unwrap().p, event_holder, eq);
        } else {
            // the shared point is the left endpoint
            sorted_events[2].unwrap().edge_type = if inner!(e1).in_out == inner!(e2).in_out {
                EdgeType::SameTransition
            } else {
                EdgeType::DifferentTransition
            };
            divide_segment(sorted_events[2].unwrap(), sorted_events[1].unwrap().p, event_holder, eq);
        }

        return;
    }

    // sorted_events.len() == 4

    if (*sorted_events[0].unwrap()) != (*(*sorted_events[3].unwrap().other).inner.get()) {
        // no line segment includes totally the other one
        sorted_events[1].unwrap().edge_type = EdgeType::NonContributing;
        sorted_events[2].unwrap().edge_type = if inner!(e1).in_out == inner!(e2).in_out {
            EdgeType::SameTransition
        } else {
            EdgeType::DifferentTransition
        };
        divide_segment(sorted_events[0].unwrap(), sorted_events[1].unwrap().p, event_holder, eq);
        divide_segment(sorted_events[1].unwrap(), sorted_events[2].unwrap().p, event_holder, eq);
        return;
    }

    // one line segment includes the other one
    sorted_events[1].unwrap().edge_type = EdgeType::NonContributing;
    unsafe { (*(*sorted_events[1].unwrap().other).inner.get()) }.edge_type = EdgeType::NonContributing;
    divide_segment(sorted_events[0].unwrap(), sorted_events[1].unwrap().p, event_holder, eq);

    unsafe { (*(*sorted_events[3].unwrap().other).inner.get()) }.edge_type =
        if inner!(e1).in_out == inner!(e2).in_out {
            EdgeType::SameTransition
        } else {
            EdgeType::DifferentTransition
        };

    divide_segment(&mut (*(*sorted_events[3].unwrap().other).inner.get()), sorted_events[2].unwrap().p, event_holder, eq);
}
