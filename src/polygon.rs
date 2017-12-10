use Point2D;
use Bbox;
use fsize;
use algorithm::{SweepEvent, PolygonType, EdgeType};
use point_chain::PointChain;

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

struct ImmutableSweepEvents<'a> {
    _internal: Vec<SweepEvent<'a>>,
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
        use std::collections::{BTreeSet, BinaryHeap};
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
        for event in &vec_of_sweep_events_subject._internal { event_queue.push(event.clone()); }
        for event in &vec_of_sweep_events_clipping._internal { event_queue.push(event.clone()); }

        // -------------------------------------------------------------------- sweep events created

        let connector = Connector::new();

        let mut sweep_line = BTreeSet::<SweepEvent>::new();

        let minimum_x_bbox_pt = self_bbox.right.min(other_bbox.right);

        // calculate the necessary events
        while let Some(event) = event_queue.pop() {
            match operation_type {
                Intersection => {
                    if event.p.x > minimum_x_bbox_pt {
                        return Some(connector.to_polygons());
                    }
                },
                Difference => {
                    if event.p.x > self_bbox.right {
                        break;
                    }
                },
                Union => {
                    if event.p.x > minimum_x_bbox_pt {
                        if !event.left {
                            // add all the non-processed line segments to the result
                            // connector.add (e->segment ());
                        }
                        continue;
                    }
                },
                Xor => { },
            }
        }

        return Some(connector.to_polygons());
    }
}

// DO NOT modify the return type, otherwise you will invalidate all internal pointers!
//
// NOTE: it may be better to use indices instead of pointer.
// However, in the end, the effect would be almost the same, since index checking is disabled.
// So it's up to testing what version (pointers or indices) is more efficient.
#[inline]
fn create_sweep_events(nodes: &[Point2D], pl: PolygonType) -> ImmutableSweepEvents {

    let vec_len = nodes.len() * 2;
    let mut new_vec = Vec::<SweepEvent>::with_capacity(vec_len);
    unsafe { new_vec.set_len(vec_len); }

    let iter1 = nodes.iter();
    let mut iter2 = nodes.iter().cycle();
    iter2.next();

    let mut cur_pt_idx = 1;
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

        let e1 = SweepEvent {
            p: cur_point,
            other: ::std::ptr::null(),
            left: e1_left,
            pl: pl,
            in_out: false,
            is_inside: false,
            edge_type: EdgeType::Normal,
        };

        let mut e2 = SweepEvent {
            p: next_point,
            other: ::std::ptr::null(),
            left: e2_left,
            pl: pl,
            in_out: false,
            is_inside: false,
            edge_type: EdgeType::Normal,
        };

        unsafe {
            {
                let e1_location = new_vec.get_unchecked_mut(cur_pt_idx - 1);
                *e1_location = e1;
                e2.other = e1_location;
            }

            #[allow(unused_assignments)]
            let mut e2_location = ::std::ptr::null_mut();
            {
                e2_location = new_vec.get_unchecked_mut(cur_pt_idx);
                *e2_location = e2;
            }
            new_vec.get_unchecked_mut(cur_pt_idx - 1).other = e2_location;
        }

        cur_pt_idx += 2;
    }

    // assert that the vector does not have moved (in memory)
    // if it did, the internal pointer would be garbage
    assert_eq!(new_vec.len(), vec_len);
    assert_eq!(new_vec.capacity(), vec_len);

    ImmutableSweepEvents { _internal: new_vec }
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
