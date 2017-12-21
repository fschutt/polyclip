use segment::Segment;
use {Point2D, Bbox, fsize};
use polygon::WindingOrder;

/// Calculate the signed area of a triangle (p0, p1, p2)
#[inline]
pub fn calculate_signed_area3(p0: &Point2D, p1: &Point2D, p2: &Point2D) -> fsize {
    (p0.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p0.y - p2.y)
}

/// Calculate the signed area of a triangle ( (0,0), p1, p2)
#[inline]
pub fn calculate_signed_area2(p0: &Point2D, p1: &Point2D) -> fsize {
    (-p1.x) * (p0.y - p1.y) - (-p1.y) * (p0.x - p1.x)
}

/// Sign of a triangle
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum Sign {
    Positive,
    Negative,
    Equal,
}

/// Calculate the sign of the triangle (p1, p2, o)
#[inline]
pub(crate) fn calculate_sign(p0: &Point2D, p1: &Point2D, o: &Point2D) -> Sign {
    let det = (p0.x - o.x) * (p1.y - o.y) - (p1.x - o.x) * (p0.y - o.y);
    if det < 0.0 {
        Sign::Negative
    } else if det > 0.0 {
        Sign::Positive
    } else {
        Sign::Equal
    }
}

/// Check if a point is inside a triangle
#[inline]
pub(crate) fn is_point_in_triangle(s: &Segment, o: &Point2D, p: &Point2D) -> bool {
    let sign_first = calculate_sign(&s.begin_pt, &s.end_pt, p);
    (sign_first == calculate_sign(&s.end_pt, o, p)) &&
    (sign_first == calculate_sign(o, &s.begin_pt, p))
}

/// Calculates the winding order of a polygon using the gaussian shoelace formula in O(n) time
///
/// # Panics
///
/// You must validate that there are at least three points in the nodes
/// (otherwise, there is no winding order, it's just a point or a line)
pub fn calculate_winding_order(nodes: &[Point2D]) -> WindingOrder {

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

/// Calculates the bounding box of all points in the nodes in O(n) time
pub fn calculate_bounding_box(nodes: &[Point2D]) -> Bbox {

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
