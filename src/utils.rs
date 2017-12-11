use segment::Segment;
use fsize;
use Point2D;

/// Signed area of the triangle (p0, p1, p2)
#[inline]
pub(crate) fn calculate_signed_area3(p0: &Point2D, p1: &Point2D, p2: &Point2D) -> fsize {
    (p0.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p0.y - p2.y)
}

/// Signed area of the triangle ( (0,0), p1, p2)
#[inline]
pub(crate) fn calculate_signed_area2(p0: &Point2D, p1: &Point2D) -> fsize {
    (-p1.x) * (p0.y - p1.y) - (-p1.y) * (p0.x - p1.x)
}

#[derive(PartialEq, Eq)]
pub(crate) enum Sign {
    Positive,
    Negative,
    Equal,
}

/// Sign of triangle (p1, p2, o)
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

#[inline]
pub(crate) fn is_point_in_triangle(s: &Segment, o: &Point2D, p: &Point2D) -> bool {
    let sign_first = calculate_sign(&s.begin_pt, &s.end_pt, p);
    (sign_first == calculate_sign(&s.end_pt, o, p)) &&
    (sign_first == calculate_sign(o, &s.begin_pt, p))
}
