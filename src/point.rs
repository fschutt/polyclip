#[derive(Debug, Clone)]
pub struct Point2D {
  pub x: f32,
  pub y: f32,
}

/// Check if lines intersect. This function is called very often,
/// takes roughly 1.27 microseconds per checks
pub fn line_intersect(p0: Point2D, p1: Point2D, p2: Point2D, p3: Point2D)
                      -> Option<Point2D>
{
    let s1_x = p1.x - p0.x;
    let s1_y = p1.y - p0.y;
    let s2_x = p3.x - p2.x;
    let s2_y = p3.y - p2.y;

    let coef_div = -s2_x * s1_y + s1_x * s2_y;
    if coef_div == 0.0 { return None; /* lines merged to single point, avoid division by 0 */ }

    let s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.x)) / coef_div;
    let t = ( s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.y)) / coef_div;

    if t >= 0.0 && t <= 1.0 && s >= 0.0 && s <= 1.0 {
        Some(Point2D {
            x: p0.x + (t * s1_x),
            y: p0.y + (t * s1_y)
        })
    } else {
        None
    }
}

#[test]
pub fn test_line_intersect_none() {
    // No Intersect
    let result1 = line_intersect(Point2D { x: 0.0,  y: 0.0 },
                                 Point2D { x: 2.0,  y: 8.0 },
                                 Point2D { x: 8.0,  y: 0.0 },
                                 Point2D { x: 0.0,  y: 20.0});
    assert!(result1.is_none());
}

#[test]
pub fn test_line_intersect_intersect() {
    // Intersect
    let result1 = line_intersect(Point2D { x: 0.0,  y: 10.0},
                                 Point2D { x: 2.0,  y: 0.0 },
                                 Point2D { x: 10.0, y: 0.0 },
                                 Point2D { x: 0.0,  y: 5.0 });
    assert!(result1.is_some());
}

#[test]
pub fn test_line_intersect_parallel_vertical() {
    // Parallel, vertical
    let result = line_intersect(Point2D { x: 0.0,  y: 0.0},
                                Point2D { x: 0.0,  y: 10.0 },
                                Point2D { x: 2.0,  y: 0.0 },
                                Point2D { x: 2.0,  y: 10.0 });
    assert!(result.is_none());
}

#[test]
pub fn test_line_intersect_parallel_diagonal() {
    // Parallel, diagonal
    let result = line_intersect(Point2D { x: 0.0,  y: 0.0},
                                Point2D { x: 5.0,  y: 5.0 },
                                Point2D { x: 2.0,  y: 0.0 },
                                Point2D { x: 7.0,  y: 5.0 });
    assert!(result.is_none());
}

#[test]
pub fn test_line_intersect_colinear_overlap() {
    // Collinear, overlap
    let result = line_intersect(Point2D { x: 0.0,  y: 0.0 },
                                Point2D { x: 5.0,  y: 5.0 },
                                Point2D { x: 2.0,  y: 2.0 },
                                Point2D { x: 7.0,  y: 7.0 });
    assert!(result.is_some());
}

#[test]
pub fn test_line_intersect_colinear_nooverlap() {
    // Collinear, no overlap
    let result = line_intersect(Point2D { x: 0.0,  y: 0.0 },
                                Point2D { x: 5.0,  y: 5.0 },
                                Point2D { x: 7.0,  y: 7.0 },
                                Point2D { x: 10.0,  y: 10.0 });
    assert!(result.is_some());
}
