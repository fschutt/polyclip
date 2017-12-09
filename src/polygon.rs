use Point2D;
use Bbox;

/// Modifying the nodes of a polygon must be done via a closure,
/// because if the points are modified, the bounding box has to be recomputed
#[derive(Debug, Clone)]
pub struct Polygon {
    pub(crate) nodes: Vec<Point2D>,
    pub(crate) bounding_box: Bbox,
}

impl Polygon {

    pub fn new(nodes: Vec<Point2D>) -> Self {
        let bbox = calculate_bounding_box(&nodes);

        Self {
            nodes: nodes,
            bounding_box: bbox,
        }
    }

    /// Substracts a polygon from the current one
    ///
    /// If the current polygon is empty, returns None.
    pub fn subtract(&self, other: &Self) -> Option<Self> {

        if self.nodes.is_empty() {
            return Some(other.clone());
        } else if other.nodes.is_empty() {
            return Some(self.clone())
        }


        None
    }

    /// Intersect the current polygon with the other one, returning the area(s)
    /// where the two polygons intersect
    pub fn intersect(&self, other: &Self) -> Option<Self> {
        None
    }

    /// Combines the current polygon with the other one
    pub fn union(&self, b: &Self) -> Option<Self> {
        None
    }

    /// XORs the current polygon with the other one
    pub fn xor(&self, b: &Self) -> Option<Self> {
        None
    }

    /// Cuts two MultiPolygons into paths, returning the polygons
    /// including all possible intersection areas.
    ///
    /// Unioning all cuts will result
    pub fn cut(&self, b: &Self) -> Option<Vec<Polygon, >> {
        None
    }
}

/// Calculates the bounding box of all points in the nodes
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
