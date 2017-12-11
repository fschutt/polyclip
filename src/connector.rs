use point_chain::PointChain;
use polygon::{WindingOrder, Polygon};
use segment::Segment;

pub(crate) struct Connector<'a> {
    open_polygons: Vec<PointChain<'a>>,
    closed_polygons: Vec<PointChain<'a>>,
}

impl<'a> Connector<'a> {

    pub(crate) fn new() -> Self {
        Self {
            open_polygons: Vec::new(),
            closed_polygons: Vec::new(),
        }
    }

    // replacement for `connector.toPolygon (result);`
    pub(crate) fn to_polygons(mut self) -> Vec<Polygon> {

        // filter empty chains
        self.open_polygons.retain(|x| !x.nodes_ref().is_empty());
        self.closed_polygons.retain(|x| !x.nodes_ref().is_empty());

        let open_poly_len = self.open_polygons.len();
        let final_poly_len = open_poly_len + self.closed_polygons.len();

        let mut final_polygons = Vec::<Polygon>::with_capacity(final_poly_len);
        unsafe { final_polygons.set_len(final_poly_len); }

        for (idx, open) in self.open_polygons.into_iter().enumerate() {
            let (nodes, is_closed) = open.into_contents();
            unsafe {
                *final_polygons.get_unchecked_mut(idx) = Polygon {
                    nodes: nodes.iter().map(|p| **p).collect(),
                    is_closed: is_closed,
                    is_hole: false, // TODO
                    winding: Some(WindingOrder::Clockwise), // TODO
                }
            }
        }

        for (idx, closed) in self.closed_polygons.into_iter().enumerate() {
            let (nodes, is_closed) = closed.into_contents();
            unsafe {
                *final_polygons.get_unchecked_mut(open_poly_len + idx) = Polygon {
                    nodes: nodes.iter().map(|p| **p).collect(),
                    is_closed: is_closed, // TODO
                    is_hole: false, // TODO
                    winding: Some(WindingOrder::Clockwise), // TODO
                }
            }
        }

        final_polygons
    }

    pub fn add_segment(&mut self, segment: Segment<'a>) {
        for (idx, open) in self.open_polygons.iter_mut().enumerate() {
            if open.link_segment(segment.clone()) {
                if open.is_closed() {
                    // self.closed_polygons.splice();
                } else {

                }

                return;
            }
        }

        /*
            void Connector::add(const Segment& s)
            {
                iterator j = openPolygons.begin ();
                while (j != openPolygons.end ()) {
                    if (j->LinkSegment (s)) {
                        if (j->closed ())
                            closedPolygons.splice (closedPolygons.end(), openPolygons, j);
                        else {
                            list<PointChain>::iterator k = j;
                            for (++k; k != openPolygons.end (); k++) {
                                if (j->LinkPointChain (*k)) {
                                    openPolygons.erase (k);
                                    break;
                                }
                            }
                        }
                        return;
                    }
                    j++;
                }
                // The segment cannot be connected with any open polygon
                openPolygons.push_back (PointChain ());
                openPolygons.back ().init (s);
            }
        */

        self.open_polygons.push(PointChain::init(segment));
    }

}
