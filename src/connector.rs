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
    pub(crate) fn to_polygons(mut self) -> Option<Vec<Polygon>> {

        // filter empty chains
        self.open_polygons.retain(|x| !x.nodes_ref().is_empty());
        self.closed_polygons.retain(|x| !x.nodes_ref().is_empty());

        if self.open_polygons.is_empty() && self.closed_polygons.is_empty() {
            return None;
        }

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

        Some(final_polygons)
    }

    pub fn add_segment(&mut self, segment: Segment<'a>) {

        let mut interesting_segment: Option<usize> = None;

        for j in 0..self.open_polygons.len() {
            if unsafe { self.open_polygons.get_unchecked_mut(j) }.link_segment(segment.clone()) {
                interesting_segment = Some(j);
                break;
            }
        }

        if let Some(j) = interesting_segment {
            if unsafe { self.open_polygons.get_unchecked(j) }.is_closed() {
                self.closed_polygons.push(self.open_polygons.remove(j));
            } else {
                // this is more or less a manual version of .retain() because
                // retain does not work on ranges
                let mut delete_last_element = false;
                {
                    let (old_chains, to_append_chains) = self.open_polygons.split_at_mut(j);
                    debug_assert!(old_chains.len() == j); // TODO

                    let old_len = old_chains.len() - 1;
                    let last_chain = unsafe { &mut old_chains.get_unchecked_mut(old_len) };

                    // this code is inspired by the `Vec::retain()` source code
                    let to_append_len = to_append_chains.len();
                    for i in 0..to_append_len {
                        if !(last_chain.link_point_chain((*unsafe { to_append_chains.get_unchecked(i) }).clone())) {
                            delete_last_element = true;
                            to_append_chains.swap(i, to_append_len - 1); // swap the current and last element
                            break;
                        }
                    }
                }

                if delete_last_element {
                    self.open_polygons.pop(); // remove the last element
                }
            }
        } else {
            // The segment cannot be connected with any open polygon
            self.open_polygons.push(PointChain::init(segment));
        }
    }
}
