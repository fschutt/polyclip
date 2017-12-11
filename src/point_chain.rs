use Point2D;
use segment::Segment;

use std::collections::VecDeque;

pub(crate) struct PointChain<'a> {
    nodes: VecDeque<&'a Point2D>,
    is_closed: bool,
}

impl<'a> PointChain<'a> {

    #[inline]
    pub(crate) fn init(initial_segment: Segment<'a>) -> Self {

        let mut deque = VecDeque::with_capacity(2);
        deque.push_back(initial_segment.begin_pt);
        deque.push_back(initial_segment.end_pt);

        Self {
            nodes: deque,
            is_closed: false,
        }
    }

    pub(crate) fn link_segment(&mut self, segment: Segment<'a>) -> bool {

        let nodes_last_idx = self.nodes.len() - 1;
        let first_elem = *self.nodes.front().unwrap();
        let last_elem = *self.nodes.back().unwrap();

        if segment.begin_pt == first_elem {
            if segment.end_pt == last_elem {
                self.is_closed = true;
            } else {
                self.nodes.push_front(segment.end_pt);
            }
            return true;
        }

        if segment.end_pt == last_elem {
            if segment.begin_pt == first_elem {
                self.is_closed = true;
            } else {
                self.nodes.push_back(segment.begin_pt);
            }
            return true;
        }

        if segment.end_pt == first_elem {
            if segment.begin_pt == last_elem {
                self.is_closed = true;
            } else {
                self.nodes.push_front(segment.begin_pt);
            }
            return true;
        }

        if segment.begin_pt == last_elem {
            if segment.end_pt == first_elem {
                self.is_closed = true;
            } else {
                self.nodes.push_back(segment.end_pt);
            }
            return true;
        }

        false
    }

    pub(crate) fn link_point_chain(&mut self, mut chain: PointChain<'a>) -> bool {

        let chain_first_elem = chain.nodes[0];
        let self_last_elem = *self.nodes.back().unwrap();

        if chain_first_elem == self_last_elem {
            chain.nodes.pop_front();
            // self.nodes.splice();
            return true;
        }

        /*
            if (chain.l.front () == l.back ()) {
                chain.l.pop_front ();
                l.splice (l.end (), chain.l);
                return true;
            }
            if (chain.l.back () == l.front ()) {
                l.pop_front ();
                l.splice (l.begin (), chain.l);
                return true;
            }
            if (chain.l.front () == l.front ()) {
                l.pop_front ();
                reverse (chain.l.begin (), chain.l.end ());
                l.splice (l.begin (), chain.l);
                return true;
            }
            if (chain.l.back () == l.back ()) {
                l.pop_back ();
                reverse (chain.l.begin (), chain.l.end ());
                l.splice (l.end (), chain.l);
                return true;
            }
            return false;
        */

        false
    }

    /// Provides read-only access to self.is_closed
    #[inline(always)]
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }

    /// Provides read-only access to self.nodes
    #[inline(always)]
    pub(crate) fn nodes_ref(&self) -> &VecDeque<&'a Point2D> {
        &self.nodes
    }

    /// Consumes the struct, returns the contents
    /// Returns: (self.nodes, self.is_closed)
    #[inline(always)]
    pub(crate) fn into_contents(self) -> (VecDeque<&'a Point2D>, bool) {
        (self.nodes, self.is_closed)
    }
}
