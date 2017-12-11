use Point2D;
use segment::Segment;

use std::collections::VecDeque;

#[derive(Clone)]
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

    /// Link a segment to the chain
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

    /// Links another point chain to the current chain
    pub(crate) fn link_point_chain(&mut self, mut chain: PointChain<'a>) -> bool {

        let chain_first_elem = *chain.nodes[0];
        let self_last_elem = *self.nodes[self.nodes.len() - 1];

        // NOTE: the C++ code uses a linked list + splice here,
        // which is of course O(1) for the first two cases,
        // instead of O(n) for a Vec. The last two cases are O(n) anyways
        //
        // however, since the PointChain gets iterated frequently,
        // a linked list is not very cache-friendly, which is why I
        // chose to use a Vec in the first place.

        if chain_first_elem == self_last_elem {
            chain.nodes.pop_front();
            chain.nodes.into_iter().for_each(|ch| self.nodes.push_back(ch));
            return true;
        }

        let chain_last_elem = *chain.nodes[chain.nodes.len() - 1];
        let self_first_elem = *self.nodes[0];

        if chain_last_elem == self_first_elem {
            self.nodes.pop_front();
            chain.nodes.into_iter().for_each(|ch| self.nodes.push_front(ch));
            return true;
        }

        if chain_first_elem == self_first_elem {
            self.nodes.pop_front();
            chain.nodes.into_iter().rev().for_each(|ch| self.nodes.push_front(ch));
            return true;
        }

        if chain_last_elem == self_last_elem {
            self.nodes.pop_back();
            chain.nodes.into_iter().rev().for_each(|ch| self.nodes.push_back(ch));
            return true;
        }

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
