use point::Point2D;

#[derive(Clone)]
pub(crate) struct Segment<'a> {
    pub(crate) begin_pt: &'a Point2D,
    pub(crate) end_pt: &'a Point2D,
}

impl<'a> Segment<'a> {
    pub(crate) fn change_orientation(&mut self) {
        ::std::mem::swap(&mut self.begin_pt, &mut self.end_pt);
    }

    pub(crate) fn set_begin(&mut self, begin: &'a Point2D) {
        self.begin_pt = begin;
    }

    pub(crate) fn set_end(&mut self, end: &'a Point2D) {
        self.end_pt = end;
    }
}
