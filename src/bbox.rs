use fsize;

/// Bounding box
#[derive(Debug, Clone)]
pub struct Bbox {
    pub(crate) top: fsize,
    pub(crate) right: fsize,
    pub(crate) bottom: fsize,
    pub(crate) left: fsize,
}

impl Bbox {

    /// Returns true if two bounding boxes overlap
    #[inline]
    pub fn overlaps(&self, other: &Self) -> bool {
        !((other.left > self.right) ||
          (other.right < self.left) ||
          (other.top < self.bottom) ||
          (other.bottom > self.top))
    }
}
