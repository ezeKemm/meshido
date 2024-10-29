use crate::prelude::*;

#[derive(Debug, Clone)]
pub struct AABB {
    pub min: Point,
    pub max: Point,
}

impl AABB {
    pub fn new(min: Point, max: Point) -> Self {
        Self { min, max }
    }

    pub fn empty() -> Self {
        Self::new(
            Point::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            Point::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        )
    }

    pub fn merge(&mut self, other: &Self) {
        self.min = Point::new(
            self.min.x.min(other.min.x),
            self.min.y.min(other.min.y),
            self.min.z.min(other.min.z),
        );
        self.max = Point::new(
            self.max.x.max(other.max.x),
            self.max.y.max(other.max.y),
            self.max.z.max(other.max.z),
        )
    }

    pub fn intersect(&self, other: &Self) -> bool {
        (self.min.x < other.max.x && self.max.x > other.min.x)
            && (self.min.y < other.max.y && self.max.y > other.min.y)
            && (self.min.z < other.max.z && self.max.z > other.min.z)
    }
}

impl Default for AABB {
    fn default() -> Self {
        Self::empty()
    }
}
