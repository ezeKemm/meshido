// type Result<T> = Result<T, Box<dyn std::error::Error>>;
pub use nalgebra::Vector3 as Vec3;

pub type Point = Vec3<f32>; // coordinates of a 3-d point
pub type Index = Vec3<usize>; // a 3-vec of the indices pointing to the Points of a face
pub type Face = Vec3<Point>; // a 3-vec of the points making up a triangular face
