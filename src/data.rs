use crate::prelude::*;
use core::fmt;

// Once Data has been built from an .obj file, we should neve have to mutate the data
pub struct Data {
    pub vertices: Vec<Point>, // A list of points in 3-d space (3-vec)
    pub faces: Vec<Index>,    // Each triangular face is stored as a list of indices to the vertices
    pub triangles: Vec<Face>, // Each triangle is separately stored as a list of points
    pub gas: f32,
}

impl Data {
    pub fn new(vertices: Vec<Point>, faces: Vec<Index>, triangles: Vec<Face>) -> Self {
        Data {
            vertices,
            faces,
            triangles,
            gas: 0.0,
        }
    }
    pub fn vert(&self) -> &Vec<Point> {
        &self.vertices
    }
    pub fn face(&self) -> &Vec<Index> {
        &self.faces
    }
    pub fn tri(&self) -> &Vec<Face> {
        &self.triangles
    }
    pub fn gas(&self) -> &f32 {
        &self.gas
    }
}

impl fmt::Display for Data {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut prt_str = String::new();
        prt_str += &format!(
            "# of Vertices: {}\n# of Faces: {}\n GasParam: {}",
            self.vertices.len(),
            self.faces.len(),
            self.gas
        );

        for vertex in self.vertices.iter() {
            prt_str += &format!("[ {}, {}, {} ]", vertex[0], vertex[1], vertex[2]);
        }

        for face in self.faces.iter() {
            prt_str += &format!("[ {}, {}, {} ]", face[0], face[1], face[2]);
        }
        write!(f, "{}", prt_str)
    }
}
