use crate::prelude::*;
use bvh::aabb::{Aabb, Bounded};
use bvh::bounding_hierarchy::BHShape;
use bvh::bvh::Bvh;
use core::fmt;
use nalgebra::Point3;

// Once Data has been built from an .obj file, we should neve have to mutate the data
pub struct Data {
    pub vertices: Vec<Point>, // A list of points in 3-d space (3-vec)
    pub faces: Vec<Index>,    // Each triangular face is stored as a list of indices to the vertices
    pub triangles: Vec<Face>, // Each triangle is separately stored as a list of points
    pub bvh: Bvh<f32, 3>,
    pub gas: f32,
    pub shapes: Vec<Triangle>,
}

impl Data {
    pub fn new(
        vertices: Vec<Point>,
        faces: Vec<Index>,
        triangles: Vec<Face>,
        bvh: Bvh<f32, 3>,
        shapes: Vec<Triangle>,
    ) -> Self {
        Data {
            vertices,
            faces,
            triangles,
            gas: 0.0,
            bvh,
            shapes,
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

    // Convert primitives to nalgebra types
    pub fn build(v: Vec<[f32; 3]>, f: Vec<[usize; 3]>) -> Result<Data, Box<dyn std::error::Error>> {
        let vertices: Vec<Point> = v.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
        let faces: Vec<Index> = f.iter().map(|k| Vec3::new(k[0], k[1], k[2])).collect();

        // Vector 'faces' points to the 3 vertices that form a triangular mesh by storing 3 indices to Vector 'vertices'
        // To improve later ease of use, 'triangles' directly stores the 3 points of the triangular mesh
        let mut triangles: Vec<Face> = Vec::new();
        let mut shapes: Vec<Triangle> = Vec::new();
        for face in faces.iter() {
            let f = Face::new(
                Point3::from(vertices[face[0] - 1]),
                Point3::from(vertices[face[1] - 1]),
                Point3::from(vertices[face[2] - 1]),
            );
            triangles.push(f);
            let triangle = Triangle {
                pts: f,
                node_index: 0,
            };
            shapes.push(triangle);
        }
        let bvh = bvh::bvh::Bvh::build(&mut shapes);
        // We keep 'vertices' and 'faces' for ease of validating uniqueness and boundedness...
        // even though 'triangles' contains all requisite data
        Ok(Data::new(vertices, faces, triangles, bvh, shapes))
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

pub struct Triangle {
    pub pts: Face,
    pub node_index: usize,
}

impl Bounded<f32, 3> for Triangle {
    fn aabb(&self) -> Aabb<f32, 3> {
        let mut aabb = Aabb::empty();

        for p in self.pts.iter() {
            aabb = aabb.grow(p);
        }
        aabb
    }
}

impl BHShape<f32, 3> for Triangle {
    fn set_bh_node_index(&mut self, index: usize) {
        self.node_index = index;
    }

    fn bh_node_index(&self) -> usize {
        self.node_index
    }
}

#[cfg(test)]
mod test {
    #[test]
    fn bvh() {
        let a0 = [1.0, 0.0, 0.0];
        let a1 = [0.0, 1.0, 0.0];
        let a2 = [0.0, 0.0, 1.0];

        let b0 = [0.0, 0.0, 0.0];
        let b1 = [-1.0, -1.0, 0.0];
        let b2 = [0.0, -1.0, -1.0];
        let v: Vec<[f32; 3]> = vec![a0, a1, a2, b0, b1, b2];
        let f: Vec<[usize; 3]> = vec![[1, 2, 3], [4, 5, 6]];

        let data = super::Data::build(v, f).expect("Failed in test:: build data");
        let bvh = data.bvh;
        bvh.pretty_print();

        assert!(bvh.nodes.len() == 3);
    }
}
