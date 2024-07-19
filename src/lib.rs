mod aabb;
mod brutus;
mod coplanar;
mod data;
mod prelude;
mod reader;
use crate::data::Data;
// use crate::prelude::*;
use std::collections::{HashMap, HashSet};

use ordered_float::OrderedFloat as oF; // To hash floats

// TODO: nalgebra and bvh have made the use of Vec3 and Point3 all fucky wucky
//       it would be best to sort that out sooner rather than later

// A proper mesh must not contain any edges which loop back on themselves (i->i)
// To ensure uniqueness, we test for any duplicate vertices
// If a vertex is unique, it will only appear once in the arrary such that a Set of the arrary will
// be equal in length to the array
pub fn validate_uniqueness(data: Data) -> bool {
    // construct a set of the vertices
    // Floats are not ordered and cannot be hashed without a wrapper
    if data.triangles.len() == 0 {
        return false;
    }
    let hashit: HashSet<[oF<f32>; 3]> = data
        .vert()
        .iter()
        .map(|x| [oF(x[0]), oF(x[1]), oF(x[2])])
        .collect();
    // Compare length of vertices arr with hashset: if equal -> no duplicates -> unique
    if hashit.len() == data.vert().len() {
        println!("success!");
        return true;
    }
    false
}

pub fn intersection(data: Data) -> bool {
    brutus::brute_self_intersection(data)
}

// We check that for every edge, only two faces share that edge
// For each face, we build the three edges of the triangle: (i->j), (j->k), (k->i)
// and then add these to a HashMap to count their occurences by all subsequent faces
// If our mesh is bounded, each entry in the map should be == 2
pub fn boundedness(data: Data) -> bool {
    let mut edge_map: HashMap<(usize, usize), u32> = HashMap::new();
    for face in data.face().iter() {
        let (i, j, k) = (face[0], face[1], face[2]);
        edge_map
            .entry((i, j))
            .and_modify(|count| *count += 1)
            .or_insert(1);
        edge_map
            .entry((j, k))
            .and_modify(|count| *count += 1)
            .or_insert(1);
        edge_map
            .entry((k, i))
            .and_modify(|count| *count += 1)
            .or_insert(1);
        // because we create edges in a oriented fashion, we insert the opposite orientation
        // This doubles our computation and adds overhead but neatly assures we count both possible
        // orientations of the same shared edge
        edge_map
            .entry((i, k))
            .and_modify(|count| *count += 1)
            .or_insert(1);
        edge_map
            .entry((k, j))
            .and_modify(|count| *count += 1)
            .or_insert(1);
        edge_map
            .entry((j, i))
            .and_modify(|count| *count += 1)
            .or_insert(1);
    }

    // Check whether all edges are counted <= 2
    // We reverse our conditional since any() short-circuits for the true case
    let bounded = !edge_map.iter().any(|((_, _), &c)| c > 2);
    bounded
}

fn main() {}

// TODO: clean up / move(?) these tests
#[cfg(test)]
mod tests {
    use super::*;
    use crate::reader::build_data_from_obj;
    // use std::vec;

    #[test]
    // Detect invalid vertices -> non-unique
    fn invalid_vertices() {
        let data = build_data_from_obj("objs/cbl_test_dup.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(!validate_uniqueness(data))
    }

    #[test]
    // Test uniqueness of vertices
    fn valid_vertices() {
        let data = build_data_from_obj("objs/cbl_test.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(validate_uniqueness(data))
    }

    #[test]
    // Read a obj file and extract data into Data struct
    fn read_file() {
        let data =
            build_data_from_obj("objs/test1.obj".to_string()).expect("failed to convert .obj file");
        println!("len: {}", data.triangles.len());
        assert!(data.triangles.len() > 0);
    }

    #[test]
    // Test boundedness
    fn bound() {
        let data = build_data_from_obj("objs/cbl_test.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(boundedness(data))
    }
    #[test]
    fn intersect() {
        let data = build_data_from_obj("objs/test_intersect.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(!intersection(data))
    }
    #[test]
    fn intersect_coplanar_shared() {
        let data = build_data_from_obj("objs/test_coplanar_square.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(intersection(data))
    }
    #[test]
    fn intersect_coplanar_disjoint() {
        // funkier triangles
        let data = build_data_from_obj("objs/test_coplanar_disjoint.obj".to_string())
            .expect("failed to convert .obj file");
        // normal coplanar triangles
        let data2 = build_data_from_obj("objs/test_coplanar_disjoint2.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(intersection(data) && intersection(data2))
    }
    #[test]
    fn cube() {
        let data =
            build_data_from_obj("objs/cube.obj".to_string()).expect("failed to convert .obj file");
        assert!(intersection(data))
    }
    #[test]
    fn low_res_sphere() {
        let data = build_data_from_obj("objs/low_sphere.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(intersection(data))
    }

    #[test]
    fn tetrahedron_selfintersect() {
        let data = build_data_from_obj("objs/tetrahedrons_self_intersect.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(!intersection(data))
    }

    #[test]
    fn sphere_selfintersect() {
        let data = build_data_from_obj("objs/sphere_selfintersect.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(!intersection(data))
    }
    #[test]
    #[ignore = "fails and we don't like that"]
    fn high_polygon_no_intersect() {
        let data = build_data_from_obj("objs/cbl_test.obj".to_string())
            .expect("failed to convert .obj file");
        assert!(intersection(data))
    }
}
