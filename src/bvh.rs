use bvh::{aabb::Aabb, ray::Ray};

use crate::{
    brutus::{check_coplanar, ray_intersect, RayTri},
    coplanar::{self, check_edge_intersect, intersect_test_coplanar},
    data::Data,
    prelude::Point,
};

fn search_mesh(data: Data) {
    let size = data.faces.len();

    for i in 0..size {
        for edge in 0..3 {
            let vert_index = data.faces[i][edge] - 1;
            let vert_index2 = data.faces[i][(edge + 1) % 3] - 1;

            let origin = data.vertices[vert_index];
            let p1 = data.vertices[vert_index2];
            let d_vec = p1 - origin; // direction vector
            let d_mag = d_vec.magnitude(); // vector magnitude
            let direction = d_vec / d_mag; // normalized direction vector as unit vector

            let ray = Ray::new(origin.into(), direction); // triangle edge as ray
        }
    }
}

fn check_self_intersection(data: &Data) -> bool {
    let size = data.faces.len();

    for i in 0..size {
        for j in (i + 1)..size {
            // should not ever have i == j
            if data.faces[i] == data.faces[j] {
                println!("Early degeneracy:: Same face");
                // return false;
            }

            if in_bounds(&data, i, j) {
                let is_intersecting = compute_intersection(&data, i, j);
                let is_intersecting2 = compute_intersection(&data, j, i);
                if is_intersecting || is_intersecting2 {
                    return false;
                }
            }
        }
    }
    true
}
fn in_bounds(data: &Data, i: usize, j: usize) -> bool {
    let node_idx1 = data.shapes[i].node_index;
    let node_idx2 = data.shapes[j].node_index;

    let bvh = &data.bvh;
    let aabb1 = bvh.nodes[node_idx1].get_node_aabb(&data.shapes);
    let aabb2 = bvh.nodes[node_idx2].get_node_aabb(&data.shapes);
    // TODO: use approx contains?
    if is_overlap(&aabb1, &aabb2) {
        return true;
    }
    false
}

fn is_overlap(bb1: &Aabb<f32, 3>, bb2: &Aabb<f32, 3>) -> bool {
    println!(
        "bb1: min: {} max: {} \nbb2: min: {} max: {}\n {} {} {} {}",
        bb1.min,
        bb1.max,
        bb2.min,
        bb2.max,
        bb1.contains(&bb2.min),
        bb1.contains(&bb2.max),
        bb2.contains(&bb1.min),
        bb2.contains(&bb1.max)
    );
    return bb1.contains(&bb2.min)
        || bb1.contains(&bb2.max)
        || bb2.contains(&bb1.min)  // Redundant check but better safe than sorry
        || bb2.contains(&bb1.max);
}

fn compute_intersection(data: &Data, i: usize, j: usize) -> bool {
    compute_ray_intersect(data, i, j)
}

fn compute_ray_intersect(data: &Data, i: usize, j: usize) -> bool {
    for edge in 0..3 {
        let vert_index = data.faces[j][edge] - 1;
        let vert_index2 = data.faces[j][(edge + 1) % 3] - 1;
        println!("edge: {}, idx: {}, {}", edge, vert_index, vert_index2);

        let (v0, v1, v2) = (
            data.vertices[data.faces[i][0] - 1],
            data.vertices[data.faces[i][1] - 1],
            data.vertices[data.faces[i][2] - 1],
        );

        let p0 = data.vertices[vert_index];
        let p1 = data.vertices[vert_index2];

        let intersect: RayTri = ray_intersect(&v0, &v1, &v2, &p0, &p1);
        match intersect {
            RayTri::Disjoint => return false,
            RayTri::Intersecting => return true,
            RayTri::Degenerate => return true,
            RayTri::Parallel => {
                if check_coplanar(&data.triangles[i], &data.triangles[j]) {
                    // Triangles are coplanar
                    println!("Parallel case: testing coplanar intersection...");
                    if intersect_test_coplanar(&data.triangles[i], &data.triangles[j]) {
                        println!("v0: {}, v1: {}, v2: {}, p0: {}, p1: {}", v0, v1, v2, p0, p1);
                        // println!("faces: {face1}, {face2}");
                        println!("triangles: {}, {}", data.triangles[i], data.triangles[j]);
                        println!("intersecting edges of coplanar faces {i}:{j}");
                        return true;
                    } else {
                        println!("coplanar but safe");
                    }
                } else {
                    // This check was added separately since the coplanar intersection
                    // algorithm can fail if testing two triangles which are not in fact
                    // coplanar
                    println!("Parallel case: testing ray intersection...");
                    // Only ray is parallel
                    // TODO: clunky and doesn't handle shared vertices very well since it
                    // sits outside the rest of the coplanar logic ... might fail in odd
                    // cases
                    if coplanar::check_point_inside(v0, v1, v2, p0)
                        || coplanar::check_point_inside(v0, v1, v2, p1)
                        || check_edge_intersect(v0, v1, p0, p1)
                        || check_edge_intersect(v1, v2, p0, p1)
                        || check_edge_intersect(v2, v0, p0, p1)
                    {
                        println!("parallel ray intersects");
                        return true;
                    }
                }
            }
        }
    }
    true
}

#[cfg(test)]
mod test {

    // node_index is correctly updated i.e. the bvh tree is properly contructed
    // -> test bvh with two disjoint triangles
    // -> test with two intersecting triangles
    // -> test with 3? (two intersect, one disjoint)
    // for sanity's sake: check compute_intersection works
    //  -> make sure all our indices are indicing
    // check aabb bounds test works ... might have to use bvh test? (is that an integration test?)

    use bvh::{aabb::Aabb, bounding_hierarchy::BoundingHierarchy};
    use nalgebra::Point3;

    use crate::bvh::{check_self_intersection, in_bounds, is_overlap};

    #[test]
    fn check_overlap() {
        let bb1 = Aabb::with_bounds(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let bb2 = Aabb::with_bounds(Point3::new(-1.0, -1.0, -1.0), Point3::new(0.5, 0.5, 0.5));
        assert!(is_overlap(&bb1, &bb2))
    }

    #[test]
    fn check_no_overlap() {
        let bb1 = Aabb::with_bounds(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let bb2 = Aabb::with_bounds(Point3::new(-1.0, -1.0, -1.0), Point3::new(-0.5, -0.5, -0.5));
        assert!(!is_overlap(&bb1, &bb2))
    }

    #[test]
    fn check_in_bounds() {
        let a0 = [1.0, 0.0, 0.0];
        let a1 = [0.0, 1.0, 0.0];
        let a2 = [0.0, 0.0, 1.0];

        let b0 = [0.0, 0.0, 0.0];
        let b1 = [-1.0, -1.0, 0.0];
        let b2 = [0.0, -1.0, -1.0];
        let v: Vec<[f32; 3]> = vec![a0, a1, a2, b0, b1, b2];
        let f: Vec<[usize; 3]> = vec![[1, 2, 3], [4, 5, 6]];

        let data = super::Data::build(v, f).expect("Failed in test:: build data");
        assert!(in_bounds(&data, 1, 2))
    }

    #[test]
    fn bvh_three() {
        let a0 = [1.0, 0.0, 0.0];
        let a1 = [0.0, 1.0, 0.0];
        let a2 = [0.0, 0.0, 1.0];

        // triangle b intersects triangle a
        let b0 = [0.75, 0.75, 0.75];
        let b1 = [-1.0, -1.0, 0.0];
        let b2 = [0.0, -1.0, -1.0];

        let c0 = [2.0, 2.0, 10.0];
        let c1 = [-2.0, -2.0, 10.0];
        let c2 = [-4.0, 4.0, 10.0];

        let v: Vec<[f32; 3]> = vec![a0, a1, a2, b0, b1, b2, c0, c1, c2];
        let f: Vec<[usize; 3]> = vec![[1, 2, 3], [4, 5, 6], [7, 8, 9]];

        let data = super::Data::build(v, f).expect("Failed in test:: build data");
        let bvh = &data.bvh;
        bvh.pretty_print();

        for (i, node) in bvh.nodes.iter().enumerate() {
            println!("i: {i} :: {:?}", node);
        }

        let aabb_parent = bvh.nodes[1].get_node_aabb(&data.shapes);
        let aabb_lc = bvh.nodes[2].get_node_aabb(&data.shapes);
        let aabb_rc = bvh.nodes[3].get_node_aabb(&data.shapes);
        let aabb_psib = bvh.nodes[4].get_node_aabb(&data.shapes);

        assert!(is_overlap(&aabb_lc, &aabb_rc)); // children nodes overlap
        assert!(!is_overlap(&aabb_psib, &aabb_lc)); // children nodes don't overlap with 3rd aabb
        assert!(!is_overlap(&aabb_psib, &aabb_parent)) // 3rd aabb and parent node of children
                                                       // don't overlap
    }

    #[test]
    fn self_intersect_aabb() {
        let a0 = [1.0, 0.0, 0.0];
        let a1 = [0.0, 1.0, 0.0];
        let a2 = [0.0, 0.0, 1.0];

        let b0 = [0.0, 0.0, 0.0];
        let b1 = [-1.0, -1.0, 0.0];
        let b2 = [0.0, -1.0, -1.0];
        let v: Vec<[f32; 3]> = vec![a0, a1, a2, b0, b1, b2];
        let f: Vec<[usize; 3]> = vec![[1, 2, 3], [4, 5, 6]];

        let data = super::Data::build(v, f).expect("Failed in test:: build data");
        // These bounding boxes overlap (they share a bound) and will trigger an intersection test
        // but should return no intersection
        assert!(check_self_intersection(&data))
    }
}
