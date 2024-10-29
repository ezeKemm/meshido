use std::f32;

use crate::coplanar::{self, check_edge_intersect, intersect_test_coplanar};
use crate::data::Data;
use crate::prelude::*;
use nalgebra::Vector3 as Vec3;

// Moller-Trumbore ray-triangle intersection algorithm: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

fn _moller_trumbore_intersection(
    origin: Vec3<f32>,
    direction: Vec3<f32>,
    triangle: Face,
) -> Option<Vec3<f32>> {
    let v0 = Point::new(triangle[0].x, triangle[0].y, triangle[0].z);
    let v1 = Point::new(triangle[1].x, triangle[1].y, triangle[1].z);
    let v2 = Point::new(triangle[2].x, triangle[2].y, triangle[2].z);
    let e1 = v1 - v0;
    let e2 = v2 - v0;

    let ray_cross_e2 = direction.cross(&e2);
    let det = e1.dot(&ray_cross_e2);

    if det > -f32::EPSILON && det < f32::EPSILON {
        return None; // This ray is parallel to this triangle.
    }

    let inv_det = 1.0 / det;
    let s = origin - v0;
    let u = inv_det * s.dot(&ray_cross_e2);
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let s_cross_e1 = s.cross(&e1);
    let v = inv_det * direction.dot(&s_cross_e1);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    // At this stage we can compute t to find out where the intersection point is on the line.
    let t = inv_det * e2.dot(&s_cross_e1);

    if t > f32::EPSILON {
        // ray intersection
        let intersection_point = origin + direction * t;
        Some(intersection_point)
    } else {
        // This means that there is a line intersection but not a ray intersection.
        None
    }
}

#[derive(Debug)]
pub enum RayTri {
    Degenerate,
    Disjoint,
    Parallel,
    Intersecting,
}

// implementation of: https://web.archive.org/web/20161130184358/http://geomalgorithms.com/a06-_intersect-2.html
// sourced from: https://stackoverflow.com/questions/36094637/how-to-detect-intersection-of-two-faces-in-3d
pub fn brute_self_intersection(data: Data) -> bool {
    // brute force search the entire mesh and compare every triangle
    // with every other triangle for intersection
    println!("len: {}, {}", data.face().len(), data.triangles.len());
    for (i, face1) in data.faces.iter().enumerate() {
        // println!("{i}\n{face1}");
        for (j, face2) in data.faces.iter().enumerate() {
            // println!("{j}\n{face2}");
            if i == j {
                // println!("matching face ... moving on {i}:{j}");
                continue;
            } else if i != j && face1 == face2 {
                println!("ERROR: Early Degeneracy :: Different indices, matching faces :: face1: {face1} matches face2: {face2}");
                return false; // duplicate faces
            }
            // perform ray_intersect with each edge of the second face
            // print!("Performing intersection test with face {i} and face {j}: {face1}, {face2}");
            for edge in 0..3 {
                // let p0 = face2[edge];
                // let p1 = if edge == 2 { face2[0] } else { face2[edge + 1] };
                // let (v0, v1, v2) = (face1[0], face1[1], face1[2]);
                // println!(
                //     "edge {edge}, vertex index: {}, face: {}",
                //     face2[edge], face2
                // );
                let p0 = data.vertices[face2[edge] - 1];
                let p1 = if edge == 2 {
                    data.vertices[face2[0] - 1]
                } else {
                    data.vertices[face2[edge + 1] - 1]
                };
                let (v0, v1, v2) = (
                    data.vertices[face1[0] - 1],
                    data.vertices[face1[1] - 1],
                    data.vertices[face1[2] - 1],
                );
                let result = ray_intersect(&v0, &v1, &v2, &p0, &p1);
                // println!("{:?}", result);
                match result {
                    RayTri::Disjoint => {
                        // println!("disjoint {}:{}", i, j);
                        continue;
                    }
                    // terminate upon detecting any self-intersects
                    RayTri::Degenerate => {
                        println!("degenerate {i},{j}");
                        return false;
                    }
                    RayTri::Intersecting => {
                        println!("intersecting {i}:{j}");
                        println!("triangles: {}, {}", data.triangles[i], data.triangles[j]);
                        println!("{}\n{}", face1, face2);
                        return false;
                    }
                    RayTri::Parallel => {
                        if check_coplanar(&data.triangles[i], &data.triangles[j]) {
                            // Triangles are coplanar
                            println!("Parallel case: testing coplanar intersection...");
                            if intersect_test_coplanar(&data.triangles[i], &data.triangles[j]) {
                                println!(
                                    "v0: {}, v1: {}, v2: {}, p0: {}, p1: {}",
                                    v0, v1, v2, p0, p1
                                );
                                println!("faces: {face1}, {face2}");
                                println!("triangles: {}, {}", data.triangles[i], data.triangles[j]);
                                println!("intersecting edges of coplanar faces {i}:{j}");
                                return false;
                            } else {
                                println!("coplanar but safe");
                                break;
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
                                return false;
                            }
                        }
                    }
                }
            }
        }
    }
    true
}

pub fn check_coplanar(face1: &Face, face2: &Face) -> bool {
    // Works bc we already know one edge of face2 is in the plane of face1
    let (a0, a1, a2) = (face1[0], face1[1], face1[2]);
    let (b0, b1, b2) = (face2[0], face2[1], face2[2]);

    let u1 = a1 - a0 / (a1 - a0).magnitude();
    let v1 = a2 - a0 / (a2 - a0).magnitude();
    let u2 = b1 - b0 / (b1 - b0).magnitude();
    let v2 = b2 - b0 / (b2 - b0).magnitude();

    let n1 = u1.cross(&v1);
    let n2 = u2.cross(&v2);

    let m1 = n1.magnitude();
    let m2 = n2.magnitude();

    let dot = n1.dot(&n2);
    let angle = dot / (m1 * m2);
    return n1 == n2;
}

#[allow(dead_code)]
// check self-intersection for coplanar triangles
// check intersection of each triangle edge with each edge of other triangle
// fn test_edge_intersect(face1: &Face, face2: &Face) -> bool {
//     println!("Entering test_edge_intersect...");
//     for e1 in 0..3 {
//         for e2 in 0..3 {
//             let a0: Point = face1[e1];
//             let b0: Point = face2[e2];
//             let a1: Point = if e1 == 2 { face1[0] } else { face1[e1 + 1] };
//             let b1: Point = if e2 == 2 { face2[0] } else { face2[e2 + 1] };
//
//             if edge_intersect(a0, a1, b0, b1) {
//                 println!("e1:{e1}, e2:{e2}\n{a0}->{a1}, {b0}->{b1}");
//                 println!("{}, {}", face1, face2);
//                 return true;
//             }
//         }
//     }
//     false
// }
#[allow(dead_code)]
// check for intersecting edges between two coplanar triangular meshes
fn edge_intersect(a0: Point, a1: Point, b0: Point, b1: Point) -> bool {
    println!("Entering edge_intersect...");

    println!("1st edge: {a0} {a1} \n2nd edge: {b0}, {b1}");
    // original implementation in C++ (https://gist.github.com/hanigamal/6556506) uses norm2() so two vectors specific don't work
    #[allow(non_snake_case)]
    let dA = a1 - a0;
    #[allow(non_snake_case)]
    let dB = b1 - b0;
    #[allow(non_snake_case)]
    let dC = b0 - a0;

    println!("dA: {dA}, dB: {dB}, dC: {dC}");

    let s = dC.cross(&dB).dot(&dA.cross(&dB)) / dA.cross(&dB).norm();
    println!(
        "s: {s}, deonominator: {}, state: {}",
        dA.cross(&dB).norm(),
        s >= 0.0 && s <= 1.0
    );
    return s >= 0.0 && s <= 1.0; // true if intersects
}

#[allow(dead_code)]
struct Log<'a> {
    v0: &'a Point,
    v1: &'a Point,
    v2: &'a Point,
    p0: &'a Point,
    p1: &'a Point,
    u: Point,
    v: Point,
    n: Point,
    w0: Point,
    dir: Point,
    intersect: Point,
    r: f32,
    a: f32,
    b: f32,
    s: f32,
    t: f32,
}

impl<'a> Log<'a> {
    fn write(self) {
        println!("triangle: {} {} {}", self.v0, self.v1, self.v2);
        println!("ray: {} {}", self.p0, self.p1);
        println!("intersect: {}", self.intersect);
        println!("r: {}, s: {}, t:{}", self.r, self.s, self.t);
    }
}

pub fn ray_intersect(
    v0: &Point, // triangle vertices
    v1: &Point,
    v2: &Point,
    p0: &Point, // "ray" two triangle vertices forming an edge
    p1: &Point,
) -> RayTri {
    // Assume a shared point (or shared edge) is not a valid intersection bc of previous checks
    // NOTE: this doesn't guarantee that the triangles in question don't intersect
    // Ex) 2 triangles who share a vertex but overlap in a shared plane
    if (p0 == v0 || p0 == v1 || p0 == v2) || (p1 == v0 || p1 == v1 || p1 == v2) {
        return RayTri::Disjoint;
    }
    // Connections to a triangle at any point beyond a vertex are a valid intersection case
    // VALID: Vertex-to-edge
    // INVALID: Vertex-to-Vertex (shared vertex)

    // Calculate triangle vectors and normal vector of the triangle's plane
    let u = v1 - v0;
    let v = v2 - v0;
    let n = u.cross(&v);

    if n == nalgebra::Vector3::zeros() {
        return RayTri::Degenerate; // degenerate triangle
    }

    let dir = p1 - p0; // compute ray's direction vector
    let w0 = p0 - v0;

    let a = -n.dot(&w0);
    let b = n.dot(&dir);

    if f32::abs(b) < 0.00000001 {
        if a == 0.0 {
            return RayTri::Parallel; // ray lies in the triangular plane (co-planar)
        }
        return RayTri::Disjoint; // ray is parallel to triangle but non-intersecting
                                 // (not necessarily coplanar...skew?)
    }
    let r = a / b; // intersection point of ray with triangle
    if r <= 0.0 || r >= 1.0 {
        // ray diverges from triangle
        // test r > 1 since ray is a segment (a triangle's edge)
        return RayTri::Disjoint;
    }

    let intersect = p0 + r * dir; // Project the intersection point onto the plane
    let uu = u.dot(&u);
    let vv = v.dot(&v);
    let uv = u.dot(&v);

    // parameterized vector from triangle vertex to the ray-plane intersection
    // w(s, t) = s*u + t*v
    let w = intersect - v0;
    let wu = w.dot(&u); // components of vector w (projections onto u and v vectors)
    let wv = w.dot(&v);
    let det = (uv * uv) - (uu * vv); // calculate determinant

    let s = ((uv * wv) - (vv * wu)) / det;
    let t = ((uv * wu) - (uu * wv)) / det;

    // our triangle is parameterized such that the point lies within the triangle if:
    // s ≥ 0, t ≥ 0, and s+t ≤ 1.0 (we consider < and not ≤ bc we don't consider the edge)
    // NOTE: Presently, we have revised it to count
    // s = 0, t = 0, and s+t = 1 (1 for each edge)
    // so we are considering the edges

    #[rustfmt::skip]
    let log = Log { v0, v1, v2, p0, p1, u, v, n, w0, dir, intersect, r, a, b, s, t };
    // Explicit check for edge intersections
    // FIX: this case results in a false positive (false intersect), figure out why and if
    // avoidable
    if s == 0.0 || t == 0.0 || (s + t) == 0.0 {
        println!("Intersection detected in ray_intersect :: edge intersection");
        log.write();
        return RayTri::Intersecting;
    }

    // Thus, the intersect with the plane is outside the triangle under the following conditions:
    // NOTE: im not really sure if the s >= 1.0 case is important???
    if s < 0.0 || s >= 1.0 {
        return RayTri::Disjoint;
    }
    if t < 0.0 || (s + t) > 1.0 {
        return RayTri::Disjoint;
    }

    // Else, the intersection lies inside the triangle
    println!("Intersection detected in ray_intersect");
    log.write();
    RayTri::Intersecting
}

#[cfg(test)]
mod test {
    use nalgebra::Point3;

    use crate::brutus::{brute_self_intersection, RayTri};

    use super::{check_coplanar, Face, Point};

    #[test]
    fn ray_intersects() {
        let v0 = nalgebra::vector![0.0, 0.0, 0.0];
        let v1 = nalgebra::vector![3.0, 0.0, 0.0];
        let v2 = nalgebra::vector![0.0, 3.0, 0.0];
        let p0 = nalgebra::vector![1.0, 1.0, 1.0];
        let p1 = nalgebra::vector![1.0, 1.0, -1.0];

        use super::ray_intersect;
        let result = ray_intersect(&v0, &v1, &v2, &p0, &p1);
        assert!({
            if let RayTri::Intersecting = result {
                true
            } else {
                false
            }
        })
    }

    #[test]
    fn ray_intersects_edge() {
        let v0 = nalgebra::vector![0.0, 0.0, 0.0];
        let v1 = nalgebra::vector![3.0, 0.0, 0.0];
        let v2 = nalgebra::vector![0.0, 3.0, 0.0];
        // Ray is perpendicular to triangle (thru xy-plane, along z-plane)
        // and intersects the triangle edge from
        // <0, 3, 0> to <0, 0, 0>
        let p0 = nalgebra::vector![0.0, 1.0, 1.0];
        let p1 = nalgebra::vector![0.0, 1.0, -1.0];

        use super::ray_intersect;
        let result = ray_intersect(&v0, &v1, &v2, &p0, &p1);
        assert!({
            if let RayTri::Intersecting = result {
                true
            } else {
                false
            }
        })
    }

    #[test]
    fn ray_parallel() {
        // Triangle in positive xy-plane with leg length of 3
        let v0 = nalgebra::vector![0.0, 0.0, 0.0];
        let v1 = nalgebra::vector![3.0, 0.0, 0.0];
        let v2 = nalgebra::vector![0.0, 3.0, 0.0];
        // A ray parallel to the triangle plane (non-intersecting)
        let p0 = nalgebra::vector![1.0, 1.0, 1.0];
        let p1 = nalgebra::vector![3.0, 3.0, 1.0];

        use super::ray_intersect;
        let result = ray_intersect(&v0, &v1, &v2, &p0, &p1);
        println!("test_result: {:?}", result);
        assert!({
            if let RayTri::Disjoint = result {
                true
            } else {
                false
            }
        })
    }
    #[test]
    fn ray_fucked() {
        // Triangle in positive xy-plane with leg length of 3
        let v0 = nalgebra::vector![0.0, 1.0, 0.0];
        let v1 = nalgebra::vector![-1.0, -1.0, 0.0];
        let v2 = nalgebra::vector![0.0, -1.0, -1.0];
        // A ray parallel to the triangle plane (non-intersecting)
        let p0 = nalgebra::vector![1.0, 0.0, 0.0];
        let p1 = nalgebra::vector![0.0, 1.0, 0.0];

        use super::ray_intersect;
        let result = ray_intersect(&v0, &v1, &v2, &p0, &p1);
        println!("test_result: {:?}", result);
        assert!({
            if let RayTri::Disjoint = result {
                true
            } else {
                false
            }
        })
    }

    #[test]
    fn test_check_coplanar() {
        let a0: Point3<f32> = Point3::new(0.0, 0.0, 0.0);
        let a1: Point3<f32> = Point3::new(1.0, 0.0, 0.0);
        let a2: Point3<f32> = Point3::new(0.0, 1.0, 0.0);

        let b0: Point3<f32> = Point3::new(3.0, 0.0, 0.0);
        let b1: Point3<f32> = Point3::new(4.0, 0.0, 0.0);
        let b2: Point3<f32> = Point3::new(3.0, 1.0, 0.0);

        let face1 = Face::new(a0, a1, a2);
        let face2 = Face::new(b0, b1, b2);
        assert!(check_coplanar(&face1, &face2))
    }

    #[test]
    fn parallel_not_coplanar() {
        let a0 = [0.0, 0.0, 0.0];
        let a1 = [1.0, 0.0, 0.0];
        let a2 = [0.0, 1.0, 0.0];

        let b0 = [0.5, -0.5, 0.0];
        let b1 = [0.5, 0.5, 0.0];
        let b2 = [0.5, -0.5, 1.0];
        let v: Vec<[f32; 3]> = vec![a0, a1, a2, b0, b1, b2];
        let f: Vec<[usize; 3]> = vec![[1, 2, 3], [4, 5, 6]];

        let data = super::Data::build(v, f).expect("Failed in test:: build data");

        assert!(!brute_self_intersection(data))
    }
}
