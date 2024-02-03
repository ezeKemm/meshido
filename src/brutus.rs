use std::f32;

use crate::data::Data;
use crate::prelude::*;
use nalgebra::Vector3 as Vec3;

// Moller-Trumbore ray-triangle intersection algorithm: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

fn _moller_trumbore_intersection(
    origin: Vec3<f32>,
    direction: Vec3<f32>,
    triangle: Face,
) -> Option<Vec3<f32>> {
    let e1 = triangle[1] - triangle[0];
    let e2 = triangle[2] - triangle[0];

    let ray_cross_e2 = direction.cross(&e2);
    let det = e1.dot(&ray_cross_e2);

    if det > -f32::EPSILON && det < f32::EPSILON {
        return None; // This ray is parallel to this triangle.
    }

    let inv_det = 1.0 / det;
    let s = origin - triangle[0];
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
enum RayTri {
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
    println!("{}, {}", data.face().len(), data.triangles.len());
    for (i, face1) in data.faces.iter().enumerate() {
        // println!("{i}\n{face1}");
        for (j, face2) in data.faces.iter().enumerate() {
            // println!("{j}\n{face2}");
            if i == j {
                println!("matching face ... moving on {i}:{j}");
                continue;
            } else if i != j && face1 == face2 {
                println!("early degeneracy");
                return false; // duplicate faces
            }
            // perform ray_intersect with each edge of the second face
            for edge in 0..3 {
                // let p0 = face2[edge];
                // let p1 = if edge == 2 { face2[0] } else { face2[edge + 1] };
                // let (v0, v1, v2) = (face1[0], face1[1], face1[2]);
                println!("edge {edge}, face index: {}", face2[edge]);
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
                        println!("disjoint {}:{}", i, j);
                        continue;
                    }
                    // terminate upon detecting any self-intersects
                    RayTri::Degenerate => {
                        println!("degenerate {i},{j}");
                        return false;
                    }
                    RayTri::Intersecting => {
                        println!("intersecting {i}:{j}");
                        println!("{}\n{}", face1, face2);
                        return false;
                    }
                    RayTri::Parallel => {
                        if test_edge_intersect(&data.triangles[i], &data.triangles[j]) {
                            println!("{face1}, {face2}");
                            println!("intersecting edges of coplanar faces {i}:{j}");
                            return false;
                        } else {
                            println!("coplanar but safe");
                            continue;
                        }
                    }
                }
            }
        }
    }
    true
}
// check self-intersection for coplanar triangles
// check intersection of each triangle edge with each edge of other triangle
fn test_edge_intersect(face1: &Face, face2: &Face) -> bool {
    for e1 in 0..3 {
        for e2 in 0..3 {
            let a0: Point = face1[e1];
            let b0: Point = face2[e2];
            let a1: Point = if e1 == 2 { face1[0] } else { face1[e1 + 1] };
            let b1: Point = if e2 == 2 { face2[0] } else { face2[e2 + 1] };

            if edge_intersect(a0, a1, b0, b1) {
                println!("e1:{e1}, e2:{e2}\n{a0}->{a1}, {b0}->{b1}");
                return true;
            }
        }
    }
    false
}

// check for intersecting edges between two coplanar triangular meshes
fn edge_intersect(a0: Point, a1: Point, b0: Point, b1: Point) -> bool {
    // original implementation in C++ (https://gist.github.com/hanigamal/6556506) uses norm2() so two vectors specific don't work
    let dA = a1 - a0;
    let dB = b1 - b0;
    let dC = b0 - a0;

    let s = dC.cross(&dB).dot(&dA.cross(&dB)) / dA.cross(&dB).norm();
    return s >= 0.0 && s <= 1.0; // true if intersects
}

fn ray_intersect(
    v0: &Point, // triangle vertices
    v1: &Point,
    v2: &Point,
    p0: &Point, // "ray" two triangle vertices forming an edge
    p1: &Point,
) -> RayTri {
    // calculate triangle vectors and normal vector of the triangle's plane
    let u = v1 - v0;
    let v = v2 - v0;
    let n = u.cross(&v);

    if n == nalgebra::Vector3::zeros() {
        return RayTri::Degenerate; // degenerate triangle
    }

    let dir = p1 - v0; // compute ray's direction vector
    let w0 = p0 - v0;
    let a = -n.dot(&w0);
    let b = n.dot(&dir);

    if f32::abs(b) < 0.00000001 {
        if a == 0.0 {
            return RayTri::Parallel; // ray lies in the triangular plane
        }
        return RayTri::Disjoint; // ray is parallel to triangle but non-intersecting
    }
    println!("a: {a}, b: {b}");
    let r = a / b; // intersection point of ray with triangle
    if r <= 0.0 || r >= 1.0 {
        // ray diverges from triangle
        // test r > 1 since ray is a segment (a triangle's edge)
        return RayTri::Disjoint;
    }

    let intersect = p0 + r * dir; // calculate the intersection point
    let uu = u.dot(&u);
    let uv = u.dot(&v);
    let vv = v.dot(&v);

    // parameterized vector from triangle vertex to the ray-plane intersection, parallel to the ray-plane
    // w(s, t) = s*u + t*v
    let w = intersect - v0;
    let wu = w.dot(&u);
    let wv = w.dot(&v);
    let det = (uv * uv) - (uu * vv); // calculate determinant

    let s = ((uv * wv) - (vv * wu)) / det;
    let t = ((uv * wu) - (uu * wv)) / det;

    // our triangle is parameterized such that the point lies within the triangle if:
    // s ≥ 0, t ≥ 0, and s+t ≤ 1.0 (we consider < and not ≤ bc we don't consider the edge)
    // Thus, the interset with the plane is outside the triangle under the following conditions:
    if s <= 0.0 || s >= 1.0 {
        return RayTri::Disjoint;
    }
    if t <= 0.0 || (s + t) >= 1.0 {
        return RayTri::Disjoint;
    }
    println!("r:{}, s:{}, t:{}", r, s, t);
    // Else, the intersection lies inside the triangle
    RayTri::Intersecting
}
