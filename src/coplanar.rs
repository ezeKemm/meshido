use crate::{prelude::*};
use nalgebra::{vector, Matrix6x5, Point3};

const THRESHOLD: f32 = 0.000001;

pub fn intersect_test_coplanar(t1: &Face, t2: &Face) -> bool {
    let a0 = Point::new(t1.x.x, t1.x.y, t1.x.z);
    let a1 = Point::new(t1.y.x, t1.y.y, t1.y.z);
    let a2 = Point::new(t1.z.x, t1.z.y, t1.z.z);

    let b0 = Point::new(t2.x.x, t2.x.y, t2.x.z);
    let b1 = Point::new(t2.y.x, t2.y.y, t2.y.z);
    let b2 = Point::new(t2.z.x, t2.z.y, t2.z.z);
    return test_coplanar(a0, a1, a2, b0, b1, b2);
}
// Returns true if the coplanar triangles intersect, false otherwise
fn test_coplanar(
    a0: Point,
    a1: Point,
    a2: Point,
    b0: Point,
    b1: Point,
    b2: Point,
) -> bool {
    // This test is simplified by recognizing partial or full overlap of triangles is unallowed ...
    // < 1 point inside counts as an intersection (so test returns on first intersection regardless)
    // The only valid cases of intersection/overlap are of >= 2 shared vertices (shared edge)
    // Warning: this case can still fail if there is overlap elsewhere
    // (i.e. third point inside the triangle...)

    let (free_vert_a, shared_vert_a, free_vert_b, shared_vert_b) =
        check_shared(a0, a1, a2, b0, b1, b2);
    println!(
        "shared verts: {}",
        shared_vert_a.len()
    );

    #[rustfmt::skip]
    return free_vert_a.len() == 0
        || (free_vert_a.len() == 1
            && perform_shared_edge(a0, a1, a2, b0, b1, b2,
                &free_vert_a, &shared_vert_a, 
                &free_vert_b, &shared_vert_b,
            ))
        || (free_vert_a.len() == 2
            && perform_shared_vertex(a0, a1, a2, b0, b1, b2,
                &free_vert_a, &shared_vert_a,
                &free_vert_b, &shared_vert_b,
            ))
        || (free_vert_a.len() == 3 && perform_unshared_intersect(a0, a1, a2, b0, b1, b2));
}

fn check_shared(
    a0: Point,
    a1: Point,
    a2: Point,
    b0: Point,
    b1: Point,
    b2: Point,
) -> (Vec<Point>, Vec<Point>, Vec<Point>, Vec<Point>) {
    // println!("Getting shared vertices");
    let mut a: Vec<Point> = vec![a0, a1, a2];
    let mut b: Vec<Point> = vec![b0, b1, b2];
    let mut shared_count = 0;
    // In the case of a shared vertex, we must check for any edge-edge intersections or vertex
    // "intersections" between the two "free" vertices / edges
    // In the case of a shared edge, we must likewise check for intersections of both variaties in
    // the third vertex

    let mut shared_a: Vec<Point> = vec![];
    let mut shared_b: Vec<Point> = vec![];
    // TODO: Functional methods?
    for i in 0..3 {
        for j in 0..3 {
            if a[i] == b[j] {
                // println!("shared vertices :: a[{i}]: {:?}, b[{j}]: {:?}", a[i], b[j]);
                shared_count += 1;
                shared_a.push(a[i]);
                shared_b.push(b[j]);
            }
        }
    }
    a.retain(|p| !shared_a.contains(p));
    b.retain(|p| !shared_b.contains(p));

    (a, shared_a, b, shared_b)
}

fn perform_shared_vertex(
    a0: Point,
    a1: Point,
    a2: Point,
    b0: Point,
    b1: Point,
    b2: Point,
    free_vert_a: &Vec<Point>,
    shared_vert_a: &Vec<Point>,
    free_vert_b: &Vec<Point>,
    shared_vert_b: &Vec<Point>,
) -> bool {
    // println!("Shared vertex case");
    // Case: shared vertex
    return check_point_inside(a0, a1, a2, free_vert_b[0])
        || check_point_inside(a0, a1, a2, free_vert_b[1])
        || check_point_inside(b0, b1, b2, free_vert_a[0])
        || check_point_inside(b0, b1, b2, free_vert_a[1])
    // Need not test all cases bc some covered by previous check
    // For example: the edges anchored by the shared vertex only intersect in the case they are
    // parallel, this overlap is covered by the edge-vertex check (and is a degenerate triangle
    // or shared edge otherwise)
        || check_edge_intersect(
            free_vert_a[0],
            free_vert_a[1],
            shared_vert_b[0],
            free_vert_b[0],
        )
        || check_edge_intersect(
            free_vert_a[0],
            free_vert_a[1],
            shared_vert_b[0],
            free_vert_b[1],
        )
        || check_edge_intersect(
            shared_vert_a[0],
            free_vert_a[0],
            free_vert_b[0],
            free_vert_b[1],
        )
        || check_edge_intersect(
            shared_vert_a[0],
            free_vert_a[1],
            free_vert_b[0],
            free_vert_b[1],
        );
}

fn perform_shared_edge(
    a0: Point,
    a1: Point,
    a2: Point,
    b0: Point,
    b1: Point,
    b2: Point,
    free_vert_a: &Vec<Point>,
    shared_vert_a: &Vec<Point>,
    free_vert_b: &Vec<Point>,
    shared_vert_b: &Vec<Point>,
) -> bool {
    // println!("Shared edge case");
    // println!(
    //     "a:: free: {}, shared: {}\nb:: free: {}, shared: {}",
    //     free_vert_a.len(),
    //     shared_vert_a.len(),
    //     free_vert_b.len(),
    //     shared_vert_b.len()
    // );
    // Case: Shared edge
    let e11 = shared_vert_a[0];
    let e12 = free_vert_a[0];

    let e21 = shared_vert_b[0];
    let e22 = free_vert_b[0];

    let e31 = shared_vert_a[1];
    let e32 = free_vert_a[0];

    let e41 = shared_vert_b[1];
    let e42 = free_vert_b[0];

    return check_point_inside(a0, a1, a2, free_vert_b[0])
        || check_point_inside(b0, b1, b2, free_vert_a[0])
        || check_edge_intersect(e11, e12, e21, e22)
        || check_edge_intersect(e11, e12, e41, e42)
        || check_edge_intersect(e31, e32, e21, e22)
        || check_edge_intersect(e31, e32, e41, e42);
}

fn perform_unshared_intersect(
    a0: Point,
    a1: Point,
    a2: Point,
    b0: Point,
    b1: Point,
    b2: Point,
) -> bool {
    // println!("Full intersect test");
    // First, check for any vertex of one triangle within the area of the other triangle
    let a: [Point; 3] = [a0, a1, a2];
    let b: [Point; 3] = [b0, b1, b2];
    let mut pa: Point;
    let mut pb: Point;
    for i in 0..3 {
        pa = a[i];
        pb = b[i];
        // Check both ways
        if check_point_inside(a0, a1, a2, pb) || check_point_inside(b0, b1, b2, pa) {
            return true;
        }
    }

    // Then, check for edge-edge intersections
    // Fuck a for loop
    return check_edge_intersect(a0, a1, b0, b1)
        || check_edge_intersect(a0, a1, b1, b2)
        || check_edge_intersect(a0, a1, b2, b0)
        || check_edge_intersect(a1, a2, b0, b1)
        || check_edge_intersect(a1, a2, b1, b2)
        || check_edge_intersect(a1, a2, b2, b0)
        || check_edge_intersect(a2, a0, b0, b1)
        || check_edge_intersect(a2, a0, b1, b2)
        || check_edge_intersect(a2, a0, b2, b0);
}

#[derive(Debug)]
#[allow(dead_code)]
enum ParallelCheck {
    Parallel,
    Colinear,
    Neither,
}

// TODO: this shit is held together by ducktape and a prayer ... fix that
fn check_parallel(a0: Point, a1: Point, b0: Point, b1: Point) -> ParallelCheck {
    let u = a1 - a0; // Vectors
    let v = b1 - b0;
    let dot = u.dot(&v); // Dot product
    let n = u.cross(&v); // Cross product of the vectors (normal)
    let magnitude = u.magnitude() * v.magnitude();
    let angle = dot / magnitude; // Angle between vectors [-1, 1]

    // Check for parallel case
    // An dot product of 1 or -1 indicates parallel (or antiparallel) vectors
    if angle == 1.0 || angle == -1.0 {
        // Vectors are colinear when they exist on the same line; for colinear line segments, consider
        // two cases: the segments overlap (intersect) or they don't (non-intersecting). The algorithm
        // is designed such that the vertex check will always precede the edge check; so the vertex
        // check will detect colinear intersecting line segments before a parallel check is called.
        // So assume an intersection cannot be missed and the colinear case only is 
        // considered to determine if the edges are actually parallel.
            if n == nalgebra::Vector3::zeros() {
                println!("colinear non-intersecting");
                return ParallelCheck::Colinear;
            }
            println!("Parallel vectors:");
            println!("|u|: {}, |v|: {}", u.magnitude(), v.magnitude());
            println!("a0: {}, a1: {}, b0: {},  b1: {}", a0, a1, b0, b1);
            println!("angle: {} | dot: {}, mag: {}", angle, dot, magnitude);
            return ParallelCheck::Parallel;
    }
    ParallelCheck::Neither // NOTE: we could make this function return a boolean
}

// TODO: use this or nah?
fn is_shared(a0: Point, a1: Point, b0: Point, b1: Point) {
    if a0 == b0 || a0 == b1 { () }
}

fn equals(lhs: f32, rhs: f32) -> bool {
    let low_end = lhs - THRESHOLD;
    let high_end = lhs + THRESHOLD;

    if rhs >= low_end && rhs <= high_end {
        println!("lhs: {lhs}, rhs: {rhs}");
        println!("low: {low_end}, high: {high_end}");
        println!("lhs- <= rhs <= lhs+ : {} && {}", rhs >= low_end, rhs <= high_end);
    }
    return rhs >= low_end && rhs <= high_end;
}

// Check that two Points equal each other within margin of error
fn point_equals(lhs: Point, rhs: Point) -> bool {
    let (x, y, z) = (lhs.x, lhs.y, lhs.z);
    let (t, u, v) = (rhs.x, rhs.y, rhs.z);
    let res = equals(x, t) && equals(y, u) && equals(z, v);
    if res {
        println!("lhs: {}, rhs: {}", lhs, rhs);
    }
    return res;
}

pub fn check_edge_intersect(a0: Point, a1: Point, b0: Point, b1: Point) -> bool {
    let (x1, y1, z1) = (a0.x, a0.y, a0.z);          // Origin point
    let (x2, y2, z2) = (b0.x, b0.y, b0.z);
    let dir1 = a1 - a0;                             // Direction vector
    let dir2 = b1 - b0;
    let (d1x, d1y, d1z) = (dir1.x, dir1.y, dir1.z); // Direction vector decomposition
    let (d2x, d2y, d2z) = (dir2.x, dir2.y, dir2.z);

    match check_parallel(a0, a1, b0, b1) {
        ParallelCheck::Parallel | ParallelCheck::Colinear => return false,
        ParallelCheck::Neither => (),              // Perform regular edge-edge intersection test
    }

    // Solve the matrix equation A*x = b for x
    #[rustfmt::skip]
    #[allow(non_snake_case)]
    let A = Matrix6x5::new(
        1.0, 0.0, 0.0, -d1x,  0.0,
        0.0, 1.0, 0.0, -d1y,  0.0,
        0.0, 0.0, 1.0, -d1z,  0.0,
        1.0, 0.0, 0.0,  0.0, -d2x,
        0.0, 1.0, 0.0,  0.0, -d2y,
        0.0, 0.0, 1.0,  0.0, -d2z,
    );

    // Calculate the pseudo-inverse of A
    #[allow(non_snake_case)]
    let pA = A.pseudo_inverse(std::f32::EPSILON).unwrap();
    let b = vector![x1, y1, z1, x2, y2, z2];
    let x = pA * b; // Solve for x
    let s = x[3]; // Get s and t
    let t = x[4];

    // In case intercept is at a shared vertex, we just ignore it ... assuming it is valid
    let intercept = Point::new(x[0], x[1], x[2]);
    if point_equals(intercept, a0) || point_equals(intercept, a1) || point_equals(intercept, b0) || point_equals(intercept, b1) {
        // println!("Bypassing intercept at shared vertex");
        return false;
    }


    // Equation solves for intersection point of a line
    // For a line segment, s and t fulfill the following bounds
    // TODO: figure out if these bounds require more (s+t <= 2?)
    if s >= 0.0 && t >= 0.0 && s <= 1.0 && t <= 1.0 {
        println!("Edge-edge intersection:");
        println!("edge A: {} {}, edge B: {} {}", a0, a1, b0, b1);
        println!("s: {s}, t: {t}, s+t: {}", s + t);
        // println!("{A}");
        // println!("{pA}*{b}");
        println!("x: {x}");
        return true;
    }
    false
}

pub fn check_point_inside(v0: Point, v1: Point, v2: Point, p0: Point) -> bool {
    let u = v1 - v0;
    let v = v2 - v0;

    let uu = u.dot(&u);
    let vv = v.dot(&v);
    let uv = u.dot(&v);

    // For each point, determine if it lies within the area of the triangle
    let w = p0 - v0;
    let wu = w.dot(&u);
    let wv = w.dot(&v);
    let det = (uv * uv) - (uu * vv);

    let s = ((uv * wv) - (vv * wu)) / det; // Parameterized coords with respect to
    let t = ((uv * wu) - (uu * wv)) / det; // triangle vectors u and v

    // NOTE: To avoid the difficult vertex-vertex case, should be able to constrain
    // s and t further: s < 1.0 && t < 1.0?
    // Does this introduce complications? How to handle these cases more elegantly
    if s >= 0.0 && t >= 0.0 && (s + t) <= 1.0 {
        print!("Points inside triangle area: ");
        println!("v0: {}, v1: {}, v2: {}, p0: {}", v0, v1, v2, p0);
        println!("s: {s}, t: {t}, s+t: {}", s + t);
        // Point lies within the triangle bounds
        return true;
    }
    // If any point lies in or on the triangle, the coplanar triangles intersect
    // NOTE: such cases as a triangle being enclosed entirely by the other triangle are possible
    // but qualify as degenerate cases and are grouped under intersection ... handle separately?
    false
}

// ************************** \\
// ************************** \\
//      ** UNIT TESTS **      \\
// ************************** \\ 
// ************************** \\

#[cfg(test)]
mod test {
    use super::{check_edge_intersect, check_point_inside, check_shared, test_coplanar};
    use crate::{coplanar::equals, prelude::Point};

    #[test]
    // One point lies within the triangle
    fn vertex_check_one() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.5, 1.0, 0.0];

        let b0 = nalgebra::vector![0.5, 0.5, 0.0]; // point inside
        let _b1 = nalgebra::vector![1.5, 0.5, 0.0];
        let _b2 = nalgebra::vector![1.0, 1.5, 0.0];

        assert!(check_point_inside(a0, a1, a2, b0))
    }

    #[test]
    // Two points lie within the triangle
    fn vertex_check_two() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.5, 1.0, 0.0];

        let b0 = nalgebra::vector![0.6, 0.5, 0.0];
        let b1 = nalgebra::vector![0.4, 0.5, 0.0];
        let _b2 = nalgebra::vector![0.5, 2.0, 0.0];

        assert!(check_point_inside(a0, a1, a2, b0) && check_point_inside(a0, a1, a2, b1))
    }

    #[test]
    fn vertex_check_three() {
        // Three points lie within the triangle
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![2.0, 0.0, 0.0];
        let a2 = nalgebra::vector![1.0, 2.0, 0.0];

        let b0 = nalgebra::vector![0.5, 0.5, 0.0];
        let b1 = nalgebra::vector![1.5, 0.5, 0.0];
        let b2 = nalgebra::vector![1.0, 1.5, 0.0];

        // A triangle that doesn't intersect but instead lies entirely within the area of a
        // coplanar triangle counts as a degenerate case and is treated as an intersection
        assert!(
            check_point_inside(a0, a1, a2, b0)
                && check_point_inside(a0, a1, a2, b1)
                && check_point_inside(a0, a1, a2, b2)
        )
    }

    #[test]
    // Triangle point meets another triangle edge
    fn vertex_check_edge() {
        let _a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let _a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.5, 1.0, 0.0];

        let b0 = nalgebra::vector![0.0, 1.0, 0.0];
        let b1 = nalgebra::vector![1.0, 1.0, 0.0];
        let b2 = nalgebra::vector![0.5, 2.0, 0.0];

        // The second test case works while the first will not,
        // thus why we test both generally
        assert!(check_point_inside(b0, b1, b2, a2))
    }

    #[test]
    // Technically, this does intersect, but returns false because it intersects at the vertex of
    // edge B and is treated as a shared vertex. The algorithm would however spot this case ahead
    // of time in the vertex check and so this case of a false negative can't be reached (I hope)
    // This test remains for the sake of reminding us of this edge (ha) case
    fn edge_check_edge_intersect_false_neg() {
        let a0 = nalgebra::vector![0.0, 1.0, 0.0];
        let a1 = nalgebra::vector![1.0, 1.0, 0.0];

        let b0 = nalgebra::vector![0.5, 0.0, 0.0];
        let b1 = nalgebra::vector![0.5, 1.0, 0.0];

        assert!(!check_edge_intersect(a0, a1, b0, b1))
    }

    #[test]
    fn edge_check_edge_intersect() {
        let a0 = nalgebra::vector![0.0, 0.5, 0.0];
        let a1 = nalgebra::vector![1.0, 0.5, 0.0];

        let b0 = nalgebra::vector![0.5, 0.0, 0.0];
        let b1 = nalgebra::vector![0.5, 1.0, 0.0];

        assert!(check_edge_intersect(a0, a1, b0, b1))
    }

    #[test]
    fn edge_check_edge_disjoint() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.0, 0.0, 0.0];
        let b1 = nalgebra::vector![2.0, 0.0, 0.0];

        assert!(!check_edge_intersect(a0, a1, b0, b1))
    }

    // TODO: parallel test cases

    #[test]
    fn check_shared_edge() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let a2 = nalgebra::vector![1.0, 1.0, 0.0]; // edge

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let b1 = nalgebra::vector![1.0, 1.0, 0.0]; // edge
        let b2 = nalgebra::vector![2.0, 0.0, 0.0];

        let free_a = vec![a0];
        let shared_a = vec![a1, a2];
        let free_b = vec![b2];
        let shared_b = vec![b0, b1];

        let (a, s_a, b, s_b) = check_shared(a0, a1, a2, b0, b1, b2);
        println!("a: {:?}, s_a: {:?}, b: {:?}, s_b: {:?}", a, s_a, b, s_b);
        assert!(free_a == a && shared_a == s_a && free_b == b && shared_b == s_b)
    }

    #[test]
    fn check_shared_vertex() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let b1 = nalgebra::vector![2.0, 1.0, 0.0];
        let b2 = nalgebra::vector![2.0, 0.0, 0.0];

        let free_a = vec![a0, a2];
        let shared_a = vec![a1];
        let free_b = vec![b1, b2];
        let shared_b = vec![b0];

        let (a, s_a, b, s_b) = check_shared(a0, a1, a2, b0, b1, b2);
        println!("a: {:?}, s_a: {:?}, b: {:?}, s_b: {:?}", a, s_a, b, s_b);
        assert!(free_a == a && shared_a == s_a && free_b == b && shared_b == s_b)
    }
    #[test]
    fn check_shared_none() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.5, 0.0, 0.0];
        let b1 = nalgebra::vector![2.5, 1.0, 0.0];
        let b2 = nalgebra::vector![2.5, 0.0, 0.0];

        let free_a = vec![a0, a1, a2];
        let shared_a: Vec<Point> = vec![];
        let free_b = vec![b0, b1, b2];
        let shared_b: Vec<Point> = vec![];

        let (a, s_a, b, s_b) = check_shared(a0, a1, a2, b0, b1, b2);
        println!("a: {:?}, s_a: {:?}, b: {:?}, s_b: {:?}", a, s_a, b, s_b);
        assert!(free_a == a && shared_a == s_a && free_b == b && shared_b == s_b)
    }

    #[test]
    fn check_shared_degenerate() {
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![0.0, 0.0, 0.0];
        let b1 = nalgebra::vector![1.0, 0.0, 0.0];
        let b2 = nalgebra::vector![0.0, 1.0, 0.0];

        let (a, _, _, _) = check_shared(a0, a1, a2, b0, b1, b2);
        assert!(a.len() == 0)
    }

    // ********************************** \\
    // ********************************** \\
    //    KINDA AN INTEGRATION TEST????   \\
    // ********************************** \\
    // ********************************** \\

    #[test]
    fn full_check_shared_edge_noint() {
        // No intersections for two triangles sharing an edge
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let a2 = nalgebra::vector![1.0, 1.0, 0.0]; // edge

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let b1 = nalgebra::vector![1.0, 1.0, 0.0]; // edge
        let b2 = nalgebra::vector![2.0, 0.0, 0.0];

        assert!(!test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    #[test]
    fn full_check_shared_vertex_noint() {
        // No intersections for two triangles sharing a vertex
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let b1 = nalgebra::vector![2.0, 1.0, 0.0];
        let b2 = nalgebra::vector![2.0, 0.0, 0.0];

        assert!(!test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn full_check_none_shared_noint() {
        // No intersection between two triangles with no shared vertices (i.e. completely disjoint)
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.5, 0.0, 0.0];
        let b1 = nalgebra::vector![2.5, 1.0, 0.0];
        let b2 = nalgebra::vector![2.5, 0.0, 0.0];

        assert!(!test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    #[test]
    fn shared_edge_edge_intersect() {
        // Test for intersection between two edges for two triangles sharing an edge
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let a2 = nalgebra::vector![1.0, 1.0, 0.0]; // edge

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let b1 = nalgebra::vector![1.0, 1.0, 0.0]; // edge
        let b2 = nalgebra::vector![0.25, 0.75, 0.0]; // intersecting edge (b2-b0)

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn shared_edge_edge_vertex_intersect() {
        // Test for intersection between a vertex and an edge for two triangles sharing an edge
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let a2 = nalgebra::vector![1.0, 1.0, 0.0]; // edge

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let b1 = nalgebra::vector![1.0, 1.0, 0.0]; // edge
        let b2 = nalgebra::vector![0.5, 0.5, 0.0]; // intersecting vertex w edge

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn shared_edge_vertex_overlap() {
        // Test intersection between vertex within triangle for two triangles sharing an edge
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let a2 = nalgebra::vector![1.0, 1.0, 0.0]; // edge

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared
        let b1 = nalgebra::vector![1.0, 1.0, 0.0]; // edge
        let b2 = nalgebra::vector![0.25, 0.25, 0.0]; // overlapping vertex

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn shared_vertex_edge_intersect() {
        // Test for intersection between two edges for two triangles sharing a vertex
        // Flipped first triangle
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 1.0, 0.0]; // shared vertex
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let b1 = nalgebra::vector![2.0, 1.0, 0.0];
        let b2 = nalgebra::vector![0.0, 0.5, 0.0]; // vertex forms intersecting edges

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn shared_vertex_edge_vertex_intersect() {
        // Test for intersection between a vertex and an edge for two triangles sharing a vertex
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let b1 = nalgebra::vector![2.0, 1.0, 0.0];
        let b2 = nalgebra::vector![0.0, 0.5, 0.0]; // intersecting vertex with edge

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn shared_vertex_vertex_overlap() {
        // Test intersection between vertex within triangle for two triangles sharing a vertex
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![1.0, 0.0, 0.0]; // shared vertex
        let b1 = nalgebra::vector![2.0, 1.0, 0.0];
        let b2 = nalgebra::vector![0.25, 0.25, 0.0]; // overlapping vertex

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    #[test]
    fn one_point_intersect() {
        // Test one point inside triangle area; test only this most general case, 
        // more points are guranteed to work based on former tests if this works
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![2.0, 0.0, 0.0];
        let a2 = nalgebra::vector![1.0, 1.0, 0.0];

        let b0 = nalgebra::vector![0.0, 1.5, 0.0];
        let b1 = nalgebra::vector![2.0, 1.5, 0.0];
        let b2 = nalgebra::vector![1.0, 0.5, 0.0]; // Point inside area of triangle a

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    #[test]
    fn triforce() {
        // Test intersections of an inscribed triangle (vertices of one triangle all touch edges of
        // the other): only vertex-edge intersects, no edge-edge intersects
        let a0 = nalgebra::vector![0.0, 0.0, 0.0]; // Exterior triangle
        let a1 = nalgebra::vector![1.5, 2.0, 0.0];
        let a2 = nalgebra::vector![3.0, 0.0, 0.0];

        let b0 = nalgebra::vector![1.5, 0.0, 0.0]; // Inscribed triangle
        let b1 = nalgebra::vector![0.75, 1.0, 0.0];
        let b2 = nalgebra::vector![2.25, 1.0, 0.0];

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn edge_only() {
        // Test intersections of two overlapping triangles with no interior vertices (only edge-edge intersections)
        let a0 = nalgebra::vector![0.0, 0.0, 0.0]; // Bottom triangle
        let a1 = nalgebra::vector![1.5, 2.0, 0.0];
        let a2 = nalgebra::vector![3.0, 0.0, 0.0];

        let b0 = nalgebra::vector![1.5, -0.5, 0.0]; // Top triangle
        let b1 = nalgebra::vector![0.0, 1.5, 0.0];
        let b2 = nalgebra::vector![3.0, 1.5, 0.0];

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn concentric() {
        // Test overlap of two triangles with no edge intersections at all
        let a0 = nalgebra::vector![0.0, 0.0, 0.0]; // Exterior triangle
        let a1 = nalgebra::vector![2.0, 0.0, 0.0];
        let a2 = nalgebra::vector![1.0, 2.0, 0.0];

        let b0 = nalgebra::vector![0.5, 0.5, 0.0]; // Nested triangle
        let b1 = nalgebra::vector![1.5, 0.5, 0.0];
        let b2 = nalgebra::vector![1.0, 1.5, 0.0];

        // A triangle that doesn't intersect but instead lies entirely within the area of a
        // coplanar triangle counts as a degenerate case and is treated as an intersection
        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn vertex_edge_intersect() {
        // Triangle point meets another triangle edge
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.5, 1.0, 0.0];

        let b0 = nalgebra::vector![0.0, 1.0, 0.0];
        let b1 = nalgebra::vector![1.0, 1.0, 0.0];
        let b2 = nalgebra::vector![0.5, 2.0, 0.0];

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }
    #[test]
    fn perfect_overlap() {
        // Two perfectly overlapping coplaanr triangles
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![0.0, 0.0, 0.0];
        let b1 = nalgebra::vector![1.0, 0.0, 0.0];
        let b2 = nalgebra::vector![0.0, 1.0, 0.0];

        assert!(test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    #[test]
    fn disjoint() {
        // Two non-verlapping coplanar triangles (no intersection)
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![1.0, 0.0, 0.0];
        let a2 = nalgebra::vector![0.0, 1.0, 0.0];

        let b0 = nalgebra::vector![2.0, 0.0, 0.0];
        let b1 = nalgebra::vector![3.0, 0.0, 0.0];
        let b2 = nalgebra::vector![2.0, 1.0, 0.0];

        assert!(!test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    #[test]
    fn square_face() {
        // Coplanar intersection test for a square face composed of two coplanar triangles
        let a0 = nalgebra::vector![0.0, 0.0, 0.0];
        let a1 = nalgebra::vector![2.0, 0.0, 0.0];
        let a2 = nalgebra::vector![2.0, 2.0, 0.0];

        let b0 = nalgebra::vector![0.0, 0.0, 0.0];
        let b1 = nalgebra::vector![2.0, 2.0, 0.0];
        let b2 = nalgebra::vector![0.0, 2.0, 0.0];
        assert!(!test_coplanar(a0, a1, a2, b0, b1, b2))
    }

    // Misc test TODO: consider moving to utils?
    #[test]
    fn sketchy_equals() {
        let x =  1.9999996; 
        let y = 1.9999998;
        let z = -0.00000013666705;
        assert!(equals(x, 2.0) && equals(y, 2.0) && equals(z, 0.0))
    }
}
