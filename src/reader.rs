use crate::prelude::*;
use crate::Data;
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

// Given a .obj file, parse into Data struct

fn get_file(path: String) -> BufReader<File> {
    println!("{path}");
    let file_path = Path::new(&path);
    println!("{:?}", file_path);
    let file = File::open(file_path).expect("file wasn't found.");
    BufReader::new(file)
}

// Parse the obj file and partition vertices and faces into separate Vectors
fn read_file(lines: Vec<String>) -> (Vec<[f32; 3]>, Vec<[usize; 3]>) {
    let mut v: Vec<[f32; 3]> = Vec::new();
    let mut f: Vec<[usize; 3]> = Vec::new();
    // let (v, f): (Vec<&String>, Vec<&String>) = lines
    //     .iter()
    //     .partition(|&l| l.split_whitespace().next().unwrap() == "v");

    for line in lines.iter() {
        // println!("{}", line);
        let l: Vec<String> = line.split_whitespace().map(|s| s.to_string()).collect();

        match l {
            _ if l.starts_with(&["v".to_string()]) => {
                let i: f32 = l[1]
                    .parse()
                    .expect("failed to parse face index i ... implement proper error handling");
                let j: f32 = l[2]
                    .parse()
                    .expect("failed to parse face index j ... implement proper error handling");
                let k: f32 = l[3]
                    .parse()
                    .expect("failed to parse face index k ... implement proper error handling");
                v.push([i, j, k]);
            }
            _ if l.starts_with(&["f".to_string()]) => {
                let i: usize;
                let j: usize;
                let k: usize;
                // .obj format allows for texture and vector normal coordinates in face data,
                // deliniated by '/'. If any face index uses '//, all face data must use '/'.
                // Thus, the first element of a split('/') will always be the face index, ignore
                // the rest.
                if l[1].contains('/') {
                    println!("Encoded attribute data detected");
                    i = l[1].split('/').collect::<Vec<&str>>()[0].parse().expect(
                        "failed to parse vertice index i ... implement proper error handling",
                    );
                    j = l[2].split('/').collect::<Vec<&str>>()[0].parse().expect(
                        "failed to parse vertice index i ... implement proper error handling",
                    );
                    k = l[3].split('/').collect::<Vec<&str>>()[0].parse().expect(
                        "failed to parse vertice index i ... implement proper error handling",
                    );
                } else {
                    i = l[1].parse().expect(
                        "failed to parse vertice index i ... implement proper error handling",
                    );
                    j = l[2].parse().expect(
                        "failed to parse vertice index j ... implement proper error handling",
                    );
                    k = l[3].parse().expect(
                        "failed to parse vertice index k ... implement proper error handling",
                    );
                }
                f.push([i, j, k]);
            }
            // bypass any line that does not record a vertex or a face
            _ => (),
        }
    }
    (v, f)
}

// Given an .obj file, extract vertex and face data and load into internal data structure
// Return Data struct
pub fn build_data_from_obj(path: String) -> Result<Data, Box<dyn std::error::Error>> {
    let input: Result<Vec<String>, _> = get_file(path).lines().collect();
    let (v, f) = read_file(input?);
    let data: Data = crate::data::Data::build(v, f)?;
    return Ok(data);
}

// // Convert primitives to nalgebra types here for later computations
// pub fn build(v: Vec<[f32; 3]>, f: Vec<[usize; 3]>) -> Result<Data, Box<dyn std::error::Error>> {
//     let vertices: Vec<Point> = v.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
//     let faces: Vec<Index> = f.iter().map(|k| Vec3::new(k[0], k[1], k[2])).collect();
//
//     // Vector 'faces' points to the 3 vertices that form a triangular mesh by storing 3 indices to Vector 'vertices'
//     // To improve later ease of use, 'triangles' directly stores the 3 points of the triangular mesh
//     let mut triangles: Vec<Face> = Vec::new();
//     for face in faces.iter() {
//         triangles.push(Vec3::new(
//             vertices[face[0] - 1],
//             vertices[face[1] - 1],
//             vertices[face[2] - 1],
//         ))
//     }
//     // We keep 'vertices' and 'faces' for ease of validating uniqueness and boundedness...
//     // even though 'triangles' contains all requisite data
//     Ok(Data::new(vertices, faces, triangles))
// }
