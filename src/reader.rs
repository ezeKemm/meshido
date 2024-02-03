use crate::prelude::*;
use crate::Data;
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

// Given a .obj file, parse into Data struct

fn get_file(path: String) -> BufReader<File> {
    let file_path = Path::new(&path);
    let file = File::open(file_path).expect("file wasn't found.");
    BufReader::new(file)
}

// Parse the obj file and partition vertices and faces into separate Vectors
fn gather(lines: Vec<String>) -> (Vec<[f32; 3]>, Vec<[usize; 3]>) {
    let mut v: Vec<[f32; 3]> = Vec::new();
    let mut f: Vec<[usize; 3]> = Vec::new();
    // let (v, f): (Vec<String>, Vec<String>) = lines
    //     .iter()
    //     .partition(|&l| l.split_whitespace().next().unwrap() == "v");
    for line in lines.iter() {
        // bypass any line that does not record a vertex or a face
        if !(line.starts_with("v") || line.starts_with("f")) {
            continue;
        }
        let v_check = String::from("v");
        let f_check = String::from("f");
        let l: Vec<String> = line.split_whitespace().map(|s| s.to_string()).collect();
        match &l[0] {
            v_check => {
                let i: f32 = l[1]
                    .parse()
                    .expect("failed to parse vertice index i ... implement proper error handling");
                let j: f32 = l[2]
                    .parse()
                    .expect("failed to parse vertice index j ... implement proper error handling");
                let k: f32 = l[3]
                    .parse()
                    .expect("failed to parse vertice index k ... implement proper error handling");
                v.push([i, j, k]);
            }
            f_check => {
                let i: usize = l[1]
                    .parse()
                    .expect("failed to parse face index i ... implement proper error handling");
                let j: usize = l[2]
                    .parse()
                    .expect("failed to parse face index j ... implement proper error handling");
                let k: usize = l[3]
                    .parse()
                    .expect("failed to parse face index k ... implement proper error handling");
                f.push([i, j, k]);
            }
            _ => {
                println!("{}", l.join(" "));
            }
        }
    }
    (v, f)
}

// Convert primitives to nalgebra types here for later computations
pub fn build_data(path: String) -> Result<Data, Box<dyn std::error::Error>> {
    let input: Result<Vec<String>, _> = get_file(path).lines().collect();
    let (v, f) = gather(input?);
    let vertices: Vec<Point> = v.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
    let faces: Vec<Index> = f.iter().map(|k| Vec3::new(k[0], k[1], k[2])).collect();

    // Vector 'faces' points to the 3 vertices that form a triangular mesh by storing 3 indices to Vector 'vertices'
    // To improve later ease of use, 'triangles' directly stores the 3 points of the triangular mesh
    let mut triangles: Vec<Face> = Vec::new();
    for face in faces.iter() {
        triangles.push(Vec3::new(
            vertices[face[0] - 1],
            vertices[face[1] - 1],
            vertices[face[2] - 1],
        ))
    }
    // We keep 'vertices' and 'faces' for ease of validating uniqueness and boundedness...
    // even though 'triangles' contains all requisite data
    Ok(Data::new(vertices, faces, triangles))
}
