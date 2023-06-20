use nalgebra::{Point3, Vector3};
use poisson_reconstruction::{PoissonReconstruction, Real};
use ply_rs as ply;
use ply_rs::ply::Property;
use linked_hash_map::LinkedHashMap;

fn convert_payload(payload: &LinkedHashMap<String, Vec<LinkedHashMap<String, Property>>>) -> Vec<Point3<Real>> {
    let mut points = Vec::new();

    for entry in payload.values() {
        for item in entry {
            let x = match &item["x"] {
                Property::Float(value) => *value as Real,
                _ => 0.0,
            };

            let y = match &item["y"] {
                Property::Float(value) => *value as Real,
                _ => 0.0,
            };

            let z = match &item["z"] {
                Property::Float(value) => *value as Real,
                _ => 0.0,
            };

            points.push(Point3::new(x, y, z));
        }
    }

    points
}


fn estimate_normals(points: &[Point3<Real>], radius: Real) -> Vec<Vector3<Real>> {
    let num_points = points.len();
    let mut normals = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let mut normal = Vector3::zeros();

        for j in 0..num_points {
            if i != j {
                let dist = (points[i] - points[j]).norm();

                if dist <= radius {
                    normal += (points[i] - points[j]).normalize();
                }
            }
        }

        normals.push(normal.normalize());
    }

    normals
}

fn main() {
    // Read the input PLY file
    let path = "./Ply/longdress_vox10_1051.ply";
    let mut f = std::fs::File::open(path).unwrap();

    // create a parser
    let p = ply::parser::Parser::<ply::ply::DefaultElement>::new();

    // use the parser: read the entire file
    let ply = p.read_ply(&mut f);

    // make sure it did work
    assert!(ply.is_ok());
    let ply = ply.unwrap();

    let points = convert_payload(&ply.payload);
    
    let normals = estimate_normals(&points, 0.4);
    println!("here");
    let poisson = PoissonReconstruction::from_points_and_normals(
        points.as_slice(), normals.as_slice(), 0.0, 4, 5, 10
    );
    println!("here1");
    let mesh_vertices = poisson.reconstruct_mesh();
    println!("here2");
    3;

    // // Extract point data from the PLY file
    // let vertex_property = ply
    //     .get_element::<[f64; 3]>("vertex")
    //     .expect("Missing vertex property");
    // let points: Vec<Point3<Real>> = vertex_property
    //     .iter()
    //     .map(|[x, y, z]| Point3::new(*x as Real, *y as Real, *z as Real))
    //     .collect();

    // // Calculate normals from the points
    // let normals: Vec<Vector3<Real>> = estimate_normals(&points, 0.9);

    // // Perform Poisson reconstruction
    // let screening = 0.0;
    // let density_estimation_depth = 4;
    // let max_depth = 5;
    // let max_relaxation_iters = 10;
    // let poisson = PoissonReconstruction::from_points_and_normals(
    //     &points,
    //     &normals,
    //     screening,
    //     density_estimation_depth,
    //     max_depth,
    //     max_relaxation_iters,
    // );
    // let mesh_vertices = poisson.reconstruct_mesh();

    // // Convert the reconstructed mesh vertices to PLY format
    // let mut reconstructed_ply = Ply::<f64>::new();
    // let vertex_property = Property::new::<[f64; 3]>("vertex");
    // reconstructed_ply.add_element_property(vertex_property);
    // for vertex in &mesh_vertices {
    //     reconstructed_ply.add_element(&[vertex.x as f64, vertex.y as f64, vertex.z as f64]);
    // }

    // // Write the reconstructed mesh to a new PLY file
    // let output_file = "output.ply";
    // reconstructed_ply
    //     .write_to_file(output_file)
    //     .expect("Failed to write output PLY file");
}
