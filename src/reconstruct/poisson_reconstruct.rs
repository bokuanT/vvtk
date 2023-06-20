use nalgebra::{Point3, Vector3};
use poisson_reconstruction::{PoissonReconstruction, Real};
use ply_rs::{ply::Ply, ply::Property, writer::Writer};

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
    let input_file = "./Ply_bun/bunny.points.ply";
    let ply = Ply::<f64>::read_from_file(input_file).expect("Failed to read input PLY file");

    // Extract point data from the PLY file
    let vertex_property = ply
        .get_element::<[f64; 3]>("vertex")
        .expect("Missing vertex property");
    let points: Vec<Point3<Real>> = vertex_property
        .iter()
        .map(|[x, y, z]| Point3::new(*x as Real, *y as Real, *z as Real))
        .collect();

    // Calculate normals from the points
    let normals: Vec<Vector3<Real>> = estimate_normals(&points, 0.9);

    // Perform Poisson reconstruction
    let screening = 0.0;
    let density_estimation_depth = 4;
    let max_depth = 5;
    let max_relaxation_iters = 10;
    let poisson = PoissonReconstruction::from_points_and_normals(
        &points,
        &normals,
        screening,
        density_estimation_depth,
        max_depth,
        max_relaxation_iters,
    );
    let mesh_vertices = poisson.reconstruct_mesh();

    // Convert the reconstructed mesh vertices to PLY format
    let mut reconstructed_ply = Ply::<f64>::new();
    let vertex_property = Property::new::<[f64; 3]>("vertex");
    reconstructed_ply.add_element_property(vertex_property);
    for vertex in &mesh_vertices {
        reconstructed_ply.add_element(&[vertex.x as f64, vertex.y as f64, vertex.z as f64]);
    }

    // Write the reconstructed mesh to a new PLY file
    let output_file = "output.ply";
    reconstructed_ply
        .write_to_file(output_file)
        .expect("Failed to write output PLY file");
}
