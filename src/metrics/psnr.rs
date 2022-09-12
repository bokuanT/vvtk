use kiddo::{distance::squared_euclidean, KdTree};
use rayon::prelude::*;

use crate::formats::pointxyzrgba::PointXyzRgba;

const RESULTS: usize = 30;
const RESOLUTION: f64 = 1023f64;

fn get_psnr(dist: f64, p: f64, factor: f64) -> f64 {
    let max_energy = p * p;
    10f64 * ((factor * max_energy) / dist).log(10f64)
}
pub struct Psnr;

impl Psnr {
    pub fn calculate_metric(
        original: &Vec<PointXyzRgba>,
        original_tree: &KdTree<f32, usize, 3>,
        reconstructed: &Vec<PointXyzRgba>,
        reconstructed_tree: &KdTree<f32, usize, 3>,
    ) -> (f64, f64) {
        let time = std::time::Instant::now();
        let drms: f32 = original
            .par_iter()
            .map(|pt| {
                let nearest_points = reconstructed_tree
                    .nearest(&[pt.x, pt.y, pt.z], RESULTS, &squared_euclidean)
                    .unwrap();
                let (dist, _) = nearest_points[0];
                dist
            })
            .sum();

        eprintln!("{:?}", std::time::Instant::now() - time);
        let n = original.len() as f64;
        let drms = drms as f64 / n;
        let psnr_drms = get_psnr(drms, RESOLUTION, 3.0);
        (drms, psnr_drms)
    }
}
