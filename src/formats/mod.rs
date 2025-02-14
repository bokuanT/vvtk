use std::fmt::Debug;

use crate::pcd::PointCloudData;
use crate::velodyne::{VelodynPoint, VelodyneBinData};

pub mod pointxyzrgba;

pub mod triangle_face;

#[derive(Clone)]
pub struct PointCloud<T> {
    pub number_of_points: usize,
    pub points: Vec<T>,
}

impl Debug for PointCloud<pointxyzrgba::PointXyzRgba> {
    // first print the number of points in one line
    // then for each T in the Vec, print in a new line
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "PointCloud<PointXyzRgba> {{")?;
        writeln!(f, "   number_of_points: {}", self.number_of_points)?;
        for point in &self.points {
            writeln!(f, "   {:?}", point)?;
        }
        writeln!(f, "}}")?;
        Ok(())
    }
}

impl<T> From<PointCloudData> for PointCloud<T> {
    fn from(pcd: PointCloudData) -> Self {
        let number_of_points = pcd.header.points() as usize;

        let mut v_clone = std::mem::ManuallyDrop::new(pcd.data);
        let points = unsafe {
            let factor = v_clone.len() / number_of_points;
            let capacity = v_clone.capacity() / factor;
            Vec::from_raw_parts(v_clone.as_mut_ptr() as *mut T, number_of_points, capacity)
        };
        Self {
            number_of_points,
            points,
        }
    }
}

impl From<VelodyneBinData> for PointCloud<pointxyzrgba::PointXyzRgba> {
    // type T: pointxyzrgba::PointXyzRgba;
    fn from(value: VelodyneBinData) -> Self {
        let number_of_points = value.data.len();
        let points = value.data.into_iter().map(|point| point.into()).collect();
        Self {
            number_of_points,
            points,
        }
    }
}

impl From<VelodynPoint> for pointxyzrgba::PointXyzRgba {
    fn from(value: VelodynPoint) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
            nx: 0.0,
            ny: 0.0,
            nz: 0.0,
            r: (value.intensity * 255.0) as u8,
            g: (value.intensity * 255.0) as u8,
            b: (value.intensity * 255.0) as u8,
            a: 255,
        }
    }
}
