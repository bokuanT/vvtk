use std::path::Path;

use ply_rs::ply::Property;

use ply_rs::ply::Header;

use crate::formats::{pointxyzrgba::PointXyzRgba, PointCloud};

pub fn read_ply_header<P: AsRef<Path>>(path_buf: P) -> Result<Header, String> {
    let vertex_parser = ply_rs::parser::Parser::<PointXyzRgba>::new();
    let f = std::fs::File::open(path_buf.as_ref())
        .expect(&format!("Unable to open file {:?}", path_buf.as_ref()));
    let mut f = std::io::BufReader::new(f);

    let header = vertex_parser.read_header(&mut f).expect(&format!(
        "Failed to read header for ply file {:?}",
        path_buf.as_ref()
    ));

    Ok(header)
}

pub fn read_ply<P: AsRef<Path>>(path_buf: P) -> Option<PointCloud<PointXyzRgba>> {
    let vertex_parser = ply_rs::parser::Parser::<PointXyzRgba>::new();
    let f = std::fs::File::open(path_buf.as_ref())
        .expect(&format!("Unable to open file {:?}", path_buf.as_ref()));
    let mut f = std::io::BufReader::new(f);

    let header = vertex_parser.read_header(&mut f).expect(&format!(
        "Failed to read header for ply file {:?}",
        path_buf.as_ref()
    ));

    let mut vertex_list = Vec::new();
    for (_, element) in &header.elements {
        if element.name.as_str() == "vertex" {
            vertex_list = match vertex_parser.read_payload_for_element(&mut f, element, &header) {
                Ok(v) => v,
                Err(e) => {
                    println!("Failed to convert {:?}\n{e}", path_buf.as_ref());
                    return None;
                }
            }
        }
    }

    Some(PointCloud {
        number_of_points: vertex_list.len(),
        points: vertex_list,
    })
}

impl ply_rs::ply::PropertyAccess for PointXyzRgba {
    fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            r: 0,
            g: 0,
            b: 0,
            a: 255,
            nx: 0.0,
            ny: 0.0,
            nz: 0.0,
        }
    }

    fn set_property(&mut self, key: &String, property: Property) {
        match (key.as_ref(), property) {
            ("x", Property::Double(v)) => self.x = v as f32,
            ("y", Property::Double(v)) => self.y = v as f32,
            ("z", Property::Double(v)) => self.z = v as f32,
            ("x", Property::Float(v)) => self.x = v,
            ("y", Property::Float(v)) => self.y = v,
            ("z", Property::Float(v)) => self.z = v,
            ("red", Property::UChar(v)) => self.r = v,
            ("green", Property::UChar(v)) => self.g = v,
            ("blue", Property::UChar(v)) => self.b = v,
            ("alpha", Property::UChar(v)) => self.a = v,
            ("nx", Property::Double(v)) => self.nx = v as f32,
            ("ny", Property::Double(v)) => self.ny = v as f32,
            ("nz", Property::Double(v)) => self.nz = v as f32,
            ("nx", Property::Float(v)) => self.nx = v,
            ("ny", Property::Float(v)) => self.ny = v,
            ("nz", Property::Float(v)) => self.nz = v,
            _ => {}
        }
    }
}
