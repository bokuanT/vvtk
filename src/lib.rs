//! # Vivo Toolkit
//#[warn(missing_docs)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

pub mod codec;
#[cfg(feature = "dash")]
pub mod dash;
pub mod downsample;
pub mod formats;
pub mod metrics;
pub mod pcd;
pub mod pipeline;
pub mod ply;
pub mod reconstruct;
pub mod render;
pub mod upsample;
pub mod utils;
pub mod velodyne;

#[test]
pub fn test_hello() {
    println!("printing: {}", concat!(env!("OUT_DIR")));
    unsafe {
        println!("test_hello is {}", hello());
    }
}