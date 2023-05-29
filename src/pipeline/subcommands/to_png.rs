use crate::pipeline::channel::Channel;
use crate::pipeline::PipelineMessage;
use crate::render::wgpu::png::PngWriter;
use clap::Parser;
use std::ffi::OsString;
use std::path::Path;
use super::Subcommand;

use crate::formats::pointxyzrgba::PointXyzRgba;
use crate::formats::PointCloud;
use crate::render::wgpu::renderer::PointCloudRenderer;
use std::num::NonZeroU32;
use std::sync::mpsc::Sender;
use wgpu::{Buffer, Device, Queue, Texture, TextureDescriptor, TextureView};
use winit::dpi::PhysicalSize;


/// Converts a folder of .pcd files to a folder of .png images
#[derive(Parser)]
struct Args {
    /// Directory to store output png images
    #[clap(short, long)]
    output_dir: OsString,
    #[clap(short = 'x', long, default_value_t = 0.0)]
    camera_x: f32,
    #[clap(short = 'y', long, default_value_t = 0.0)]
    camera_y: f32,
    #[clap(short = 'z', long, default_value_t = 1.3)]
    camera_z: f32,
    #[clap(long = "yaw", default_value_t = -90.0, allow_hyphen_values = true)]
    camera_yaw: f32,
    #[clap(long = "pitch", default_value_t = 0.0)]
    camera_pitch: f32,
    #[clap(long, default_value_t = 1600)]
    width: u32,
    #[clap(long, default_value_t = 900)]
    height: u32,
}

pub struct ToPng<'a> {
    writer: PngWriter<'a>,
}

impl<'a> ToPng<'a> {
    pub fn from_args(args: Vec<String>) -> Box<dyn Subcommand> {
        let Args {
            output_dir,
            camera_x,
            camera_y,
            camera_z,
            camera_yaw,
            camera_pitch,
            width,
            height,
        }: Args = Args::parse_from(args);

        Box::from(ToPng {
            writer: PngWriter::new(
                output_dir,
                camera_x,
                camera_y,
                camera_z,
                camera_yaw,
                camera_pitch,
                width,
                height,
            ),
        })
    }
}

impl Subcommand for ToPng<'_> {
    fn handle(&mut self, messages: Vec<PipelineMessage>, channel: &Channel) {
        for message in messages {
            match &message {
                PipelineMessage::PointCloud(pc) => {
                    self.writer.write_to_png(pc);
                }
                _ => {}
            }
            channel.send(message);
        }
    }
}


// pub fn pc_to_png(to_png: &mut ToPng, pc: PointCloud<PointXyzRgba>, filename: &str) {
//     if to_png.point_renderer.is_none() {
//         to_png.point_renderer = Some(PointCloudRenderer::new(
//             &to_png.device,
//             to_png.texture_desc.format,
//             &pc,
//             to_png.size,
//             &to_png.camera_state,
//         ));
//     }
//     let point_renderer = to_png.point_renderer.as_mut().unwrap();
//     point_renderer.update_vertices(&to_png.device, &to_png.queue, &pc);
//     let mut encoder = to_png
//         .device
//         .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });

//     point_renderer.render(&mut encoder, &to_png.texture_view);
//     encoder.copy_texture_to_buffer(
//         wgpu::ImageCopyTexture {
//             aspect: wgpu::TextureAspect::All,
//             texture: &to_png.texture,
//             mip_level: 0,
//             origin: wgpu::Origin3d::ZERO,
//         },
//         wgpu::ImageCopyBuffer {
//             buffer: &to_png.output_buffer,
//             layout: wgpu::ImageDataLayout {
//                 offset: 0,
//                 bytes_per_row: NonZeroU32::new(to_png.u32_size * to_png.size.width),
//                 rows_per_image: NonZeroU32::new(to_png.size.height),
//             },
//         },
//         to_png.texture_desc.size,
//     );

//     to_png.queue.submit(Some(encoder.finish()));
//     {
//         let buffer_slice = to_png.output_buffer.slice(..);
//         buffer_slice.map_async(wgpu::MapMode::Read, |_| {});
//         to_png.device.poll(wgpu::Maintain::Wait);

//         let data = buffer_slice.get_mapped_range();

//         use image::{ImageBuffer, Rgba};
//         let buffer = ImageBuffer::<Rgba<u8>, _>::from_raw(
//             to_png.size.width,
//             to_png.size.height,
//             data,
//         )
//         .unwrap();

//         let filename = format!("{}.png", filename);
//         to_png.count += 1;
//         let output_path = Path::new(&to_png.output_dir);
//         buffer.save(output_path.join(Path::new(&filename))).unwrap();
//     }
//     to_png.output_buffer.unmap();

// }