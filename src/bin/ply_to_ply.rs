extern crate iswr;
use clap::{App, Arg};

fn main() -> std::io::Result<()> {
    let matches = App::new("ply_to_ply")
        .about("Write data to ply file in ascii form")
        .arg(
            Arg::with_name("input")
                .short("i")
                .long("input")
                .takes_value(true)
                .multiple(false)
                .help("File directory for input"),
        )
        .arg(
            Arg::with_name("form")
                .short("f")
                .long("form")
                .takes_value(true)
                .multiple(false)
                .help("Form of output (ascii/binary)"),
        )
        .arg(
            Arg::with_name("output")
                .short("o")
                .long("output")
                .takes_value(true)
                .multiple(false)
                .help("File directory for output"),
        )
        .get_matches();

    let input = matches.value_of("input");
    let form = matches.value_of("form");
    let output = matches.value_of("output");

    iswr::tool::reader::read(input).write(form, output)
}
