//! A Rust gpsctl: the same options as the C program.  Try `gpsctl --help`.
fn main() {
    let args: Vec<String> = std::env::args().collect();
    std::process::exit(gpsctl_rs::cli::run(&args));
}
