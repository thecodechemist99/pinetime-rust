//! This build script copies the `memory.x` file from the crate root into a directory where
//! the linker can always find it at build time.

use chrono;
use std::{
  env,
  fs::File,
  io::Write,
  path::PathBuf,
};

fn main() {
    // Put memory layout in the output directory and ensure it's on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // create rs file with current UTC time
    File::create(out.join("utc.rs"))
      .unwrap()
      .write_fmt(format_args!("const UTC_TIME: i64 = {:?};", chrono::offset::Local::now().timestamp()))
      .unwrap();

    // By default, Cargo will re-run a build script whenever any file in the project changes.
    // Specify `memory.x` here to ensure it is only re-run when `memory.x` is changed.
    // println!("cargo:rerun-if-changed=memory.x");
}