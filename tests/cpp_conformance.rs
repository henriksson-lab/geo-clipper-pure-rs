use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use clipper_rust::{
    ClipType, Clipper, ClipperOffset, EndType, IntPoint, JoinType, Path as ClipperPath, Paths,
    PolyFillType, PolyType, area,
};

fn total_abs_area(paths: &Paths) -> f64 {
    paths.iter().map(|path| area(path).abs()).sum()
}

fn oracle_source() -> &'static str {
    r#"
#include <cmath>
#include <iostream>
#include "clipper.hpp"

using namespace ClipperLib;

static double total_abs_area(const Paths& paths) {
  double result = 0.0;
  for (Paths::size_type i = 0; i < paths.size(); ++i) {
    result += std::fabs(Area(paths[i]));
  }
  return result;
}

int main() {
  Paths solution;

  Clipper union_clipper;
  Path a;
  a << IntPoint(0, 0) << IntPoint(10, 0) << IntPoint(10, 10) << IntPoint(0, 10);
  Path b;
  b << IntPoint(5, 5) << IntPoint(15, 5) << IntPoint(15, 15) << IntPoint(5, 15);
  union_clipper.AddPath(a, ptSubject, true);
  union_clipper.AddPath(b, ptSubject, true);
  bool union_ok = union_clipper.Execute(ctUnion, solution, pftNonZero);
  std::cout << "union " << union_ok << " " << solution.size() << " " << total_abs_area(solution) << "\n";

  solution.clear();
  ClipperOffset offset;
  Path square;
  square << IntPoint(0, 0) << IntPoint(10, 0) << IntPoint(10, 10) << IntPoint(0, 10);
  offset.AddPath(square, jtMiter, etClosedPolygon);
  offset.Execute(solution, 1.0);
  std::cout << "offset " << solution.size() << " " << total_abs_area(solution) << "\n";

  solution.clear();
  ClipperOffset offset_negative;
  offset_negative.AddPath(square, jtMiter, etClosedPolygon);
  offset_negative.Execute(solution, -1.0);
  std::cout << "offset_negative " << solution.size() << " " << total_abs_area(solution) << "\n";

  return 0;
}
"#
}

fn build_cpp_oracle(manifest_dir: &Path) -> Option<PathBuf> {
    let repo_root = manifest_dir;
    let out_dir = manifest_dir.join("target").join("cpp-conformance");
    fs::create_dir_all(&out_dir).ok()?;
    let source = out_dir.join("oracle.cpp");
    let binary = out_dir.join("oracle");
    fs::write(&source, oracle_source()).ok()?;

    let status = Command::new("g++")
        .arg("-std=c++11")
        .arg(&source)
        .arg(repo_root.join("Clipper/cpp/clipper.cpp"))
        .arg("-I")
        .arg(repo_root.join("Clipper/cpp"))
        .arg("-o")
        .arg(&binary)
        .status()
        .ok()?;

    if status.success() { Some(binary) } else { None }
}

#[test]
fn rust_matches_cpp_for_basic_union_and_offset_cases() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let Some(binary) = build_cpp_oracle(&manifest_dir) else {
        eprintln!("skipping C++ conformance test because oracle build failed");
        return;
    };

    let output = Command::new(binary).output().expect("run C++ oracle");
    assert!(output.status.success());
    let stdout = String::from_utf8(output.stdout).expect("oracle stdout is utf8");
    let mut lines = stdout.lines();

    let union_line = lines.next().expect("union oracle line");
    let union_parts: Vec<&str> = union_line.split_whitespace().collect();
    assert_eq!(union_parts[0], "union");
    assert_eq!(union_parts[1], "1");
    let cpp_union_count: usize = union_parts[2].parse().unwrap();
    let cpp_union_area: f64 = union_parts[3].parse().unwrap();

    let offset_line = lines.next().expect("offset oracle line");
    let offset_parts: Vec<&str> = offset_line.split_whitespace().collect();
    assert_eq!(offset_parts[0], "offset");
    let cpp_offset_count: usize = offset_parts[1].parse().unwrap();
    let cpp_offset_area: f64 = offset_parts[2].parse().unwrap();

    let offset_negative_line = lines.next().expect("negative offset oracle line");
    let offset_negative_parts: Vec<&str> = offset_negative_line.split_whitespace().collect();
    assert_eq!(offset_negative_parts[0], "offset_negative");
    let cpp_offset_negative_count: usize = offset_negative_parts[1].parse().unwrap();
    let cpp_offset_negative_area: f64 = offset_negative_parts[2].parse().unwrap();

    let mut rust_union = Clipper::new();
    rust_union
        .add_path(
            &[
                IntPoint::new(0, 0),
                IntPoint::new(10, 0),
                IntPoint::new(10, 10),
                IntPoint::new(0, 10),
            ],
            PolyType::Subject,
            true,
        )
        .unwrap();
    rust_union
        .add_path(
            &[
                IntPoint::new(5, 5),
                IntPoint::new(15, 5),
                IntPoint::new(15, 15),
                IntPoint::new(5, 15),
            ],
            PolyType::Subject,
            true,
        )
        .unwrap();
    let mut rust_union_solution = Vec::new();
    rust_union
        .execute_into(
            ClipType::Union,
            &mut rust_union_solution,
            PolyFillType::NonZero,
        )
        .unwrap();

    let mut rust_offset = ClipperOffset::new(2.0, 0.25);
    let square: ClipperPath = vec![
        IntPoint::new(0, 0),
        IntPoint::new(10, 0),
        IntPoint::new(10, 10),
        IntPoint::new(0, 10),
    ];
    rust_offset.add_path(&square, JoinType::Miter, EndType::ClosedPolygon);
    let rust_offset_solution = rust_offset.execute(1.0).unwrap();

    let mut rust_offset_negative = ClipperOffset::new(2.0, 0.25);
    rust_offset_negative.add_path(&square, JoinType::Miter, EndType::ClosedPolygon);
    let rust_offset_negative_solution = rust_offset_negative.execute(-1.0).unwrap();

    assert_eq!(rust_union_solution.len(), cpp_union_count);
    assert_eq!(total_abs_area(&rust_union_solution), cpp_union_area);
    assert_eq!(rust_offset_solution.len(), cpp_offset_count);
    assert_eq!(total_abs_area(&rust_offset_solution), cpp_offset_area);
    assert_eq!(
        rust_offset_negative_solution.len(),
        cpp_offset_negative_count
    );
    assert_eq!(
        total_abs_area(&rust_offset_negative_solution),
        cpp_offset_negative_area
    );
}
