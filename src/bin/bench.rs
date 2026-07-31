use std::env;
use std::time::Instant;

use geo_clipper_pure_rs::{
    ClipType, Clipper, ClipperOffset, ClipperOptions, EndType, IntPoint, JoinType, Path, Paths,
    PolyFillType, PolyType, area, closed_paths_from_poly_tree_into, open_paths_from_poly_tree_into,
};

#[derive(Clone, Copy)]
struct Summary {
    ok: bool,
    paths: usize,
    points: usize,
    area_abs: f64,
    area_signed: f64,
    checksum: i128,
}

fn summarize(ok: bool, paths: &Paths) -> Summary {
    let mut points = 0;
    let mut area_abs = 0.0;
    let mut area_signed = 0.0;
    let mut checksum = 0_i128;

    for path in paths {
        let path_area = area(path);
        area_abs += path_area.abs();
        area_signed += path_area;
        points += path.len();

        for (idx, pt) in path.iter().enumerate() {
            let weight = idx as i128 + 1;
            checksum += weight * pt.x as i128;
            checksum += (weight + 17) * pt.y as i128;
            checksum += (pt.x as i128) * (pt.x as i128 + 31);
            checksum -= (pt.y as i128) * (pt.y as i128 - 17);
        }
    }

    Summary {
        ok,
        paths: paths.len(),
        points,
        area_abs,
        area_signed,
        checksum,
    }
}

fn dump_paths(paths: &Paths) {
    let mut normalized: Vec<String> = paths
        .iter()
        .map(|path| {
            path.iter()
                .map(|pt| format!("{},{}", pt.x, pt.y))
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect();
    normalized.sort();
    for path in normalized {
        println!("path {}", path);
    }
}

fn rect(left: i64, top: i64, right: i64, bottom: i64) -> Path {
    vec![
        IntPoint::new(left, top),
        IntPoint::new(right, top),
        IntPoint::new(right, bottom),
        IntPoint::new(left, bottom),
    ]
    .into()
}

fn rect_grid(cols: i64, rows: i64, step: i64, size: i64, xoff: i64, yoff: i64) -> Paths {
    let mut paths = Paths::with_capacity((cols * rows) as usize);
    for y in 0..rows {
        for x in 0..cols {
            let left = xoff + x * step;
            let top = yoff + y * step;
            paths.push(rect(left, top, left + size, top + size));
        }
    }
    paths
}

fn vertical_strips(count: i64, width: i64, height: i64, gap: i64) -> Paths {
    let mut paths = Paths::with_capacity(count as usize);
    for i in 0..count {
        let left = i * gap;
        paths.push(rect(left, 0, left + width, height));
    }
    paths
}

fn horizontal_strips(count: i64, width: i64, height: i64, gap: i64) -> Paths {
    let mut paths = Paths::with_capacity(count as usize);
    for i in 0..count {
        let top = i * gap;
        paths.push(rect(0, top, width, top + height));
    }
    paths
}

fn star(cx: i64, cy: i64, outer: i64, inner: i64, vertices: usize) -> Path {
    let mut path = Path::with_capacity(vertices);
    for i in 0..vertices {
        let angle = (i as f64) * std::f64::consts::TAU / vertices as f64;
        let radius = if i % 2 == 0 { outer } else { inner } as f64;
        path.push(IntPoint::new(
            cx + (radius * angle.cos()).round() as i64,
            cy + (radius * angle.sin()).round() as i64,
        ));
    }
    path
}

fn star_grid(cols: i64, rows: i64, step: i64, vertices: usize) -> Paths {
    let mut paths = Paths::with_capacity((cols * rows) as usize);
    for y in 0..rows {
        for x in 0..cols {
            paths.push(star(x * step, y * step, 34, 17, vertices));
        }
    }
    paths
}

fn jitter(value: i64) -> i64 {
    let mut x = value as u64;
    x = x
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((x >> 61) as i64) - 3
}

fn jittered_rect_grid(cols: i64, rows: i64, step: i64, size: i64) -> Paths {
    let mut paths = Paths::with_capacity((cols * rows) as usize);
    for y in 0..rows {
        for x in 0..cols {
            let seed = y * cols + x;
            let left = x * step + jitter(seed);
            let top = y * step + jitter(seed + 17);
            paths.push(vec![
                IntPoint::new(left, top),
                IntPoint::new(left + size + jitter(seed + 31), top + jitter(seed + 43)),
                IntPoint::new(
                    left + size + jitter(seed + 47),
                    top + size + jitter(seed + 59),
                ),
                IntPoint::new(left + jitter(seed + 71), top + size + jitter(seed + 83)),
            ]);
        }
    }
    paths
}

fn open_diagonals(count: i64, span: i64, step: i64) -> Paths {
    let mut paths = Paths::with_capacity((count * 2) as usize);
    for i in 0..count {
        let offset = i * step;
        paths.push(vec![
            IntPoint::new(0, offset),
            IntPoint::new(span / 2, offset + span / 3),
            IntPoint::new(span, offset + span),
        ]);
        paths.push(vec![
            IntPoint::new(offset, 0),
            IntPoint::new(offset + span / 3, span / 2),
            IntPoint::new(offset + span, span),
        ]);
    }
    paths
}

fn run_union_dense() -> Summary {
    let mut clipper = Clipper::new();
    let subjects = rect_grid(60, 60, 10, 18, 0, 0);
    clipper
        .add_paths(&subjects, PolyType::Subject, true)
        .unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Union, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn run_touching_rect_grid() -> Summary {
    let mut clipper = Clipper::new();
    let subjects = rect_grid(90, 90, 10, 10, 0, 0);
    clipper
        .add_paths(&subjects, PolyType::Subject, true)
        .unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Union, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn jittered_sliver_union_sized(size: i64) -> (bool, Paths) {
    let mut clipper = Clipper::new();
    let subjects = jittered_rect_grid(size, size, 13, 16);
    clipper
        .add_paths(&subjects, PolyType::Subject, true)
        .unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Union, &mut solution, PolyFillType::NonZero)
        .unwrap();
    (true, solution)
}

fn run_jittered_sliver_union_sized(size: i64) -> Summary {
    let (ok, solution) = jittered_sliver_union_sized(size);
    summarize(ok, &solution)
}

fn run_jittered_sliver_union() -> Summary {
    run_jittered_sliver_union_sized(56)
}

fn run_intersection_grid() -> Summary {
    let mut clipper = Clipper::new();
    let subjects = vertical_strips(90, 9, 900, 10);
    let clips = horizontal_strips(90, 900, 9, 10);
    clipper
        .add_paths(&subjects, PolyType::Subject, true)
        .unwrap();
    clipper.add_paths(&clips, PolyType::Clip, true).unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Intersection, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn run_nested_holes() -> Summary {
    let mut clipper = Clipper::new();
    clipper
        .add_path(&rect(-2000, -2000, 2000, 2000), PolyType::Subject, true)
        .unwrap();
    let holes = rect_grid(32, 32, 115, 58, -1800, -1800);
    let islands = rect_grid(32, 32, 115, 24, -1783, -1783);
    clipper.add_paths(&holes, PolyType::Clip, true).unwrap();
    clipper
        .add_paths(&islands, PolyType::Subject, true)
        .unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Difference, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn run_difference_holes() -> Summary {
    let mut clipper = Clipper::new();
    clipper
        .add_path(&rect(-20, -20, 920, 920), PolyType::Subject, true)
        .unwrap();
    let clips = rect_grid(38, 38, 23, 11, 15, 15);
    clipper.add_paths(&clips, PolyType::Clip, true).unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Difference, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn run_strict_simple_stars() -> Summary {
    let mut clipper = Clipper::with_options(ClipperOptions::new().strictly_simple(true));
    let subjects = star_grid(22, 22, 38, 20);
    clipper
        .add_paths(&subjects, PolyType::Subject, true)
        .unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Union, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn run_open_paths_clip() -> Summary {
    let mut clipper = Clipper::new();
    let closed = Paths::from(vec![rect(80, 80, 920, 920)]);
    let open = open_diagonals(180, 1100, 5);
    clipper.add_paths(&closed, PolyType::Clip, true).unwrap();
    clipper.add_paths(&open, PolyType::Subject, false).unwrap();
    let polytree = clipper
        .execute_polytree(ClipType::Intersection, PolyFillType::NonZero)
        .unwrap();
    let ok = true;
    let mut solution = Paths::new();
    open_paths_from_poly_tree_into(&polytree, &mut solution);
    summarize(ok, &solution)
}

fn run_large_coord_xor() -> Summary {
    let origin = 1_200_000_000_i64;
    let mut clipper = Clipper::new();
    let subjects = rect_grid(34, 34, 30_000_000, 48_000_000, origin, origin);
    let clips = rect_grid(
        34,
        34,
        30_000_000,
        48_000_000,
        origin + 12_000_000,
        origin + 12_000_000,
    );
    clipper
        .add_paths(&subjects, PolyType::Subject, true)
        .unwrap();
    clipper.add_paths(&clips, PolyType::Clip, true).unwrap();
    let mut solution = Paths::new();
    clipper
        .execute_into(ClipType::Xor, &mut solution, PolyFillType::NonZero)
        .unwrap();
    summarize(true, &solution)
}

fn run_offset_stars() -> Summary {
    let mut offset = ClipperOffset::new(2.0, 0.25);
    let subjects = star_grid(26, 26, 90, 48);
    offset.add_paths(&subjects, JoinType::Round, EndType::ClosedPolygon);
    let solution = offset.execute(7.0).unwrap();
    summarize(true, &solution)
}

fn run_offset_open_round() -> Summary {
    let mut offset = ClipperOffset::new(2.0, 0.25);
    let subjects = open_diagonals(220, 500, 8);
    offset.add_paths(&subjects, JoinType::Round, EndType::OpenRound);
    let solution = offset.execute(9.0).unwrap();
    summarize(true, &solution)
}

fn run_polytree_closed_nested() -> Summary {
    let mut clipper = Clipper::new();
    clipper
        .add_path(&rect(-1500, -1500, 1500, 1500), PolyType::Subject, true)
        .unwrap();
    let holes = rect_grid(24, 24, 115, 52, -1300, -1300);
    clipper.add_paths(&holes, PolyType::Clip, true).unwrap();
    let polytree = clipper
        .execute_polytree(ClipType::Difference, PolyFillType::NonZero)
        .unwrap();
    let ok = true;
    let mut solution = Paths::new();
    closed_paths_from_poly_tree_into(&polytree, &mut solution);
    summarize(ok, &solution)
}

fn run_case(name: &str) -> Summary {
    if let Some(size) = name.strip_prefix("jittered_sliver_union_") {
        return run_jittered_sliver_union_sized(size.parse().expect("invalid jittered grid size"));
    }

    match name {
        "union_dense" => run_union_dense(),
        "touching_rect_grid" => run_touching_rect_grid(),
        "jittered_sliver_union" => run_jittered_sliver_union(),
        "intersection_grid" => run_intersection_grid(),
        "difference_holes" => run_difference_holes(),
        "nested_holes" => run_nested_holes(),
        "strict_simple_stars" => run_strict_simple_stars(),
        "open_paths_clip" => run_open_paths_clip(),
        "large_coord_xor" => run_large_coord_xor(),
        "offset_stars" => run_offset_stars(),
        "offset_open_round" => run_offset_open_round(),
        "polytree_closed_nested" => run_polytree_closed_nested(),
        _ => panic!("unknown benchmark case: {name}"),
    }
}

fn cases() -> &'static [&'static str] {
    &[
        "union_dense",
        "touching_rect_grid",
        "jittered_sliver_union",
        "intersection_grid",
        "difference_holes",
        "nested_holes",
        "strict_simple_stars",
        "open_paths_clip",
        "large_coord_xor",
        "offset_stars",
        "offset_open_round",
        "polytree_closed_nested",
    ]
}

fn main() {
    let mut args = env::args().skip(1);
    let selected = args.next().unwrap_or_else(|| "all".to_string());
    let iterations = args
        .next()
        .map(|value| value.parse::<usize>().expect("iterations must be a number"))
        .unwrap_or(3);

    let selected_cases: Vec<&str> = if selected == "all" {
        cases().to_vec()
    } else {
        vec![selected.as_str()]
    };

    for case in selected_cases {
        let start = Instant::now();
        let mut summary = run_case(case);
        for _ in 1..iterations {
            summary = run_case(case);
        }
        if env::var_os("CLIPPER_BENCH_DUMP").is_some() {
            if let Some(size) = case.strip_prefix("jittered_sliver_union_") {
                let (_, solution) =
                    jittered_sliver_union_sized(size.parse().expect("invalid jittered grid size"));
                dump_paths(&solution);
            }
        }
        let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
        println!(
            "engine=rust case={} iterations={} ok={} paths={} points={} area_abs={:.3} area_signed={:.3} checksum={} elapsed_ms={:.3}",
            case,
            iterations,
            summary.ok,
            summary.paths,
            summary.points,
            summary.area_abs,
            summary.area_signed,
            summary.checksum,
            elapsed_ms
        );
    }
}
