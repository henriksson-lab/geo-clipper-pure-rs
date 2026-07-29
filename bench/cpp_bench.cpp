#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include "clipper.hpp"

using namespace ClipperLib;

struct Summary {
  bool ok;
  size_t paths;
  size_t points;
  double area_abs;
  double area_signed;
  long double checksum;
};

static Summary summarize(bool ok, const Paths& paths) {
  Summary result{ok, paths.size(), 0, 0.0, 0.0, 0.0L};

  for (Paths::size_type path_idx = 0; path_idx < paths.size(); ++path_idx) {
    const Path& path = paths[path_idx];
    const double path_area = Area(path);
    result.area_abs += std::fabs(path_area);
    result.area_signed += path_area;
    result.points += path.size();

    for (Path::size_type idx = 0; idx < path.size(); ++idx) {
      const IntPoint& pt = path[idx];
      const long double weight = static_cast<long double>(idx + 1);
      result.checksum += weight * static_cast<long double>(pt.X);
      result.checksum += (weight + 17.0L) * static_cast<long double>(pt.Y);
      result.checksum += static_cast<long double>(pt.X) *
                         static_cast<long double>(pt.X + 31);
      result.checksum -= static_cast<long double>(pt.Y) *
                         static_cast<long double>(pt.Y - 17);
    }
  }

  return result;
}

static void dump_paths(const Paths& paths) {
  std::vector<std::string> normalized;
  normalized.reserve(paths.size());
  for (Paths::size_type path_idx = 0; path_idx < paths.size(); ++path_idx) {
    std::ostringstream out;
    const Path& path = paths[path_idx];
    for (Path::size_type idx = 0; idx < path.size(); ++idx) {
      if (idx != 0) out << ' ';
      out << path[idx].X << ',' << path[idx].Y;
    }
    normalized.push_back(out.str());
  }
  std::sort(normalized.begin(), normalized.end());
  for (std::vector<std::string>::const_iterator it = normalized.begin();
       it != normalized.end(); ++it) {
    std::cout << "path " << *it << "\n";
  }
}

static Path rect(cInt left, cInt top, cInt right, cInt bottom) {
  Path path;
  path << IntPoint(left, top) << IntPoint(right, top) << IntPoint(right, bottom)
       << IntPoint(left, bottom);
  return path;
}

static Paths rect_grid(cInt cols, cInt rows, cInt step, cInt size, cInt xoff,
                       cInt yoff) {
  Paths paths;
  paths.reserve(static_cast<size_t>(cols * rows));
  for (cInt y = 0; y < rows; ++y) {
    for (cInt x = 0; x < cols; ++x) {
      const cInt left = xoff + x * step;
      const cInt top = yoff + y * step;
      paths.push_back(rect(left, top, left + size, top + size));
    }
  }
  return paths;
}

static Paths vertical_strips(cInt count, cInt width, cInt height, cInt gap) {
  Paths paths;
  paths.reserve(static_cast<size_t>(count));
  for (cInt i = 0; i < count; ++i) {
    const cInt left = i * gap;
    paths.push_back(rect(left, 0, left + width, height));
  }
  return paths;
}

static Paths horizontal_strips(cInt count, cInt width, cInt height, cInt gap) {
  Paths paths;
  paths.reserve(static_cast<size_t>(count));
  for (cInt i = 0; i < count; ++i) {
    const cInt top = i * gap;
    paths.push_back(rect(0, top, width, top + height));
  }
  return paths;
}

static Path star(cInt cx, cInt cy, cInt outer, cInt inner, size_t vertices) {
  Path path;
  path.reserve(vertices);
  for (size_t i = 0; i < vertices; ++i) {
    const double angle = static_cast<double>(i) * 2.0 * 3.141592653589793238 /
                         static_cast<double>(vertices);
    const double radius = i % 2 == 0 ? static_cast<double>(outer)
                                     : static_cast<double>(inner);
    path << IntPoint(cx + static_cast<cInt>(std::llround(radius * std::cos(angle))),
                     cy + static_cast<cInt>(std::llround(radius * std::sin(angle))));
  }
  return path;
}

static Paths star_grid(cInt cols, cInt rows, cInt step, size_t vertices) {
  Paths paths;
  paths.reserve(static_cast<size_t>(cols * rows));
  for (cInt y = 0; y < rows; ++y) {
    for (cInt x = 0; x < cols; ++x) {
      paths.push_back(star(x * step, y * step, 34, 17, vertices));
    }
  }
  return paths;
}

static cInt jitter(cInt value) {
  unsigned long long x = static_cast<unsigned long long>(value);
  x = x * 6364136223846793005ULL + 1442695040888963407ULL;
  return static_cast<cInt>(x >> 61) - 3;
}

static Paths jittered_rect_grid(cInt cols, cInt rows, cInt step, cInt size) {
  Paths paths;
  paths.reserve(static_cast<size_t>(cols * rows));
  for (cInt y = 0; y < rows; ++y) {
    for (cInt x = 0; x < cols; ++x) {
      const cInt seed = y * cols + x;
      const cInt left = x * step + jitter(seed);
      const cInt top = y * step + jitter(seed + 17);
      Path path;
      path << IntPoint(left, top)
           << IntPoint(left + size + jitter(seed + 31),
                       top + jitter(seed + 43))
           << IntPoint(left + size + jitter(seed + 47),
                       top + size + jitter(seed + 59))
           << IntPoint(left + jitter(seed + 71),
                       top + size + jitter(seed + 83));
      paths.push_back(path);
    }
  }
  return paths;
}

static Paths open_diagonals(cInt count, cInt span, cInt step) {
  Paths paths;
  paths.reserve(static_cast<size_t>(count * 2));
  for (cInt i = 0; i < count; ++i) {
    const cInt offset = i * step;
    Path a;
    a << IntPoint(0, offset) << IntPoint(span / 2, offset + span / 3)
      << IntPoint(span, offset + span);
    paths.push_back(a);
    Path b;
    b << IntPoint(offset, 0) << IntPoint(offset + span / 3, span / 2)
      << IntPoint(offset + span, span);
    paths.push_back(b);
  }
  return paths;
}

static Summary run_union_dense() {
  Clipper clipper;
  Paths subjects = rect_grid(60, 60, 10, 18, 0, 0);
  clipper.AddPaths(subjects, ptSubject, true);
  Paths solution;
  const bool ok = clipper.Execute(ctUnion, solution, pftNonZero);
  return summarize(ok, solution);
}

static Summary run_touching_rect_grid() {
  Clipper clipper;
  Paths subjects = rect_grid(90, 90, 10, 10, 0, 0);
  clipper.AddPaths(subjects, ptSubject, true);
  Paths solution;
  const bool ok = clipper.Execute(ctUnion, solution, pftNonZero);
  return summarize(ok, solution);
}

static bool jittered_sliver_union_sized(cInt size, Paths& solution) {
  Clipper clipper;
  Paths subjects = jittered_rect_grid(size, size, 13, 16);
  clipper.AddPaths(subjects, ptSubject, true);
  solution.clear();
  return clipper.Execute(ctUnion, solution, pftNonZero);
}

static Summary run_jittered_sliver_union_sized(cInt size) {
  Paths solution;
  const bool ok = jittered_sliver_union_sized(size, solution);
  return summarize(ok, solution);
}

static Summary run_jittered_sliver_union() {
  return run_jittered_sliver_union_sized(56);
}

static Summary run_intersection_grid() {
  Clipper clipper;
  Paths subjects = vertical_strips(90, 9, 900, 10);
  Paths clips = horizontal_strips(90, 900, 9, 10);
  clipper.AddPaths(subjects, ptSubject, true);
  clipper.AddPaths(clips, ptClip, true);
  Paths solution;
  const bool ok = clipper.Execute(ctIntersection, solution, pftNonZero);
  return summarize(ok, solution);
}

static Summary run_nested_holes() {
  Clipper clipper;
  clipper.AddPath(rect(-2000, -2000, 2000, 2000), ptSubject, true);
  Paths holes = rect_grid(32, 32, 115, 58, -1800, -1800);
  Paths islands = rect_grid(32, 32, 115, 24, -1783, -1783);
  clipper.AddPaths(holes, ptClip, true);
  clipper.AddPaths(islands, ptSubject, true);
  Paths solution;
  const bool ok = clipper.Execute(ctDifference, solution, pftNonZero);
  return summarize(ok, solution);
}

static Summary run_difference_holes() {
  Clipper clipper;
  clipper.AddPath(rect(-20, -20, 920, 920), ptSubject, true);
  Paths clips = rect_grid(38, 38, 23, 11, 15, 15);
  clipper.AddPaths(clips, ptClip, true);
  Paths solution;
  const bool ok = clipper.Execute(ctDifference, solution, pftNonZero);
  return summarize(ok, solution);
}

static Summary run_strict_simple_stars() {
  Clipper clipper(ioStrictlySimple);
  Paths subjects = star_grid(22, 22, 38, 20);
  clipper.AddPaths(subjects, ptSubject, true);
  Paths solution;
  const bool ok = clipper.Execute(ctUnion, solution, pftNonZero);
  return summarize(ok, solution);
}

static Summary run_open_paths_clip() {
  Clipper clipper;
  Paths closed;
  closed.push_back(rect(80, 80, 920, 920));
  Paths open = open_diagonals(180, 1100, 5);
  clipper.AddPaths(closed, ptClip, true);
  clipper.AddPaths(open, ptSubject, false);
  PolyTree polytree;
  const bool ok = clipper.Execute(ctIntersection, polytree, pftNonZero);
  Paths solution;
  OpenPathsFromPolyTree(polytree, solution);
  return summarize(ok, solution);
}

static Summary run_large_coord_xor() {
  const cInt origin = 1200000000LL;
  Clipper clipper;
  Paths subjects = rect_grid(34, 34, 30000000LL, 48000000LL, origin, origin);
  Paths clips = rect_grid(34, 34, 30000000LL, 48000000LL, origin + 12000000LL,
                          origin + 12000000LL);
  clipper.AddPaths(subjects, ptSubject, true);
  clipper.AddPaths(clips, ptClip, true);
  Paths solution;
  const bool ok = clipper.Execute(ctXor, solution, pftNonZero);
  return summarize(ok, solution);
}

static Summary run_offset_stars() {
  ClipperOffset offset(2.0, 0.25);
  Paths subjects = star_grid(26, 26, 90, 48);
  offset.AddPaths(subjects, jtRound, etClosedPolygon);
  Paths solution;
  offset.Execute(solution, 7.0);
  return summarize(true, solution);
}

static Summary run_offset_open_round() {
  ClipperOffset offset(2.0, 0.25);
  Paths subjects = open_diagonals(220, 500, 8);
  offset.AddPaths(subjects, jtRound, etOpenRound);
  Paths solution;
  offset.Execute(solution, 9.0);
  return summarize(true, solution);
}

static Summary run_polytree_closed_nested() {
  Clipper clipper;
  clipper.AddPath(rect(-1500, -1500, 1500, 1500), ptSubject, true);
  Paths holes = rect_grid(24, 24, 115, 52, -1300, -1300);
  clipper.AddPaths(holes, ptClip, true);
  PolyTree polytree;
  const bool ok = clipper.Execute(ctDifference, polytree, pftNonZero);
  Paths solution;
  ClosedPathsFromPolyTree(polytree, solution);
  return summarize(ok, solution);
}

static Summary run_case(const std::string& name) {
  const std::string jitter_prefix = "jittered_sliver_union_";
  if (name.compare(0, jitter_prefix.size(), jitter_prefix) == 0) {
    const std::string size = name.substr(jitter_prefix.size());
    return run_jittered_sliver_union_sized(
        static_cast<cInt>(std::strtoll(size.c_str(), nullptr, 10)));
  }

  if (name == "union_dense") return run_union_dense();
  if (name == "touching_rect_grid") return run_touching_rect_grid();
  if (name == "jittered_sliver_union") return run_jittered_sliver_union();
  if (name == "intersection_grid") return run_intersection_grid();
  if (name == "difference_holes") return run_difference_holes();
  if (name == "nested_holes") return run_nested_holes();
  if (name == "strict_simple_stars") return run_strict_simple_stars();
  if (name == "open_paths_clip") return run_open_paths_clip();
  if (name == "large_coord_xor") return run_large_coord_xor();
  if (name == "offset_stars") return run_offset_stars();
  if (name == "offset_open_round") return run_offset_open_round();
  if (name == "polytree_closed_nested") return run_polytree_closed_nested();
  throw std::runtime_error("unknown benchmark case: " + name);
}

static const std::vector<std::string>& cases() {
  static const std::vector<std::string> names{
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
      "polytree_closed_nested"};
  return names;
}

int main(int argc, char** argv) {
  const std::string selected = argc > 1 ? argv[1] : "all";
  const size_t iterations =
      argc > 2 ? static_cast<size_t>(std::strtoull(argv[2], nullptr, 10)) : 3;

  std::vector<std::string> selected_cases;
  if (selected == "all") {
    selected_cases = cases();
  } else {
    selected_cases.push_back(selected);
  }

  for (const std::string& name : selected_cases) {
    const clock_t start = clock();
    Summary summary = run_case(name);
    for (size_t i = 1; i < iterations; ++i) {
      summary = run_case(name);
    }
    if (std::getenv("CLIPPER_BENCH_DUMP")) {
      const std::string jitter_prefix = "jittered_sliver_union_";
      if (name.compare(0, jitter_prefix.size(), jitter_prefix) == 0) {
        const std::string size = name.substr(jitter_prefix.size());
        Paths solution;
        jittered_sliver_union_sized(
            static_cast<cInt>(std::strtoll(size.c_str(), nullptr, 10)),
            solution);
        dump_paths(solution);
      }
    }
    const double elapsed_ms =
        1000.0 * static_cast<double>(clock() - start) / CLOCKS_PER_SEC;

    std::cout << std::fixed << std::setprecision(3)
              << "engine=cpp"
              << " case=" << name << " iterations=" << iterations
              << " ok=" << (summary.ok ? "true" : "false")
              << " paths=" << summary.paths << " points=" << summary.points
              << " area_abs=" << summary.area_abs
              << " area_signed=" << summary.area_signed
              << " checksum=" << std::setprecision(0) << summary.checksum
              << std::setprecision(3) << " elapsed_ms=" << elapsed_ms << "\n";
  }

  return 0;
}
