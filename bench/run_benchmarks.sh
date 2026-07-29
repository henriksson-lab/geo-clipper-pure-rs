#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CRATE_DIR="${REPO_ROOT}/clipper-rust"
OUT_DIR="${REPO_ROOT}/clipper-rust/target/bench"
ITERATIONS="${1:-3}"

CASES=(
  union_dense
  touching_rect_grid
  intersection_grid
  difference_holes
  nested_holes
  strict_simple_stars
  open_paths_clip
  large_coord_xor
  offset_stars
  offset_open_round
  polytree_closed_nested
)

mkdir -p "${OUT_DIR}"

cargo build --manifest-path "${CRATE_DIR}/Cargo.toml" --release --features bench-bin --bin bench

g++ -O3 -DNDEBUG -std=c++11 \
  "${SCRIPT_DIR}/cpp_bench.cpp" \
  "${REPO_ROOT}/Clipper/cpp/clipper.cpp" \
  -I "${REPO_ROOT}/Clipper/cpp" \
  -o "${OUT_DIR}/cpp_bench"

parse_summary() {
  local line="$1"
  local key="$2"
  printf '%s\n' "${line}" | tr ' ' '\n' | awk -F= -v key="${key}" '$1 == key { print $2 }'
}

run_timed() {
  local engine="$1"
  local case_name="$2"
  local stdout_file="${OUT_DIR}/${engine}-${case_name}.out"
  local time_file="${OUT_DIR}/${engine}-${case_name}.time"

  if [[ "${engine}" == "rust" ]]; then
    /usr/bin/time -f 'rss_kb=%M wall_s=%e' -o "${time_file}" \
      "${CRATE_DIR}/target/release/bench" "${case_name}" "${ITERATIONS}" \
      > "${stdout_file}"
  else
    /usr/bin/time -f 'rss_kb=%M wall_s=%e' -o "${time_file}" \
      "${OUT_DIR}/cpp_bench" "${case_name}" "${ITERATIONS}" \
      > "${stdout_file}"
  fi
}

printf 'case,parity,rust_elapsed_ms,cpp_elapsed_ms,rust_wall_s,cpp_wall_s,rust_rss_kb,cpp_rss_kb,rust_paths,cpp_paths,rust_points,cpp_points,rust_area_abs,cpp_area_abs,rust_checksum,cpp_checksum\n'
failures=0

for case_name in "${CASES[@]}"; do
  run_timed rust "${case_name}"
  run_timed cpp "${case_name}"

  rust_line="$(cat "${OUT_DIR}/rust-${case_name}.out")"
  cpp_line="$(cat "${OUT_DIR}/cpp-${case_name}.out")"
  rust_time="$(cat "${OUT_DIR}/rust-${case_name}.time")"
  cpp_time="$(cat "${OUT_DIR}/cpp-${case_name}.time")"

  rust_paths="$(parse_summary "${rust_line}" paths)"
  cpp_paths="$(parse_summary "${cpp_line}" paths)"
  rust_points="$(parse_summary "${rust_line}" points)"
  cpp_points="$(parse_summary "${cpp_line}" points)"
  rust_area="$(parse_summary "${rust_line}" area_abs)"
  cpp_area="$(parse_summary "${cpp_line}" area_abs)"
  rust_checksum="$(parse_summary "${rust_line}" checksum)"
  cpp_checksum="$(parse_summary "${cpp_line}" checksum)"

  parity="fail"
  if [[ "${rust_paths}" == "${cpp_paths}" &&
        "${rust_points}" == "${cpp_points}" &&
        "${rust_area}" == "${cpp_area}" &&
        "${rust_checksum}" == "${cpp_checksum}" ]]; then
    parity="ok"
  else
    failures=$((failures + 1))
  fi

  rust_elapsed="$(parse_summary "${rust_line}" elapsed_ms)"
  cpp_elapsed="$(parse_summary "${cpp_line}" elapsed_ms)"
  rust_rss="$(parse_summary "${rust_time}" rss_kb)"
  cpp_rss="$(parse_summary "${cpp_time}" rss_kb)"
  rust_wall="$(parse_summary "${rust_time}" wall_s)"
  cpp_wall="$(parse_summary "${cpp_time}" wall_s)"

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${case_name}" "${parity}" "${rust_elapsed}" "${cpp_elapsed}" \
    "${rust_wall}" "${cpp_wall}" "${rust_rss}" "${cpp_rss}" \
    "${rust_paths}" "${cpp_paths}" "${rust_points}" "${cpp_points}" \
    "${rust_area}" "${cpp_area}" "${rust_checksum}" "${cpp_checksum}"
done

printf '\nDetailed outputs are in %s\n' "${OUT_DIR}"

if [[ "${failures}" -ne 0 ]]; then
  printf '%s parity failure(s)\n' "${failures}" >&2
  exit 1
fi
