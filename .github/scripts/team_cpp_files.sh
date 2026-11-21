#!/usr/bin/env bash
# Prints the C and C++ files tracked by git in the team's ROS packages, NUL-separated.
# Tomas's packages (those that come from main) are skipped. See AGENTS.md, section 2.
#
# Usage: team_cpp_files.sh [MAIN_REF]
#   MAIN_REF  defaults to origin/main

set -euo pipefail

main_ref="${1:-origin/main}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$(git rev-parse --show-toplevel)"

main_names="$(bash "${script_dir}/main_package_names.sh" "${main_ref}" HEAD)"

while IFS= read -r -d '' manifest; do
  package_dir="$(dirname "${manifest}")"
  package_name="$(awk -F'[<>]' '/<name>/ { print $3; exit }' "${manifest}")"
  if grep -qxF "${package_name}" <<< "${main_names}"; then
    continue
  fi
  git ls-files -z -- \
    "${package_dir}/*.c" "${package_dir}/*.cc" "${package_dir}/*.cpp" "${package_dir}/*.cxx" \
    "${package_dir}/*.h" "${package_dir}/*.hh" "${package_dir}/*.hpp" "${package_dir}/*.hxx"
done < <(git ls-files -z -- src | grep -z '/package.xml$' || true)
