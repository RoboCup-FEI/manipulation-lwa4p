#!/usr/bin/env bash
# Formats the C and C++ files of the team's ROS packages: clang-format with
# .github/format/clang-format.yaml, then ament_uncrustify --reformat (ROS 2 style).
# The result passes ament_uncrustify and ament_cpplint. Running it twice changes nothing.
# Tomas's packages are skipped. See AGENTS.md, section 4.
#
# Usage: format_cpp.sh [--check] [MAIN_REF]
#   --check   CI mode: fail and show the diff instead of keeping the changes
#   MAIN_REF  defaults to origin/main
#
# Needs clang-format-18 (Ubuntu 24.04: sudo apt install clang-format) and a sourced
# ROS 2 Jazzy environment with ament_uncrustify.

set -euo pipefail

check=0
if [[ "${1:-}" == "--check" ]]; then
  check=1
  shift
fi
main_ref="${1:-origin/main}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

for tool in clang-format-18 ament_uncrustify; do
  if ! command -v "${tool}" > /dev/null; then
    echo "error: ${tool} not found. Install clang-format 18 and source /opt/ros/jazzy/setup.bash." >&2
    exit 2
  fi
done

mapfile -d '' -t files < <(bash "${script_dir}/team_cpp_files.sh" "${main_ref}")
if (( ${#files[@]} == 0 )); then
  echo "OK: the team's packages have no C/C++ files."
  exit 0
fi

if (( check )) && ! git diff --quiet -- "${files[@]}"; then
  echo "error: --check needs these files without uncommitted changes." >&2
  exit 2
fi

clang-format-18 -i --style="file:${repo_root}/.github/format/clang-format.yaml" "${files[@]}"
# Its own report ("files with code style divergence") is noise here: the diff below counts.
ament_uncrustify --reformat "${files[@]}" > /dev/null 2>&1 || true

if ! (( check )); then
  echo "Formatted ${#files[@]} C/C++ files. Review and commit the changes."
  exit 0
fi

if git diff --quiet -- "${files[@]}"; then
  echo "OK: ${#files[@]} C/C++ files in the team's packages are formatted."
  exit 0
fi

git diff --stat -- "${files[@]}"
git diff -- "${files[@]}"
changed="$(git diff --name-only -- "${files[@]}")"
if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
  while IFS= read -r path; do
    echo "::error file=${path}::Not formatted. Run .github/scripts/format_cpp.sh and commit the result."
  done <<< "${changed}"
fi
git checkout -- "${files[@]}"
echo
echo "These files are not formatted:"
echo "${changed}" | sed 's/^/  /'
echo
echo "Fix: source /opt/ros/jazzy/setup.bash && .github/scripts/format_cpp.sh"
echo "Then commit and push. The diff above shows exactly what the command changes."
exit 1
