#!/usr/bin/env bash
# Prints the names of the ROS packages that come from main (Tomas-Jurov/lwa4p_driver),
# one per line. CI uses the list to skip Tomas's packages. See AGENTS.md, section 2.
#
# Usage: main_package_names.sh [MAIN_REF] [HEAD_REF]
#   MAIN_REF  defaults to origin/main
#   HEAD_REF  defaults to HEAD
#
# Like check_main_files_untouched.sh, it reads the last MAIN_REF commit that HEAD_REF
# already contains (the merge base), not the tip of MAIN_REF.

set -euo pipefail

main_ref="${1:-origin/main}"
head_ref="${2:-HEAD}"

if ! base="$(git merge-base "${main_ref}" "${head_ref}")"; then
  echo "error: ${head_ref} shares no history with ${main_ref}." >&2
  exit 2
fi

manifests="$(git ls-tree -r --name-only "${base}" -- src | grep '/package.xml$' || true)"
while IFS= read -r manifest; do
  [[ -n "${manifest}" ]] || continue
  # Read the whole file first: awk exits at the first match and would break the pipe.
  content="$(git show "${base}:${manifest}")"
  awk -F'[<>]' '/<name>/ { print $3; exit }' <<< "${content}"
done <<< "${manifests}"
