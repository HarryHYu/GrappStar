#!/bin/bash
# Coordinator helper: merge every worker branch (origin/w/*) into the current branch (base-v1).
# Stops at the first conflict and names it. Usage: scripts/merge_workers.sh [worker ...]
# With no names it merges all workers in a fixed order (spec readers first, outputs last).
set -u
cd "$(dirname "$0")/.."
ORDER=(fw-base fw-dart physics jetson jetson-vision systest cad-infra cad-drum cad-reel cad-launch cad-frame sensors docs review)
[ $# -gt 0 ] && ORDER=("$@")
cur=$(git rev-parse --abbrev-ref HEAD)
if [ "$cur" != "base-v1" ]; then echo "on $cur, not base-v1: stop"; exit 2; fi
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then echo "working tree not clean: stop"; exit 2; fi
git fetch -q origin base-v1 || { echo "fetch base-v1 failed"; exit 2; }
git merge -q --ff-only origin/base-v1 2>/dev/null || true
for w in "${ORDER[@]}"; do
    git fetch -q origin "w/$w" 2>/dev/null || { echo "SKIP  $w (no branch)"; continue; }
    n=$(git rev-list --count HEAD..origin/w/$w)
    if [ "$n" = "0" ]; then echo "UP    $w (nothing new)"; continue; fi
    if git merge -q --no-edit -m "Merge w/$w into base-v1" "origin/w/$w" >/dev/null 2>&1; then
        echo "MERGE $w ($n commits)"
    else
        echo "CONFLICT $w:"; git diff --name-only --diff-filter=U | sed 's/^/    /'
        echo "Resolve, commit, then rerun for the rest (or: git merge --abort)."; exit 1
    fi
done
# generated or local files the merges must not bring in
bad=$(git diff --name-only origin/base-v1..HEAD | grep -E '(^|/)(__pycache__|\.venv)/|\.pyc$' || true)
[ -n "$bad" ] && { echo "WARNING: generated files came in:"; echo "$bad" | sed 's/^/    /'; }
echo "Done. Next: light checks, then git push origin base-v1"
