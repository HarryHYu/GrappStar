#!/bin/sh
# Test the owners' unmerged branches together before the coordinator merges them:
# a worktree of origin/base-v1 with firmware/base from w/fw-base, firmware/dart from w/fw-dart and jetson/ from w/jetson.
#   systest/preview.sh [scenario|all]      results in systest/runs_preview/
set -e
cd "$(dirname "$0")/.."
WT=${PREVIEW_DIR:-/tmp/grappstar_preview}
git fetch -q origin base-v1 w/fw-base w/fw-dart w/jetson
rm -rf "$WT"; git worktree prune
git worktree add -q --detach "$WT" origin/base-v1
( cd "$WT" && git checkout -q origin/w/fw-base -- firmware/base && git checkout -q origin/w/fw-dart -- firmware/dart \
  && git checkout -q origin/w/jetson -- jetson )
echo "preview: base-v1 $(git rev-parse --short origin/base-v1), fw-base $(git rev-parse --short origin/w/fw-base), fw-dart $(git rev-parse --short origin/w/fw-dart), jetson $(git rev-parse --short origin/w/jetson)"
SYSTEST_ROOT="$WT" SYSTEST_RUNS="${SYSTEST_RUNS:-$(pwd)/systest/runs_preview}" exec python3 systest/harness.py --scenario "${1:-nominal}"
