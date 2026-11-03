#!/bin/sh
# Every scenario in N parallel shards (default 3). Logs: systest/runs<suffix>/shard<i>.log, results<_shard i>.json.
#   systest/batch.sh [N]                 on this checkout
#   PREVIEW=1 systest/batch.sh [N]       on base-v1 + the owners' branches (preview.sh's worktree, rebuilt first)
cd "$(dirname "$0")/.."
N=${1:-3}
if [ -n "$PREVIEW" ]; then
  WT=${PREVIEW_DIR:-/tmp/grappstar_preview}
  git fetch -q origin base-v1 w/fw-base w/fw-dart w/jetson
  rm -rf "$WT"; git worktree prune; git worktree add -q --detach "$WT" origin/base-v1
  ( cd "$WT" && git checkout -q origin/w/fw-base -- firmware/base && git checkout -q origin/w/fw-dart -- firmware/dart && git checkout -q origin/w/jetson -- jetson )
  export SYSTEST_ROOT="$WT" SYSTEST_RUNS="$(pwd)/systest/runs_preview"
  echo "preview: base-v1 $(git rev-parse --short origin/base-v1), fw-base $(git rev-parse --short origin/w/fw-base), fw-dart $(git rev-parse --short origin/w/fw-dart), jetson $(git rev-parse --short origin/w/jetson)" > "$SYSTEST_RUNS.version"
fi
OUT=${SYSTEST_RUNS:-systest/runs}; mkdir -p "$OUT"
python3 systest/harness.py --scenario nominal --list >/dev/null    # import check
python3 -c "import sys; sys.argv=['x']; sys.path.insert(0,'systest'); import harness; harness.build()"
i=0; while [ $i -lt $N ]; do
  python3 systest/harness.py --scenario all --shard $i/$N > "$OUT/shard$i.log" 2>&1 &
  i=$((i+1)); sleep 5
done
wait
grep -h -E "^  (PASS|FAIL) " "$OUT"/shard*.log | sort -k2
