#!/bin/sh
# Stop every running harness, shim and dart sim started by systest (batch shards included).
for pat in "systest/batch.sh" "systest/harness.py" "build/base_shim" "sim/kedge_sim"; do
  for p in $(pgrep -f "$pat"); do [ "$p" != "$$" ] && kill "$p" 2>/dev/null; done
done
sleep 1
