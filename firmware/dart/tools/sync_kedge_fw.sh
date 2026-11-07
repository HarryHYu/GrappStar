#!/bin/sh
# Copies the canonical dart firmware (firmware/dart) into the old kedge/fw folder, so kedge/make_dist.sh, setup_mac.sh
# and anyone opening kedge/fw get the current firmware, never the old v3 one. Edit firmware/dart only, then run this.
# firmware/dart/test/run_tests.sh fails while the two copies differ.
set -e
cd "$(dirname "$0")/../../.."
cp firmware/dart/kedge_dart/kedge_dart.ino firmware/dart/kedge_dart/kedge_logic.h kedge/fw/kedge_dart/
cp firmware/dart/test/test_logic.cpp kedge/fw/test/test_logic.cpp
rm -f kedge/fw/test/test_logic
( cd kedge/fw/test && g++ -std=c++17 -O1 -I../kedge_dart -o /tmp/kedge_fw_test_logic.$$ test_logic.cpp && /tmp/kedge_fw_test_logic.$$ > last_run.txt; rc=$?; rm -f /tmp/kedge_fw_test_logic.$$; rm -rf out; exit $rc )
tail -1 kedge/fw/test/last_run.txt
echo "kedge/fw/kedge_dart and kedge/fw/test/test_logic.cpp now match firmware/dart"
