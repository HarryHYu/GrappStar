#!/bin/sh
# debug one test group with the event log: ./dbg.sh cock
cd "$(dirname "$0")" && g++ -std=c++17 -O1 -g -fsanitize=address,undefined -o test_base test_base.cpp && SIMLOG=1 ./test_base "$@"
