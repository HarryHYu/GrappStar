#!/bin/sh
# Build and run the PC tests for the base firmware (g++ 9 or newer, python 3).
set -e
cd "$(dirname "$0")"
echo "== params generator"
python3 test_gen_params.py
echo "== firmware logic (simulation)"
g++ -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -fsanitize=address,undefined -o test_base test_base.cpp
./test_base
g++ -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -fsanitize=address,undefined -I. -o test_fuzz test_fuzz.cpp && ./test_fuzz 300
