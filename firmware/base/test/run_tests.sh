#!/bin/sh
# Build and run the PC tests for the base firmware logic (g++ 9 or newer).
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -fsanitize=address,undefined -o test_base test_base.cpp
./test_base
