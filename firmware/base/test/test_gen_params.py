#!/usr/bin/env python3
"""Tests for tools/gen_params.py: value shapes, the derived aim table, and that the output compiles as C++."""
import os, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import gen_params  # noqa: E402

passed = failed = 0
def check(cond, msg):
    global passed, failed
    if cond: passed += 1
    else: failed += 1; print("  FAIL", msg)

def gen(name):
    out = tempfile.mktemp(suffix=".h")
    warn, n = gen_params.generate(os.path.join(HERE, "fixtures", name), out)
    return open(out).read(), warn, out

def compiles(header):
    src = tempfile.mktemp(suffix=".cpp")
    open(src, "w").write('#include "%s"\nint main(){return 0;}\n' % header)
    r = subprocess.run(["g++", "-std=c++17", "-Wall", "-Werror", "-Wno-unused-variable", "-c", src, "-o", os.devnull], capture_output=True, text=True)
    if r.returncode: print(r.stderr[:2000])
    return r.returncode == 0

print("gen_params: sample shapes")
h, warn, path = gen("params_sample.py")
check("#define P_WIPER_STALL_A 18.0f" in h, "dict float")
check("#define P_WIPER_MEASURED 0" in h, "bool")
check('#define P_WIPER_NOTE "12 V"' in h, "string")
check("// P_WIPER_HOLES: not set" in h, "None")
check("static const float P_BED[] = {256.0f, 256.0f, 260.0f};" in h and "#define P_BED_N 3" in h, "tuple")
check('#define P_SPRAG_ROLES_1 "cock"' in h, "int-keyed dict")
check('static const char* const P_NAMES[] = {"a", "b"};' in h, "string list")
check("static const float P_STEPS_T[] = {0.0f, 1.5f};" in h and "#define P_STEPS_N 2" in h, "list of dicts")
check("static const float P_GRID[][2]" in h, "2-D list")
check(any("P_MIXED" in w for w in warn), "mixed list warned")
check("HELPER" not in h, "functions skipped")
check("#define P_AIMTAB_N 181" in h and "#define P_AIMTAB_CYCLE_DEG 10800.0f" in h, "four-bar aim table derived (ratio 30)")
check(compiles(path), "sample header compiles")
print("gen_params: explicit table and callable")
h2, w2, p2 = gen("params_table.py")
check("#define P_AIMTAB_N 3" in h2 and "P_AIMTAB_PITCH[] = {15.0f, 60.0f, 15.0f}" in h2, "explicit table")
check(compiles(p2), "table header compiles")
h3, w3, p3 = gen("params_callable.py")
check("#define P_AIMTAB_N 181" in h3, "callable sampled every 2 deg")
check(compiles(p3), "callable header compiles")
print("gen_params: the real params.py")
real = os.path.join(HERE, "..", "..", "..", "base", "spec", "params.py")
out = tempfile.mktemp(suffix=".h")
wr, nr = gen_params.generate(real, out)
check(nr > 10 and compiles(out), "real params.py generates a header that compiles")
print("\n%d checks, %d passed, %d failed: %s" % (passed + failed, passed, failed, "FAILURES" if failed else "ALL PASS"))
sys.exit(1 if failed else 0)
