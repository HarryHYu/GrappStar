#!/usr/bin/env python3
"""Generate firmware/base/grappstar_base/params_gen.h from base/spec/params.py.

Usage (from anywhere):  python3 firmware/base/tools/gen_params.py [path/to/params.py] [out.h]
Defaults: <repo>/base/spec/params.py and <repo>/firmware/base/grappstar_base/params_gen.h.

Every top-level UPPER_CASE value in params.py becomes C macros, flattened with underscores:
  WIPER = dict(stall_a=18.0)        ->  #define P_WIPER_STALL_A 18.0f
  BED = (256.0, 256.0, 260.0)       ->  static const float P_BED[] = {...};  #define P_BED_N 3
  lists of dicts with numbers       ->  one array per key (P_NAME_KEY[]) plus P_NAME_N
  None                              ->  a comment (the firmware then uses its own default, see config.h)
Strings are emitted as string macros when short and plain ASCII. Booleans become 1 / 0.
The firmware only ever uses P_* names behind #ifdef, so a missing or renamed value falls back to the firmware
default (config.h) instead of breaking the build; check_params() below lists the names the firmware expects so a
rename shows up as a WARNING here.

Derived tables (computed here so the firmware never does geometry):
  - aim table: pitch (deg) against aim-train wiper angle, from AIM (see derive_aim()).
"""
import math
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEF_IN = os.path.join(REPO, "base", "spec", "params.py")
DEF_OUT = os.path.join(REPO, "firmware", "base", "grappstar_base", "params_gen.h")

# names the firmware reads (config.h / base.h). Missing ones only warn: config.h has a default for each.
EXPECTED = [
    "P_WIPER_STALL_A", "P_WIPER_FREE_A", "P_WIPER_N0_RPM", "P_BATTERY_V_FULL",
]


def cname(parts):
    s = "_".join(str(p) for p in parts)
    out = []
    for ch in s:
        out.append(ch.upper() if ch.isalnum() else "_")
    name = "".join(out)
    while "__" in name:
        name = name.replace("__", "_")
    return "P_" + name.strip("_")


def is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))


def fnum(v):
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    r = repr(float(v))
    if "e" in r or "E" in r:
        return "%.9gf" % v if "." in ("%.9g" % v) or "e" in ("%.9g" % v) else "%.9g.0f" % v
    return r + "f"


def cstr(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit(lines, parts, v, warn):
    name = cname(parts)
    if v is None:
        lines.append("// %s: not set in params.py (firmware default used)" % name)
    elif isinstance(v, bool) or is_num(v):
        lines.append("#define %s %s" % (name, fnum(v)))
    elif isinstance(v, str):
        if len(v) <= 80 and all(32 <= ord(c) < 127 for c in v):
            lines.append("#define %s %s" % (name, cstr(v)))
    elif isinstance(v, dict):
        for k, x in v.items():
            emit(lines, parts + [k], x, warn)
    elif isinstance(v, (list, tuple)):
        if len(v) == 0:
            lines.append("#define %s_N 0" % name)
        elif all(is_num(x) or isinstance(x, bool) for x in v):
            lines.append("static const float %s[] = {%s};" % (name, ", ".join(fnum(float(x)) for x in v)))
            lines.append("#define %s_N %d" % (name, len(v)))
        elif all(isinstance(x, str) for x in v):
            lines.append("static const char* const %s[] = {%s};" % (name, ", ".join(cstr(x) for x in v)))
            lines.append("#define %s_N %d" % (name, len(v)))
        elif all(isinstance(x, dict) for x in v):
            keys = []
            for x in v:
                for k in x:
                    if k not in keys:
                        keys.append(k)
            lines.append("#define %s_N %d" % (name, len(v)))
            for k in keys:
                col = [x.get(k) for x in v]
                if all(is_num(c) or isinstance(c, bool) for c in col):
                    lines.append("static const float %s[] = {%s};" % (cname(parts + [k]), ", ".join(fnum(float(c)) for c in col)))
                elif all(isinstance(c, str) for c in col):
                    lines.append("static const char* const %s[] = {%s};" % (cname(parts + [k]), ", ".join(cstr(c) for c in col)))
        elif all(isinstance(x, (list, tuple)) and all(is_num(y) for y in x) for x in v) and len({len(x) for x in v}) == 1:
            w = len(v[0])
            lines.append("static const float %s[][%d] = {%s};" % (name, w, ", ".join("{" + ", ".join(fnum(float(y)) for y in x) + "}" for x in v)))
            lines.append("#define %s_N %d" % (name, len(v)))
        else:
            warn.append("skipped %s (mixed list)" % name)
    else:
        pass  # functions, modules, classes: not data


def derive_aim(P, lines, warn):
    """Aim table: pitch against aim-train wiper angle over one aim cycle.

    Accepted forms in params.py (first that exists wins):
      AIM["table"] = [(wiper_deg, pitch_deg), ...]     explicit table, wiper_deg rising over one cycle
      AIM["pitch_of"] = callable(wiper_deg) -> pitch    sampled every 2 deg over AIM["cycle_deg"] (default 360)
      AIM crank-rocker: crank, coupler, rocker, ground (link lengths, mm), ratio (wiper deg per crank deg),
        rocker_to_pitch = (offset_deg, sign)            solved here (four-bar, open branch)
    """
    A = P.get("AIM")
    if not isinstance(A, dict):
        lines.append("// aim table: AIM not set in params.py (firmware placeholder table used)")
        return
    tab = None
    cyc = float(A.get("cycle_deg", 360.0) or 360.0)
    if A.get("table"):
        tab = [(float(a), float(b)) for a, b in A["table"]]
    elif callable(A.get("pitch_of")):
        n = int(cyc / 2.0)
        tab = [(i * cyc / n, float(A["pitch_of"](i * cyc / n))) for i in range(n + 1)]
    elif all(k in A for k in ("crank", "coupler", "rocker", "ground")):
        a, b, c, d = (float(A[k]) for k in ("crank", "coupler", "rocker", "ground"))
        ratio = float(A.get("ratio", 1.0))
        off, sgn = A.get("rocker_to_pitch", (0.0, 1.0))
        n = 180
        tab = []
        for i in range(n + 1):
            th = 2 * math.pi * i / n
            # crank pin, ground pivots at (0,0) and (d,0)
            px, py = a * math.cos(th), a * math.sin(th)
            dx, dy = d - px, -py
            L = math.hypot(dx, dy)
            if L > b + c or L < abs(b - c) or L == 0:
                warn.append("aim four-bar does not assemble at crank %.1f deg" % math.degrees(th))
                return
            base = math.atan2(dy, dx)
            gam = math.acos((L * L + c * c - b * b) / (2 * L * c))
            rocker = base + math.pi - gam                     # open branch
            tab.append((math.degrees(th) * ratio, float(off) + float(sgn) * math.degrees(rocker)))
        cyc = 360.0 * ratio
    if not tab:
        lines.append("// aim table: AIM has no table / pitch_of / four-bar (firmware placeholder table used)")
        return
    if any(tab[i + 1][0] <= tab[i][0] for i in range(len(tab) - 1)):
        warn.append("aim table wiper angles must rise; table skipped")
        return
    lines.append("// aim table (derived): pitch_deg against aim-train wiper angle over one aim cycle of %.1f deg" % cyc)
    lines.append("#define P_AIM_TABLE_N %d" % len(tab))
    lines.append("#define P_AIM_CYCLE_DEG %s" % fnum(cyc))
    lines.append("static const float P_AIM_TABLE_WIPER[] = {%s};" % ", ".join(fnum(round(x, 3)) for x, _ in tab))
    lines.append("static const float P_AIM_TABLE_PITCH[] = {%s};" % ", ".join(fnum(round(y, 3)) for _, y in tab))


def generate(inp, out):
    P = runpy.run_path(inp)
    lines, warn = [], []
    head = [
        "// GENERATED by firmware/base/tools/gen_params.py from base/spec/params.py. Do not edit: edit params.py and rerun.",
        "// params.py VERSION: %s" % P.get("VERSION", "?"),
        "#pragma once",
        "",
    ]
    for k, v in P.items():
        if not k.isupper() or k.startswith("_") or callable(v) or k == "VERSION":
            continue
        emit(lines, [k], v, warn)
    lines.append("")
    derive_aim(P, lines, warn)
    defined = {l.split()[1] for l in lines if l.startswith("#define ")}
    defined |= {l.split()[3].split("[")[0] for l in lines if l.startswith("static const")}
    for e in EXPECTED:
        if e not in defined:
            warn.append("firmware expects %s (default used)" % e)
    body = "\n".join(head + ['#define P_VERSION %s' % cstr(str(P.get("VERSION", "?")))] + lines) + "\n"
    with open(out, "w") as f:
        f.write(body)
    return warn, len([l for l in lines if l.startswith(("#define", "static const"))])


if __name__ == "__main__":
    inp = sys.argv[1] if len(sys.argv) > 1 else DEF_IN
    out = sys.argv[2] if len(sys.argv) > 2 else DEF_OUT
    warn, n = generate(inp, out)
    for w in warn:
        print("WARNING:", w)
    print("wrote %s (%d values) from %s" % (os.path.relpath(out, REPO), n, os.path.relpath(inp, REPO)))
