#!/usr/bin/env python3
"""Print the summary table of all review/r*.md topic files (for REPORT.md)."""
import glob
import re

print("| Topic | File | Reviewed at | Confirmed | Refuted |")
print("|---|---|---|---|---|")
for f in sorted(glob.glob("review/r*.md")):
    s = open(f).read()
    title = s.splitlines()[0][2:]
    c = re.search(r"Confirmed: (\d+)\. Refuted and dropped: (\d+)", s)
    sev = {k: len(re.findall(r"^\| \d+ \| " + k + " ", s, re.M)) for k in ["critical", "high", "medium", "low"]}
    base = re.search(r"commit ([0-9a-f]{7})", s).group(1)
    counts = ", ".join(f"{v} {k}" for k, v in sev.items() if v)
    print(f"| {title} | {f.split('/')[-1]} | {base} | {c.group(1)} ({counts}) | {c.group(2)} |")
