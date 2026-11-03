#!/usr/bin/env python3
"""Add a 'Now' status column to a topic file from a staleness-check workflow result.

Usage: add_status.py topic.md kept.json stale_task.output newer_commit
kept.json is the list of kept findings (index i) the checkers read; the task output holds
[{"i", "status", "where", "note"}].
"""
import json
import sys

md, kept, out, commit = sys.argv[1:5]
K = json.load(open(kept))
R = json.load(open(out))["result"]
by_title = {K[r["i"]]["title"]: r for r in R}
L = open(md).read().split("\n")
res = []
for line in L:
    if line.startswith("| # | Sev |"):
        line = "| # | Sev | File:line | Owner | Finding | Now (" + commit + ") |"
    elif line.startswith("|---|---|---|---|---|"):
        line = "|---|---|---|---|---|---|"
    elif line.startswith("| ") and line.split("|")[1].strip().isdigit():
        title = line.rsplit("|", 2)[1].strip()
        r = next((v for t, v in by_title.items() if t.replace("—", ", ").strip() == title), None)
        line = line + f" {r['status'].replace('_', ' ') if r else '?'} |"
    elif line.startswith("## ") and line[3:4].isdigit():
        title = line.split(". ", 1)[1]
        r = next((v for t, v in by_title.items() if t.replace("—", ", ").strip() == title), None)
        if r:
            res.append(line)
            res.append("")
            res.append(f"- Now ({commit}): **{r['status'].replace('_', ' ')}** at `{r['where']}`. {r['note']}")
            continue
    res.append(line)
s = "\n".join(res)
import re
s = re.sub(r"Each finding is being re-checked\nagainst base-v1 [0-9a-f]+; the results will be added as a \"Now\" column\.",
           f"Each finding was re-checked against\nbase-v1 {commit} by one checker subagent each; see the Now column.", s)
open(md, "w").write(s)
c = {}
for r in R:
    c[r["status"]] = c.get(r["status"], 0) + 1
print(c)
