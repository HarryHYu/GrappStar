#!/usr/bin/env python3
"""Collect every finding whose Now column says 'still open' or 'partly fixed' into review/OPEN_ITEMS.md,
grouped by owner. Reads the topic files themselves (table rows and the '- Now (...)' lines)."""
import glob
import re

items = []
for f in sorted(glob.glob("review/r*.md")):
    s = open(f).read()
    topic = f.split("/")[-1]
    hdr = re.search(r"^\| # \| Sev .*Now \(([0-9a-f]+)\) \|$", s, re.M)
    if not hdr:
        continue
    commit = hdr.group(1)
    secs = re.split(r"^## (?=\d+\. )", s, flags=re.M)
    notes = {}
    for sec in secs[1:]:
        num = int(sec.split(".", 1)[0])
        m = re.search(r"^- Now \([0-9a-f]+\): \*\*(.+?)\*\* at `(.*?)`\. (.*)$", sec, re.M)
        if m:
            notes[num] = (m.group(2), m.group(3))
    for row in re.findall(r"^\| (\d+) \| (\w+) \| `(.+?)` \| (.+?) \| (.+) \| (still open|partly fixed) \|$", s, re.M):
        num, sev, loc, owner, title, st = row
        where, note = notes.get(int(num), ("", ""))
        items.append(dict(topic=topic, num=num, sev=sev, loc=loc, owner=owner.strip(), title=title.strip(),
                          st=st, where=where, note=note, commit=commit))
order = {"critical": 0, "high": 1, "medium": 2, "low": 3}
owners = sorted({i["owner"] for i in items})
L = ["# Open review items", "",
     "Every confirmed review finding that a checker subagent found still open or only partly fixed on the newest",
     "base-v1 it read (commit in each row). Fixed and moot items are not listed; see the topic files.", "",
     f"Total: {len(items)} ({sum(i['st'] == 'still open' for i in items)} still open, "
     f"{sum(i['st'] == 'partly fixed' for i in items)} partly fixed).", ""]
for o in owners:
    mine = sorted([i for i in items if i["owner"] == o], key=lambda i: (order.get(i["sev"], 9), i["topic"]))
    L += [f"## {o} ({len(mine)})", "", "| Sev | Status | Topic # | Finding | Now at | What remains |", "|---|---|---|---|---|---|"]
    for i in mine:
        L.append(f"| {i['sev']} | {i['st']} | {i['topic']} #{i['num']} | {i['title']} | `{i['where'][:90]}` ({i['commit']}) | {i['note'][:400].replace('|', '/')} |")
    L.append("")
open("review/OPEN_ITEMS.md", "w").write("\n".join(L))
print(len(items), owners)
