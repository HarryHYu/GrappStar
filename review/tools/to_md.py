#!/usr/bin/env python3
"""Turn a review workflow result (JSON) into review/<topic>.md.

Usage: python3 review/tools/to_md.py result.json out.md "Title" "base commit"
The JSON is {"key": ..., "findings": [...]} as returned by the review workflow.
"""
import json
import sys

ORDER = {"critical": 0, "high": 1, "medium": 2, "low": 3}


def clean(s):
    # Harry's rule: no em dashes anywhere.
    return str(s).replace("—", ", ").replace("–", "-").strip()


def main():
    src, out, title, base = sys.argv[1:5]
    data = json.load(open(src))
    fs = data["findings"]
    kept = [f for f in fs if f.get("kept")]
    dropped = [f for f in fs if not f.get("kept")]
    for f in kept:
        # Use the skeptics' severity if both agreed it is real and both rated it lower.
        sev = [v["severity_if_real"] for v in f.get("verdicts", []) if not v.get("refuted")]
        if sev and all(ORDER[s] > ORDER[f["severity"]] for s in sev):
            f["severity_adj"] = min(sev, key=lambda s: ORDER[s])
        else:
            f["severity_adj"] = f["severity"]
    kept.sort(key=lambda f: (ORDER[f["severity_adj"]], f["file"], f["line"]))
    L = [f"# {clean(title)}", "",
         f"Reviewed at base-v1 commit {base}. Method: independent finder subagents, one per lens; every finding of",
         "medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than",
         "two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics",
         "agreed it is real but rated it lower.", "",
         f"Confirmed: {len(kept)}. Refuted and dropped: {len(dropped)}.", "",
         "| # | Sev | File:line | Owner | Finding |", "|---|---|---|---|---|"]
    for i, f in enumerate(kept, 1):
        L.append(f"| {i} | {f['severity_adj']} | `{f['file']}:{f['line']}` | {clean(f['owner'])} | {clean(f['title'])} |")
    L.append("")
    for i, f in enumerate(kept, 1):
        votes = f.get("verdicts", [])
        vtxt = "unverified (low)" if not f.get("verified") else \
            f"{sum(1 for v in votes if not v['refuted'])} of {len(votes)} skeptics confirmed"
        L += [f"## {i}. {clean(f['title'])}", "",
              f"- File: `{f['file']}:{f['line']}`",
              f"- Severity: {f['severity_adj']} (claimed {f['severity']}); {vtxt}",
              f"- Owner: {clean(f['owner'])}", "",
              f"Scenario: {clean(f['scenario'])}", "",
              f"Evidence: {clean(f['evidence'])}", "",
              f"Fix: {clean(f['fix'])}", ""]
        for v in votes:
            tag = "refuted" if v["refuted"] else "confirmed"
            L.append(f"> Skeptic ({tag}): {clean(v['reasoning'])[:600]}")
            L.append(">")
        L.append("")
    if dropped:
        L += ["## Refuted (dropped)", ""]
        for f in dropped:
            L.append(f"- `{f['file']}:{f['line']}` {clean(f['title'])}")
        L.append("")
    open(out, "w").write("\n".join(L))
    print(f"{out}: {len(kept)} kept, {len(dropped)} dropped")


if __name__ == "__main__":
    main()
