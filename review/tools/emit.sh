#!/bin/bash
# emit.sh <task-output-file> <topic-file-name> "<title>" <base-commit-reviewed>
set -e
SP=/tmp/claude-0/-home-user-GrappStar/19f18bb2-0910-575b-b022-2f967c765c97/scratchpad
python3 -c "import json,sys; json.dump(json.load(open('$1'))['result'], open('$SP/cur.json','w'))"
python3 review/tools/to_md.py $SP/cur.json "review/$2" "$3" "${4:?base commit the review ran on}"
