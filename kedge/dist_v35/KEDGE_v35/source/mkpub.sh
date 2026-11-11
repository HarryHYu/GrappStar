#!/bin/bash
# Viewer artifact files for one dart version: ./mkpub.sh out34 pub34  (index.html = viewer.html without the page skeleton, + parts/meshes/audit json)
set -e
cd "$(dirname "$0")"
OUT=$1; PUB=$2
mkdir -p $PUB
grep -v -E '^<!doctype html>$|^<html lang="en">$|^<head>$|^<meta charset|^<meta name="viewport"|^</head>$|^<body>$|^</body>$|^</html>$' viewer.html > $PUB/index.html
cp $OUT/parts.json $OUT/meshes.json $OUT/audit.json $PUB/
ls -la $PUB
