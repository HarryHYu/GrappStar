#!/bin/bash
# GrappStar mission render on a Mac (Apple Silicon, Blender 5.3).
# Double-click in Finder (first time: right-click > Open), or in Terminal:
#   bash render_mac.command [hd720|cy720|final|uhd|preview|hero] [eevee|cycles] [start end]   (default hd720 eevee;
#   cy720 = Cycles on the GPU at 720p with the camera look, frames_cy720/)
cd "$(dirname "$0")"
B="/Applications/Blender.app/Contents/MacOS/Blender"
if [ ! -x "$B" ]; then echo "Blender is not in Applications. Install Blender 5.3 (Apple Silicon) from blender.org first."; read -p "Press Enter to close"; exit 1; fi
F="grappstar_mission.blend"
if [ ! -f "$F" ]; then echo "Put grappstar_mission.blend in this folder first: $(pwd)"; read -p "Press Enter to close"; exit 1; fi
PRESET="${1:-hd720}"; ENGINE="${2:-eevee}"
EXTRA=""; [ -n "$3" ] && EXTRA="--start $3"; [ -n "$4" ] && EXTRA="$EXTRA --end $4"
mkdir -p frames
caffeinate -i "$B" -b "$F" -P preset.py -- --preset "$PRESET" --engine "$ENGINE" $EXTRA --out "$(pwd)/frames/"
echo "Done. Frames and grappstar_mission.mp4 are in $(pwd)/frames"
read -p "Press Enter to close"
