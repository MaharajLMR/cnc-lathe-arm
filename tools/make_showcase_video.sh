#!/usr/bin/env bash
# Build the full demo from a run of tools/run_cycle_demo.sh:
#   title -> the machine -> the cell -> the pick -> the load -> end card.
# Also writes the README GIF, which is the chuck shot on its own.
#
# Usage: tools/make_showcase_video.sh <run_dir> [out_basename]
#   <run_dir> has frames/wide, frames/machine, frames/pick, frames/closeup
set -eu
RUN=${1:?usage: tools/make_showcase_video.sh <run_dir> [out_basename]}
OUT=${2:-docs/load_cycle_gazebo}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
FONT_R=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
W=960; H=640
BG=0x1b1f24

mkdir -p "$(dirname "$OUT")"

card() {  # card <out.mp4> <seconds> <title> <subtitle>
  ffmpeg -y -loglevel error -f lavfi -i "color=c=$BG:s=${W}x${H}:d=$2:r=20" -vf "\
drawtext=fontfile=$FONT:text='$3':fontcolor=white:fontsize=38:x=(w-text_w)/2:y=h/2-48,\
drawtext=fontfile=$FONT_R:text='$4':fontcolor=0xc4c9ce:fontsize=22:x=(w-text_w)/2:y=h/2+18" \
    -c:v libx264 -pix_fmt yuv420p "$1"
}

clip() {  # clip <frames_dir> <out.mp4> <caption> <fps> [first] [count]
  local sel="setpts=N/($4*TB)"
  if [ $# -ge 6 ]; then sel="select='between(n\\,$5\\,$(($5 + $6)))',setpts=N/($4*TB)"; fi
  ffmpeg -y -loglevel error -framerate "$4" -i "$1/frame_%05d.png" -vf "\
$sel,\
scale=${W}:${H}:force_original_aspect_ratio=decrease,pad=${W}:${H}:(ow-iw)/2:(oh-ih)/2:color=$BG,\
drawtext=fontfile=$FONT_R:text='$3':fontcolor=white:fontsize=22:box=1:boxcolor=${BG}cc:boxborderw=10:x=24:y=h-text_h-24" \
    -c:v libx264 -pix_fmt yuv420p -r 20 "$2"
}

n_frames() { ls "$1" | wc -l; }
PICK=$(n_frames "$RUN/pick")
CLOSE=$(n_frames "$RUN/closeup")

card "$TMP/0.mp4" 3 "cnc-lathe-arm" "loading a horizontal CNC lathe, ROS 2 Jazzy and Gazebo Harmonic"
# the machine camera is sampled a third as often, so play it slower
clip "$RUN/machine" "$TMP/1.mp4" "the machine: slant bed, 8-station turret, tailstock" 9
clip "$RUN/wide"    "$TMP/2.mp4" "the cell: 5-axis jib arm, job tray, lathe" 20
# the pick is the front of the run, the load is the back of it
clip "$RUN/pick"    "$TMP/3.mp4" "picking a billet out of the tray" 20 0 $((PICK * 2 / 3))
clip "$RUN/closeup" "$TMP/4.mp4" "sliding it into the chuck along the spindle" 20 $((CLOSE / 3)) "$CLOSE"
card "$TMP/5.mp4" 3 "MaharajLMR/cnc-lathe-arm" "rebuild of our RANE hackathon arm"

for p in 0 1 2 3 4 5; do printf "file '%s'\n" "$TMP/$p.mp4"; done > "$TMP/list.txt"
ffmpeg -y -loglevel error -f concat -safe 0 -i "$TMP/list.txt" -c copy "$OUT.mp4"

# README GIF: the chuck shot, which is the part worth looking at
ffmpeg -y -loglevel error -i "$TMP/4.mp4" \
  -vf "fps=9,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer" \
  "$OUT.gif"

ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$OUT.mp4"
ls -la "$OUT.mp4" "$OUT.gif"
