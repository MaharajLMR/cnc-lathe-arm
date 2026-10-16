#!/usr/bin/env bash
# Encode recorded overhead frames into an MP4 and a README-sized GIF.
# Usage: tools/make_demo_video.sh <frames_dir> <out_basename> [speedup]
set -eu
FRAMES=$1
OUT=$2
SPEED=${3:-4}            # frames were kept at 2.5 fps sim time; play back faster
FPS=$((5 * SPEED / 2))
ffmpeg -y -loglevel error -framerate "$FPS" -i "$FRAMES/frame_%05d.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 23 "$OUT.mp4"
ffmpeg -y -loglevel error -i "$OUT.mp4" \
  -vf "fps=8,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer" \
  "$OUT.gif"
ls -la "$OUT.mp4" "$OUT.gif"
