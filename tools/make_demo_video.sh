#!/usr/bin/env bash
# Encode a recorded run into an MP4 and a README-sized GIF: the wide shot of
# the cell, then the close-up of the billet going into the chuck.
#
# Usage: tools/make_demo_video.sh <run_dir> <out_basename> [speedup]
#   <run_dir> is the output of tools/run_cycle_demo.sh (has frames/wide and
#   frames/closeup). An older frames dir with the PNGs straight in it still
#   works and is encoded as a single shot.
set -eu
RUN=$1
OUT=$2
SPEED=${3:-4}            # frames are kept at 2.5 fps of sim time
FPS=$((5 * SPEED / 2))
W=960; H=640
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

clip() {  # clip <frames_dir> <out.mp4>
  ffmpeg -y -loglevel error -framerate "$FPS" -i "$1/frame_%05d.png" \
    -vf "scale=${W}:${H}:force_original_aspect_ratio=decrease,pad=${W}:${H}:(ow-iw)/2:(oh-ih)/2:color=0x202428" \
    -c:v libx264 -pix_fmt yuv420p -crf 23 "$2"
}

if [ -d "$RUN/wide" ] && [ -d "$RUN/closeup" ]; then
  clip "$RUN/wide" "$TMP/0.mp4"
  clip "$RUN/closeup" "$TMP/1.mp4"
  printf "file '%s'\nfile '%s'\n" "$TMP/0.mp4" "$TMP/1.mp4" > "$TMP/list.txt"
  ffmpeg -y -loglevel error -f concat -safe 0 -i "$TMP/list.txt" -c copy "$OUT.mp4"
  # the GIF is the close-up only: it is the shot that shows the load
  GIF_SRC="$TMP/1.mp4"
else
  clip "$RUN" "$OUT.mp4"
  GIF_SRC="$OUT.mp4"
fi

ffmpeg -y -loglevel error -i "$GIF_SRC" \
  -vf "fps=8,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer" \
  "$OUT.gif"
ls -la "$OUT.mp4" "$OUT.gif"
