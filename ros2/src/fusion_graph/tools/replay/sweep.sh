#!/bin/bash
# Parameter sweep on the replay harness. usage: sweep.sh <overlay> <bag> <tag-prefix> "<label>|<extra -p args>" ...
SP=/private/tmp/claude-501/-Users-cedric-Dev-git-mowglinext/dc127f49-0c42-4bf1-b152-f0c95d5d0747/scratchpad
OVL=$1; BAG=$2; PFX=$3; shift 3
IMG=ghcr.io/mowglinext/mowglinext/mowgli-ros2:feat-lidar-map-anchor
OUT="$SP/sweep_${PFX}.log"; : > "$OUT"
i=0
for cfg in "$@"; do
  i=$((i+1)); label="${cfg%%|*}"; extra="${cfg#*|}"
  tag="${PFX}_$i"
  docker run --rm --network none --name "sw-$tag" -e BAG="$BAG" -e ANCHOR=true -e RATE=1 -e TAG="$tag" -e OVERLAY="$OVL" \
    -e PLAY_EXTRA_TOPICS="${PLAY_EXTRA_TOPICS:-}" -e EXTRA="-p lidar_anchor_shadow_mode:=true $extra" -v "$SP:/data" "$IMG" bash /data/replay.sh 2>&1 \
    | grep -E "candidates under|max \||at RTK|node cpu" | sed "s/^/[$label] /" | tee -a "$OUT" &
  # at most 3 concurrent replays
  while [ "$(jobs -rp | wc -l)" -ge 3 ]; do sleep 10; done
done
wait
echo "=== sweep $PFX done ==="; cat "$OUT"
