#!/bin/bash
# 使い方: ./organizecsv.sh --apply
APPLY=false
if [[ "$1" == "--apply" ]]; then
  APPLY=true
fi

shopt -s nullglob
for entry in *; do
  if [[ $entry =~ ([0-9]{8}) ]]; then
    date=${BASH_REMATCH[1]}
    year=${date:0:4}
    month=${date:4:2}
    day=${date:6:2}
    target="${month}-${day}-${year}"

    echo "Found: '$entry' -> target folder: '$target'"

    if $APPLY; then
      mkdir -p "$target"
      mv -v -- "$entry" "$target/"
    fi
  fi
done
