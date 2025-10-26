#!/bin/bash
# 使い方: ./organize_png.sh --apply
# --apply を付けると実際に移動します。付けないと dry-run（確認用）

APPLY=false
if [[ "$1" == "--apply" ]]; then
  APPLY=true
fi

shopt -s nullglob
# ディレクトリ内の .png ファイルをループ
for file in *.png; do
  # ファイル名に8桁の日付が含まれるか
  if [[ $file =~ ([0-9]{8}) ]]; then
    date=${BASH_REMATCH[1]}
    year=${date:0:4}
    month=${date:4:2}
    day=${date:6:2}
    target="${month}-${day}-${year}"

    echo "Found: '$file' -> target folder: '$target'"

    if $APPLY; then
      mkdir -p "$target"
      mv -v -- "$file" "$target/"
    fi
  fi
done
