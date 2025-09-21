#!/bin/bash
# 作業したい親ディレクトリに移動してから実行

for dir in */; do
  if [[ $dir =~ ([0-9]{8}) ]]; then
    date=${BASH_REMATCH[1]}
    year=${date:0:4}
    month=${date:4:2}
    day=${date:6:2}
    target="${month}-${day}-${year}"   # 実際のフォルダ名 (例: 09-20-2025)

    # まとめ先フォルダ作成
    mkdir -p "$target"

    # フォルダを移動
    mv "$dir" "$target/"
  fi
done
