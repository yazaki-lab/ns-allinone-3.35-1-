import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルのパス
file1 = "random_AP2user5.csv"
file2 = "myargo_AP2user5_upglade.csv"

# データ読み込み
df1 = pd.read_csv(file1)
df2 = pd.read_csv(file2)

# 改善率データを抽出
improvement1 = df1["ImprovementRate"]
improvement2 = df2["ImprovementRate"]

# フォントサイズ設定
plt.rcParams.update({"font.size": 14})

# ======================
# 1. 箱ひげ図
# ======================
plt.figure(figsize=(6, 5))
plt.boxplot([improvement1, improvement2],
            labels=["Random", "MyArgo"],
            patch_artist=True,
            boxprops=dict(facecolor="lightblue"),
            medianprops=dict(color="red"),
            positions=[1, 1.5],  # 箱の間隔を狭める
            widths=0.35)         # 箱の幅を小さめに
plt.title("Boxplot of Improvement Rate")
plt.ylabel("Improvement Rate (%)")
plt.grid(True, linestyle="--", alpha=0.6)
plt.savefig("boxplot.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 2. ヒストグラム
# ======================
plt.figure(figsize=(6, 5))
plt.hist(improvement1, bins=20, alpha=0.6, label="Random", color="blue")
plt.hist(improvement2, bins=20, alpha=0.6, label="MyArgo", color="orange")
plt.title("Histogram of Improvement Rate")
plt.xlabel("Improvement Rate (%)")
plt.ylabel("Frequency")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.6)
plt.savefig("histogram.png", dpi=300, bbox_inches="tight")
plt.close()
