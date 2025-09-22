import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルのパス
file1 = "random_AP2user5.csv"
file2 = "myargo_AP2user5_0946.csv"

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
            tick_labels=["Random", "MyArgo"],
            patch_artist=True,
            boxprops=dict(facecolor="lightblue"),
            medianprops=dict(color="red"),
            positions=[1, 1.5],  # 箱の間隔を狭める
            widths=0.35)         # 箱の幅を小さめに

# 平均値を計算してプロット
means = [improvement1.mean(), improvement2.mean()]
plt.plot([1, 1.5], means, "D", color="green", label="Mean")  # "D" はダイヤマーカー

# 平均値の数値を表示（少し上にずらして見やすくする）
for x, y in zip([1, 1.5], means):
    plt.text(x, y + 0.5, f"{y:.2f}", ha="center", va="bottom", color="green", fontsize=12)

plt.title("Boxplot of Improvement Rate")
plt.ylabel("Improvement Rate (%)")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig("boxplot.png", dpi=300, bbox_inches="tight")
plt.close()
