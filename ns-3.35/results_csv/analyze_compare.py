import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルのパス
file1 = "random_AP2user5_09231259.csv"
file2 = "myargo_AP2user5_09231242.csv"

# データ読み込み
df1 = pd.read_csv(file1)
df2 = pd.read_csv(file2)

# フォントサイズ設定
plt.rcParams.update({"font.size": 14})

# ======================
# 1. ImprovementRate の箱ひげ図
# ======================
improvement1 = df1["ImprovementRate"]
improvement2 = df2["ImprovementRate"]

plt.figure(figsize=(6, 5))
plt.boxplot([improvement1, improvement2],
            tick_labels=["Random", "proposed"],
            patch_artist=True,
            boxprops=dict(facecolor="lightblue"),
            medianprops=dict(color="red"),
            positions=[1, 1.5],
            widths=0.35)

means = [improvement1.mean(), improvement2.mean()]
plt.plot([1, 1.5], means, "D", color="green", label="Mean")
for x, y in zip([1, 1.5], means):
    plt.text(x, y + 0.5, f"{y:.2f}", ha="center", va="bottom", color="green", fontsize=12)

plt.title("Boxplot of Improvement Rate")
plt.ylabel("Improvement Rate (%)")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig("boxplot_improvement.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 2. TotalPositionVariance の箱ひげ図
# ======================
variance1 = df1["TotalPositionVariance"]
variance2 = df2["TotalPositionVariance"]

plt.figure(figsize=(6, 5))
plt.boxplot([variance1, variance2],
            tick_labels=["Random", "proposed"],
            patch_artist=True,
            boxprops=dict(facecolor="lightblue"),
            medianprops=dict(color="red"),
            positions=[1, 1.5],
            widths=0.35)

means = [variance1.mean(), variance2.mean()]
plt.plot([1, 1.5], means, "D", color="green", label="Mean")
for x, y in zip([1, 1.5], means):
    plt.text(x, y + 0.5, f"{y:.2f}", ha="center", va="bottom", color="green", fontsize=12)

plt.title("Boxplot of Total Position Variance")
plt.ylabel("Total Position Variance")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig("boxplot_variance.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 3. ImprovementRate のヒストグラム
# ======================
plt.figure(figsize=(6, 5))
plt.hist(improvement1, bins=20, alpha=0.6, label="Random", color="blue")
plt.hist(improvement2, bins=20, alpha=0.6, label="proposed", color="orange")
plt.title("Histogram of Improvement Rate")
plt.xlabel("Improvement Rate (%)")
plt.ylabel("Frequency")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig("hist_improvement.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 4. TotalPositionVariance のヒストグラム
# ======================
plt.figure(figsize=(6, 5))
plt.hist(variance1, bins=20, alpha=0.6, label="Random", color="blue")
plt.hist(variance2, bins=20, alpha=0.6, label="proposed", color="orange")
plt.title("Histogram of Total Position Variance")
plt.xlabel("Total Position Variance")
plt.ylabel("Frequency")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig("hist_variance.png", dpi=300, bbox_inches="tight")
plt.close()