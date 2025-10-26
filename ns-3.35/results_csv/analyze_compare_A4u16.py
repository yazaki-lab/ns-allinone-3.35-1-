import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# タイムスタンプ作成（例: 20250923_145930）
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

# CSVファイルのパス
file1 = "random_AP4user100_09251413.csv"
file2 = "myargo_AP4user100_classroom.csv"

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
            tick_labels=["Random", "Proposed"],
            patch_artist=True,
            boxprops=dict(facecolor="lightblue"),
            medianprops=dict(color="red"),
            positions=[1, 1.5],
            widths=0.35)

means = [improvement1.mean(), improvement2.mean()]
plt.plot([1, 1.5], means, "D", color="green", label="Mean")
for x, y in zip([1, 1.5], means):
    plt.text(x, y + 0.5, f"{y:.2f}", ha="center", va="bottom", color="green", fontsize=12)

plt.ylabel("Improvement Rate (%)")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig(f"boxplot_improvement_{timestamp}.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 2. MovementDistance の箱ひげ図
# ======================
distance1 = df1["MovementDistance"]
distance2 = df2["MovementDistance"]

plt.figure(figsize=(6, 5))
plt.boxplot([distance1, distance2],
            tick_labels=["Random", "Proposed"],
            patch_artist=True,
            boxprops=dict(facecolor="lightblue"),
            medianprops=dict(color="red"),
            positions=[1, 1.5],
            widths=0.35)

means = [distance1.mean(), distance2.mean()]
plt.plot([1, 1.5], means, "D", color="green", label="Mean")
for x, y in zip([1, 1.5], means):
    plt.text(x, y + 0.5, f"{y:.2f}", ha="center", va="bottom", color="green", fontsize=12)

plt.ylabel("Travel Distance")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.savefig(f"boxplot_travel_distance_{timestamp}.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 3. ImprovementRate のヒストグラム
# ======================
plt.figure(figsize=(6, 5))
plt.hist(improvement1, bins=30, alpha=0.5, label="Random", color="blue")
plt.hist(improvement2, bins=30, alpha=0.5, label="Proposed", color="orange")
plt.xlabel("Improvement Rate (%)")
plt.ylabel("Frequency")
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend()
plt.savefig(f"hist_improvement_{timestamp}.png", dpi=300, bbox_inches="tight")
plt.close()

# ======================
# 4. MovementDistance のヒストグラム
# ======================
plt.figure(figsize=(6, 5))
plt.hist(distance1, bins=30, alpha=0.5, label="Random", color="blue")
plt.hist(distance2, bins=30, alpha=0.5, label="Proposed", color="orange")
plt.xlabel("Travel Distance")
plt.ylabel("Frequency")
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend()
plt.savefig(f"hist_travel_distance_{timestamp}.png", dpi=300, bbox_inches="tight")
plt.close()
