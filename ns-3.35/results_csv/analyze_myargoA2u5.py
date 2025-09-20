import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む
df = pd.read_csv("myargo_AP2user5.csv")

# データの基本統計量を表示
print("=== 基本統計量 ===")
print(df.describe())

# 改善率 (ImprovementRate) の箱ひげ図
plt.figure(figsize=(6, 5))
df["ImprovementRate"].plot(kind="box", vert=True)
plt.ylabel("Improvement Rate (%)")
plt.title("Distribution of Improvement Rate")
plt.grid(True, axis="y", linestyle="--", alpha=0.7)
plt.show()

# 改善率のヒストグラム
plt.figure(figsize=(6, 5))
df["ImprovementRate"].hist(bins=20)
plt.xlabel("Improvement Rate (%)")
plt.ylabel("Frequency")
plt.title("Histogram of Improvement Rate")
plt.grid(True, linestyle="--", alpha=0.7)
plt.show()
