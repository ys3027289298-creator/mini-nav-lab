#!/usr/bin/env python3
"""Static PNG preview of the two decision curves (same data/palette as chart.html)."""
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

with open("results.json") as f:
    R = json.load(f)

sizes = ["small_32x24", "medium_64x48", "large_128x96"]
xlabels = ["small\n32x24", "medium\n64x48", "large\n128x96"]
modes = [
    ("fixed",     "#2a78d6", "fixed (total order)"),
    ("hash",      "#eb6834", "hash (salt order)"),
    ("float",     "#1baf7a", "float (strict relax)"),
    ("thread",    "#eda100", "thread (no join)"),
    ("float_tie", "#e87ba4", "float_tie (float+salt)"),
]
x = [0, 1, 2]

fig, (axA, axB) = plt.subplots(2, 1, figsize=(11, 8.2), dpi=130)
fig.patch.set_facecolor("#f9f9f7")
for ax in (axA, axB):
    ax.set_facecolor("#fcfcfb")
    ax.grid(True, color="#e1e0d9", linewidth=0.8)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.spines["left"].set_color("#c3c2b7")
    ax.spines["bottom"].set_color("#c3c2b7")
    ax.tick_params(colors="#52514e", labelsize=10)

# curve 1: determinism
for key, color, label in modes:
    vals = [R["sizes"][i]["modes"][key]["determinism_pct"] for i in range(3)]
    axA.plot(x, vals, marker="o", markersize=7, linewidth=2, color=color, label=label)
axA.set_ylim(-4, 108)
axA.yaxis.set_major_locator(MultipleLocator(25))
axA.set_xticks(x); axA.set_xticklabels(xlabels)
axA.set_ylabel("determinism %", color="#52514e")
axA.set_title("Curve 1: determinism achievement rate (zero diff frames required to pass)",
              loc="left", fontsize=12, color="#0b0b0b", pad=10)
axA.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), frameon=False, fontsize=9.5)
# annotate the 100% overlap explicitly
axA.annotate("fixed & float overlap at 100% (all replays bit-identical)",
             xy=(2, 100), xytext=(0.55, 86), fontsize=9, color="#52514e",
             arrowprops=dict(arrowstyle="->", color="#898781", lw=0.9))

# curve 2: ordering overhead
ov = [R["sizes"][i]["bench"]["overhead_pct"] for i in range(3)]
axB.plot(x, ov, marker="o", markersize=8, linewidth=2, color="#2a78d6",
         label="heap vs linear-scan overhead")
axB.axhline(0, color="#52514e", linewidth=1.3)
for xi, v in zip(x, ov):
    axB.annotate(f"{v:+.1f}%", (xi, v), textcoords="offset points",
                 xytext=(0, 12 if v >= 0 else -18), ha="center",
                 fontsize=10, fontweight="bold", color="#2a78d6")
axB.set_ylim(-32, 62)
axB.set_xticks(x); axB.set_xticklabels(xlabels)
axB.set_ylabel("overhead %", color="#52514e")
axB.set_title("Curve 2: enforced-ordering overhead (same spec, outputs cross-checked, 0 mismatch)",
              loc="left", fontsize=12, color="#0b0b0b", pad=10)
axB.legend(loc="upper right", frameon=False, fontsize=9.5)

fig.tight_layout(h_pad=3.2)
fig.savefig("curves_preview.png", bbox_inches="tight", facecolor=fig.get_facecolor())
print("wrote curves_preview.png")
