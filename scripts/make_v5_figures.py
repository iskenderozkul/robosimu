"""Build v5 figures and summary table from reports/v5_results.jsonl."""
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

os.makedirs("figures/v5", exist_ok=True)
df = pd.DataFrame([json.loads(l) for l in open("reports/v5_results.jsonl")])
df["ok"] = df.settled & (df.final_mm <= 5)
df["mt"] = df.mt_s.fillna(5.0)

rng = np.random.default_rng(42)
rows = []
for (dl, c), g in df.groupby(["delay", "ctl"]):
    bs = [np.mean(rng.choice(g.os_pct.values, len(g))) for _ in range(4000)]
    lo, hi = np.percentile(bs, [2.5, 97.5])
    rows.append(dict(delay=dl, ctl=c, n=len(g), success=g.ok.mean(), os_mean=g.os_pct.mean(),
                     os_ci95_lo=lo, os_ci95_hi=hi, os_max=g.os_pct.max(),
                     mt_median=g.mt.median(), final_median_mm=g.final_mm.median()))
pd.DataFrame(rows).round(3).to_csv("reports/v5_summary_table.csv", index=False)

lab = {"pg": "Perceptual 1/e governor (no oracle)", "pg_mis": "same, delay under-reported x0.5",
       "gov": "Governor (oracle)", "pid30": "PID tuned @30 ms", "fixed30": "Fixed gain tuned @30 ms"}
fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
for c in lab:
    sub = df[df.ctl == c]
    if sub.empty:
        continue
    g = sub.groupby("delay")
    d = np.array(sorted(sub.delay.unique())) * 1000
    ax[0].plot(d, g.ok.mean().values * 100, "o-", label=lab[c])
    ax[1].plot(d, g.os_pct.max().values, "o-")
    ax[2].plot(d, g.mt.median().values, "o-")
ax[1].set_yscale("log")
for a, t in zip(ax, ["Success (settled <=5 mm) %", "Max overshoot % (log)", "Median movement time (s)"]):
    a.set_xlabel("Visual delay (ms)"); a.set_title(t, fontsize=10); a.grid(alpha=.3)
ax[0].legend(fontsize=7, loc="lower left")
plt.tight_layout(); plt.savefig("figures/v5/fig_v5_benchmark.png", dpi=180); plt.close()

names = {"pg": "full", "abl_n1": "- averaging", "abl_broy": "+ Broyden",
         "abl_nodelay": "- delay term", "abl_noL": "- servo lag"}
A = df[df.delay.isin([0.06, 0.15]) & df.ctl.isin(names)]
if not A.empty:
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6)); x = np.arange(len(names)); w = .38
    for j, dl in enumerate([0.06, 0.15]):
        s = A[(A.delay == dl) & (A.seed == 0)].groupby("ctl")
        ax[0].bar(x + (j - .5) * w, [s.os_pct.max().get(k, np.nan) for k in names], w, label=f"{int(dl*1000)} ms")
        ax[1].bar(x + (j - .5) * w, [s.mt.median().get(k, np.nan) for k in names], w)
    for a, t in zip(ax, ["Max overshoot %", "Median movement time (s)"]):
        a.set_xticks(x); a.set_xticklabels(names.values(), fontsize=8); a.set_title(t, fontsize=10)
    ax[0].legend(fontsize=8); plt.tight_layout(); plt.savefig("figures/v5/fig_v5_ablation.png", dpi=180); plt.close()

t = np.linspace(0, 8, 4000); dt = t[1] - t[0]; lag = int(1 / dt)
fig, ax = plt.subplots(figsize=(5, 3.4))
for kd in (0.2, 1 / math.e, 0.6, 1.2):
    x = np.ones_like(t)
    for i in range(1, len(t)):
        x[i] = x[i - 1] - dt * kd * (x[i - 1 - lag] if i - 1 - lag >= 0 else 1.0)
    ax.plot(t, x, label=f"k*delta = {kd:.3f}")
ax.axhline(0, color="k", lw=.6); ax.set_xlabel("t / delta"); ax.set_ylabel("e(t)/e0"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/v5/fig_v5_theory.png", dpi=180); plt.close()
print("figures/v5 and reports/v5_summary_table.csv written")
