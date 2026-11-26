# A3 Pareto — c_max(delta) ve cepheye uzaklik

* Kriter: en yüksek `c` — aşma ≤ 0.5% (TEST hedefleri, seeds=[0, 1, 2])
* Klasik sınır: `1/e = 0.3679` · H3 toleransı: ±0.05

| Gecikme (ms) | c_max(dps) | c_max(gov) | \|c_max-1/e\|(dps) | H3 | dps cephe uzaklığı | gov cephe uzaklığı |
|---|---|---|---|---|---|---|
| 30 | 0.3 | 0.6 | 0.068 | FAIL | 1.6% | 0.0% |
| 60 | 0.3 | 0.6 | 0.068 | FAIL | 8.7% | 0.0% |
| 100 | 0.35 | 0.6 | 0.018 | PASS | 0.0% | 0.0% |
| 150 | 0.35 | 0.6 | 0.018 | PASS | 3.1% | 0.0% |
| 200 | 0.35 | 0.6 | 0.018 | PASS | 5.2% | 0.0% |

> H3: tüm gecikmelerde `|c_max(dps) − 1/e| ≤ 0.05` ise desteklenir; aksi hâlde yasa `c(δ)` düzeltmesiyle güncellenir veya iddia daraltılır (kapı G1, `TODO_dogru_yol.md`).
