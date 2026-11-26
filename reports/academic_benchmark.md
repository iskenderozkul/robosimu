# Standart Akademik Benchmark Raporu (5 Senaryo × 6 Kontrolcü + 2 Ablasyon)

* **Model:** `human_arm_like_v1` (`posture=retracted_compact`, tohumlar `seeds=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]`)
* **Dış bağlantı:** `https://github.com/iskenderozkul/robosimu` (kod, model, bu raporun üretildiği komut)
* **Ölçütler:** Son hata $e_f$ (mm, %95 bootstrap GA, 10 000 tekrar, seed=42), Hedef aşma (%), Yerleşme süresi $T_s$ (s, tolerans $\varepsilon=5\text{ mm}$; yerleşmeyen koşul süre tavanında skorlanır), ISO 9241-9 / MacKenzie Fitts $TP = ID_e / T_{\text{eff}}$ (bit/s, $W_t = 40\text{ mm}$, $W_e = \max(W_t, 4.133\sigma_e)$), $E_{\text{init}}$ (mm), $\%T_{\text{comp}}$, minimum Yoshikawa manipülabilite $w_{\min}$ ve tepe tork $\tau_{\max}$ (Nm).

## Kabul Kriterleri ve İstatistiksel Protokol (Önceden Tanımlı — Pre-defined)

1. **Örneklem:** Her hücrede $n=10$ bağımsız gürültü seed'i ($0\ldots n-1$); aynı seed'ler tüm kontrolcülere uygulanır (eşleşik tasarım).
2. **Başarı ölçütü:** $T_s$ sonlu (5 mm banda oturma) **ve** $e_f \le 5\text{ mm}$; yerleşmeyenler $T_s = \text{süre tavanı}$ skorlanır (Fitts $T_{\text{eff}}$ ile aynı kural).
3. **Güven aralıkları:** $e_f$ için yüzdelik bootstrap %95 GA (10 000 tekrar, seed=42); oranlar için Wilson GA.
4. **Karşılaştırmalar:** hdap vs diğer 5 kontrolcü, 5 senaryoda 25 eşleşik karşılaştırma; iki-yönlü log-ortalama permütasyon testi (10 000 permütasyon, seed=42); **Holm step-down** düzeltmesi ($\alpha=0.05$).
5. **Sabitler:** $dt=1\text{ kHz}$ iç servo, `control_dt=10 ms`, `v_max=0.60 m/s`, `eps=4 mm`, geometri ve gürültü parametreleri `robosimu/benchmark.py`'de dondurulmuştur; değişiklik = yeni sürüm.

## Doğrulanabilir Tekrar Çalıştırma (GitHub / CLI & Tarayıcı)

```bash
git clone https://github.com/iskenderozkul/robosimu.git
cd robosimu
python -m robosimu.cli benchmark models/human_arm.json --out reports --seeds 10
python -m robosimu.cli workbench models/human_arm.json --out figures/human_arm_workbench.html
```

## S1: Nominal Binoküler Stereo (60 ms, 0.25 px)

> Nominal visuomotor reach from retracted_compact under 60 ms visual delay.

| Kontrolcü | Başlangıç→Hedef (cm) | Son Hata $e_f$ (mm) [GA95] | Aşma (%) | $T_s$ (s) | Oturma % | Fitts $TP$ (bit/s) | $E_{\text{init}}$ (mm) | $\%T_{\text{comp}}$ | $w_{\min}$ | Tepe Tork (Nm) | Bel / Paralaks |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `hdap` | 27.9 | 0.81 ± 0.26 [0.64–0.97] | 1.33 ± 0.13 | 0.92 | 100% | 3.34 | 147.0 | 71% | 0.0326 | 25.6 | bel=+0.0°, B_boy=64.0mm |
| `hlsl` | 27.9 | 0.97 ± 0.36 [0.75–1.19] | 3.44 ± 0.09 | 1.28 | 100% | 2.42 | 161.3 | 81% | 0.0326 | 25.6 | bel=+0.0°, B_boy=64.0mm |
| `governor` | 27.9 | 0.78 ± 0.33 [0.58–0.99] | 3.53 ± 0.09 | 1.25 | 100% | 2.47 | 160.8 | 80% | 0.0326 | 27.7 | bel=+0.0°, B_boy=64.0mm |
| `pid` | 27.9 | 1.56 ± 0.96 [1.03–2.21] | 8.25 ± 0.20 | 1.69 | 100% | 1.82 | 155.9 | 85% | 0.0326 | 27.8 | bel=+0.0°, B_boy=64.0mm |
| `lqr` | 27.9 | 39.64 ± 17.87 [28.82–50.61] | 8.91 ± 1.32 | > 3.0 (N/A) | 0% | 0.53 | 146.5 | 91% | 0.0326 | 28.8 | bel=+0.0°, B_boy=64.0mm |
| `fixed` | 27.9 | 1.16 ± 0.65 [0.80–1.58] | 8.70 ± 0.21 | 1.63 | 100% | 1.86 | 155.2 | 85% | 0.0326 | 27.8 | bel=+0.0°, B_boy=64.0mm |

## S2: Yüksek Gecikme Stresi (120 ms, 0.35 px — Lambert-W)

> High-latency stress test verifying kappa <= 1/e monotone damping without retuning.

| Kontrolcü | Başlangıç→Hedef (cm) | Son Hata $e_f$ (mm) [GA95] | Aşma (%) | $T_s$ (s) | Oturma % | Fitts $TP$ (bit/s) | $E_{\text{init}}$ (mm) | $\%T_{\text{comp}}$ | $w_{\min}$ | Tepe Tork (Nm) | Bel / Paralaks |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `hdap` | 27.9 | 0.62 ± 0.31 [0.44–0.82] | 0.37 ± 0.08 | 0.75 | 100% | 4.00 | 143.7 | 61% | 0.0326 | 22.0 | bel=+0.0°, B_boy=64.0mm |
| `hlsl` | 27.9 | 0.64 ± 0.27 [0.48–0.81] | 0.41 ± 0.06 | 0.75 | 100% | 4.00 | 169.8 | 67% | 0.0326 | 22.1 | bel=+0.0°, B_boy=64.0mm |
| `governor` | 27.9 | 0.61 ± 0.32 [0.42–0.81] | 0.99 ± 0.08 | 0.87 | 100% | 3.50 | 149.9 | 70% | 0.0326 | 27.9 | bel=+0.0°, B_boy=64.0mm |
| `pid` | 27.9 | 31.25 ± 18.57 [19.46–42.53] | 20.91 ± 0.16 | 3.49 | 10% | 0.54 | 23.4 | 34% | 0.0326 | 32.6 | bel=+0.0°, B_boy=64.0mm |
| `lqr` | 27.9 | 77.60 ± 3.33 [75.28–79.15] | 20.56 ± 0.66 | > 3.5 (N/A) | 0% | 0.26 | 90.5 | 65% | 0.0326 | 30.1 | bel=+0.0°, B_boy=64.0mm |
| `fixed` | 27.9 | 30.03 ± 11.54 [23.11–37.49] | 21.33 ± 0.17 | > 3.5 (N/A) | 0% | 0.51 | 23.1 | 40% | 0.0326 | 32.9 | bel=+0.0°, B_boy=64.0mm |

## S3: Sınır / Tekillik Yakını Uzak Uzanma (human_wdls vs plain_dls)

> Near-boundary extended reach preserving manipulability margin w_min.

| Kontrolcü | Başlangıç→Hedef (cm) | Son Hata $e_f$ (mm) [GA95] | Aşma (%) | $T_s$ (s) | Oturma % | Fitts $TP$ (bit/s) | $E_{\text{init}}$ (mm) | $\%T_{\text{comp}}$ | $w_{\min}$ | Tepe Tork (Nm) | Bel / Paralaks |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `hdap` | 41.7 | 1.49 ± 1.11 [0.86–2.21] | 0.44 ± 0.17 | 1.84 | 100% | 2.65 | 293.0 | 81% | 0.0326 | 34.3 | bel=+0.0°, B_boy=64.0mm |
| `hlsl` | 41.7 | 1.39 ± 0.96 [0.80–2.01] | 0.60 ± 0.12 | 2.22 | 100% | 2.08 | 293.0 | 85% | 0.0326 | 34.4 | bel=+0.0°, B_boy=64.0mm |
| `governor` | 41.7 | 1.39 ± 0.96 [0.80–2.01] | 0.60 ± 0.12 | 2.22 | 100% | 2.08 | 293.0 | 85% | 0.0326 | 34.4 | bel=+0.0°, B_boy=64.0mm |
| `pid` | 41.7 | 2.81 ± 2.01 [1.72–4.14] | 4.19 ± 0.20 | 3.33 | 90% | 1.05 | 291.8 | 93% | 0.0326 | 34.7 | bel=+0.0°, B_boy=64.0mm |
| `lqr` | 41.7 | 43.90 ± 13.07 [35.87–51.88] | 7.05 ± 1.06 | > 4.5 (N/A) | 0% | 0.40 | 283.4 | 94% | 0.0326 | 41.8 | bel=+0.0°, B_boy=64.0mm |
| `fixed` | 41.7 | 2.38 ± 2.25 [1.26–3.92] | 4.40 ± 0.19 | 3.60 | 90% | 0.99 | 291.9 | 93% | 0.0326 | 34.2 | bel=+0.0°, B_boy=64.0mm |

## S4: Tek Göz Aktif Boyun Paralaksı vs Sabit Tek Göz (+%28 Ölçek Hatası)

> Monocular reach with +28% initial target size prior bias: active neck glide vs static eye.

| Kontrolcü | Başlangıç→Hedef (cm) | Son Hata $e_f$ (mm) [GA95] | Aşma (%) | $T_s$ (s) | Oturma % | Fitts $TP$ (bit/s) | $E_{\text{init}}$ (mm) | $\%T_{\text{comp}}$ | $w_{\min}$ | Tepe Tork (Nm) | Bel / Paralaks |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `hdap` | 27.9 | 1.77 ± 0.53 [1.46–2.12] | 2.70 ± 0.29 | 1.11 | 100% | 2.71 | 150.5 | 73% | 0.0326 | 31.8 | bel=+0.0°, B_boy=58.8mm |
| `hlsl` | 27.9 | 1.59 ± 0.50 [1.30–1.91] | 2.79 ± 0.36 | 1.09 | 100% | 2.76 | 159.7 | 74% | 0.0326 | 31.8 | bel=+0.0°, B_boy=58.8mm |
| `governor` | 27.9 | 1.59 ± 0.50 [1.30–1.91] | 2.79 ± 0.36 | 1.09 | 100% | 2.76 | 159.7 | 74% | 0.0326 | 31.8 | bel=+0.0°, B_boy=58.8mm |
| `pid` | 27.9 | 1.04 ± 0.44 [0.77–1.31] | 7.87 ± 0.28 | 1.26 | 100% | 2.42 | 58.3 | 63% | 0.0326 | 31.6 | bel=+0.0°, B_boy=58.8mm |
| `lqr` | 27.9 | 48.41 ± 16.79 [36.77–57.01] | 11.03 ± 0.76 | > 4.0 (N/A) | 0% | 0.35 | 144.1 | 91% | 0.0326 | 30.4 | bel=+0.0°, B_boy=58.8mm |
| `fixed` | 27.9 | 0.91 ± 0.55 [0.61–1.27] | 8.22 ± 0.30 | 1.23 | 100% | 2.45 | 58.1 | 62% | 0.0326 | 31.6 | bel=+0.0°, B_boy=58.8mm |
| `hdap_mono_static` | 27.9 | 171.24 ± 0.78 [170.74–171.69] | 13.03 ± 0.46 | > 4.0 (N/A) | 0% | 0.12 | 168.2 | 94% | 0.0326 | 30.5 | bel=+0.0°, B_boy=0.0mm |

## S5: Yanal Küre-Dışı Senkron Bel + Kol Uzanma ([0.42, -0.42, 0.86] m)

> Contralateral target beyond static shoulder envelope requiring synchronized torso yaw.

| Kontrolcü | Başlangıç→Hedef (cm) | Son Hata $e_f$ (mm) [GA95] | Aşma (%) | $T_s$ (s) | Oturma % | Fitts $TP$ (bit/s) | $E_{\text{init}}$ (mm) | $\%T_{\text{comp}}$ | $w_{\min}$ | Tepe Tork (Nm) | Bel / Paralaks |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `hdap` | 35.7 | 0.80 ± 0.31 [0.61–0.99] | 0.41 ± 0.14 | 1.02 | 100% | 3.44 | 224.9 | 73% | 0.0326 | 32.9 | bel=-58.0°, B_boy=64.0mm |
| `hlsl` | 35.7 | 1.33 ± 0.89 [0.86–1.94] | 0.59 ± 0.10 | 1.61 | 100% | 2.24 | 224.9 | 82% | 0.0326 | 33.6 | bel=-58.0°, B_boy=64.0mm |
| `governor` | 35.7 | 1.33 ± 0.89 [0.86–1.94] | 0.59 ± 0.10 | 1.61 | 100% | 2.24 | 224.9 | 82% | 0.0326 | 33.6 | bel=-58.0°, B_boy=64.0mm |
| `pid` | 35.7 | 11.80 ± 5.35 [8.58–15.23] | 5.16 ± 0.22 | 3.36 | 10% | 0.86 | 216.7 | 92% | 0.0326 | 36.3 | bel=-58.0°, B_boy=64.0mm |
| `lqr` | 35.7 | 29.33 ± 19.09 [18.10–41.57] | 6.98 ± 1.14 | 3.49 | 10% | 0.64 | 220.8 | 92% | 0.0326 | 40.2 | bel=-58.0°, B_boy=64.0mm |
| `fixed` | 35.7 | 10.49 ± 4.26 [8.06–13.32] | 5.42 ± 0.19 | > 3.5 (N/A) | 0% | 0.89 | 215.4 | 92% | 0.0326 | 36.3 | bel=-58.0°, B_boy=64.0mm |
| `hdap_no_waist` | 65.4 | 51.92 ± 0.01 [51.91–51.92] | 0.00 ± 0.00 | > 3.5 (N/A) | 0% | 0.58 | 525.5 | 93% | 0.0034 | 51.4 | bel=+0.0°, B_boy=64.0mm |

## İstatistiksel Karşılaştırma — hdap vs Klasik Kontrolcüler

Hareket süresi $T_s$ üzerinden iki-yönlü log-ortalama permütasyon testi (10 000 permütasyon, seed=42; yerleşmeyen koşullar süre tavanında skorlanır), **Holm step-down** düzeltmesi ($\alpha=0.05$, 25 eşleşik karşılaştırma):

| Senaryo | hdap vs | $p$ (ham) | $p$ (Holm) | Sonuç |
|---|---|---|---|---|
| S1 | `hlsl` | 0.0001 | 0.0025 | **anlamlı** |
| S1 | `governor` | 0.0001 | 0.0025 | **anlamlı** |
| S1 | `pid` | 0.0001 | 0.0025 | **anlamlı** |
| S1 | `lqr` | 0.0001 | 0.0025 | **anlamlı** |
| S1 | `fixed` | 0.0001 | 0.0025 | **anlamlı** |
| S2 | `hlsl` | 0.8009 | 1.0000 | anlamsız |
| S2 | `governor` | 0.0001 | 0.0025 | **anlamlı** |
| S2 | `pid` | 0.0001 | 0.0025 | **anlamlı** |
| S2 | `lqr` | 0.0001 | 0.0025 | **anlamlı** |
| S2 | `fixed` | 0.0001 | 0.0025 | **anlamlı** |
| S3 | `hlsl` | 0.4040 | 1.0000 | anlamsız |
| S3 | `governor` | 0.4040 | 1.0000 | anlamsız |
| S3 | `pid` | 0.0017 | 0.0170 | **anlamlı** |
| S3 | `lqr` | 0.0001 | 0.0025 | **anlamlı** |
| S3 | `fixed` | 0.0012 | 0.0132 | **anlamlı** |
| S4 | `hlsl` | 0.5282 | 1.0000 | anlamsız |
| S4 | `governor` | 0.5282 | 1.0000 | anlamsız |
| S4 | `pid` | 0.0303 | 0.1818 | anlamsız |
| S4 | `lqr` | 0.0001 | 0.0025 | **anlamlı** |
| S4 | `fixed` | 0.0239 | 0.1673 | anlamsız |
| S5 | `hlsl` | 0.0027 | 0.0243 | **anlamlı** |
| S5 | `governor` | 0.0027 | 0.0243 | **anlamlı** |
| S5 | `pid` | 0.0001 | 0.0025 | **anlamlı** |
| S5 | `lqr` | 0.0001 | 0.0025 | **anlamlı** |
| S5 | `fixed` | 0.0001 | 0.0025 | **anlamlı** |

> Yorumlama: $p_{\text{Holm}} \le 0.05$ olan karşılaştırmalarda hdap, ilgili kontrolcüye kıyasla **istatistiksel olarak anlamlı** farklı yerleşme süresi gösterir. Bu tablo çoklu karşılaştırma düzeltmesi uygulanmış aile içi bir testtir; tekil $p$ değerlerine değil, Holm sütununa bakın.
