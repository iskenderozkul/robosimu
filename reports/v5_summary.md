# v5 — Oracle-sız Algısal 1/e Governor: benchmark ve ablasyon

Tarih: 23.11.2026 · Kod: `robosimu/perceptual_governor.py` · Betikler: `scripts/run_v5.py`, `scripts/make_v5_figures.py` · Ham veri: `reports/v5_results.jsonl` · Özet tablo: `reports/v5_summary_table.csv` · Şekiller: `figures/v5/`

## Kurulum
Model `six_axis_arm.json`, Q0 = [0, −0.3, 0.6, 0, 0.6, 0]; 1 kHz eklem servosu (wn = 10 rad/s, ζ = 1); stereo piksel gürültüsü 0.25 px, disparite 0.35 px; 12 bağımsız TEST hedefi (4–10 cm; seed 1001), ayrık TUNE kümesi (seed 2002); δ ∈ {30, 60, 100, 150, 200} ms; süre 5 s; başarı = 5 mm'ye yerleşme. pg / pg_mis / gov için 2 gürültü tohumu (n = 24/hücre); pid30 / fixed30 ve ablasyon için 1 tohum (n = 12).

## Ana sonuç (başarı / en büyük aşma / medyan MT)

| δ (ms) | pg (oracle yok) | pg_mis (δ yarısı) | gov (oracle) | pid30 | fixed30 |
|---|---|---|---|---|---|
| 30 | 100 % / 3.5 % / 1.06 s | 100 % / 3.9 % / 0.94 s | 92 % / 18.6 % / 3.88 s | 83 % / 29.7 % | 83 % / 29.7 % |
| 60 | 100 % / 2.8 % / 1.30 s | 100 % / 3.7 % / 1.02 s | 100 % / 9.9 % / 0.42 s | 58 % / 53.3 % | 67 % / 53.7 % |
| 100 | 100 % / 2.0 % / 1.67 s | 100 % / 3.3 % / 1.17 s | 100 % / 5.3 % / 0.62 s | 17 % / 134.8 % | 17 % / 135.0 % |
| 150 | 100 % / 0.9 % / 2.12 s | 100 % / 3.0 % / 1.36 s | 100 % / 3.7 % / 0.96 s | 0 % / 211 % | 0 % / 213 % |
| 200 | 92 % / 0.6 % / 2.52 s | 100 % / 2.5 % / 1.52 s | 100 % / 3.0 % / 1.27 s | 0 % / 251 % | 0 % / 251 % |

pg: 120 uzanmanın 118'i başarılı. İki başarısızlık aynı hedefte (200 ms, iki tohum), 6.1 ve 6.3 mm'de, hâlâ yakınsıyordu. Ortalama aşma pg 0.04–0.71 %, gov 1.3–10.8 %; eşleşik permütasyon testi her gecikmede p < 10⁻⁴ (Holm sonrası da).

## Ablasyon (tohum 0, n = 12)

| Varyant | 60 ms: max aşma / MT / başarı | 150 ms |
|---|---|---|
| Tam yasa | 2.8 % / 1.30 s / 100 % | 0.9 % / 2.12 s / 100 % |
| Algısal ortalama yok (n = 1) | 5.6 % / 1.24 s / 100 % | 5.0 % / 1.99 s / 100 % |
| Çevrimiçi Broyden açık | 33.8 % / 0.84 s / 100 % | 9.6 % / 1.42 s / 92 % |
| Gecikme terimi yok | 11.6 % / 0.88 s / 100 % | 41.9 % / 0.80 s / 100 % |
| Öğrenilen servo gecikmesi yok | 0.2 % / 3.20 s / 83 % | 0.0 % / 4.25 s / 75 % |

## Dürüst okuma
1. Oracle-sız uzanma çalışıyor ve test edilen en güvenli yasa.
2. Bedel hız: 60–200 ms'de oracle'lı governor'dan 2.0–3.1 kat yavaş.
3. Güvenliği gecikme terimi taşıyor; çevrimiçi Jacobian güncellemesi güvenliği bozuyor.
4. δ yarısı bildirilince bozulma yok; fazla tahmin test edilmedi (yapı gereği yalnız yavaşlatır).
5. Servo'yu saf gecikme saymak bir yaklaşıklık; ikinci mertebe servo için kesin sınır 1/e'nin altında (c ≈ 0.31–0.35). Ölçülen aşma ≤ 3.5 %; bu bir ispat değil, ampirik marj.
6. Sınırlar: yalnız simülasyon, tek başlangıç duruşu, tek kol modeli, boyun ve kesintili bakış kapalı, baz çizgiler tek gecikmede ayarlı.
