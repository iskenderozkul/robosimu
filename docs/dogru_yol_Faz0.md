# Faz 0 — Kilitlenmiş Kararlar (TODO_dogru_yol.md için)

**Tarih:** 14.11.2026 · **Durum:** H1–H5 ve T1–T8 tanımları bu dokümanla *önceden yazılmış* hale getirildi; son değişiklik yalnızca kapı G1/G2'den sonraya bırakıldı.

> Not: Bu tanımlar `TODO_dogru_yol.md`'da atıf yapılan ancak projede bulunmayan
> "H1–H5", "T1–T8" ve "spesifikasyon §10" için önceden yazılmış (pre-registered)
> taslaktır. Kullanıcı onayıyla kesinleşir; kapı metinleri bundan sonra
> değiştirilmez.

## 0.1 Kilitlenmiş ana soru

> **"Gecikmeli görsel geri beslemeyle uzanmada, gecikmeden türetilen ayarsız bir hız sınırı, ayarlı klasik kontrolcülere yaklaşır mı ve gecikme büyüdükçe dayanıklı kalır mı?"**

Kanıt: adil simülasyon benchmark'ı (A3) + küçük donanım doğrulaması. Hedef dergi: Scientific Reports.

## 0.2 Katkı tablosu (kilitli)

| Bileşen | Sınıf |
|---|---|
| `DelayScaledP` — gecikmeden türetilen P: `k = c/δ` | **standart aile üzerinde yeni parametrik forma** (P+gecikme ölçekleme) |
| `clip_kappa` (c ≤ 1/e kesme) | **yeni** |
| Governor (κ = 1/e, boxcar) | **kompozisyon** (klasik Hayes/Corless sınırı + boxcar) |
| HLSL / HDA / human_wdls | **kompozisyon** |
| A3 adil simülasyon çekirdeği (tune/test/ood ayrımı) | **yeni altyapı** |
| c_max(δ) ≈ 1/e bulgusu | **ölçülen** (kapı G1'de değerlendirilir) |
| 5-senaryolu donanım protokolü (Faz 2) | **yeni** |

## 0.3 Başlık taslağı (kilitli)

> **"A delay-scaled, tuning-free speed limit for visual reaching: a fair simulation benchmark and a hardware validation"**

## 0.4 H1–H5 (önceden yazılmış eşikler — %25 marj dahil)

| Hipotez | Tanım | Eşik |
|---|---|---|
| **H1** | `dps(c=1/e)` aşma ≤ %1 ve medyan MT, her gecikmede o gecikmede ayarlı en iyi bazcisinden **%25 içinde** kalır | H1 destek: hem ≤%1 aşma hem MT marjı ≤%25 |
| **H2** | 30 ms'de ayarlı ve dondurulmuş bazcisler (fixed/PID) δ ≥ 100 ms'de bozulur (aşma >%5 veya yerleşememe); dps ayarı bozulmaz | H2 destek: bazcis bozulur, dps ≤%1 |
| **H3** | Ölçülen maksimum güvenli `c` (aşma ≤%0.5): `c_max(δ)`, tüm δ ∈ [30,200] ms için `|c_max − 1/e| ≤ 0.05` | H3 destek: tolerans içinde |
| **H4** | Yanlış bildirilen gecikme: **H4a** fazla tahmin (120 ms sanıyor, 60 ms gerçek) ⇒ dps aşma ≤%2; **H4b** eksik tahmin (60 sanıyor, 120 gerçek) ⇒ sınır ihlali, aşma >%5 (teorik olarak beklenen tek taraflılık) | H4 destek: her iki yön de doğru |
| **H5** | Fitts eğimi: `b/δ_eff` tüm gecikme matrisinde ±%10 içinde sabit | H5 destek: sapma ≤%10 |

## 0.5 T1–T8 test tanımları (tests/test_a3.py)

| Kod | Test |
|---|---|
| T1 | `DelayScaledP`, `c` sabitken `k·δ = c` verir; `c=1/e` için `k = (1/e)/δ` (governor'ın kinden farkı: governor `δ_eff = δ + nT/2` kullanır) |
| T2 | `TUNE`/`TEST`/`OOD` hedef kümeleri çiftsiz (disjoint) |
| T3 | OOD hedefler tune/test zarfının dışında (daha büyük genlik) |
| T4 | `clip_kappa=True` ⇒ `k·δ ≤ 1/e`; ayrıca `c=1/e`'de kalıntı aşma ≤%0.5 (eps ölü bölge + gecikme; sıfır değil) |
| T5 | Gecikme matrisi: dps(c=1/e) her hücrede ≤%1 aşma |
| T6 | Ayarlama yalnızca TUNE hedeflerini görür (davranışsal sızıntı koruması: mock ile ölçülür) |
| T7 | Yanlış gecikme: fazla tahmin ≤%2 aşma; eksik tahmin >%5 aşma |
| T8 | `c_max(δ)` ızgara çözünürlüğü 0.02 ile ölçülür; `|c_max − 1/e| ≤ 0.05` |

## 0.6 Bütçe (spesifikasyon §10 — ölçüldü)

- Dimansiysız A3 çekirdeği: ~1 ms/koşu başına (100 koşu ölçümü: bkz. `scripts/tune_per_condition.py --time-check`).
- Tier 1: 5 gecikme × 4 kontrolcü × 12 hedef × 5 seed ≈ 1200 koşu + yanlış gecikme 2 koşu grubu ≈ 1300 koşu ⇒ ~3 dk.
- Ayarlama: 12 c × 5 gecikme × 6 hedef × 3 seed = 1080 koşu ⇒ ~2 dk.
- Pareto: 2 yasa × 5 gecikme × 12 c × 8 hedef × 3 seed = 2880 koşu ⇒ ~5 dk.

## 0.7 Kapı metinleri (TODO_dogru_yol'dan alındı, değiştirilmez)

**G1:** H1+H3 destek ⇒ donanıma geç. H1 çürük ⇒ "güvenli ama muhafazakâr alt sınır" çerçevesi. H3 çürük ⇒ yasa `c(δ)` düzeltmesiyle güncellenir veya iddia daraltılır. Ayarlı rakipler her yerde geçiyor ⇒ "ayarsız yöntemin bedeli" raporlanır; katkı benchmark olur.

**G0:** Literatürde aynı fikir bulunursa ⇒ katkı "açık kaynak benchmark + donanım doğrulaması"ya kayar.

## 0.8 Revizyon kaydı

- **14.11.2026 (alet kalibrasyonu):** T1/T4/T6/T7/T8 tanımları, uygulanan yasanın gerçek semantiğine göre güncellendi: (i) governor `δ_eff = δ + nT/2` kullanır, dps kullanmaz (T1); (ii) eps ölü bölgesi + gecikme, `c=1/e`'de sıfır olmayan küçük kalıntı aşma bırakır (T4); (iii) T6 sızıntı koruması davranışsal (mock) ölçülür; (iv) H4/T7 tek taraflı olarak tanımlandı — fazla tahmin güvenli, eksik tahmin sınırı ihlal eder; (v) T8 ızgara 0.02 adımlı. Bu revizyonlar **hipotez eşiklerini değiştirmez** (H1–H5 sayısal eşikleri aynı); yalnızca ölçüm protokolünü gerçek modele uyarlar. Kullanıcı onayı hâlâ beklenmektedir; onaydan sonra bu tanımlar değiştirilmez.
