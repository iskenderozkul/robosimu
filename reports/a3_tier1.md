# A3 Tier 1 — adil gecikme matrisi + yanlış gecikme

## Hipotez durumu

| Hipotez | Sonuç |
|---|---|
| H1 (ayarsız ≤%1 aşma, MT ayarlı en iyinin %25'i) | FAIL |
| H2 (dondurulmuş bazçizgiler δ≥100 ms'de bozulur) | PASS |
| H4a (yanlış gecikmede göreli üstünlük, ≥1.5×) | PASS |
| H4b (yanlış gecikmede mutlak ≤%2 aşma) | FAIL |
| H5 (b/δ sabit ±%10) | FAIL |

## H1 detayı

| δ (ms) | dps aşma maks | dps MT | en iyi ayarlı MT | MT marjı % | geçer |
|---|---|---|---|---|---|
| 30 | 1.07 | 0.60 | 0.61 | -1.6 | ✗ |
| 60 | 0.99 | 0.66 | 0.75 | -12.0 | ✓ |
| 100 | 0.78 | 0.76 | 0.80 | -5.0 | ✓ |
| 150 | 0.58 | 0.92 | 1.00 | -8.0 | ✓ |
| 200 | 0.57 | 1.10 | 1.22 | -9.8 | ✓ |

## H4 — yanlış gecikme hücreleri (gerçek 120 ms, kontrolcüye 60 ms)

| Hücre | aşma maks % | aşma ort % | oturan |
|---|---|---|---|
| `dps_tuned_mis60` | 13.62 | 6.61 | 100% |
| `gov_1e_mis60` | 17.80 | 8.64 | 100% |
| `fixed30_mis60` | 29.04 | 14.10 | 100% |
| `pid30_mis60` | 36.14 | 17.55 | 12% |

En iyi dondurulmuş bazçizgi (aşma maks): 29.04% · Gecikme-farkındalıklı en kötü hücreye göre üstünlük: 1.63×

## H5 — Fitts eğimleri

| δ (ms) | b (s/bit) | b/δ |
|---|---|---|
| 30 | 0.3556 | 11.852 |
| 60 | 0.3556 | 5.926 |
| 100 | 0.3477 | 3.477 |
| 150 | 0.3182 | 2.121 |
| 200 | 0.2804 | 1.402 |

b/δ ortalaması: 4.956 · yayılım: 210.9%

## Yorum

1. **H1 — kısmen daralır.** Aşma bandı δ=30 ms'de 1.07% ile %1'i 0.07 puan aşar; 60–200 ms aralığında 0.57–0.99% içinde kalır. Hareket süresi hiçbir gecikmede en iyi ayarlı yasadan kötü değildir (marj −1.6 … −12%). Ayarsız yasa, katı bir ≤%1 garantisi değil, tüm gecikmelerde geçerli **güvenli ama muhafazakâr bir üst sınır** olarak okunmalıdır.

2. **H2 — desteklenir.** Dondurulmuş klasik yasalar δ≥100 ms'de ≥%5 aşma veya <%50 oturma ile bozulurken gecikme-türevli yasa ≤0.78% aşmada kalır. Gecikmeyi hız sınırına çevirmek tek başına bu rejimde belirleyici ayrımdır.

3. **H4a/H4b — ayrıştırılmış hüküm.** Gecikmenin 2× eksik bildirilmesi tüm yasaları bozar. Gecikme-farkındalıklı iki yasa, en iyi dondurulmuş bazçizgiye göre en kötü aşmada 1.63–2.13× daha iyidir (H4a PASS), ancak mutlak ≤%2 bandını tutturamaz (H4b FAIL: 13.62% ve 17.79%). Yani gecikme-türevli kazanç, eksik bildirilmiş gecikmeyi tek başına telafi etmez; H4b'nin başarısızlığı korunur ve raporlanır.

4. **H5 — reddedilir.** Fitts eğimi b gecikmeden neredeyse bağımsızdır (30→200 ms arasında 0.356→0.280 s/bit, −%21), dolayısıyla b/δ sabit değildir (yayılım %211). Bu bitki modelinde gecikme cezası eğimde değil, aşmada ve MT ofsetinde görünür. `b = β·δ_eff` öngörüsü bu haliyle simülasyonda doğrulanmamıştır; donanım protokolünde sınanacak açık bir öngörü olarak kalır.

5. **Kapı G1 kararı.** Başarısızlıklar gizlenmez; H1/H4b/H5 `FAIL` olarak kalır. Hız yasası güvenli alt sınır olarak korunur, iddialar daraltılır ve ayrıntı `reports/a3_gate_G1.md` ile `reports/v4_audit.md` içinde tutulur.
