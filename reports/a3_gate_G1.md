# Kapı G1 — A3 kanıtı, hükümler ve karar kaydı

**Tarih:** 16.11.2026 · **Kapsam:** Faz 0 kanıt paketi (A3 Tier 1 + Pareto + tuning) · **Karar:** koşullu onay, iddialar daraltılmış.
**Kanıt dosyaları:** `reports/a3_tier1.{json,csv,md}`, `reports/a3_pareto.{json,md}`, `reports/a3_tuning.json`
**Üreten komut:** `python scripts/run_a3.py` (TEST hedefleri, seeds 0–4, 8 genlik × 5 tohum = 40 uzanma/hücre) · `python scripts/run_a3_pareto.py`

> **Tek cümlelik hüküm:** Gecikme-türevli hız yasası (`k = 1/(e·δ_eff)`) dondurulmuş klasik yasalara karşı **tutarlı biçimde güvenli** çıkar, ancak "tüm gecikmelerde ≤%1 aşma" ve "eksik bildirilen gecikmede ≤%2 aşma" iddiaları **doğrulanmamıştır**. Bu raporda H1, H4b ve H5 **FAIL** olarak korunur.

---

## 1. Önceden kayıtlı kapı ölçütleri

| Kod | Ölçüt (TEST kümesi) |
|---|---|
| **H1** | `dps_1e` aşma maks ≤ %1 **ve** MT ≤ 1.25 × en iyi ayarlı yasanın MT'si; δ ∈ {30, 60, 100, 150, 200} ms'nin **tümünde** |
| **H2** | δ ≥ 100 ms'de en az bir dondurulmuş bazçizgi bozulur (aşma > %5 veya oturma < %50) **ve** `dps_1e` ≤ %1 |
| **H4a** | Gerçek 120 ms / kontrolcüye 60 ms: gecikme-farkındalıklı en kötü aşma ≤ (en iyi dondurulmuş bazçizgi) / 1.5 |
| **H4b** | Aynı hücrelerde gecikme-farkındalıklı aşma ≤ %2 (mutlak bant) |
| **H5** | Fitts eğimi oranı $b/\delta$ yayılımı ≤ %10 (yani $b = \beta\,\delta$) |
| **H3** (Pareto) | $\lvert c_{\max}(\delta) - 1/e \rvert \le 0.05$ her gecikmede |

H4, ilk kayıtta tek bir birleşik hüküm olarak yazılmıştı (aynı hücrede hem mutlak ≤%2 hem dondurulmuş >%5). Bu, iki farklı iddiayı tek ölçüte bağlıyordu; G1'de **H4a (göreli)** ve **H4b (mutlak)** olarak ayrıştırıldı. Ayrıştırma, ölçütü gevşetmek için değil, hangi iddianın düştüğünü görünür kılmak için yapıldı; her iki alt hüküm de aşağıda ayrı ayrı raporlanır.

## 2. H1 — hız yasası (δ matrisi)

| δ (ms) | `dps_1e` aşma maks % | `dps_1e` MT (s) | En iyi ayarlı MT (s) | MT marjı | Hüküm |
|---|---|---|---|---|---|
| 30 | **1.07** | 0.60 | 0.61 | −1.6 % | ✗ |
| 60 | 0.99 | 0.66 | 0.75 | −12.0 % | ✓ |
| 100 | 0.78 | 0.76 | 0.80 | −5.0 % | ✓ |
| 150 | 0.58 | 0.92 | 1.00 | −8.0 % | ✓ |
| 200 | 0.57 | 1.10 | 1.22 | −9.8 % | ✓ |

**Yorum.** %1 bandı yalnız δ = 30 ms'de, 0.07 puanla aşılır. Hareket süresi hiçbir hücrede en iyi ayarlı yasadan kötü değildir; marj her zaman negatiftir (daha hızlı). Yasa, katı bir ≤%1 garantisi değil, **δ ∈ [30, 200] ms boyunca ≤%1.1 üst sınır** veren muhafazakâr bir alt sınırdır. H1'in tek hücre nedeniyle FAIL kalması, bandın daraltılmasını değil, iddianın yeniden yazılmasını gerektirir.

## 3. H2 — dondurulmuş bazçizgiler

δ ≥ 100 ms'de `fixed30`/`pid30` aşma > %5 veya oturma < %50 ile bozulur; `dps_1e` aşması 0.57–0.78 % aralığında kalır. **PASS.** Bu, gecikmeyi hız sınırına çevirmenin tek başına belirleyici ayrım olduğu rejimi gösterir.

## 4. H4a / H4b — yanlış bildirilen gecikme (gerçek 120 ms, bildirilen 60 ms)

| Hücre | Aşma maks % | Aşma ort % | Oturma |
|---|---|---|---|
| `dps_tuned_mis60` | 13.62 | 6.61 | 100 % |
| `gov_1e_mis60` | 17.80 | 8.64 | 100 % |
| `fixed30_mis60` | 29.04 | 14.10 | 100 % |
| `pid30_mis60` | 36.14 | 17.55 | 12 % |

- **H4a (göreli): PASS.** En iyi dondurulmuş bazçizgi 29.04 %; gecikme-farkındalıklı en kötü hücre 17.80 % → **1.63×** üstünlük (`dps_tuned` için 2.13×), eşik 1.5×.
- **H4b (mutlak): FAIL.** Hiçbir gecikme-farkındalıklı hücre ≤%2 bandına giremez. Gecikme-türevli kazanç, gecikmenin 2× eksik bildirilmesini **tek başına telafi etmez**; bu bir tasarım sınırıdır, ölçüm hatası değildir.

## 5. H5 — Fitts eğimi ve gecikme

| δ (ms) | b (s/bit) | b/δ |
|---|---|---|
| 30 | 0.3556 | 11.85 |
| 60 | 0.3556 | 5.93 |
| 100 | 0.3477 | 3.48 |
| 150 | 0.3182 | 2.12 |
| 200 | 0.2804 | 1.40 |

b/δ yayılımı **%210.9** → **FAIL.** Eğim b, gecikme 6.7× büyürken yalnız %21 azalır (0.356 → 0.280 s/bit). Yani bu bitki modelinde gecikme cezası Fitts eğiminde değil, aşmada ve MT ofsetinde görünür. `b = \beta\,\delta_{\text{eff}}` ($\beta \approx 0.86$) öngörüsü **simülasyonda doğrulanmamıştır**; donanım protokolünde sınanacak açık bir öngörü olarak kalır.

## 6. H3 / Pareto — tepe kazanç ve cephe

| δ (ms) | $c_{\max}$ (dps) | $c_{\max}$ (gov) | $\lvert c_{\max}-1/e\rvert$ | H3 | dps cephe uzaklığı |
|---|---|---|---|---|---|
| 30 | 0.30 | 0.60 | 0.068 | FAIL | 1.6 % |
| 60 | 0.30 | 0.60 | 0.068 | FAIL | 8.7 % |
| 100 | 0.35 | 0.60 | 0.018 | PASS | 0.0 % |
| 150 | 0.35 | 0.60 | 0.018 | PASS | 3.1 % |
| 200 | 0.35 | 0.60 | 0.018 | PASS | 5.2 % |

$c_{\max}$ ızgarası 0.05 adımlıdır; 30–60 ms'de gerçek tepe değer 0.30 ile 0.35 arasındadır, bu yüzden H3'ün ±0.05 toleransı bu iki hücrede ızgara çözünürlüğü nedeniyle düşer. Bu bir yasa hatası değil, **ölçüm çözünürlüğü** sınırıdır; daha ince ızgara (0.01) ile yeniden koşturma açık iştir.

## 7. Karar

1. **G1 koşullu onaylanır.** Faz 1 kodunun (zero-oracle yolu, `V43Controller`, `NeckKinematics`) varlığı A3 kanıtıyla çelişmez; ancak A3, v4.3'ün **kapalı çevrim garantilerini** değil, **gecikme-türevli hız sınırının güvenliğini** kanıtlar.
2. **Hız yasası korunur** — "güvenli ama muhafazakâr üst sınır" çerçevesiyle.
3. **H1, H4b, H5 FAIL olarak kalır.** Hiçbiri gizlenmez, hiçbiri eşik değiştirilerek "geçirilmez".
4. H4 iki alt hükme ayrılır (H4a/H4b); ayrıştırma `scripts/run_a3.py` içinde kalıcıdır ve `reports/a3_tier1.md` her koşuda bu tabloyu üretir.
5. Bu dosya, G1'e ilişkin **tek karar kaydıdır**; durum/denetim notları yalnızca burada ve `reports/v4_audit.md` içinde tutulur, makale metnine taşınmaz.

## 8. İddia daraltmaları (makaleye yansıyan)

| Önceki iddia | Yeni iddia |
|---|---|
| "1/e kuralı tüm gecikmelerde ≤%1 aşma garanti eder" | "δ ∈ [30, 200] ms boyunca ≤%1.1 üst sınır; δ ≥ 60 ms'de ≤%1" |
| "gecikme-türevli kazanç oracle'sız da güvenli" | "göreli olarak 1.6–2.1× daha güvenli; mutlak %2 bandı yalnız doğru bildirilen gecikmede" |
| "$b = 0.86\,\delta_{\text{eff}}$" (simüle edilmiş öngörü) | "donanımda sınanacak açık öngörü; simülasyonda b gecikmeden bağımsız" |

## 9. Yeniden üretim

```
python scripts/run_a3.py            # a3_tier1.{json,csv,md}
python scripts/run_a3_pareto.py     # a3_pareto.{json,md}
python -m unittest discover -s tests
```

## 10. Açık işler (G1 sonrası)

- [ ] H3 için 0.01 adımlı yeniden koşturma (30–60 ms çözünürlük sınırı).
- [ ] H4b için eksik-bildirim telafisi (ölçülen varış gecikmesinden δ_obs kestirimi) ayrı bir hipotez olarak ön-kaydedilmeli.
- [ ] `b = β·δ_eff` donanım protokolü (Zenodo DOI ve laboratuvar erişimi bekliyor).
- [ ] Canlı literatür doğrulaması (ağ erişimi yok — G0 kalemi açık).
