# Faz 1 — Oracle-sız (sıfır-oracle) yol denetimi

Tarih: 23.11.2026 · Model: `models/six_axis_arm.json`, Q0 = [0, -0.3, 0.6, 0, 0.6, 0], stereo varsayılan, gürültü 0, süre 3–4 s.

## 1. Bulgu: mevcut `V43Controller` hedefe ulaşmıyor

Arayüz doğru kısıtlanmış (yalnız `TimestampedObservation`: piksel + zaman damgası; `set_delay` yok). Ama kapalı döngü çalışmıyor.

| Hedef ofseti (m) | Gecikme | V43 son hata (mm) | Governor (oracle'lı) son hata (mm) |
|---|---|---|---|
| (0.08, 0, 0) | 60 ms | 77.7 | 0.87 |
| (0, 0.08, 0) | 60 ms | 95.6 | 1.05 |
| (0.05, 0.05, -0.05) | 60 ms | 98.9 | 0.74 |
| (0.08, 0, 0) | 120 ms | 77.7 | 0.90 |

Hız sınırı `vmax=0.5` piksel ölçeğine göre yaklaşık 1000 kat küçük kalıyor: kontrolcü metre/s sınırını piksel/s hatasına uyguluyor ve kol neredeyse hiç kıpırdamıyor. `vmax` yükseltildiğinde ise kol uzaklaşıyor (son hata 1–1.3 m). Bunun sebebi sabit "beden şeması" Jacobian'ının (`J_r0`) bu modelin geometrisiyle uyuşmaması: işaretler/eksenler yanlış ve Broyden güven bölgesi (`delta_j_max=0.5`) bu yanlışı düzeltmeye izin vermiyor.

**Sonuç:** Raporlardaki V4.3 testleri (firewall, Broyden sınırı, zaman hizalama) birim düzeyinde geçiyor olabilir; ama **uçtan uca bir uzanma başarısı hiçbir yerde ölçülmemiş**. Makalede "oracle-sız çalışır" iddiası bu hâliyle yazılamaz.

## 2. Prototip: `BlindBabbleGovernor` (deneme)

Fikir: insan bebeğinin motor babbling'i gibi, kontrolcü başlangıçta her ekleme kısa bir ileri-geri darbe verir; efferent kopya (kendi verdiği komutların integrali) ile piksel değişimini eşleştirip Jacobian'ı **kendisi öğrenir**. Sonra Broyden ile günceller ve `k = (1/e)/(δ_obs + τ_servo)` kazancıyla hedefe gider. Analitik J, eklem açısı, metre ve gerçek gecikme kullanılmaz; gecikme zaman damgasından ölçülür.

| Hedef ofseti (m) | 60 ms son hata (mm) | 60 ms aşma | 120 ms son hata (mm) | 120 ms aşma |
|---|---|---|---|---|
| (0.08, 0, 0) | 20.2 | 0.0 % | 107.9 | 0.0 % |
| (0, 0.08, 0) | 3.1 | 0.0 % | 13.8 | 0.0 % |
| (0, 0, 0.08) | 3.6 | 0.6 % | 8.7 | 1.9 % |
| (0.05, 0.05, -0.05) | 10.3 | 0.0 % | 64.7 | 0.0 % |
| (-0.06, 0.04, 0.03) | 5.3 | 0.0 % | 20.7 | 0.0 % |

Okuma: aşma neredeyse yok (1/e güvenliği oracle'sız da korunuyor gibi), ama **doğruluk yetersiz**. 60 ms'de 5 hedeften 2'si 5 mm'ye girmeye yakın; 120 ms'de çoğu girmiyor. Muhtemel nedenler: (a) babbling sonrası kol başlangıç noktasına tam dönmüyor ve öğrenilen J yerel kalıyor; (b) sönümleme `lam` piksel ölçeğine göre ayarlı değil; (c) süre 4 s yetmiyor (yavaş ama güvenli yaklaşım).

## 3. Sonraki adımlar

1. V43'ün `vmax`'ını piksel biriminde tanımla ya da kaldır; `J_r0`'ı babbling ile değiştir.
2. Prototipi 30+ bağımsız hedef × 5 tohum × 5 gecikme ile ölç; başarı oranı, son hata, aşma, MT.
3. Uçtan uca testi (`tests/test_no_oracle_reach.py`) ekle: oracle-sız kontrolcü 60 ms'de hedeflerin ≥%80'inde 5 mm'ye girmeli. Geçemezse makalede FAIL olarak raporlanır.
4. Ancak bu kapı geçildikten sonra ablasyon merdiveni (L0–L6) çalıştırılır.
