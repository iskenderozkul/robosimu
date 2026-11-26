# İnsansı Hız Sınıfları (Aciliyet Seviyeleri) — Ürün Dokümanı

Bu doküman, simülasyon kurgusundaki (workbench) **“İnsansı Hız Sınıfı (Aciliyet Seviyesi)”** menüsünü ve alt durumlarını (görev bağlamı) tanımlar. Sınıflar insan hareketinin gerçek sınıflandırmasına karşılık gelir: monoseynaptik balistik reflekten (~100–250 ms) karmaşık cerrahi mikro-ulaşıma (>3 s) kadar uzanan hız–doğruluk uzayı.

> İngilizce özet: five urgency classes for the human-like arm (reflex → fast → normal → careful → precision), each with task-context sub-modes. Parameters are task constraints (speed cap, joint rate cap, speed-budget scale, arrival tolerance, brake onset), not a biological measurement. Doc written for the open-source simulator `robosimu`; see also `models/human_arm.json`.

## Nasıl ayarlanır

1. `python -m robosimu workbench models/human_arm.json --out figures/human_arm_workbench.html` ile workbench'i üret.
2. **Kolun İlk Pozisyonu** (askıda sarkan kol vb.) ve **Hedef** slider'larını/koordinat kutularını ayarla.
3. **İnsansı Hız Sınıfı** seç; **Alt Durum** buna göre otomatik dolar. Detayli slider'lar (v_max, q̇_max, κ ölçeği, ε, φ_min, γ, ter_P, süre) sınıfa göre otomatik ayarlanır; hepsini elle de düzenleyebilirsin.
4. **Simülasyonu Başlat** — kol ilk pozisyondan hedefe uzanır, KPIs/compare tablosu dolar; CSV ve `.md` rapor indirilebilir.

CLI tarafında `--posture hanging_natural --target "0.45 0.0 1.10" --vmax 1.0` gibi geçersiz kılabilirsiniz; `--write-config` ile kayıtlı bir kurguyu `--config` ile tekrar oynatabilirsiniz.

## Formül arka planı

- **Hız bütçesi:** `v = min(v_max, φ · min(k_hi·|e|, κ/δ_eff · (ε + |e|)))`,  `δ_eff = δ + n·T/2`,  `κ = kScale / e`
  - `kScale > 1` (reflex) `κ > 1/e` yapar: aşma kabuledilir, tek-atım (open-loop) balistik davranışın modelidir.
  - `kScale = 1.00` (normal) `κ = 1/e` — Proposition 1'in monoton, aşmasız rejimi.
  - `kScale < 1` (careful, precision) ek güvenlik payı verir.
- **φ — frenleme eğrisi:** `φ = 1` eğer kalan yol oranı `p_rem < ter_P` (son fren bölgesi), aksi halde `φ = max(φ_min, min(1, 0.5·(timeRamp + p_exp)))`,  `p_exp = ((1−p_rem+0.18)/0.55)^γ`
  - `γ` frenleme rampasının üssüdür; küçük γ = yavaş başlangıç ivmelenmesi, büyük γ = geç, balistik ivmelenme.
  - `ter_P` frenleme bölgesinin başladığı yol oranıdır; `φ_min` φ'nın taban değeridir (son ulaşım hızını sınırlar).
- **ε (varış toleransı)** ve **süre sınırı (`dur`)** sınıfa özeldir: reflex–fast hata sayısını, normal–precision doğruluğu önceler.
- **n (ortalama kareleri / averaging budget)** gürültüden hesaplanır: `n = clip(ceil((2σ/ε)²), 1, floor(2δ/T))`; sezgisel: gürültü arttıkça kare sayısı artar, görüş gecikmesi de artmış olur (δ_eff = δ + nT/2).

## Sınıflar ve alt durumlar

| # | Sınıf (UI) | Tipik süre | v_max (m/s) | q̇_max (rad/s) | κ çarpanı | ε (mm) | φ_min | γ | ter_P | Süre bütçesi | Kullanım |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | **Acil / Refleks** (Balistik, ~120–250 ms) | 120–250 ms | 1.15 | 3.0 | 1.18 (>1/e) | 8.0 | 0.68 | 0.35 | 0.28 | 1.6 s | Tehlike/korunma anında omurilik-beyin sapı düzeyinde tetiklenen hareket. Görsel düzeltme neredeyse yok; doğruluk yerine hız. **Aşma kabul edilir.** |
| 2 | **Hızlı / Atik Ulaşma** | 250–500 ms | 0.80 | 2.2 | 1.05 | 5.0 | 0.45 | 0.45 | 0.38 | 2.0 s | Erken ve güçlü ivmelenme, gecikmeye bağlı kararlı frenleme. Hafif aşma riski kabul edilebilir. |
| 3 | **Normal / Doğal Günlük Ulaşma** | 500–1000 ms | 0.50 | 1.4 | 1.00 (=1/e) | 4.0 | 0.28 | 0.55 | 0.45 | 2.8 s | Klasik çan-eğrisi hız profili (Flash & Hogan 1985), tam `κ = 1/(e·δ_eff)` ile aşmasız varış. **Varsayılan.** |
| 4 | **Dikkatli / Yumuşak Taşıma** | 1–2 s | 0.30 | 0.9 | 0.78 | 3.0 | 0.18 | 0.72 | 0.60 | 3.8 s | Sarsıntısız, düşük ivmeli hareket: dolu bardak, kırılgan nesne. Uzun ve erken frenleme bölgesi. |
| 5 | **Hassas / Cerrahi Mikro-Ulaşma** | 2–5 s | 0.16 | 0.55 | 0.55 | 1.5 | 0.12 | 0.88 | 0.75 | 5.0 s | Milimetrik tolerans; yolun büyük çoğunluğu geri beslemeli fren bölgesindedir. |

### 1. Acil / Refleks — alt durumlar

| Alt durum | Anlamı | Ayarlama (çarpımsal) |
|---|---|---|
| `catch_falling` | Düşen nesneyi yakalama | `vmax × 1.00`, `ε × 1.00` (taban değerler) |
| `threat_grab` | Tehlikeli nesneyi hızla kapma / itme | `vmax × 1.15`, `ε × 1.4`, `φ_min × 1.15` |
| `protective` | Koruyucu el uzatma (çarpışma önleme) | `vmax × 0.9`, `ε × 1.6`, `ter_P × 0.8` |

### 2. Hızlı / Atik — alt durumlar

| Alt durum | Anlamı | Ayarlama |
|---|---|---|
| `quick_pick` | Masadan hızla nesne alma | taban değerler |
| `sport` | Spor / oyun (top yakalama hazırlığı) | `vmax × 1.1`, `φ_min × 1.2` |
| `repetitive` | Tekrarlı montaj / ayıklama temposu | `vmax × 0.9`, `ε × 0.8` |

### 3. Normal — alt durumlar

| Alt durum | Anlamı | Ayarlama |
|---|---|---|
| `comfort` | Günlük konforlu ulaşma (bardak, telefon) | taban değerler |
| `heavy_load` | Ağır yük ile ulaşma (tork bütçesi korunur) | `vmax × 0.75`, `q̇_max × 0.75`, `φ_min × 0.8` |
| `fatigue` | Yorgun / yaşlı birey benzeri | `vmax × 0.65`, `κ × 0.9`, `γ × 1.2` |

### 4. Dikkatli / Yumuşak — alt durumlar

| Alt durum | Anlamı | Ayarlama |
|---|---|---|
| `full_glass` | Ağzına kadar dolu bardak taşıma | taban değerler |
| `fragile` | Kırılgan nesne (cam, yumurta) | `vmax × 0.85`, `ε × 0.8` |
| `human_contact` | İnsana temas / bakım (hasta yardımı) | `vmax × 0.7`, `κ × 0.9`, `ter_P × 1.15` |

### 5. Hassas / Cerrahi — alt durumlar

| Alt durum | Anlamı | Ayarlama |
|---|---|---|
| `surgical` | Cerrahi mikro-ulaşma | taban değerler |
| `needle` | İğneye iplik geçirme / ince montaj | `vmax × 0.8`, `ε × 0.7` |
| `lab` | Laboratuvar pipetleme | `vmax × 1.2`, `ε × 1.3` |

Çarpanlar `v_k ← v_k · mod_k` şeklinde uygulanır (`φ_min` ≤ 0.9, `ter_P` ≤ 0.85 sınırlıdır); yani `0.75` → %25 yavaşlama, `1.2` → %20 hızlanma anlamına gelir.

## Beklenen davranış (insan kolu modeli, `models/human_arm.json`)

Askıda sarkan koldan rafa uzanma (`0.45 0.0 1.10`, 60 ms gecikme) için ölçülenler:

| sınıf | v_max | settling | aşım | notlar |
|---|---|---|---|---|
| normal (CLI `--vmax 0.6`) | 0.6 m/s | 2.6 s | %0 | taban |
| fast (`--vmax 1.0`) | 1.0 m/s | 2.5 s | %0.008 | koni-şeklinde ivmelenme |
| careful (`--vmax 0.3`) | 0.3 m/s | 3.5 s | %0.01 | erken frenleme |
| precision (`--vmax 0.16`) | 0.16 m/s | 6.1 s | %0.007 | geri besleme ağırlıklı |

Aynı hedefte sabık fixed kazanç (30 ms'te ayarlı) 120 ms gecikmede hiç yerleşmez ve %4.5 aşım yapar; governor 2.5 s'te yerleşir ve aşmasız varır.

## Sınırlamalar (dürüstçe)

- Bunlar **görev parametresi sınıflarıdır**, biyolojik bir ölçüm değildir; gerçek insan refleksleri vestibular/proprioseptif yollar ve kas elastisitesiyle çalışır, bu modelin tek eksen/`visual servo` soyutlaması onlar yerine geçmez.
- Reflex sınıfı bir **kasıtlı istisnadır**: `κ > 1/e` monoton-varış garantisini bozar; aşma kabul edilir (balistik model). Grafiklerdeki “0 aşım” iddiası normal sınıf ve altı içindir.
- Sınıf seçimi v_max'ı kısıtlar; kolu fiziksel olarak hızlandırmaz — gerçek tepkiler kas kuvveti ve sinir-kas gecikmesiyle sınırlıdır.
- Süre bütçesi (`dur`) sadece planlama üst sınırıdır; simülasyon dışı davranışı temsil etmez.

## Kaynaklar

- Schmidt, R. A., Lee, T. D. *Motor Learning and Performance* (5th ed.) — refleks ve istemli hareketler hiyerarşisi.
- Flash, T., Hogan, N. (1985). The coordination of arm movements. *J. Neuroscience* 5(7):1688–1703 — çan-eğrisi hız profili.
- Fitts, P. M. (1954). *J. Exp. Psychol.* 47:381–391 — hız–doğruluk takası (sınıflar arasındaki tradeoff bu ölçüt ile ölçülür).
- Sünklorenk / Armstrong — kas-programı (open-loop) kavramı (reflex/balistik sınıfları için sezgi).
