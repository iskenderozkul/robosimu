# V4.3 Kod Denetimi — `robosimu/` (Faz 0, Kapı G0)

**Tarih:** 16.11.2026 · **Kaynak şartname:** `md docs/MASTER_SPEC_V4_3_ROBOSIMU.md` · **Kapsam:** yalnızca okuma/rapor; sıfır kod değişikliği.

> **Özet yargı:** Mevcut `robosimu` şartname v4.3'ün **çekirdek fizik/kinematomotor alt yapısına** (6-DOF kol, eklem sınırları, motor modelleri, iki göz pinhole stereo, gecikmeli gözlem, `1/e` sınırı) sahip; ancak şartnameye özgü **dört algoritmik bileşenin** (interpolasyonlu sıra-korumalı Broyden, korelasyon-farkındalıklı pencere seçimi, hata-ölçekli bel–kol WDLS, aşamalı el sinerjisi) **hiçbiri kodda yok**. Üstelik v4.3'ün "sıfır-oracle" kuralı ile mevcut kod **çelişiyor** (set_delay + metre cinsinden rel_base_gap_m sızmakta). Bu rapor bu durumu netleştirir.

---

## Soru 1 — Gerçek Kinematik Envanter

| Alt-uzay | Kod | Derece | Kaynak |
|---|---|---|---|
| Trunk (bel) | `link1_trunk_shoulder_yaw` (j1, ±60°) | $d_{\text{trunk}} = 1$ | `models/human_arm.json:114-130` |
| Omuz + Dirsek + Bilek | `link2_upper_arm`, `link3_forearm`, `link4_wrist_flexion`, `link5_wrist_radial_ulnar` (j2–j5) | $d_{\text{arm}} = 4$ | `models/human_arm.json:151-278` |
| El | `link6_hand_rotation` (j6, ±30°) | $d_{\text{hand}} = 1$ (yalnız döndürme; **sinerji matrisi yok**) | `models/human_arm.json` (link6) |
| **Toplam** | 6 serbestlik dereceli kol | $\sum = 6$ | `robosimu/model.py:71` (`ArmModel.n`) |
| Boyun (pan/tilt) | **modelde yok** — yalnızca görsel baş (`state.gazeYaw/gazePitch`) | $d_{\text{neck}} = 0$ (fiziksel) / 2 (görsel) | `workbench_html.py` (gaze state) |

Şartname §3.1'deki $d_{\text{neck}}=2$ (pan, tilt) ve el sinerjisi $\hat q_{\text{hand}} = q_0 + S_{\text{syn}}\alpha$ **kodda karşılık içermez**.

## Soru 2 — Oracle Sızıntı Envanteri (satır satır)

| Sızıntı | Konum | Şartnameyle çelişimi |
|---|---|---|
| `set_delay(delays[k])` — kontrolcüye gerçek gecikme | `simulator.py:135-138` | Şartname §3.3: "görüntü paketleri zaman damgalı gelir (`set_delay` kullanılmaz)" |
| `obs_st.rel_cam_gap_m` — metre cinsinden göreli konik fark | `simulator.py:161` | Şartname §3.2: yalnız piksel büyüklükleri/ortalamaları |
| `obs_st.rel_base_gap_m` — metre cinsinden taban farkı | `simulator.py:164` | Aynı |
| `q` (eklem açıları) — kontrolcü `q_ref` yoluyla resolved-rate IK'e beslenir | `simulator.py` döngüsü + `stereo.py` (`HumanLoomingController`) | Şartname §3.1: kontrolcü $q_{\text{true}}$ okuyamaz |
| `J` (analitik Jacobian) — `human_wdls` içinde kullanılır | `stereo.py` (`human_wdls` yolu) | Şartname: yalnız öğrenilen $\hat J$ |

**Not:** `set_delay`/`rel_*_gap_m` yalnızca **görsel gözlem** üreten yolaklar içindir; `hdap`/`hlsl` gibi yasaların çoğu yine de bu sızıntıyı kullanır. Bu, v4.3'ün "kanıtsız garanti yasağı" ile de çelişir (şartname'nin H1–H5'i bu sızıntıyı ortadan kaldırmadan kurulamaz).

## Soru 3 — Kamera ve Projeksiyon Modeli (`stereo.py`)

- Yapı: `StereoCameraConfig` (`stereo.py:55-110`): `baseline_m=0.065`, `focal_px=580`, `cam_pos=(-0.12, 0.0, 0.62)`, `look_at`, `imgW/imgH` (640×480 varsayılan), `fps`, piksel/disparite gürültüsü, `vision_mode` (binocular/mono_left/mono_right/mono_static), boyun paralaksı (`neck_baseline_m`, `neck_parallax_*`).
- Projeksiyon: iki pinhole kamera; `u_L/u_R` hesabı `stereo.py:288-332` civarı; disparite $d = u_L - u_R$; derinlik $Z = fB/d$ yorumu `stereo.py:555-561` (mono modda sentetik taban).
- Görünür alan: hedef/el retina projeksiyonları `stereo.py:393-442` (`observe`); mono modda boyut-uzaklık belirsizliği `stereo.py:518-561`.
- **Fark:** şartname $\bar u, \bar v, d$ (piksel ortalaması + disparite) kullanır; kod metre cinsinden `rel_cam_gap_m`/`rel_base_gap_m` üretir. Birim/boyut uyuşmazlığı → `TODO_OWNER` (bkz. Soru 6).

## Soru 4 — Kavrama, Durma, Başarı Ölçütleri

| Ölçüt | Kod durumu | Kaynak |
|---|---|---|
| Aşım (overshoot) | `_overshoot_pct(p, p0, target)` — komut adımının yüzdesi + penetrasyon (m) | `simulator.py:59-60, 265` |
| Yerleşme | `settling_time_s` (ε = 5 mm bandı) | `simulator.py:285` |
| Final hata | `final_error_m` | `simulator.py:286` |
| Enerji | `energy_j`, `pulse_efficiency` | `simulator.py:281-291` |
| Zamanaşımı (horizon) | `cfg.duration` (varsayılan 2.5-3.0 s) | `simulator.py` SimConfig |
| **Temas/kavrama başarısı** | **YOK** — ne `contact` ne `grasp_success` ne `false_grasp` tanımlı | — |
| Erken kapanma / penetrasyon sınırı | **YOK** (yalnız overshoot/penetrasyon ölçümü var) | — |

Şartname §3.6'nın `Evaluation` adımı ($r_{\text{grasp\_tol}}$, `false_grasp`) ve §3.3'ün `stale_observation`/`is_admissible` bayrakları **kodda yok**.

## Soru 5 — Mevcut Kontrolcüler ve Testler

**Kontrolcüler** (`controllers.py`): `fixed` (:17), `governor` (:47, κ=1/e, boxcar), `adaptive` (:180), `smith` (:211), `broyden` (:258), `lqr` (:312), `pid` (:347), `dps` (:136, gecikme-ölçekli P — bu oturumda eklendi), `hlsl`/`hdap` (`stereo.py`). **`make_controller`** (:384-410) fabrika.

**Testler:** `python -m unittest discover -s tests` → 70 test (bu oturumde 61→70; `test_a3.py` T1–T8 dahil). Komut: `make test`.

## Soru 6 — Uyumluluk / TODO_OWNER listesi

| # | Uyuşmazlık | Şartname | Kod | Aksiyon |
|---|---|---|---|---|
| U1 | Etkin gecikme | $\delta_{\text{eff}} = \delta_{\text{obs}} + \frac{n^*-1}{2}T$ | $\delta + \frac{n}{2}T$ (ve per-axis $\delta + n_j T/2$) | Tek formülde birleştir |
| U2 | Pencere üst sınırı | $n_{\max} = \lfloor 2\delta_{\text{obs}}/T \rfloor + 1$ | $\lfloor 2\delta/T \rfloor$ (min 1) | +1 ekle |
| U3 | Pencere seçimi | AR(1) $V_j^U(n)$ admissibility + $\min\mathcal N_j$ | $n = \mathrm{clip}(\lceil(2\sigma/\varepsilon)^2\rceil, 1, n_{\max})$ | Korelasyon modülü yaz |
| U4 | Stale/bayrak | `stale_observation`, `is_admissible` | yok | Ekle |
| U5 | Başlangıç yumuşatma | $\psi(t)=\tanh^2(t/\tau_{\text{onset}})$ | yok (governor'da; HLSL'de $\phi(p)$ var) | Modele ekle |
| U6 | Broyden | interpolasyonlu halka, boyun ayrımı, Frobenius güven kümesi, leak | `BroydenCartesian` (tek skaler $\lambda$, interpolasyon yok) | Yeniden yaz |
| U7 | WDLS | $W^{-1}(\|e\|)$, $w_{\text{trunk}}(e)$, ikincil düzenleme | `human_wdls` (tork-bütçeli, sabit ağırlık) | Üzerine inşa et |
| U8 | El sinerjisi | 3 fazlı (Approach/Pre-Grasp/Closure) + Evaluation | yok | Yaz |
| U9 | Motor kayması | $\Delta_{\text{scale}}, b_{\text{drift}}, w_{\text{motor}}$, deadzone | yok | Simülatör{e ekle |
| U10 | Sıfır-oracle | yasak | `set_delay`, `rel_*_gap_m` açık | Faz 1'de kapat |
| U11 | Birimler | piksel/ $s$ | metre karışımı (`sigma` m, `eps` m; `sigma_cam_m`) | Sözşekme sabiti |

---

## Kapı G0 için karar metni

- Şartname **Faz 0'ı tamamlamıştır** (bu rapor). Kod değişikliği yok.
- **Öneri:** Faz 1'e geçmeden önce (i) U10'u (oracle sızıntısı) ve U1–U5'i (governor Pencere/δ_eff) kapsayan küçük bir "temizlik" commit'i, (ii) $d_{\text{neck}}$/el sinerjisi için veri modeli uzantısı onaylanmalıdır.
- **Uyarı:** Mevcut HLSL/HDA makalesi (v4) ile v4.3 şartnamesi **farklı programlardır**; v4.3'ün H1–H5'i (broyden/gaze/WDLS/sinergy) ile v4'ün H1–H5'i (gecikme-ölçekli yasa dayalı) aynı adda farklı hipotezlerdir. İsim çakışması önlenmeli.

---

## Kapı G1 — A3 kanıtı (16.11.2026)

**Karar kaydı:** `reports/a3_gate_G1.md` · **Kanıt:** `reports/a3_tier1.{json,csv,md}`, `reports/a3_pareto.{json,md}`, `reports/a3_tuning.json`

| Hipotez | Sonuç | Ölçüm |
|---|---|---|
| H1 (ayarsız ≤%1 aşma, MT ≤1.25×) | **FAIL** | δ=30 ms'de 1.07 %; 60–200 ms'de 0.57–0.99 % |
| H2 (dondurulmuş bazçizgiler δ≥100 ms'de bozulur) | **PASS** | dondurulmuş > %5 aşma / %12 oturma; `dps_1e` ≤ 0.78 % |
| H4a (göreli üstünlük, ≥1.5×) | **PASS** | 17.80 % vs 29.04 % → 1.63× |
| H4b (mutlak ≤%2, yanlış gecikme) | **FAIL** | 13.62 % ve 17.80 % |
| H5 (b/δ sabit ±%10) | **FAIL** | b 0.356→0.280 s/bit; b/δ yayılımı %211 |

- **Hüküm:** gecikme-türevli hız yasası `k = 1/(e·δ_eff)` **güvenli ama muhafazakâr bir üst sınır** olarak korunur; "tüm gecikmelerde ≤%1" ve "yanlış bildirilen gecikmede ≤%2" iddiaları daraltılır.
- **`b = β·δ_eff` ($\beta\approx0.86$) simülasyonda doğrulanmadı**; donanım protokolünde sınanacak açık öngörü olarak kalır. Fitts eğimi bu bitki modelinde gecikmeden neredeyse bağımsızdır.
- **H4** tek birleşik hükümden **H4a (göreli)** + **H4b (mutlak)** olarak ayrıştırıldı; ayrıştırma `scripts/run_a3.py` içinde kalıcıdır.
- **Faz 1 kodu** (`robosimu/v43.py`, `stereo.NeckKinematics`, `SimConfig.zero_oracle` yolu, `exact_group_delay=False` varsayılanı) bu kanıtla çelişmez; legacy `δ_eff = δ + nT/2` yolu ve 80 eski test korunur (toplam 92 test).
