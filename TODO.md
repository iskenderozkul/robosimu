# TODO — robosimu & İnsansı Stereo Perspektif (HLSL) Sistemi

Durum: `[x]` tamamlandı ve doğrulandı.

## Aşama 1 — Mimari Sadeleştirme ve Antropomorfik Kamera / Kol Geometrisi
- [x] **Omuz Üstü ve Hafif Gerisi İki Göz (Stereo) Konumlandırması (`robosimu/stereo.py`):**
  - Kameralar robotun karşısından alınıp insan anatomisine uygun olarak ilk eklemin (omuzun) üstüne ve hafif gerisine (`cam_pos = (-0.12, 0.0, 0.62) m`) yerleştirildi.
  - Kullanıcının kamera konumunu ($X_{\text{cam}}, Z_{\text{cam}}$), iki göz açıklığını ($B$) ve robot kol uzunluklarını (Üst kol $L_2$, Ön kol $L_3$) simülasyon ekranından canlı değiştirebilmesi sağlandı.
- [x] **Aktif Kamera Mikro-Hareketi (Paralks) ve Uzanma Öncesi Singularity Tahmini (`predict_reachability`):**
  - Kamera küçük bir mikro-salınım ($\Delta c$) yaparak hedefin optik paralks kaymasını ($\Delta u_{\text{par}}$) ve perspektif boyutunu ölçer; kolun mevcut uzunluklarıyla ($L_2, L_3$) singularity sınırına girmeden hedefe uzanıp uzanamayacağını ($D_{\text{est}} \le 0.88 \sum L_i$, $w_{\text{pred}} \ge w_{\min}$) hareket başlamadan önce tahmin eder.

## Aşama 2 — Elin Perspektifte Küçülmesi (Optical Looming) ve İki Göz Görünümü
- [x] **Sol Göz ve Sağ Göz Perspektif Pencereleri (`cvLeftEye`, `cvRightEye`):**
  - Hem hedef objeyi hem de robotun en ucundaki tutucuyu (grabber/el) iki gözde bir arada gösterir.
  - El omuzdan ileriye (hedefe) doğru uzandıkça kameradan uzaklaştığı için, elin görünür piksel çapının ($s_g(t) = f W_g / Z_g(t)$) **perspektifte küçülmesi** ve normalize ölçek oranının ($\rho_s(t) = \frac{s_g(t)/W_g}{s_t/W_t} \to 1.000$) hedefe yakınsaması hem Python motorunda hem de HTML simülasyonunda modellendi.

## Aşama 3 — PID ve LQR'a Alternatif Yeni İnsansı Kontrol Modeli (HLSL)
- [x] **HLSL (`HumanLoomingController` — İnsansı Çan-Eğrisi Hız Rampası + $1/e$ Gecikme Yöneticisi):**
  - Sabit kartesyen (X, Y, Z) basamak komutu yerine, ne zaman hızlanıp ne zaman yavaşlayacağını bilen çan-eğrisi modülasyonu $\phi(t, p)$ ile $k_j = \frac{1}{e \cdot \delta_{\text{eff},j}}$ monoton frenleme zarfını birleştiren yeni kontrol modeli tasarlandı ve `controllers.py` / `stereo.py` içine entegre edildi.
- [x] **Singularity ve Tork-Bütçeli Eklem Koordinasyonu (`human_wdls`):**
  - Motorların anlık tork/hız rezervini ($h_i$) ve manipülabilite gradyanını ($\nabla_q w(q)$) kullanan insansı eklem koordinasyonu doğrulandı.

## Aşama 4 — Modern, Sade ve Anlaşılır Simülasyon Ekranı (`figures/arm_viewer.html`)
- [x] **Klasik AI Tasarımından Arındırılmış Modern Bilimsel Arayüz (`robosimu/workbench_html.py`):**
  - Karmaşık sekmeler ve gereksiz detaylar kaldırıldı; sıcak laboratuvar kağıdı (`#f6f4ef`), yüksek kontrastlı tipografi (`Source Serif 4`, `IBM Plex Sans`, `JetBrains Mono`) ve herkesin ilk bakışta anlayacağı 3 sütunlu net iş akışı kuruldu.
  - Herkes için anlamlı 3 temel grafik (A: İnsansı Çan-Eğrisi Hız Rampası, B: Kamerada Elin Perspektif Küçülmesi $s_g \to s_t$, C: Singularity İndeksi $w(q)$ ve Omuz Torku) ve tek tıkla **HLSL vs. Governor vs. PID vs. LQR vs. Sabit Kazanç (P)** karşılaştırma tablosu eklendi.

## Aşama 5 — Q1 Akademik Makale (Önceki Makaleyi Koruyarak Kopyalama + Editör/Hakem Raporu)
- [x] Önceki makale dosyaları (`iskender-v3/manuscript/main.md` ve `main_tr.md`) silinmeden korundu.
- [x] Q1 Dergi Editörü, 2 Hakem ve Disiplinlerarası Okuyucu Değerlendirme Raporu (`review/EDITORIAL_REVIEW_v4_STEREO_LOOMING.md`) oluşturuldu.
- [x] Tıp, mekanik, robotik ve matematik okuyucularının rahatlıkla anlayabileceği yeni Q1 makaleler (`iskender-v3/manuscript/main_v4_stereo_looming.md`, `main_v4_stereo_looming_tr.md` ve `.docx` sürümleri) gerçek bilimsel referanslarla (Fitts 1954, Flash & Hogan 1985, Lee 1976, Nakamura & Hanafusa 1986, Harris & Wolpert 1998, Khatib 2004, Levine 2018, Chi 2023) yazıldı.

## Aşama 6 — İnsansı Ölçekli Kol (İnsan Uzuv Oranları) ve Sarkık Kol Başlangıç Pozunun Simülasyona Girilmesi
- [x] **`models/human_arm.json` — insan uzuv oranlarıyla 6R kol:** üst kol 0.31 m, önkol 0.26 m, el 0.12 m; omuz yüksekliği 0.90 m (gövde), kol kütlesi ~5 kg; `q2 = q3 = 0` ve tüm q = 0 **dikey sarkan kol** pozudur, yerçekimi torku ≈ 0.
- [x] **El bileği eklem eksenleri simüle edildi** (bilek fleksiyonu / ulnar-radyal deviasyon / pronasyon-supinasyon): askıdaki sarkan elin parmakları aşağı doğru yönelir (EE ekseni dikey).
- [x] **Model JSON'ına isimli başlangıç pozları gömüldü** (`hanging_natural` - dirsek hafif bükük, `hanging_vertical` - tam düz, `ready_desk` - masa başı, `reach_high` - yukarı uzanma); workbench "Kolun İlk Pozisyonu" dropdown'ı artık bu isimli pozlardan beslenir.
- [x] **`--posture`, `--q0`, `--target` CLI parametreleri** (`simulate`, `html`/`workbench` komutlarında) — kolu isimli pozla, serbest eklem açılarıyla veya mutlak Cartesian hedef koordinatıyla başlatmak mümkün.
- [x] **Sarkık koldan rafa uzanma demosu** (`--posture hanging_natural --target "0.45 0.0 1.10"`): governor 60 ms'de 2.6 s settling, %0 aşım, 0 doygunluk; sabit kazanç (30 ms'te tuned) 120 ms'de hiç yerleşmez (%4.5 aşım).
- [x] **Hız sınıfları belgelendi** (`docs/insans_hiz_profilleri.md`): refleks → hızlı → normal → dikkatli → hassas 5 sınıfın parametreleri, alt durumları ve çarpımsal modülasyon çarpanları; formül açıklamaları (κ çarpanı > 1/e = balistik istisna, φ frenleme eğrisi, δ_eff = δ + nT/2); insan motor literatürü ile ilişkilendirme.
- [x] `figures/human_reach_shelf.{gif,png}`, `figures/human_arm_workbench.html`, `tests/test_human_arm.py` (12 yeni test, toplam 60 test)

## Aşama 7 — Timeline Sıfırlama, Kanvastan Sürükleme, İki Göz Ekranının İyileştirilmesi
- [x] **Timeline sıfırlama butonu ("Başa Sar", `btnResetTimeline`)**: animasyon bir kez oynayıp sonuna gelince `frameIdx` sona takılıyor ve Oynat görünürde hiçbir şey yapmıyordu; artık "Başa Sar" timeline'ı t = 0'a döner ve Oynat da bitişten sonra 0'dan yeniden başlar.
- [x] **Hedef ve tutucu (el) kanvas üzerinde sürüklenebilir**: `Sürükleme düzlemi` X–Z / Y–Z / X–Y butonlarıyla kartezyen düzlem seçilir; hedef halkası sürüklendiğinde hedef konumu, tutucu sürüklendiğinde kolun ilk pozu canlı değişir.
- [x] **Tutucunun sürüklenmesi DLS ile IK çözer**: sürüklerken anlık sönümlü en küçük kareler ile 6 eklem çözülür; kol singularity sınırında sıkışmaz, eklem slider'ları güncellenir.
- [x] **Parametrik ayar (sol panel)**: 6 eklem slider'ı (derece) + mutlak `x y z` başlangıç el konumu alanları + "IK Çöz" butonu; her değişiklik anında sola (ve poza/kanvasa) yansır.
- [x] **İki göz ekranı iyileştirildi**: hedef üzerinde ortak fiksasyon çarpidazı; el ve hedef için monocular (boyuttan Z: `s ∝ f/Z`) ve binocular (uyumsuzluktan Z: `Z = f·B/Δu`) derinlik ölçümleri yan yana gösterilir, "uyum %" göstergesiyle iki göz doğrulamasının approach hareketini güncellemesi görselleştirilir; hafif periferik vinyet (odak dışı yumuşama) eklendi.
- [x] **Kuşbakışı stereo paneli (`cvStereoTop`)**: omuz, iki göz ve hedef/tutucunun yukarıdan görünümü; görme eksenleri ve kesişim noktası, `Δu_g`, `Z_disp`, `Z_size`, `uyum %` canlı metrikleriyle birlikte.
- [x] **"CNC" ismi kaldırıldı**: her yerde doğru teknik isimler kullanılır — sabit kazanç baseline'ı **Sabit Kazanç (P)**; parametrik yol + zaman kuralı için robotik terim **trajiktori** (parametrik trajiktori / hareket profili, trajiktori planlaması); hiçbir UI/docs/rapor/TODO/comment CNC kelimesi içermez.

## Aşama 11 — Gerçek 3B Pinhole Göz Görünümü (Göz Ekranlarının Baştan Yazılması)
- [x] **Göz ekranları artık 2B şema değil, gerçek zamanlı 3B projeksiyon**: her göz için ayrı pinhole kamera (göz bebeği konumu, toe-in fiksasyonu); gerçek oda — sıcak fildişi duvar, boy pencere (güneş lekesi + dikme), tahta zemin + perspektif ızgara, raf tahtası (damar çizgileri + ön yüz + süpürgelik), **terakota kupa** (gövde silüeti, ağız halkası + kahve, sap, speküler ışık şeridi, raftaki temas gölgesi); kol **gölgeli kapsüller** (omuz→dirsek→bilek→el, çift kat çizgi + yönsel gradyan) ve **küre eklem başları** (radyal gradyan + speküler nokta), üç parmak; elde derinliğe bağlı solma (depth-α); vinyet.
- [x] **Gaze modelleme hatası giderildi**: simülasyon döngüsü `updateGazeState` çağrısına her zaman el pozunu geçiyordu — gazeMode='target' (hedefe kilitli) modunda bile kafa eli takip ediyordu; artık 'target' = kafa hedefe sabit (hareketsiz), 'hand' = pürüzsüz pursuit. Görünen varsayılan = **Ele odaklı** (demo gözleği kolu takip eder; CLI/makale varsayılanı hedefe kilitli kaldı).
- [x] **Görüş alanının dürüst sınırı**: omuz-üstü kamera hedefe baktığında sarkık kol görüş alanının dışındadır (kafayı −18°'den fazla aşağı eğmez — insan gibi); kol uzanma başlayınca göz hizasına yükselir ve göz ekranlarına girer ("kol yükselir, göz ekranlarında belirir").
- [x] Doğrulama: render yolu exception'sız, 6/6 kontrolör iki sayfada hedefe ulaşıyor, 60/60 test OK, `node --check` temiz.

## Aşama 16 — Teorem: Üç Reddiye (enkoder yok, sayısal mesafe yok, sürekli bakış yok) — *tasarım, kod yok*
- [x] **Kavramsal belge:** `docs/teorem_ve_felsefe.md` — teorem tek cümlede, üç reddiye, klasik/teorem karşılaştırma tablosu, algı ve darbe sözleşmeleri, bakış durum makinesi (S0–S6 + refleks), teorem modu UI tasarımı, geçiş planı, açık sorular.
- [x] **Dürüst envanter:** mevcut simülasyonun nerede olduğu tablo hâlinde yazıldı — (1) kör eklem *kısmen* (bitki `q` paylaşıyor, resolved-rate IK `q_ref` kullanıyor), (2) sayısal mesafe yokluğu *henüz yok* (`rel_base_gap_m` metre olarak veriliyor), (3) kesintili bakış *kısmen* (sakkad görselleştiriliyor, kontrolü etkilemiyor), (4) hata+telafi *var* (E_init/T_comp), (5) teorem modu *yok*.
- [x] **Kabuller belgesine işlendi:** `docs/kabuller_ve_gereksinimler.md` §0 (üç reddiye + karşılaştırma tablosu + envanter) ve §7'ye İngilizce **Core thesis** paragrafı.
- [x] **Makale motivasyonları:** `main_v4_stereo_looming.md` §1.1b (İngilizce), `main_v4_stereo_looming_tr.md` §1.1b (Türkçe), `main_tr.md` §1.1'e kısa program paragrafı. `main.md` bilinçli olarak **dokunulmadı** (dondurulmuş v3).
- [ ] **Kod (sonraki adım, onay bekliyor):** ateş duvarı (`Observation`), göz nitelik paketi, bakış durum makinesi + sample-and-hold, darbe aktüatör arayüzü, darbe tablosu + İlk Uyanış kalibrasyonu, teorem modu UI, klasik/teorem yan yana karşılaştırma.

## Aşama 20 — MASTER_SPEC v4.3 (İskender-V4) — yeni yol haritası
- [x] **Faz 0 (zorunlu ilk adım):** `md docs/MASTER_SPEC_V4_3_ROBOSIMU.md` okundu; `reports/v4_audit.md` üretildi (6 soru, kaynak satırlarıyla). **Kapı G0: onaylandı.**
- [x] **Faz 1 — Çekirdek prototip (kod):** `robosimu/v43.py` (`V43Controller`, doğrusal interpolasyonlu `EfferenceRingBuffer`, sıra-korumalı Broyden, `outputs_joint_velocity`), `stereo.NeckKinematics` (2-DOF boyun, `J_neck: 3×2`), `SimConfig.zero_oracle` yolu; `exact_group_delay=False` varsayılanı legacy `δ_eff = δ + nT/2` davranışını korur. `tests/test_v4_3_core.py` (12 test, 5'i zorunlu) → toplam **92/92 OK** (96 s).
- [x] **Faz 2 — Korelasyon farkındalıklı governor (kod):** AR(1) kalibrasyonu, $V_j^U(n)$ ve $\min\mathcal{N}_j$ pencere seçimi, `exact_group_delay=True` yolunda $\delta_{\text{eff}} = \delta_{\text{obs}} + (n^*-1)T/2$, $k = 1/(e\,\delta_{\text{eff}})$ ve bayat veri dondurması.
- [ ] **Faz 3 — Tam mimari:** bel–kol WDLS, motor kayması ($\Delta_{\text{scale}}$, $b_{\text{drift}}$) ve 3-aşamalı el sinerjisi (H2, H4, H5) — *WDLS/sinerji kısmı açık*.
- [ ] **Faz 4 — Tam benchmark + raporlama:** eşleşik Monte Carlo, Wilson %95 GA, bootstrap, yayın figürleri.
- [x] **Görsel/illüstrasyon işi v2 (16.11.2026):** iki panelli 920×404 şema sıfırdan çizildi — Panel A ölçülü L-zincir ($d_1$, $y_0$, $L_2$, $L_3$, $L_{\text{el}}$), $\theta_{\text{bel}}$ yayı, $R_{\max}$ yayı (30°–90°), $F_0/F_{ee}/F_t$ üçlüleri; Panel B göz/retina/ışın/$\Delta u$/$Z$ + boyun paralaksı alt paneli + 6-DOF şerit. Etiketler beyaz hale (`lab-halo`) ile ışınların üzerinde okunur kalır; ölçü çizgileri 90° dirsekli; şema yazı tipi Times New Roman; sayılar Türkçe ondalık virgülle.
- [x] **Tek kaynak optik/kol ölçüleri:** `B = 65 mm`, `f = 580 px`, `L_el = 12 cm`, $R_{\max} = 60{,}7$ cm; sembol rehberi, parametre sözlüğü, pinhole örneği ($s_t \approx 56$ px) ve uzuv profili etiketleri (12/13/10 cm) bunlarla hizalandı.
- [x] **Donanım kartları:** `A. Kamera/Göz Rigi` (ön + yan görünüm, ölçülü $\Delta X = -10$ cm ve $\Delta Z = +22$ cm) ve `B. Aktüatör + Tutma Freni` (kesit + $\tau_b$ hapları: j₁ 1,0 · j₂ 42,6 · j₃ 14,6 · j₄ 1,6 · j₅ 1,0 · j₆ 0,05 Nm).
- [x] **Makale §2.8 temizliği:** EN/TR başlıklarından "ve Durum" çıkarıldı; Tablo 2.8a artık **şartname** tablosu (durum/denetim sütunu yok); "henüz koşturulmadı", "bugün ihlal ediliyor", "bekliyor" gibi geliştirme/durum ifadeleri metinden çıkarıldı, H1–H5 "yanlışlama matrisi" olarak yeniden yazıldı. Donanım ölçüleri $B=65$ mm, $f=580$ px, $L_{\text{el}}=12$ cm'e çekildi (EN §2.8, TR §2.8 ve Boxed Assumptions). `lint_manuscript.py` PASS.
- [x] **A3 kapı G1:** H4 tek birleşik hükümden **H4a (göreli üstünlük, PASS)** + **H4b (mutlak ≤%2, FAIL)** olarak ayrıldı (`scripts/run_a3.py` içinde kalıcı); `reports/a3_tier1.md` yorum bölümüyle yeniden üretildi; `reports/a3_gate_G1.md` karar kaydı yazıldı; `reports/v4_audit.md` G1 bölümü eklendi. **H1/H4b/H5 FAIL olarak korunur**; hız yasası "güvenli ama muhafazakâr üst sınır" olarak çerçevelenir; `b = β·δ_eff` simülasyonda doğrulanmadı, donanımda sınanacak açık öngörü.
- [ ] **Push edilmedi** — kullanıcı onayı bekliyor; `.docx` yenilemesi bekliyor.

## Aşama 19 — v5 Görsel/Açıklayıcı Katman: SE(3) İllüstrasyonu (yeniden çizim), Sembol Rehberi, Matematik İllüstrasyonları, Başa Sar + Benchmark Düğmeleri, Benchmark Akademik Protokol (push kullanıcı onayı bekliyor)
- [x] **Zoom hatası giderildi:** 3B sahnede sabit piksel bedenler artık `PU = state.pxPerUnit` ölçekleniyor — kafa, boyun, omuz kapağı, klavikula, göz çukuru, iris/pupil, burun/ağız, kaynak etiketi fontu tutarlı. (Kafa eskiden `headR = 34` sabitti.)
- [x] **Panel A (3B Uzaysal SE(3)) yeniden çizildi:** izometrik ızgara, pedestal, θ_bel kadranı, y₀/d₁ ölçü hatları, kol zinciri, R_max küre yayı, F_ee ve F_hedef çerçeveleri, SO(3)/SE(3) canlı bilgi şeridi, örnek sembol satırı — tümü tutarlı kalınlık/renk dili (`workbench_html.py`).
- [x] **Panel B (Göz İzdüşümü):** altbilgi satırına sembol açıklamaları eklendi (E_L/E_R, B, B_boyun, f, s_g/s_t, ρ_s, Δu_mp, F_ee/F_hedef).
- [x] **Global "Sembol Rehberi" çekmecesi:** başlık çubuğunda her yerden açılır; 4 grup (Uzamsal Kinematik / Optik-Stereo / Kontrol / Biyomekanik-Kıyas), ~64 giriş, formül-İngilizce anlamlarıyla birlikte.
- [x] **Matematik mini-kartlara illüstrasyon:** her 4 karta da 150×92 inline SVG eklendi (similar-triangles looming, monoküler bardak çifti, gecikme/k·δ>1/e vs ≤1/e aşma eğrileri, j₁–j₃↔J_v / j₄–j₆↔J_ω bölünümü).
- [x] **Bakış zaman çizelgesi (diagram içine gömülü):** S0 ARA → S1 ELE → S2 HEDEFE → S3 KÖR UÇUŞ (δ) → S4 YENİDEN BAK → S5 KOMP → S6 KİLİT; altına diyagram omurgası açıklaması eklendi. "XYZ Koordinatsız Retinal Looming Kontrol Döngüsü" matematiksel olarak doğrulandı (k ≤ 1/e, Z = f·B/Δd, ρ_s = (s_g/W_g)/(s_t/W_t), e_ω = ½Σ x̂×x̂_d, [v;ω] = J(q)θ̇).
- [x] **Başa Al / Akademik Benchmark düğmeleri sahne HUD'una eklendi** (Başlat'ın yanında): `Başa Al` 0. kareye sarar; `Akademik Benchmark` Sekme 5'e geçip JS 5-senaryo testini çalıştırır (`window.__activateTab`).
- [x] **Zoom'da telemetri/eksen etiketleri ölçeklendi**; Tam Ekran overlay artık overlay açılıp kapandığında kanvasları yeniden oturttur (623→923 px testi PASS).
- [x] **Benchmark metodoloji kartı (Tab 5):** görev gerekçesi + 15 kaynakla (Fitts 1954, MacKenzie 1992, ISO 9241-9, Crossman & Goodeve 1983, Flash & Hogan 1985, Corless 1996, Yoshikawa 1985, Nakamura 1986, Wampler 1986, Khatib 2004, Efron 1979, Holm 1979, Wilson 1927, Good 2005, Woodworth 1899).
- [x] **Makale:** EN §3.7 ve TR §3.7'ye "Protocol justification / Protokol gerekçesi" paragrafı + Kaynaklar'a 26–34 arası 9 yeni atıf eklendi (kitap içi atıf biçimiyle).
- [x] **Doğrulama:** 70/70 birim testi, başsız UI 39/39 PASS, sıfır JS hatası (başsız DOM taraması), tam sayfa 1920×3450 ekran görüntüsü (kontrollü).
- [ ] **Push edilmedi** — kullanıcı "yükleriz" diyene kadar beklemede (bkz. Aşama 18 notu).

## Aşama 18 — Akademik Benchmark v1.1 (10 tohum, %95 GA, Holm) + Teorem Şeridi Dalgalanma Düzeltmesi — *yapıldı, push kullanıcı onayı bekliyor*
- [x] **Dalgalanma (flicker) düzeltmesi:** `.theorem-strip` `flex-wrap: nowrap` + `overflow: hidden` + sabit 29 px yükseklik; `.th-badge` nowrap/flex:0; ilk rozet `text-overflow: ellipsis`; `TEOREM KİPİ AKTİF` metni kısaltıldı (eski ~200 karakterlik tek satır taşıyor/sarıyordu). Metrik rozetleri artık eklendiğinde bile yerleşim değişmiyor.
- [x] **Benchmark v1.1 (`robosimu/benchmark.py`):** hücre başına %95 **bootstrap güven aralığı** (e_f; 10 000 tekrar, seed=42), `settled_rate_pct` + `final_error_mm_ci95_lo/hi` alanları, 25 eşleşik karşılaştırma (hdap vs hlsl/governor/pid/lqr/fixed, 5 senaryo), **Holm step-down** düzeltmeli iki-yönlü log-ortalama permütasyon testi (10 000 permütasyon, seed=42), yerleşmeyen koşullar senaryo süre tavanında skorlanır. `--seeds` CLI varsayılanı 10.
- [x] **Rapor yenilendi:** `python -m robosimu.cli benchmark models/human_arm.json --out reports --seeds 10` → 32 satır (~13 dk); `reports/academic_benchmark.md|.csv|.json` güncel; satırlara Oturma % sütunu eklendi (S2 pid 10/10'da yalnız 1 yerleşme, hdap %100).
- [x] **Makale:** EN §3.7 + TR §3.7 — `seeds=[0..9]`, Tablo 5 sayıları tazelendi, GA95 + `p_Holm` protokolü, bulgu (iv) holm anlamlı/anlamsız karşılaştırmalar, **`https://github.com/iskenderozkul/robosimu`** açıkça yazıldı, CLI komutu `--seeds 10`; okuyucu CLI ve Tab 5 tarayıcı adımlarıyla sonuçları kendisi görebilir.
- [x] **Doğrulama:** 70/70 test, `lint_manuscript.py` PASS, workbench yeniden üretildi, Teorem Kipi tek satır/29 px (DOM ölçümü).
- [ ] **Push edilmedi** — kullanıcı "yükleriz" deyince atılacak (`a647906`'dan beri tüm çalışma commit bekliyor).

## Aşama 17 — v5 Enstrümantasyon Paketi: Frenler, x0.1 Telemetri, Koniler, Hayalet Robot — *tasarım yazıldı, kod onay bekliyor*
- [x] **Doküman:** `docs/teorem_ve_felsefe.md` §2.4 (fren + efference copy + gözün ele baktığı dört istisna), §4.5 (v5 arayüz/gösterim paketi), §6 (M1–M10 modüler sıra ve bağımlılıklar).
- [x] **Kabuller:** `docs/kabuller_ve_gereksinimler.md` §3.1c (frenler, efference copy, dört istisna, dürüst durum notu) + §2 sensör tablosuna "eklem tutma freni" satırı.
- [x] **Makaleler:** EN §2.7 + §3.6, TR §2.5 + §3.3 — kör başlangıç, dört istisna, biçimsel bakış tetiği, dört karşılaştırma ölçütü ($E_{init}$, $\%T_{comp}$, $L_{vis}$, $\eta_{pulse}$) ve enstrüman/durum tablosu. "Kapsam" paragrafları ile ölçülen–taahhüt ayrımı korundu.
- [x] **M1 — Zaman ölçeği + telemetri HUD:** `x1 / x0.75 / x0.5 / x0.25 / x0.1` oynatım hızı düğmeleri eklendi (**yalnızca oynatım**; fizik ve metrikler değişmez — kullanıcı onayıyla). Sahne HUD'u: zaman, bakış etiketi (EL/HEDEF/TARAMA; yörüngede kare başına saklanıyor, `tr.gaze`), ρ_s, disparite, δ gecikme, fren durumu. *Not: "darbe (grup, ms)" alanı M6 (pulse aktüatörü) gelmeden eklenemez; bakış etiketi göreli 3B derinlik konileri gibi sayısal değildir.*
- [x] **M2 — Fren paneli:** `scripts/compute_brake_torques.py` ile `brake_torque_nm` otomatik hesaplandı (τ_b = 1.5 × max|τ_g(q)|, 15.000 rastgele örnek + tüm postürler; j1=1.0, j6=0.05 Nm alt sınır) ve `models/human_arm.json`'a yazıldı. UI: `Frenler AKTİF (Tutma)` / `Serbest (Enerji Kesintisi)` düğmeleri; eklem başına τ_b girdileri + **Otomatik** onayı (açıldığında girdiler kilitli); SERBEST'te Simülasyonu Başlat devre dışı, kol sarkık pozisyona alınır, banner uyarı verir; İlk Uyanış tamamlanınca frenler **kendiliğinden yeniden devreye girer**. *Dürüst sınır: serbest düşmenin dinamik entegrasyonu yok — poz, sarkık postüre atlanıyor (aynı biçimde etiketlendi).*
- [x] **Arayüz — üst sıra yeniden dizilimi:** gözler üstte tam genişlik yan yana (SOL | SAĞ), 3B sahne altında tek parça; dikey `eyesSplitter` ile göz/sahne yüksekliği sürüklenebilir. Hedef: "iki gözü ve hareketi en ideal şekilde görebilmek". Alt görünüşler ve alt ayar şeridi korundu; 1920×1080'te sahne 344 px'e çıktı.
- [x] **Arayüz — global ayar yönetimi:** üstte `💾 Kaydet` (tüm ayarları `localStorage`'a yazar) ve `↺ Varsayılanlar` (kaydı siler, hepsini varsayılana döndürür); ayar satırlarının etiketine **ayar-başı ↺ sıfırlama düğmesi** (37 adet, otomatik ekleniyor).
- [ ] **M3 — Ateş duvarı:** `Observation` nesnesi; kontrolcü ile bitki arasındaki `q`/`p` paylaşımı kesilir (yalnızca nitelik paketi + efference kestirimi).
- [ ] **M4 — Nitelik paketi:** $\rho_s = s_{el}/s_{hedef}$ eşikleriyle `çok_uzak…temas`; `sol/orta/sağ`, `üst/orta/alt`, `önde/hizada/geride`, `görünürlük`, `güven`; eşiklerde histerezis (sınır titremesini önlemek için).
- [ ] **M5 — Bakış durum makinesi:** S0 Ara → S1 Ele bak → S2 Hedefe bak → S3 Kör uçuş → S4 Yeniden bak (önce el, sonra hedef) → S5 Kompanzasyon → S6 Kilitle; sample-and-hold + sakkadik bastırma; HUD'da faz ve bakış zaman çizelgesi.
- [ ] **M6 — Darbe aktüatörü:** `(grup, süre, kutup)` → tork zarfı; HDA zarfı darbenin *içinde* (darbe = HDA hız zarfının ayrık integrali); `qd_ref` yolu kaldırılır; kas enerji akışı görselleştirmesi.
- [ ] **M7 — Darbe tablosu + İlk Uyanış kalibrasyonu:** kaba elle tablo (`nitelik, aciliyet → süre`) + mikro darbelerle görsel ince kalibrasyon; eksik tablo = büyük ilk hata (istenen davranış).
- [ ] **M8 — Teorem modu UI:** sayısal koordinatlar gizli; göreceli derinlik konileri (tepe açısı oranı = $\rho_s$), `ELE BAKIYOR / HEDEFE BAKIYOR / KÖR UÇUŞ` faz etiketi, bakış sayacı; DOM'da sayısal konum dizgisi kalmadığını doğrulayan otomatik test.
- [ ] **M9 — Hayalet Robot + 4 eksenli kıyaslama:** aynı sahne/hedefte enkoder+IK+PID ikizi (yarı saydam); $E_{init}$, $\%T_{comp}$, görsel işlem yükü $L_{vis}$, darbe enerji verimliliği $\eta_{pulse}$ grafikleri.
- [ ] **M10 — Ablasyon + istatistik:** teorem mimarisi vs aynı kontrolcüye metre + sürekli bakış verilen koşul; beklenti: ikinci koşulda balistik/kompanzasyon imzası kaybolur, birinci koşul 60→120 ms gecikmede yeniden ayarsız çalışır. $N$ tekrar, dağılım + bootstrap/Wilson.

## Aşama 15 — Yan Yana Göz+Sahne, Alt Sırada Üstten/Yandan Görünüş ve Tam Ölçeklenebilir Paneller
- [x] **Üst sıra**: SOL göz | SAĞ göz | 3B robot sahnesi yan yana (kullanıcı isteği: "sağ sol ve robot simülasyonunu yan yana"). Her biri kendi pencere kromuna sahip (macOS noktaları, − daralt, ✕ kapat).
- [x] **Alt sıra**: yeni **YANDAN GÖRÜNÜŞ (X–Z)** paneli (ortografik, omuz→dirsek→bilek zinciri, j0..j6 etiketleri, hedef/tutucu, 0.2 m ölçek çubuğu) + mevcut kuşbakışı **ÜSTTEN GÖRÜNÜŞ (X–Y)** paneli yan yana.
- [x] **4 sürüklenebilir ayırıcı**: gözler↔sahne genişliği, üst sıra↔görünüşler yüksekliği, üstten↔yandan genişliği, görünüşler↔ayar şeridi yüksekliği. Hepsi klavye/fare ile canlı ölçekler, `ResizeObserver` benzeri yeniden çizim.
- [x] **Kanvaslar kapsayıcıya tam oturur** (`fitCanvasSize`): göz kameraları, 3B sahne, üstten ve yandan görünüş artık gerçek en-boy oranında ve tam alan kullanır (siyah letterbox yok).
- [x] Header'da 5 panel geri-gösterme düğmesi: Sol Göz / Sağ Göz / 3B Sahne / Üstten / Yandan.
- [x] Sahne telemetrisi sağ-üste, "Kafa (Stereo Gözler)" etiketi kafanın sağına taşındı; HUD düğmeleriyle çakışma giderildi.
- [x] Doğrulama: **20/20 tarayıcı etkileşim testi** (5 kanvas, 4 ayırıcı, kapat/geri getir, daralt/genişlet, yatay taşma yok, **tam ekran aç/kapa**), 61/61 Python testi, node syntax OK, 7 kontrolör yakınsıyor.
- [x] **Tam ekran overlay onarımı (kritik HTML hatası):** Uzuv düzenleyici (`#limbPanel`) kabuğunda `</section>` yazım hatası vardı; `#sceneOverlay` bu panelin *içine* yuvalanıyordu. Panel `display:none` olduğu için overlay `0×0` kalıyor ve tam ekran boş açılıyordu. Kapanış `</div>` ile düzeltildi (overlay artık `body` çocuğu, 1887×984).
- [x] **Tam ekranda kanvas oturtma:** `fitCanvasSize` overlay sırasında erken dönüyordu; göz kanvasları 620×280 bitmap olarak `object-fit:contain` ile oturup yanlarda siyah bant bırakıyordu. Overlay'de de oturtma etkinleştirildi (göz kanvası 921 px = kutu genişliği).
- [x] **Duyarlı yükseklikler:** `--views-h: clamp(150px, 25vh, 300px)`, `--bottom-h: clamp(120px, 17vh, 190px)` ve `.row-views`/`#layerBottom` için `flex-shrink`. 1366×768'de üst sıra 266 px (önceden 180 px), 1920×1080'de 453 px.
- [x] **Küçük ekran cilası:** `#pwScene.compact` (<760 px genişlik veya <360 px yükseklik) HUD'u küçültür; 900 px altı yükseklikte sahne içi ipucu/açıklama satırları gizlenir (kanvas 117→170 px). Sahne telemetrisi 640 px altında kısa biçime, 640–760 px arası tam biçime geçer; "Kafa (Stereo Gözler)" etiketi <700 px'de çizilmez.
- [x] **`<select id="selController">` `hud-select` sınıfı eklendi:** genel `select { width:100%; background:var(--card-alt) }` kuralı HUD seçicisini beyaz/tam genişlik yapıyordu; artık koyu, 330 px (compact 250 px).


## Aşama 14 — Pencere Kromu (macOS-stili başlık çubukları) ve Daralt/Kapat Paneller
- [x] **Üç ana panel gerçek pencere kromuna kavuştu**: koyu lacivert gradyan başlık çubukları (sol: kırmızı/sarı/yeşil noktalar), ✕ kapat ve − daralt/genişlet (panel-body'yi gizler), ortalanmış başlıklar.
- [x] **Üst ince formula şeridi**: "1. Elin Perspektif Küçülmesi / 2. Hedef Boyu & Paralks / 3. Perspektif Ölçek Yakınsaması" metni GÖZLER panelinden alınıp header'ın hemen altına, tek satırlık ince şerit olarak taşındı (ekran sığdırımı).
- [x] **Header'da panel göster/gizle düğmeleri** (Gözler / 3B Sahne / Kuşbakışı) — kapatılan panel tek tıkla geri gelir.
- [x] Göz paneli varsayılan yüksekliği 215px; splitter'la sürüklenebilir aralıklar 120–460px (her iki splitter aktif).
- [x] Doğrulama: 61/61 test OK, node syntax OK, tüm kontrolörler yakınsar.

## Aşama 13 — 3 Katmanlı Arayüz, İlk Uyanış Kalibrasyonu, E_init/T_comp Metrikleri ve Analitik Dashboard
- [x] **3 katmanlı arayüz düzeni**: **Üst** = GÖZLER paneli (SOL/SAĞ kamera + kuşbakışı, simülasyonun hemen üstünde yan yana, tam genişlik grid; 14" 1080p sığması için 620×380'e çekildi), **Orta** = 3B sahne (Başlat/Durdur, İlk Uyanış ve Kontrol Modeli düğmeleri artık sahnenin içinde yarı saydam HUD çipleri olarak — üst menüdekiler kaldırıldı), **Alt** = tüm ayar gruplarının yatay şeridi (scrollsuz, esnek sarım). ⛶ overlay **üstte gözler + altta 3B robot** olarak yeniden düzenlendi (önce gözler yan yana, altında simülasyon — gözler artık robotun yanında değil üstünde); tüm id'ler korundu, 61/61 test OK.
- [x] **1080p civarı sıkıştırma (max-height medya sorgusu)**: başlık/alt bilgi çubukları küçültüldü, assessment bar sadeleştirildi.
- [x] **İlk Uyanış (Self-Calibration) düğmesi ve 3 durumlu başlangıç algoritması**: Durum 1 (fren var → prior = son duruş), Durum 2 (frensiz → yerçekimi sarkma dengesi), Durum 3 (mikro darbeler → optik akışla el tespiti → visuomotor eşleme). `state.calibration` ile darbe sayısı/süre/konum hatası kaydı; akış banner'da canlı gösterilir.
- [x] **Başlat/Durdur tek düğmede**: çalışırken buton "Durdur"a döner, durdurunca "Simülasyonu Başlat".
- [x] **Refleks döngüsü 20–50 ms'ye çekildi** (`retroDelay = clamp(delay×0.35, 20, 50)`) — insan refleks arkı seviyesi.
- [x] **Yeni metrikler**: `eInitMm` (tepe-hız anında |hedef − el|), `tBallistic`, `tComp`, `tCompPct` — Python/JS motorlarında aynı tanım; karşılaştırma tablosuna 2 sütun (E_init mm, T_comp %), KPI kartlarına 2 hücre, Markdown/CSV raporlarına dahil.
- [x] **Analitik Dashboard (4 akademik grafik, CSV/PNG dışa aktarım)**: A) zaman–hata + faz ayrımı (balistik=turuncu/kompanzasyon=turkuaz), B) E_init model karşılaştırması (ort ± 1σ, 10 tekrar/model), C) %T_comp yığılmış sütun (4 mesafe × 3 model), D) darbe süresi–açısal sapma saçılım (7 süre × 8 tekrar). PNG = birleşik 2×2 görüntü, CSV = tüm veri kümeleri.
- [x] **Doğrulama**: node syntax OK · tüm kontrolörler iki sayfada yakınsar · e2e duman testi: dashboard 62 sim × hesaplanıyor, kalibrasyon 6/6 darbe & visuomotor hazır.
- [ ] Görsel teyit kullanıcıda: Ctrl+F5 ile göz paneli boyutları, HUD çipleri ve dashboard grafiklerinin ekran görüntüsü gerekiyor.

## Aşama 12 — Gerçekçi Göz Ekranları, Kafa–Kol Ayrımı ve İnsansı Dinamik Yaklaşım (HDA)
- [x] **Göz ekranları gerçek 3B pinhole projeksiyona çevrildi** (2B şema değil): oda (duvar + yan duvar + boy pencere + güneş lekesi + süpürgelik), tahta zemin + perspektif ızgara, damarlı raf, terakota kupa (gövde helis/koyu-açık gradyan, ağız halkası + kahve, sap, temas gölgesi, speküler şerit), kol gölgeli kapsül segmentleri (omuz→dirsek→bilek→el) + küre eklem başları + 3 parmak; el izi turkuaz; fiksasyon retikülü; kompakt telemetri yongası (Z_boy/Z_disp + uyum%); vinyet.
- [x] **Kafa–kol düzlemi ayrımı uygulandı**: `models/human_arm.json`'a `base_origin = [0, +0.19, 0]` (sol omuz); gövde/kafa y = 0 merkezli, kol y = +0.19'de; yerçekimi torku sarkık pozda ≈ 0 kalır (doğrulandı).
- [x] **İnsan kafası simülasyonda çizilir**: y = 0 merkezde, z ≈ 1.13 m; gölgeli küre + göz yuvaları (beyaz + iris + pupil + speküler nokta) + burun + ağız; gözler gaze ile döner (yaw ±32°, pitch kısıtlı); shoulder-girdle (kol yaka) 3B'de açıkta — kafa ile kol düzlemde değil.
- [x] **`auto` bakış modu (varsayılan)**: kol-hedef arasındaki açısal separasyona göre hedefe↔ele sırayla bakar (20° üstü → 0.26 s tutuş, 8–20° → 0.42 s tutuş, <8° → mid sabit, son 12 mm → hedef kilidi) — insanın uzanırken önce kupayı, sonra eli izlemesi; `mid` / `target` / `hand` hâlâ seçilebilir; defaults güncellendi.
- [x] **HDA — İnsansı Dinamik Yaklaşım (`hdap`)**: hız zarfı artık sınıfa ait sabit değil, **mesafe+ciddiyete** göre değişir: `v_cap(D) = v_max · clamp(sqrt(D/0.30), 0.35, 1)`; sinyal-bağımlı komut gürültüsü (JS), asimetrik çan (kalkış hızlı → geniş tepe → yumuşak kuyruk), son fazda sınıf-natural sönüm; **trifazik aktüasyon sıralaması** (proksimal→distal→ko-kontraksiyon) tüm eklemlerde hız-kap çarpanı olarak uygulanır (Python+JS, IK çıktısıyla birlikte, motor hız limitleri asla aşılmaz).
- [x] **Gaze bug giderildi**: simülasyon döngüsü `updateGazeState`'e her zaman el pozunu geçiyordu — `target` modunda bile kafa eli takip ediyordu; artık `target` = kafa sabit, `hand` = pursuit, `mid` = ikisi, `auto` = akıllı alterasyon.
- [x] **Kabuller doc güncellendi**: §3.1 (göz modu), §3.3 (HDA algoritması — formül+oranlar), §3.4 (trifazik sıralama + "aktüatör sıralaması insanla uyumlu olmalı" gereksinimi), §3.5 (sakkadik alterasyon), §3.6 (`base_origin`), §3.7 (dinamik hız haritası özeti), §4.2 (davranış beklentileri), §7 (İngilizce Assumptions paragrafı: HDA + triphasic + base_origin + auto-gaze dahil).
- [x] **Makale (`main_v4_stereo_looming.md`) güncellendi**: §2.5 (insan ölçeği kol, anatomik limit tablosu, DH çapası sınırı), §2.6 (HDA: hız zarfı formülü + trifazik tablo + sinyal-bağımlı gürültü + alterasyon kuralları), §3.5 (asılı koldan raf deneyi + Fig. 9 + gerçek sayılar), §1.3+Abstract (6. katkı), §4.3 (Boxed Assumptions), Conclusion.
- [x] Doğrulama: 60/60 test OK · hdap @60ms: 2.73 s / %0.16 aşım / 1.5 mm · hdap @120ms: 2.87 s / %0 / 1.0 mm · hdap JS: 2.13 s · tüm kontrolörler hedefe ulaşıyor · `node --check` temiz.
- [x] **Göz ekranları büyütüldü**: 340×210 → 470×290 (varsayılan), overlay'de 880×544; iki göz + kuşbakışı ekranda daha geniş yer kaplıyor (CSS width:100%).

## Aşama 10 — Sahne Tam Ekran (3B Kart + Gözler + Temel Ayarlar), Local UCS Döndürme ve Kullanıcı Eklem Sınırı Kaydı
- [x] **3B İnsansı Robot Kol kartı tam ekranı** (`⛶` kart başlığında): tüm sayfayı kaplayan yerleşik **overlay** — ortada 3B sahne (1440×800 doğal çözünürlük), sağda **SOL + SAĞ göz kamerası ve kuşbakışı görüş** + canlı **Akademik Performans Özeti**, solda yalnızca temel ayarlar (Kontrol Modeli + hız sınıfı + hedef X/Y/Z + Algı Gecikmesi + Gürültü + Payload + ayrıntısız <details> rampa ayarları); grafikler ve karşılaştırma tablosu gizlenir; `Esc` / ✕ Kapat ile kapanır. Overlay, DOM hareketini kullanır (kanvas kimlikleri korunur) — Fullscreen API'ye bağımlı değil, her tarayıcıda çalışır.
- [x] **Local UCS ekseni etrafında DÖNDÜRME modu** (`⟲ Eksen Etrafında Döndür` / `↔ Eksende Ötele` seçicisi): seçili eklemin lokal eksen oku etrafında tüm kolu döndürür — açı, ekran-merkezli yaydan `atan2` ile okunur; EE hedefi, seçili eksen doğrusu etrafında döndürülmüş başlangıç EE noktası olarak IK'ya verilir ve sönümlü IK ile çözülür; **eklem limitlerine her adımda uyulur** (büyük aşırı-sürüklemede tüm q sınırların içinde kaldı).
- [x] **Eksen kilidi onarımı**: hedef erişilemez olunca kol doğruyu bırakıyordu; ikinci aşamalı IK (focus'u başlangıç doğrusuna en yakın noktaya çek) ile geniş sürüklemede **perp ≤ 5.4 mm**. IK artık her adımda bir önceki çözümle başlatılıyor (eksen kayması sürekli).
- [x] **Kullanıcı eklem sınırı düzenleyici** (`2c. Eklem Sınırlarını Düzenle`): 6 eklem × alt/üst sınır girişi; **Uygula** anında simülasyona/IK'ya/kaydırıcılara işler ve mevcut pozu kelepçeler; **Kaydet** ile isimli profil localStorage'a; **Yükle / Sil**; **JSON İndir** ile gerçek yeni model kaydı (`--model dosya.json` ile CLI'da kullanılabilir). Tarayıcıda uçtan uca doğrulandı (değiştir/uygula/kaydet/yükle/sil/sıfırla).
- [x] **Tarayıcı doğrulaması**: gizmo snap seçimi, eksen sürükleme, döndürme sürükleme, sınır düzenleyici, overlay aç/kapat — tümü headless Chromium'da gerçek mouse olaylarıyla test edildi.

## Aşama 9 — İnsan Kolu 6 DOF'unun Anatomik Eksenlere Oturtulması ve Tam Ekran Modu
- [x] **Eklem DOF'ları gerçek eksenlerine göre yeniden etiketlendi ve limitlendi** (DH zinciri sayısal doğrulandı):
      j4 → **bilek fleksiyon/ekstansiyon** (ekseni mediolateral, eski isim `wrist_roll` yanlıştı): −70° … +75° ·
      j5 → **bilek radyal/ulnar sapma** (ekseni antero-posterior): −40° (ulnar) … +20° (radial) ·
      j6 → **el eksenel rotasyonu** (ekseni ön kol): ±30° (eski ±110° anatomik olarak imkânsızdı).
- [x] **DH çapası dürüst sınırı belgelendi**: dikey-gövde/yatay-omuz DH çapasında **gerçek önkol pronasyon/supinasyon ekseni elde edilemez**; glenohumeral abdüksiyon/addüksiyon ve izole omuz rotasyonu da modellendi değil. `models/human_arm.json` açıklamasına, `docs/kabuller_ve_gereksinimler.md` §3.2a + İngilizce Assumptions paragrafına yazıldı (makalede olduğu gibi görünür).
- [x] **`ready_desk` pozu** j5=25° → 15° (yeni anatomik sınıra uyduruldu); tüm isimli pozlar limit içinde; FK değişmedi (aynı DH), 60/60 test OK.
- [x] **Tam Ekran modu (`⛶ Tam Ekran` başlık düğmesi, Esc ile çıkış)**: 3B sahne doğal 1440×800 çözünürlükte büyür; sağda SOL/SAĞ göz ekranları + kuşbakışı görüş (680×420) ve canlı Akademik Performans Özeti kalır; solda yalnızca temel ayarlar görünür (Kontrol Modeli + hız sınıfı + Detaylı rampa ayarları <details> içinde gizli + Hedef X/Y/Z + Algı Gecikmesi + Gürültü + Payload + süre slider'ı + 4 hazır senaryo); grafikler ve karşılaştırma tablosu gizlenir; tarayıcı Fullscreen API yoksa sınıf tabanlı yerleşim devreye girer.
- [x] **Altı eksenli endüstriyel görüntüleyiciler** (`figures/arm_viewer*.html`) yeniden üretildi; insan sayfası değişmedi.

## Aşama 8 — İnsan Kolu Serbestliklerinin (Eklem Limitlerinin) Anatomik Sınırlara Oturtulması
- [x] **`models/human_arm.json` — tüm eklemler anatomik aralıklara çekildi** (insan anatomisine göre):
      j1 omuz kuşağı ±60° · j2 omuz fleksiyon −35° … +175° · j3 dirsek 0° … +140° · j4 bilek f/e −70° … +75° · j5 bilek r/u −40° … +20° · j6 el eksenel rotasyonu ±30°.
    - Dirsek hiperekstansiyonu, omuzun dolanması/geriye dönüşü, elin aşırı fleksiyonu gibi **insan kolu yapamayacağı pozlar** artık girilmez.
- [x] **Kısıtlar her iki motora da işlendi**: Python integratoru ve JS simülasyon döngüsü artık `q` ve `qd`'yi her adımda eklem sınırlarına **kelepçeler** (`np.clip` / `Math.min|max`); sınır aşımı olan eklemin hızı sıfırlanır.
- [x] **`docs/kabuller_ve_gereksinimler.md` 3.2a maddesi** eklendi: eklem-eklem insan anatomisi sınır tablosu — makalede assumptions bölümüne eklenmeye hazır; İngilizce Assumptions paragrafına da sınırlar işlendi.
- [x] **Workbench eklem editörü tamamlandı** (bu tur):
    - **Snap ile eklem seçimi**: imleç yaklaşınca en yakın eklem turuncu halkayla vurgulanır; 18 px içinde tıklayınca seçilir; boş alana tıklayınca seçim temizlenir.
    - **Local UCS gizmo**: seçili eklemde kırmızı X / yeşil Y / mavi Z eksen okları; oklardan tutulup sürüklendiğinde kol (sönümlü en küçük kareler IK'si ile) yalnızca o eksen doğrultusunda bükülür.
    - **Eklem kilitleme**: `2b. Eklem Seçimi, Kilitleme & Düzlem Hizalama` kartında j1–j6 onay kutuları; kilitli eklem canvas'ta kırmızı halka + "L" işareti alır ve tüm IK yollarında (gripper sürükleme, IK Çöz, gizmo eksen sürükleme, simülasyon döngüsü) sabit tutulur.
    - **Düzleme hizalama**: XY/XZ/YZ referans düzlemlerinden biri seçilip düğmeye basılınca kilitli olmayan eklemler Gauss-Newton ile ilgili düzleme en yakın poza oturur; tam hizalanamazsa ipucu yazar.
    - **Uzuv yüzer menüsü**: uzvun üzerine tıklayınca açılan panelde slider + metre cinsinden hassas değer girişi; anında FK güncellenir. **Çocuk / Ergen / Yetişkin** tek tık antropometrik profil; **Custom** profiller localStorage'da isimle kaydet/yükle/sil.
- [x] **Refleks sınıfı tepki süresi optimize**: refleks katmanı ~25–40 ms etkin gecikmeli kortikal-baypaslı yoldan çalışır.
- [x] **Biyomimetik Pulse kontrolörü** (`selController: 'bio'`): kas-sıçrama duty-cycle + Harris-Wolpert sinyal bağımlı gürültü; hedefi ±1.8 mm kaçırabilen, insan gibi kusursuz olmayan biyomimetik davranış; karşılaştırma tablosuna ve dışa aktarılan raporlara eklendi.
- [x] **Doğrulama**: 6 kontrolörün tümü (hlsl, governor, pid, lqr, fixed, bio) sarkık koldan raf hedefine ulaşıp sonlanır (nihai hata: hlsl 0.8 / governor 0.9 / lqr 6.0 / fixed 1.3 / bio 2.1 mm; PID %8.7 aşımla bilinen kararsızlık); `python -m unittest discover -s tests` → **60/60 OK**; `figures/human_arm_workbench.html` yeniden üretildi ve node duman testiyle doğrulandı.

## Aşama 6 — v5 Bilimsel Çalışma Tezgahı, Ordinal Algı, S0–S6 & Üç Reddiye Tamamlaması (M1–M10)
- [x] **Koordinatsız Ordinal Algı Gözlemcisi (`robosimu/stereo.py` & JS):** `PROXIMITY_CLASSES` (`ÇOK UZAK`–`TEMAS`), simetrik $\min(\rho_s, 1/\rho_s)$ eşikleri `(0.45, 0.78, 0.94, 0.985)` + histerezis, yatay/dikey/derinlik sırası, yaklaşma hızı ve güven paketi (`OrdinalPerception`, `compute_ordinal_perception`).
- [x] **S0–S6 Kesintili Bakış & 4-Eksenli Metrikler (`robosimu/simulator.py` & JS):** `SimConfig.intermittent_gaze`, örnek-tutma (`S3 KÖR UÇUŞ`), mekanik iş $W_{\text{mek}}$, `visual_load_hz` ($L_{\text{vis}}$), `pulse_efficiency` ($\eta_{\text{pulse}}$) ve `ReachMetrics` / `compute_metrics`.
- [x] **v5 Editoryal Akademik Arayüz (`robosimu/workbench_html.py`):** Sahte `.pw-dots` ve ölü `#manualSliders` temizlendi; `PANEL I–V` açıklamaları, **Üç Reddiye Kavram Çekmecesi** (`#btnConceptGuide`), **Teorem Modu (`#btnViewMode` + `#theoremStrip`)**, 5-sekmeli alt masa (`#bottomTabBar` + `#btnShowAllTabs`), 3B **Hayalet İkiz PID** (`#chkGhostRobot`, `opts.isGhost` izolasyonuyla), **Göreceli Derinlik Konileri** (`#chkDepthCones`), **Kas Enerji Akışı**, retina `EL / HEDEF / TARAMA` nişangahı, manuel `brk_0..5` + yumuşak 12 adımlı sarkma (`sagArm`), sıralı varsayılan sıfırlama (`bindSettingsStore`), düzeltilmiş `screenToWorldOnPlane` ve 10-sütunlu karşılaştırma + `.md`/`.csv` dışa aktarımı tamamlandı.
- [x] **Doğrulama:** `figures/human_arm_workbench.html`, `figures/arm_viewer.html` ve `figures/arm_viewer_120ms.html` yeniden üretildi; gömülü JS (`260,692` karakter) `node --check` ile hatasız doğrulandı; `python -m unittest discover -s tests -v` → **64/64 OK**.
