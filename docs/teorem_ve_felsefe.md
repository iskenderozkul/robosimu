# Teorem ve Felsefe — Üç Reddiye

> Bu belge, geliştirilen insansı kontrol teorisinin **kavramsal çekirdeğini** tek yerde toplar ve
> `docs/kabuller_ve_gereksinimler.md` ile makale taslaklarının motivasyon bölümlerine kaynak olur.
> Durum: **tasarım notu** — bu belgedeki mekanizmalar henüz kodlanmadı; mevcut simülasyonun
> bu teoremin neresinde durduğu §5'te dürüstçe listelenmiştir.

---

## 1. Tek cümlede teorem

> Klasik robot dünyayı **ölçer ve hesaplar**; insan **görür ve hatırlar**.
> Ölçüm mutlaktır, sayısaldır, gürültüsüzdür ve süreklidir. Görme görelidir, niteliklidir (kategoriktir),
> gürültülüdür ve **kesintilidir**. İnsansı hareket, bu ikinci tarafın doğal sonucudur — taklit edilecek
> bir "yörünge" değil, eksik bilgiyle çalışmanın kaçınılmaz biçimidir.

Klasik zincir: `kamera → (X,Y,Z) → ters kinematik → eklem açısı → servo`.
Bizim zincir: `göz → (göreli simge) → darbe süresi → kas`.
Aradaki fark bir "kontrolcü ayarı" farkı değil, **bilgi temsili** farkıdır. PID/LQR/IK tartışması bu
farkın yanında ikincil kalır: onlar aynı (yanlış) temsilin farklı düzelticileridir.

---

## 2. Üç reddiye

### 2.1 Enkoder yok — eklem kördür
Robot, ekleminin kaç derecede olduğunu **bilmez**. Motor yalnızca kendisine verilen elektrik
enerjisinin **süresi (darbe/pulse)** kadar kasılır. Sonuç: konum bilgisi mutlak değil, **verilen
darbelerin integrali** kadardır; sürtünme, yerçekimi, yük ve ısı bu integrali kaydırır.
Hareket bu yüzden milimetrik değil, **nispi ve doğal olarak hatalıdır.**

Bunun bedeli ve ödülü aynı şeydir: hata kaçınılmaz olduğu için sistem **hatayı görmeye ve
düzeltmeye** mecbur kalır. Kusursuz servo, düzeltmeyi gereksiz kılar; gereksiz düzeltme insansı
davranışı da ortadan kaldırır.

### 2.2 Sayısal mesafe yok — stereo göz nitelik üretir
İki göz, el ile hedef arasındaki mesafeyi **santimetre olarak hesaplamaz.** Gözün ürettiği şey bir
sayı değil, bir **nitelik demetidir**:

- *"El hedefe göre **çok geride** ve **biraz solda**."*
- *"El hedefinkinden **büyük** görünüyor"* (yani hedefe göre daha yakın / daha geride).
- *"Mesafe **kapandı**, artık **çok yakın**."*

Yani sayısal derinlik yerine **sıralı (ordinal) kategoriler** ve **göreli oranlar** kullanılır:
`çok uzak / uzak / yakın / temas öncesi / temas`, `sol / orta / sağ`, `üst / orta / alt`, `önde / hizada / geride`.
Metre cinsinden hiçbir büyüklük kontrolcüye **girmez**; varsa da yalnızca deneyi yapan bizim
(araştırmacının) ölçümüdür, robotun bilgisi değildir.

Bu, doğruluk kaybı değil **temsil kazancıdır**: nitelik temsili gürültüye dayanıklıdır, gecikmeyi
saklayabilir (bir sonraki bakışa kadar geçerlidir) ve dil/öğrenme ile doğrudan eşleşir.

### 2.3 Kesintili göz kontrolü — sürekli izleme yok
Gözler her milisaniye hem eli hem hedefi taramaz. Bakış **olay tabanlı**dır:

1. **Bul:** El nerede? (frensizse yerçekimiyle sarkmış; frenliyse son bölgede; ilk uyanışta mikro
   darbelerle kendini arayarak.)
2. **Hedefe bak:** Göreli farkı **zihinde tart** (sayıya dökmeden).
3. **Tahmini süre kadar elektrik ver:** Kol **balistik** gider; bu sırada görsel hesap yoktur.
4. **Yaklaşınca ya da sapınca tekrar bak:** **önce ele**, sonra **hedefe** — iki kısa sakkad.
5. **Kompanzasyon darbesi:** Kalan göreli boşluğu kapatacak kısa darbe. Gerekirse 4'e dön.

Bu döngünün doğal sonucu, literatürdeki "balistik + düzeltme" ayrımıdır: ilk hamle **İlk Yaklaşım
Hatası**nı üretir, ikinci bakış **Kompanzasyon**u tetikler. Hata bir kusur değil, mekanizmanın
parçasıdır; düzeltme de bir "güvenlik katmanı" değil, kontrolün kendisidir.

### 2.4 Başlangıç bilgisi: frenler + efference copy — "eline bakmadan bilmek"

İnsan, bardağa uzanırken önce kendi eline bakmaz; **doğrudan hedefe bakar**. Enkodersiz robotun da
aynısını yapabilmesi için üç varsayım gerekir:

1. **Eklem mikro-frenleri (izometrik kas tonusu analoğu).** Darbe bittiği anda devreye giren düşük
   enerjili tutma frenleri kolu yerçekimine karşı tutar; sürekli akım harcanmaz, kol düşmez.
   Frenler **aktif (tutma)** veya **serbest (enerji kesintisi → yerçekimi düşüşü)** olabilir.
   Bu, "başlangıç durumu"nu belirleyen tek fiziksel olaydır.
2. **İçsel motor hafızası (efference copy).** Gönderilen her darbenin süresi/kutbu kaydedilir.
   Frenler aktifken el, bu darbelerin kümülatif integraliyle **yaklaşık olarak bilinir**; robot
   eline bakmadan hedefe kilitlenip ilk balistik darbeyi gönderebilir. Bilgi *mutlak* değil,
   *yaklaşık ve kaydırılabilir*dir — istenen de budur.
3. **Gözün ele baktığı istisnalar (yalnızca dört durum):**
   - **İlk Uyanış / Kalibrasyon:** enerji ilk kez verildiğinde veya frenler boşalıp kol sarktığında
     konum bilinmez; mikro darbeler + optik akış ile el-göz eşlemesi sıfırdan kurulur.
   - **Dış darbe / sapma:** beklenmeyen bir bozucu (çarpma, yük değişimi, fren kayması) efference
     kopyanın tahminini geçersiz kılar.
   - **Kompanzasyon başlangıcı:** balistik faz bittikten sonra "nereye vardım?" sorusu yalnızca
     bakarak yanıtlanır (önce el, sonra hedef).
   - **Güven düşüşü:** örtüşme, düşük ışık, gürültü artışı → tahmin güvenilmez, göz doğrular.

   Bunların dışında göz **hedefte**dir. Yani "eline bakmak" bir rutin değil, **bir hatadan şüphelenme**
   davranışıdır; bu yüzden bakış sayısı aynı zamanda bir **hata göstergesi**dir (teorem modunda sayaç).

---

## 3. Klasik ile karşılaştırma

| Konu | Klasik (CNC / PID / LQR / IK) | Teorem (bizim) |
|---|---|---|
| Konum bilgisi | Enkoder, mutlak açı | Darbe geçmişi + ileri model (efferent kopya) |
| Hedef bilgisi | Kamera → (X,Y,Z) metre | Göz → sıralı göreli nitelikler |
| Ters kinematik | Çözülür, açı emri verilir | **Yok**; motor grubuna süre emri verilir |
| Bakış | Sürekli, tam kare | Kesintili: bul → hedef → kör uçuş → el → hedef |
| Doğruluk | Hedef: sıfır hata | Hedef: **hata + telafi**; sıfır hata insansı değil |
| Gecikme | Bozucu, bastırılır | **Tasarım girdisi** (δ_eff → κ = 1/e) |
| Bozucu | Reddedilir (disturbance rejection) | Kullanılır: hata, öğrenmenin yakıtıdır |
| Gözlemlenebilirlik | Tam durum varsayılır | Durum **kısmi** ve **gecikmeli**; tahmin zorunlu |
| Başarısızlık | Arıza | Beklenen ve öğretici |

Bu tablo makalenin motivasyonunun omurgasıdır: **biz daha iyi bir PID yazmıyoruz; PID'in varsaydığı
bilgiyi robota vermiyoruz.** Geriye kalan davranış, bilgi eksikliğinden doğan insansı davranıştır.

---

## 4. Simülasyonda nasıl kurulur? (tasarım, kod değil)

### 4.1 Algı sözleşmesi — "sayı yok" kuralı
Göz modülü (yalnızca göz modülü!) piksel ve metre ile çalışabilir; dışarıya **yalnızca** şu paketi verir:

| Alan | Değer kümesi | Nereden |
|---|---|---|
| `mesafe_sınıfı` | `çok_uzak / uzak / yakın / temas_öncesi / temas` | el/hedef retinal boyut oranı ρ_s = s_el / s_hedef |
| `yatay` | `sol / orta / sağ` (+ `az / çok`) | iki gözdeki yatay farkın işareti ve kabalığı |
| `dikey` | `üst / orta / alt` (+ `az / çok`) | dikey fark |
| `derinlik` | `önde / hizada / geride` | disparite işareti |
| `görünürlük` | `el_var / el_yok / hedef_var / hedef_yok / örtük` | görüş alanı + sakkad sırasında bastırma |
| `güven` | `düşük / orta / yüksek` | gürültü + örtüşme |

Kontrolcü bu paketin dışında hiçbir şey görmez: **ne q, ne p, ne metre, ne derece.** Bu bir "API
disiplini"dir; yanlışlıkla sızmayı önlemek için bitki ile kontrolcü arasına **ateş duvarı** konur.

### 4.2 Darbe (pulse) aktüatör sözleşmesi — "açı emri yok" kuralı
Kontrolcünün çıktısı `qd_ref` değil, şu üçlüdür: **(motor grubu, süre ms, yön/kutup)**.

- **Motor grupları (sinerji tabanı):** gövde dönüşü, omuz fleksiyonu, dirsek fleksiyonu, bilek,
  kavrama. Altı eklem tek tek sürülmez; 4–5 sinerji sürülür (insan da tek tek kas sürmez).
- **Darbe tablosu:** `(nitelik, aciliyet sınıfı) → süre`. Örn. `mesafe=çok_uzak, sınıf=normal` → 320 ms;
  `mesafe=yakın` → 90 ms; `mesafe=temas_öncesi` → 35 ms.
- **Öğrenme/ilk uyanış:** Darbe tablosunun kaba hâli mikro darbelerle (İlk Uyanış) kalibre edilir;
  bu, enkoderin yerini alan tek "ölçüm"dür ve tamamen görseldir.
- **Bozucular açık:** yerçekimi, sürtünme, yük, motor doygunluğu. Bunlar bilinçli olarak modellenir;
  ilk yaklaşım hatası buradan doğar.
- **Efferent kopya:** Verilen her darbe kaydedilir; ileri model "el şimdi nerede olmalı"yı buradan
  tahmin eder. Göz kapalıyken (kör uçuşta) tek bilgi kaynağı budur.

### 4.3 Bakış durum makinesi — "sürekli izleme yok" kuralı

| Durum | Ne yapar | Süre | Görsel çıktı |
|---|---|---|---|
| S0 **Ara** | Kafa sakkad dizisi; el görüş alanına girene kadar | değişken | `el_yok` → `el_var` |
| S1 **Ele bak** | Elin konumu ve görünür boyutu kaydedilir | 120–200 ms | göz görüntüsü el foveada |
| S2 **Hedefe bak** | Sakkad (~30–60 ms) + hedef fiksasyonu | 120–200 ms | göz görüntüsü hedef foveada |
| S3 **Kör uçuş** | Balistik darbe; görsel işleme **kapalı** | 150–350 ms | göz görüntüsü donuk/karartılmış |
| S4 **Yeniden bak** | Önce el, sonra hedef (iki sakkad) | ~250–400 ms + δ | iki kısa fiksasyon |
| S5 **Kompanze et** | Kalan nitelik farkına göre kısa darbe | 30–120 ms | fiksasyon hedefte kalır |
| S6 **Kilitle** | `mesafe=temas` ve `yatay/dikey=orta` ise bitir | — | hedef foveada sabit |
| R **Refleks** | S3–S5 atlanır; tek atım | 20–50 ms | bakış yok |

Kritik kural: **görsel örnekleme yalnızca fiksasyon sonunda üretilir** (sample-and-hold). Fiksasyonlar
arasında kontrolcü **son gözlemi tutar** (zero-order hold). Böylece:
- "balistik + kompanzasyon" davranışı ayarlanarak değil, **zamanlamadan** doğar;
- E_init ve T_comp **emergent** metrikler olur (sonradan eklenen etiketler değil);
- bakış sayısı bir **maliyet**tir: az bakış = hızlı ama hatalı, çok bakış = yavaş ama isabetli.

### 4.4 İzleyiciye gösterim — teorem modu
Ekranda iki görünüm yan yana:

- **Mühendis görünümü:** metrikler (cm, derece, m/s, hata mm) — bugünkü dashboard.
- **Teorem görünümü:** sayı yok. Sadece
  - gözün o an **ne yaptığı**: `ELE BAKIYOR` / `HEDEFE BAKIYOR` / `KÖR UÇUŞ`;
  - algı paketi: `ÇOK UZAK · SOL · ALT`;
  - motor çıktısı: `OMUZ 240 ms · DİRSEK 120 ms`;
  - birikimli hata: yalnızca "ilk yaklaşım hatası" ve "kaç bakışta düzeldi" sayısı.

Amaç: izleyicinin "koordinat nerede?" sorusunu kendi kendine sorması ve cevabı **ekranda
görememesi**. Teorem, ölçümün yokluğuyla görünür olur.

### 4.5 Arayüz ve gösterim yenilikleri (v5 paketi)

**(a) Üç katman + telemetri (uygulandı, M1–M5).** Gözler üstte tam genişlik yan yana (`PANEL I SOL GÖZ` | `PANEL II SAĞ GÖZ`), 3B sahne ortada (`PANEL III`), üstten/yandan görünümler altta (`PANEL IV`–`V`) ve alt kontrol alanı **5 odaklanmış sekme + Tüm Sekmeleri Aç** düzenindedir. `TELEMETRİ HUD` (`#telemetryHud`) artık zaman, bakış hedefi, `S0–S6` bakış fazı (`#telGazePhase`), son görsel örnek yaşı (`#telSampleAge`, `TAZE` / `HOLD`), ordinal yakınlık sınıfı (`#telOrdinal`), aktif kas sinerji darbesi (`#telPulse`), efference-copy belirsizliği (`#telEffSigma`), $\rho_s$, disparite, $\delta$ ve fren durumunu canlı gösterir. Her iki göz kanvasında da renkli `EL / HEDEF / TARAMA` odak nişangahı ve `S3 KÖR UÇUŞ · ÖRNEK TUTULUYOR` rozeti çizilir.

**(b) Fren paneli (uygulandı, M2 + manuel düzenleme & yumuşak sarkma).** `Frenler AKTİF (Tutma)` / `SERBEST (Enerji Kesintisi)` düğmeleri vardır; `brake_torque_nm` eklem başına otomatik hesaplanır ($\tau_b = 1.5 \times \max|\tau_g(q)|$, `scripts/compute_brake_torques.py`) ve `brk_0..5` kutularından manuel olarak değiştirilebilir (`chkBrakeAuto` kapatıldığında `state.links[i].brake_torque_nm` canlı güncellenir). Frenler bırakıldığında kol 12 adımlı yumuşak yerçekimi sarkması (`sagArm`) önizlemesine geçer ve İlk Uyanış tamamlanana dek `Simülasyonu Başlat` kilitlenir; tam donanımsal serbest düşme rölesi ise Faz-2 donanım çalışmasıdır.

**(c) Zaman ölçeği x1 / x0.75 / x0.5 / x0.25 / x0.1 (uygulandı, M1).** Yalnızca oynatım hızını böler — fizik ve metrikler ($E_{\text{init}}$, $T_{\text{comp}}$) değişmez; x0.1 ağır çekimde S0–S6 sakkadları ve darbe başlangıç/bitişleri ayrı ayrı çözünür olur.

**(d) Kinematik editör (uygulandı).** Manyetik yakalama, tutulan eksende Local X/Y/Z UCS gizmo, boşluğa tıklayınca seçimi bırakma, "Seçili Düzlemde Hizala", eklem kilidi, uzuv uzat/kısalt, Çocuk/Ergen/Yetişkin/Custom profilleri ve düzeltilmiş `screenToWorldOnPlane` ters projeksiyonu (`project3D` ile tam uyumlu).

**(e) Global ayar yönetimi (uygulandı).** Üstte `Kaydet` (`localStorage` anahtarı `ikjal_ayarlar_v1`) ve `<select>` dahil doğru sıra-duyarlı (`selSpeedTier` $\to$ `selSpeedSub`) `Varsayılanlar` sıfırlaması + ayar başı `↺` düğmeleri.

**(f) Sayı yerine görsel dil & Üç Reddiye rehberi (uygulandı, M6–M9).**
- **Kavram & Okuma Rehberi Çekmecesi (`#btnConceptGuide`):** Üç Reddiye'yi, her panelin neyi kanıtladığını ve canlı deney akışını özetler.
- **Teorem Modu (`#btnViewMode`, `Teorem Modu: AÇIK (Metrikler Gizli)`):** `.metric-only` öğelerini ve kanvaslardaki metrik sayıları (`cm`, `px`, `derece`) gizler; Teorem Şeridinde (`#theoremStrip`) yalnızca ordinal algı + `S0–S6` durumunu gösterir.
- **Göreceli derinlik konileri (`#chkDepthCones`):** Sol/sağ gözlerden ele (camgöbeği) ve hedefe (turuncu) uzanan derinlik konileri.
- **Kas enerji akışı:** 3B kolda aktif eklem grubu üzerinde amber enerji akışı kılıfı.
- **Hayalet İkiz Robot (`#chkGhostRobot`):** Paylaşılan bakış durumunu kirletmeden (`opts.isGhost` anlık kopya izolasyonuyla) çalışan kesik kırmızı klasik PID + enkoder ikizi.

**(g) Akademik metrikler (uygulandı, M9).** `robosimu/simulator.py` (`ReachMetrics`, `compute_metrics`) ve tarayıcı analiz paneli artık 4 dik ekseni de hesaplar ve raporlar: $E_{\text{init}}$ (mm), $\%T_{\text{comp}}$, etkin görsel yük $L_{\text{vis}}$ (`visual_load_hz`, Hz) ve darbe verimi $\eta_{\text{pulse}}$ (`pulse_efficiency`, %). `#tblCompare` tablosu ve `.md` / `.csv` dışa aktarımları bu 10 sütunu içerir.

**(h) Sabit sekme çubuğu, genişletilmiş duruş/görev senaryoları ve tek göz + aktif boyun hareket paralaksı (uygulandı, v5.1).** `.bottom-bar` kaydırmada sabit (`position: sticky`), sekme düğmeleri sarmalanmaz (`flex-wrap: nowrap`) ve her tıklamada katmanı açıp yalnızca seçilen sekmeyi gösterir; `models/human_arm.json` içine 5 yeni anatomik başlangıç duruşu (`side_guard`, `low_table_pick`, `shelf_pre_reach`, `cross_body_rest`, `retracted_compact`) eklenerek toplam 9 duruş sağlanmıştır; `SPEED_TIERS` altındaki *Alt Durum (Görev Bağlamı)* seçimleri gerçekçi bağlamları (`posture`, `target`, `payload`, `delay`, `visionMode`) taşır; `stereo.py` (`vision_mode="mono_left"|"mono_right"|"mono_static_left"`) ve tezgâhtaki `#selVisionMode` + Ön Ayar 5 (`mononeck`), tek göz kapalıyken boyun sinüsoidal taramasıyla ($A_y=38\text{ mm}$, $f_n=2.2\text{ Hz}$) iki farklı baş pozundan sentetik paralaks üçgeni ve retinal looming füzyonu üretir.

---

## 5. Bugünkü simülasyon bu teoremin neresinde? (güncel dürüst envanter)

| İlke | Durum | Açıklama |
|---|---|---|
| 1. Enkoder yok | **Simülasyonda Uygulandı** (Fiziksel donanım Faz-2) | `models/human_arm.json` fren torku (`brake_torque_nm`) içerir; frenler/efference-copy $\Sigma_{\text{eff}}$ + İlk Uyanış akışı v5 tezgahında canlı çalışır; aktif kas darbesi 3B görünürdür. Alttaki sayısal fizik entegratörü (`simulate_reach`) katı cisim dinamiğini entegre eder; gerçek darbe sürücü kartı Faz-2'dedir. |
| 2. Sayısal mesafe yok | **Uygulandı (`robosimu/stereo.py` + JS)** | `compute_ordinal_perception` ve `computeOrdinalPerceptionJS`, 5 yakınlık sınıfını (`ÇOK UZAK`..`TEMAS`, eşikler `0.45, 0.78, 0.94, 0.985`, histerezisli) + yön/looming/güven paketini üretir ve Teorem Modunda tüm sayısal etiketler gizlenir. |
| 3. Kesintili bakış | **Uygulandı (`intermittent_gaze=True` + S0–S6)** | S0–S6 sakkadik durum makinesi ve `S3 KÖR UÇUŞ` örnek-tutma döngüsü hem Python simülatöründe ($L_{\text{vis}} < 100\text{ Hz}$) hem JS motorunda çalışır. |
| 4. Hata + telafi | **Evet** | $E_{\text{init}}$, $\%T_{\text{comp}}$, $L_{\text{vis}}$ ve $\eta_{\text{pulse}}$ dört dik eksende ölçülür ve 8 kontrolcü arasında karşılaştırılır. |
| 5. İzleyiciye gösterme | **Uygulandı (v5 UI)** | Kavram Çekmecesi, Teorem Modu, Hayalet İkiz, Derinlik Konileri, Kas Enerji Akışı, Odak Nişangahı ve 5 sekmeli laboratuvar düzeni tamamlandı (`node --check` + 64/64 `unittest`). |

---

## 6. Geçiş planı — modüler sıra (durum özeti)

| # | Modül | Durum | Uygulanan Çıktı |
|---|---|---|---|
| M1 | **Zaman ölçeği + telemetri HUD** | **Tamamlandı** | `x1..x0.1` ağır çekim + 11 satırlı `#telemetryHud` |
| M2 | **Fren paneli + yumuşak sarkma** | **Tamamlandı** | `brake_torque_nm`, `brk_0..5` manuel düzenleme, 12 adımlı yumuşak sarkma & İlk Uyanış |
| M3 | **Gözlem & simülasyon izolasyonu** | **Tamamlandı** | `opts.isGhost` ile bakış/efference durumunun izole edilmesi + `OrdinalPerception` arayüzü |
| M4 | **Göz nitelik paketi** (`compute_ordinal_perception`) | **Tamamlandı** | 5 sınıf (`ÇOK UZAK`..`TEMAS`), simetrik $\min(\rho_s, 1/\rho_s)$ ve histerezis |
| M5 | **Bakış durum makinesi** (`S0–S6` + `intermittent_gaze`) | **Tamamlandı** | S3 kör uçuş örnek-tutma + `L_vis` ($< 100\text{ Hz}$) hesabı + retina nişangahı |
| M6 | **Kas enerji akışı & sinerji göstergesi** | **Tamamlandı** | 3B kolda aktif eklem grubu enerji kılıfı + HUD `#telPulse` |
| M7 | **Efference $\Sigma_{\text{eff}}$ & İlk Uyanış** | **Tamamlandı** | Sakkad kör uçuşunda $\Sigma_{\text{eff}}$ drift izleme ve fren bırakma sonrası optik-akış kalibrasyonu |
| M8 | **Teorem Modu UI & Derinlik Konileri** | **Tamamlandı** | `#btnViewMode`, `#theoremStrip`, `.metric-only` maskeleme, `#chkDepthCones`, `#btnConceptGuide` |
| M9 | **Hayalet İkiz Robot & 4-Eksenli Kıyas** | **Tamamlandı** | `#chkGhostRobot`, `ReachMetrics` (`e_init_m`, `t_comp_pct`, `visual_load_hz`, `pulse_efficiency`), 10-sütun tablo + `.md`/`.csv` |
| M10 | **Doğrulama & Makale Eşitleme** | **Tamamlandı** | 64/64 `unittest` OK (`robosimu`), 31/31 OK (`iskender-v3`), `node --check` OK, EN+TR makale + `.docx` eşitlemesi |

---

## 7. Açık sorular / riskler

- **Nitelik çözünürlüğü:** 5 mesafe sınıfı 6-DOF bir kol için yeterli mi, yoksa 7–9 sınıf mı gerekir?
- **Sinerji tabanı:** 4–5 grup, hedefe 3B ulaşmayı bozmadan yeterli mi? (Prototipte 6 grup + kavrama.)
- **Kalibrasyon maliyeti:** İlk Uyanış ne kadar sürmeli ki "insansı" kalsın, "kullanışsız" olmasın?
- **Bakış gecikmesi:** δ (60 ms) + sakkad süresi, kompanzasyon sayısını kaça çıkarır? (Hedef: 1–3.)
- **Tekrarlanabilirlik:** Nitelik tabanlı sistem aynı hedefe her seferinde aynı sayıda bakışta varır mı?
  (Makale için istatistik gerekir: bakış sayısı dağılımı, E_init dağılımı.)
- **Dürüstlük:** Bitki hâlâ gerçek `q` ile simüle edilir. Bu bir sahne hilesi değil, fizik motorudur;
  ama makalede "robot bunu bilmiyor" derken **arayüzün** bunu garanti ettiği açıkça yazılmalıdır.
