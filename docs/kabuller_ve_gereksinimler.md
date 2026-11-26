# Robosimumuzun Kabulleri ve Gereksinimleri — HLSL İnsansı Kol

Bu belge, simülasyonumuzun ve (ileride yazılacak HLSL makalesi) yazımının üstüne oturacağı **kabul edilen varsayımları, dışlanan sensörleri ve robotun davranışına yönelik gereksinimleri** tanımlar. Kısa özet:

> Robotumuzun davranışını **kabaca insansı**, görsel olarak **insansı**, geribildirimce **insan öğrenmesine benzer** (durum tahmini + görsel doğrulama) ve **ölçümleri yalnızca kendi gözleriyle** yaptığını kabul ederiz. Kusursuzluk değil, **yetenekli motor becerileri** hedeflenir.

---

## 0. Teorem — üç reddiye: enkoder yok, sayı yok, sürekli bakış yok

> **Kısa hâli:** Klasik robot dünyayı **ölçer ve hesaplar**; insan **görür ve hatırlar**. Ölçüm mutlaktır,
> sayısaldır ve süreklidir; görme görelidir, niteliklidir (kategoriktir) ve **kesintilidir**. İnsansı
> hareket, taklit edilen bir yörünge değil, bu eksik bilgiyle çalışmanın zorunlu sonucudur.

Klasik zincir `kamera → (X,Y,Z) → ters kinematik → açı emri → servo` iken bizim zincirimiz
`göz → göreli nitelik → darbe süresi → kas` biçimindedir. Fark bir kontrolcü ayarı değil, **bilgi
temsili** farkıdır; PID/LQR/IK tartışması bu farkın yanında ikincildir (aynı yanlış temsilin farklı
düzelticileridir).

**Reddiye 1 — Enkoder yok (kör eklem):** Eklemin kaç derecede olduğu bilinmez. Motor, kendisine verilen
elektriğin **süresi (darbe)** kadar kasılır; konum bilgisi darbelerin integrali kadardır ve sürtünme,
yerçekimi, yük ile kayar. Hareket bu yüzden milimetrik değil, **nispi ve doğal olarak hatalıdır** —
ve hata kaçınılmaz olduğu için sistem onu **görmek ve düzeltmek** zorundadır.

**Reddiye 2 — Sayısal mesafe yok (göreli stereo algı):** İki göz mesafeyi santimetre olarak ölçmez;
**sıralı nitelikler** üretir: `çok uzak / uzak / yakın / temas öncesi / temas`, `sol / orta / sağ`,
`üst / orta / alt`, `önde / hizada / geride`, artı `görünürlük` ve `güven`. Metre cinsinden hiçbir
büyüklük kontrolcüye girmez; varsa da yalnızca **deneyi yapan araştırmacının** ölçümüdür.

**Reddiye 3 — Kesintili göz kontrolü:** Bakış olay tabanlıdır: **bul → hedefe bak → kör uçuş
(balistik darbe) → önce ele sonra hedefe bak → kısa kompanzasyon darbesi → kilitle.** Görsel örnek
yalnızca **fiksasyon sonunda** üretilir (sample-and-hold); fiksasyonlar arasında kontrolcü son gözlemi
tutar. Böylece "balistik + düzeltme" davranışı ayarlanarak değil, **zamanlamadan** doğar.

| Konu | Klasik (CNC / PID / LQR / IK) | Teorem (bizim) |
|---|---|---|
| Konum bilgisi | Enkoder, mutlak açı | Darbe geçmişi + ileri model (efferent kopya) |
| Hedef bilgisi | Kamera → (X,Y,Z) metre | Göz → sıralı göreli nitelikler |
| Ters kinematik | Çözülür, açı emri verilir | **Yok**; motor grubuna süre emri verilir |
| Bakış | Sürekli, her kare | Kesintili: bul → hedef → kör uçuş → el → hedef |
| Doğruluk hedefi | Sıfır hata | **Hata + telafi**; sıfır hata insansı değil |
| Gecikme | Bastırılacak bozucu | **Tasarım girdisi** (δ_eff → κ = 1/e) |
| Gözlemlenebilirlik | Tam durum varsayımı | **Kısmi ve gecikmeli** durum; tahmin zorunlu |
| Başlangıç durumu | Enkoder sorgusu (homing) | **Fren + efference copy**; göz yalnızca istisnada ele bakar |

**Bugünkü durum (dürüst envanter):** Enkoder yokluğu **kısmen** sağlanmıştır (dış döngü gecikmeli
görsel gözlemle çalışır, fakat bitki `q` paylaşır ve resolved-rate IK `q_ref` kullanır); sayısal mesafe
yokluğu **henüz yoktur** (kontrolcüye `rel_base_gap_m` metre olarak verilir); kesintili bakış
**kısmen** vardır (sakkad modeli görselleştirilir ama kontrolü etkilemez); hata + telafi **vardır**
(E_init / T_comp). Tam tasarım `docs/teorem_ve_felsefe.md` §4–§6'da, geçiş planı §6'dadır.

---

## 1. Kavramsal model — robot nasıl “düşünür”?

- İki stereo kamera (insan gözüne benzer, omuz üstü/arkası asılmış, Eklem-1'in 1.12 m üstünde, baseline 64 mm), `f = 520 px`, image 640×480.
- Tek hedef, tek hedef — reaching problemine kilitli.
- Robota **hiçbir dış dünya mutlak ölçümü verilmez**: ne koordinat encoder'ı, ne dışarıdan koordinat ölçümü, ne lazer. Tek dış kaynak: **kendi iki gözüyle gördüğü** göreli görünüm (looming, disparity).
- Gözlere gelen görüntü, **gecikmeli ve gürültülüdür** (varsayılan 60 ms, 100 Hz, σ = 0.35 px; derinlik ekseni 2.5× daha gürültülü).
- Robotun komutu **tek bir hız profili**dir: `v = min(v_max, φ(t,p) · min(k_hi·|e|, κ/δ_eff · (ε+|e|)))`,  `φ` çan-eğrisi frenleme fonksiyonu, `κ = 1/e`, `δ_eff = δ + nT/2`.
- **Refleks dışı hiçbir sınıfta κ ≤ 1/e aşılmaz** — monoton varış garantisi burada yatar.

## 2. Sensörler — kesinlikle kullanılmayanlar

Vücutta karşılığı olmayan, yüksek-hassas mutlak ölçüm gerektirenler **yasaktır**:

| Sensör | Neden yasak |
|---|---|
| Mutlak açı / pozisyon enkoderleri | İnsan enkoder ölçmez; kas iğciği uzunluk/gerilim ölçer |
| Dışarıdan koordinat/mocap ölçümü (ör. hareket yakalama) | Göz yok sayılır; kendi görüşünün dışında bilgi |
| Lazer / mutlak mesafe sensörü | Tek göz pinhole geometrisiyle mesafe sağlanmaz |
| Harici kuvvet/tork matrisi (6-eksen FT sensörü) | Kas kasımında uzunluk-gerilim var, fakat mutlak joint torku ölçmek biyolojik değil |
| Göz arkasına yerleşik mutlak doğum | — |

Kullananılanlar:

| Sensör | Biyolojik karşılığı | Not |
|---|---|---|
| 0. / 1. göze benzer stereo kamera gözlük benzeri rig | İki göz, insan görme | İnsan görmek için kafayı döndürerek vurgu yapar; biz de yapıyoruz |
| Efferent (motor) kopya | Efferent kopi (korteks-içi motor komutunun iç kaydı) | Konum kestiriminin **tek** iç kaynağı; Faz 4'te zorunlu |
| — | Golgi tendon organı / kas iğciği | Rodea kullanılmıyorsa da mümkün (yapı/donanım yaklaşırsa) |
| Eklem tutma freni (mekanik/elektromanyetik) | İzometrik kas tonusu (eklem sertliği) | **Yeni (v5):** enerji kesilince kolu tutar; biyolojik karşılığı düşük enerjili ko-kontraksiyon |

## 3. Varsayımlar (bu simülasyonda ve makalede geçerli)

### 3.1c Frenler, efference copy ve "eline bakmadan başlama" (v5 varsayımı)
Robot, harekete başlarken **eline bakmaz**; elinin nerede olduğunu iki şeyden bilir:

1. **Eklem mikro-frenleri.** Darbe bittiği anda devreye giren düşük enerjili tutma frenleri kolu yerçekimine karşı tutar (sürekli akım yok). Frenler `Aktif (Tutma)` veya `Serbest (Enerji Kesintisi → yerçekimi düşüşü)` olabilir; ikincisi konum bilgisini geçersiz kılar.
2. **İçsel motor hafızası (efference copy).** Gönderilen her darbenin süresi ve kutbu kaydedilir; elin konumu bu darbelerin kümülatif integralinden **yaklaşık** olarak kestirilir. Kestirim mutlak değil, kaydırılabilirdir (sürtünme, yerçekimi, yük) — istenen budur.

**Gözün ele baktığı dört istisna:** (i) **İlk Uyanış / kalibrasyon** — enerji ilk kez verildiğinde veya frenler boşalıp kol sarktığında; mikro darbeler + optik akış ile el-göz eşlemesi kurulur. (ii) **Dış darbe / beklenmeyen sapma** — efference tahmini geçersiz. (iii) **Kompanzasyon başlangıcı** — balistik faz bitti, "nereye vardım?" sorusu. (iv) **Güven düşüşü** — örtüşme, düşük ışık, gürültü artışı.

> **Durum (dürüst, v5.1):** Fren paneli ve `brake_torque_nm` **uygulandı** (1.5× otomatik, eklem başına; scripts/compute_brake_torques.py). Efference copy kestirimi hâlâ **kullanılmıyor**; serbest düşmenin dinamik simülasyonu yok (kol sarkık postüre atlanır, başlangıç tahmini geçersiz sayılır); `İlk Uyanış` tamamlanınca frenler kendiliğinden devreye girer.

### 3.1 Görsel ve mekanik
- **Görüş yeterli** ve tek dış kaynak; hedefin konumu asla doğrudan hesaplanmaz, yalnızca **göreli görünümden** çıkar.
- Kamera rigi insanın görme organı gibi davranır: varsayılan **otomatik alterasyondur** (`auto` — kafa hedef ile el arasında ~150°/s yumuşak pursuit; iki nokta da foveaya sığdığında birlikte sabitlenir); `target` (rafa kilitli), `hand` (tutucuya takip), `mid` (ikisi ortada) seçilebilir; detaylar 3.5'te.
- **Kafa hareketi rate-limited** (~150°/s max, smooth pursuit); anlamlı hedef değilse kafa sabit kalır.
- **Refleks sınıfı hariç** hiçbir aciliyet sınıfında κ = 1/e sınırı aşılmaz.
- **Refleks döngü gecikmesi 20–50 ms'ye düşürüldü** (kortikal pipeline bypass; omurga-benzeri kestirme yol, `state.speedTierName === 'reflex'` için `retroDelay = clamp(delay × 0.35, 20 ms, 50 ms)`).
  - Refleks durumları **kasıtlı** serbest bırakılmıştır: balistik pomp/tect, tek atım, görsel düzeltme yok, ε aşımı kabul edilir.
- **δ_eff = δ + n·T/2** — frame period'u olan filtre ve frame kayıt süresi atlanamaz.
- **Hız sınıfları ve alt durumları** (5 sınıf, 15 alt bağlam): `docs/insans_hiz_profilleri.md`.
- **Görev**: 0.15 m nominal mesafedeki tek hedef, sabit (hareketsiz hedef).
- **Bitki**: mükemmel rijit cisim, DH kinematiği, 1 kHz joint servo, J−T sürücü, dinamik kütle matrisi RNEA ile (JS tarafında static gravity + inertia + friction linear model).
- **Payloads** 0–4 kg; el/uç ekseni insan bileği gibi yönelimi üç eksende serbest, mekanik sınırlar eller/kürek gibi.

### 3.1b Başlangıç Konumu Kestirimi ve "İlk Uyanış (Self-Calibration)"
Robotik sistemdeki enkoderlerin yerine geçen konum belirleme zinciri (üç aşamalı):
1. **Durum 1 — Öncül durum hafızası (prior):** Son aktüatör enerji/duruş geçmişi tutulur. Holding-fren (`brake_torque_nm > 0`) varsa kolun en son bilinen bölgede olduğu varsayılır; stereo kameralar o bölgeye odaklanarak eli doğrudan tespit eder.
2. **Durum 2 — Yerçekimi sarkması (frensiz):** Potansiyel minimumu (doğal sarkma pozisyonu) çıkarımı yapılır, görsel arama bu alt bölgeden başlatılır.
3. **Durum 3 — İlk Uyanış (Self-Calibration):** Enkoder olmadan robot ilk kez enerjilendiğinde, "İlk Uyanış" düğmesiyle eklemlere ardışık mikro enerji darbeleri uygulanır; uzuvlarda oluşan küçük salınımlar optik akış/hareket segmentasyonu ile algılanarak el-uzay eşlemesi (visuomotor mapping) sıfırdan kurulur. Kalibrasyon metrikleri (darbe sayısı, süre, konum hatası, tespit edilen piksel hareketleri) kaydedilir ve raporlara yazılır.

**Performans metrikleri — balistik/kompanzasyon ayrımı:**
- **İlk Yaklaşım Hatası (E_init):** İlk hızlı (açık-çevrim/balistik) enerji darbesinin sonundaki tepe-hız anında elin hedefe olan öklid mesafesi; sonraki ince düzeltmeler bu skora dahil değildir.
- **Kompanzasyon Süresi (T_comp):** t_ballistic (tepe hız anı) → t_target (hedefin tolerans bandına ilk giriş anı) arasındaki süre; **%T_comp = T_comp / T_total** olarak raporlanır ve grafiklerde/ihracatlarda gösterilir.
- Balistik faz turuncu, kompanzasyon fazı turkuaz olarak renklendirilir (Analitik Dashboard Grafik A).

### 3.2a Eklem Serbestlikleri ve Sınırlar (insan anatomisi)
İnsan kolunun serbestlikleri bilinçli olarak sınırlandırılmıştır; robot hiçbir durumda insan anatomisinde mümkün olmayan bir poza giremez (dirsek hiperekstansiyonu, omuzun 360° dönmesi, elin 110°+ fleksiyonu, gövdeyi dolanma). Eklem eksenleri anatomiktir (DH zinciri doğrulandı): j1 dikey eksen (gövde/omuz kuşağı dönüşü), j2/j3 mediolateral eksen (omuz ve dirsek fleksiyonu), j4 bilekte mediolateral eksen (bilek fleksiyon/ekstansiyon), j5 antero-posterior eksen (bilek radyal/ulnar sapma), j6 ön kol ekseni (el eksenel rotasyonu).

| # | Eklem | İnsanda | Sınır (derece) | Eksen |
|---|---|---|---|---|
| j1 | Gövde / omuz kuşağı dönüşü | Trunk rotation + skapular protraksiyon (omuzun transvers düzlem bileşeni) | ±60° | Dikey (Z) |
| j2 | Omuz fleksiyon / ekstansiyon | Kolu öne-yukarıya (overhead) ve arkaya sınırlı | −35° … +175° | Mediolateral |
| j3 | Dirsek | Yalnızca fleksiyon (hiperekstansiyon yok) | 0° … +140° | Mediolateral |
| j4 | Bilek fleksiyon / ekstansiyon | Bilek anatomik sınırı (fleksiyon ~80°, ekstansiyon ~70°) | −70° … +75° | Mediolateral |
| j5 | Bilek radyal / ulnar sapma | Radial deviation ~15–20°, ulnar deviation ~35–40° | −40° (ulnar) … +20° (radial) | Antero-posterior |
| j6 | El eksenel rotasyonu | Ön kol pronasyonuna ek, sınırlı el ekseni dönüşü | ±30° | Ön kol ekseni |

**Ulaşım kısıtları (dürüst sınırlar):**
- **Gerçek önkol pronation/supination bu DH zincirinde gösterilemez.** j4'ün sınır işaretlemesi (eski `link4_wrist_roll`) eksen mediolateral olduğundan gerçekte bilek fleksiyonuydu; etiket ve sınır anatomiye göre düzeltildi. Pron/sup ekseni ön kol doğrultusunda olmalıdır; dikey-gövde/yatay-omuz DH çapasında bu eksen hiçbir zaman elde edilemez (twist α yalnızca yatay x ekseni etrafında döner). Sonuç: avuç içi öne → arkaya çevrilemez.
- **Glenohumeral abdüksiyon/adduksiyon ve izole iç/dış rotasyon** modellendi değil (omuzun 3 serbestliğinden yalnızca fleksiyon/ekstansiyon + gövde j1 ile temsil edilir). Yana açma eksiktir; ancak gövde rotasyonu (j1) yanal erişim bileşenini kısmen karşılar.
- Bu iki eksik, DH tabanlı genel bir modelin doğal sonucudur ve makale varsayım bölümünde açıkça yazılmalıdır.

Endüstriyel/6R kolu (`models/six_axis_arm.json`) insan kolu anatomisiyle karıştırılmamalıdır; onun sınırları endüstriyel tasarım çalışması içindir ve insan biçimindeki hareketin gereği değildir.

### 3.3 İnsansı Dinamik Yaklaşım Profili (HDA — Human-like Dynamic Approach)
Sabit-hızlı veya tek-üstel sönümlemeli (klasik endüstriyel) hız profilleri bu modelin hedefi değildir; hedef, insanın normal uzanma hareketinin (çan-eğrisi, tek tepe, asimetrik taban) mesafe ve **ciddiyete** (aciliyet sınıfına) göre dinamik olarak ayarlanmasıdır. Algoritma dört bileşenden oluşur ve `robosimu`'da `HumanApproachController` (`hdap`) olarak uygulanır; işaretlenen tüm formüller hem Python hem JS motorunda aynıdır.

**(a) Mesafe ve aciliyet uyarlamalı dinamik hız zarfı.** Tepe hız sınırı sabit bir sınıf sabiti değildir:
  v_cap(D) = v_max(sınıf) · clamp(sqrt(D / D_ref), D_min, 1)   ; D_ref = 0.30 m, D_min = 0.35, sqrt ≈ genelleştirilmiş Fitts-serbest genleşme üssü (insan genlik/peak-hızı ilişkisinin korumacı yaklaşımı)

Yani yakındaki hedeflere insan normalde olduğundan daha yavaş, uzaklara ise sınıfa kadar hızlı gider; aciliyet sınıfı (reflex > fast > normal > careful > precision) yine de mutlak tavanı belirler. Azalma aşaması (durma fazı) v_max'ı değil, k·e'yi takip eder — sonlara doğru hız üstel sönüm ile düşer, sabit hızla gelip ani frenleme yoktur.

**(b) Asimetrik çan-eğrisi şekillendirme.** φ (çıkış/kalkış-rampası) sınıfın γ üssü, ter_P frenleme bölgesi eşiği ve φ_min tabanıyla, kalan mesafe oranına p = |e|/|e0| göre hesaplanır; p < ter_P bölgesinde φ = 1 (sınıfın doğal sönümü tam hakim) ve aksi durumda φ = max(φ_min, 0.5·(zaman rampası + ilerleme rampası)). Sonuç: erken hızlı ivmelenme, geniş bir tepe, uzun ve yumuşak frenleme kuyruğu — insanın normal uzanma kinematiği (Flash & Hogan 1985).

**(c) Sinyal-bağımlı komut gürültüsü (Harris & Wolpert 1998).** Komut hızı artıkça komutu bozan gürültü doğrusal büyür (σ_cmd ∝ v); JS motorunda etkin, Python benchmark motorunda deterministik karşılaştırma için kapalıdır (varsayılan). Gürültü asla yön işaretini değiştirecek ölçekte değildir; sadece büyüklüğü modüle eder.

**(d) Trifazik aktüatör sıralama (bkz. 3.4).** İnsanın kas trifazik EMG paterniyle uyumlu, proksimalden distala doğru aktüatör sıralaması; giriş fazında omuz/dirsek baskın, uçuş fazında bilek, bitiş fazında ko-kontraksiyon.

**Neden "sabit hız sonra yavaşlama" değil?** Araç/portal profilleri (constant-then-decel) yüksek komut ivmesi ve sarsıntı yaratır ve insan kinematiğine aykırıdır; bu modelde ivme, hızın kümülatif integrali olarak değil φ(p) dolayımıyla kademeli artar, τ(q) (tepe tork) ve sarsıntı (m/s²) ayrıca raporlanır. Kabul testi: `overshoot < %0.5` (reflex hariç), `final_error < 5 mm` (reflex hariç), `mt` sınıfın tipik bandı içinde (`docs/insans_hiz_profilleri.md`).

### 3.4 Trifazik Aktüatör Sıralama ve İnsan-Uygun Katkı Ağırlıkları
İnsanın tek bir hareketi tek kasla değil, **trifazik EMG dizisiyle** yapılır: agonist aktivasyonu (itme) → antagonist (frenleme) → ko-kontraksiyon (sertlik). Modele karşılığı, her eklemin hız-kap çarpanlarının kalan mesafe oranına p = |e|/|e0| göre yumuşak geçişli olarak değiştirilmesidir:

| Faz | p aralığı | j1 | j2 | j3 | j4 | j5 | j6 | Fizyolojik karşılık |
|---|---|---|---|---|---|---|---|---|
| İtme (ballistic) | ≥ 0.60 | 1.00 | 1.00 | 0.85 | 0.60 | 0.60 | 0.50 | merkezî kaslar (gövde/omuz) baskın |
| Taşıma | 0.25–0.60 | 0.90 | 1.00 | 0.90 | 0.75 | 0.75 | 0.65 | omuz/dirsek eşit ağır, bilek stabilize |
| Yakalama | 0.06–0.25 | 0.60 | 0.70 | 0.85 | 1.00 | 1.00 | 0.85 | distal kaslar (bilek/el) hassas ayar |
| Son ko-kontraksiyon | ≤ 0.06 | 0.55 | 0.65 | 0.80 | 0.90 | 0.90 | 0.75 | global eklem sertliği (grip güvenliği) |

Ağırlıklar iki köşe (anchor) arasında doğrusal interpolasyonla yumuşak geçer — basamak değişimi yoktur. Sıralama, her eklemin `speed_limit`'ini asla aşmaz (human_arm.json'daki motor hız limitleri mutlaktır); yalnızca komutlandırılan tepe q̇'yu faz içinde düşürür. İnsan gibi davranma sınırı: **model hiçbir durumda insan kaskadının veremediği bir tepkiyi vermez** — tüm eklemler anatomik akış içinde hareket eder (3.2a), refleks sınıfı hariç tek komutta monotondur ve κ ≤ 1/e sabititir.

**Aktüatör sıralama gereksinimi (MUST):** Uygulama, aktüatör aktivasyon sırasını her zaman (itme→taşıma→yakalama→ko-kontraksiyon) faz desenine göre yetkilendirir; dışarıdan gelen komutlar (UI sürükleme, IK) bu sırayı bozmaz — IK çıktısı komut hızları her adımda bu çarpanlarla kırpılır.

### 3.5 Görsel Dikkat Alterasyonu — Sakkadlar, Foveasyon ve Otomatik Bakış (`auto`)
İnsan, el hedeften uzaktayken gözünü sırayla kupaya ve ele çevirir; ikisi birden aynı foveaya sığdığında sabitler. Modelin çalışma davranışı:

1. Kol ve hedef arasındaki **açısal ayrım** (gözbebeğinden ölçülen, derece) eşiklidir: > 20° → sırayla bakılma, 0.26 s tutuş (hızlı sakkad), 8–20° → 0.42 s tutuş (yavaş geçiş), < 8° → ikisi birden ortada ("mid" sabitlenme), son 12 mm → hedef kilidi ("target").
2. Bakış noktası her 0.18 s'den sık değişmez (en yavaş sakkad sınırı); kafa dönüşü ~150°/s yumuşak pursuit sınırı altındadır (3.2).
3. Sakkadlar arasındaki "hedef kilidi" insanın hedef-önceliği prensibidir: kol henüz yoldayken hedef genellikle sabitlenir ve kol yaklaştıkça izlenir — davranış tam olarak koda işlenmiştir (`gazeMode = 'auto'`, varsayılan).
4. `target` (hedefe kilitli, "kafa rafdaki bardağa sabit", en az hareket), `hand` (tutucuyu takip, en aktif kafa), `mid` (ikisi ortada) modları seçilebilir; üçü de bağımsız çalışır ve göz ekranlarına aynı şekilde yansır.

Kafa ve kol **aynı dik düzlemde değildir**: insan anatomisinde kafa ortada, omuz ve kollar iki yanadır; modelde gövde/kafa y = 0 düzlemindedir, sol kol (taban) y = +0.19 m'ye monte edilmiştir (3.6). Bu, hem fiziksel olarak doğru bağlanma noktası verir hem de gözün kolu (yanında) ve hedefi (karşısında) birlikte görebilmesini sağlar — gözler y = 0 merkezli, kafa yüksekliği 1.13 m; kamera yansıtmalarını etkilemeyen tek kaynak `base_origin`'dir (FK köşesi).

### 3.6 Kafa–Kol Ayrımı ve `base_origin` (omuz yaka)
İnsan anatomisine göre: gövde (trunk) y = 0 merkezli, kafa iki omuz arasında ortada (y = 0) durur; kol yaka (scapular girdle) ile **y = +0.19 m**'ye bağlıdır (sol omuz). Böylece:
- Kol ve kafa dik düzlemde çakışmaz; gözün bakış alanında kol sağdan/soldan çerçeveye girer.
- Model yerçekimi yaklaşımı ve FK sıradüzeni değişmez (yalnızca taban köşesi translate edilir): sarkık kol pozunda yerçekimi torku ≈ 0 kalır (doğrulandı: max |τ| = 0.136 Nm < 0.5).
- Gözler gövde/kafa merkezindedir (y = 0); stereopsis ve looming hesabı gerçek insan göz konumlarına (y = 0, omuz üstü) dayanır.
- `base_origin` yalnızca bu modelde uygulanır (`models/human_arm.json`); endüstriyel `six_axis_arm.json` etkilenmez.

### 3.7 Ciddiyet/Aciliyet Sınıfları ve Dinamik Hız Haritası — Bağıntı Özeti
Hız haritası sabit hız sonra yavaşlama (trapezoidal araç profili) değildir; üç çarpanla tanımlanır ve sınıf başına sabitlenir:
  v(t) ≤ min(v_max(sınıf)·d(D), φ(p) · min(k_hi·|e|, k(δ_eff)·(ε+|e|)))
  d(D) = clamp(sqrt(D0 / D_ref), 0.35, 1)
Sınıflar (5) ve alt bağlamlar (15) `docs/insans_hiz_profilleri.md`'de; `reflex` sınıfı κ > 1/e ile balistiktir ve tek komutta görsel düzeltme almaz (monoton varış garantisi yalnızca normal ve altı için geçerlidir).


### 3.2 Endirimler (kullanılan rakamlar)
- Görsel gecikme: 60 ms (değersiz 20–180 ms)
- Gürültü: 0.35 px (σ, her satıra; z-ekseni 2.5×), çözünürlük 640 × 480, f = 520 px
- Baseline: 64 mm (yetişkin)
- Pozlama: hedef `0.45, 0.10, 0.85` m; omuz yüksekliği 0.90 m; üst kol 0.31 m, ön kol 0.26 m, el 0.12 m
- Kafa dönüş sınırları: yaw ±32°, pitch +18°/−28°, hız ≤ 150°/s
- Varış toleransı: sınıfa bağlı (4 mm normal)

## 4. Gereksinimler — robot nasıl davranmalı?

### 4.1 Gecikmez ama kusursuz değil
> Kusursuzluk arayışı insanı yavaşlatır; biz **yetkin ama her zaman doğru olmak zorunda değiliz**.

- Robot bir **insan nasıl yapıyorsa öyle** yapmak için algılama-öğrenme yolunda ilerleyecek.
- Göz kapandığında elinin nerde olduğunu **nasıl biliyorsa** bizim robotumuz da kolunun kendi vücuduna göre nerede olduğunu **efferent kopi + öğrenilmiş ileri model** sayesinde bilecek.
- Yetenekli motor becerileri gelişmiş bir tasarım olacak; hassas işler yapabilecek; ancak hatasız olma zorunluluğu yok.

### 4.2 Beklentiler (davranış)
- **Değişen tehditlere ilk tepkisi refleksler** olmalı, ölçümlenmeden atılan hareket.
- **Göz kapalıyken bile hesap edilen yeri tahmin edecek**: efferent kopi + öğrenilmiş ileri model durumu drift'ler ama çok değil; görsel düzeltme büyük hatalarda baskın.
- **İki göz iki bağımsız gözlem** yapmalı; gözlerden gelen verinin çaprazlama doğrulanması (cross-eye validation) hareketin hızını/amplity'sini ayarlamada kullanılacak (gözü bağlanan bir robot koluna *yalpalama* olsun).
- **Hedefe uzanma insansı**: hız profili çan–eğrisi, monotone; aşma yok; α sınıflara göre doğal-frenleme (φ).
- **Görsel feedback baskın**: hareketi gözden aldığı verilerle şekillendirilecek, servo dış ekleme yok.
- **Aktüatör sıralaması insan hareketiyle uyumludur**: itme (proksimal/agonist) → taşıma → yakalama (distal) → ko-kontraksiyon faz sırası her komutta uygulanır (3.4); insanın üretemeyeceği tepkiler robotta üretilmez — tüm eklemler anatomik aralıklar içinde, hız/tork limitlerine uyarak hareket eder.
- **Hız haritası dinamiktir**: mesafe ve aciliyet sınıfına göre v(D, sınıf) değişken, iniş sabit-sonra-yavaşlama değil.

## 5. Sınırlamalar — bu simülasyonun yapmaları ve yapmamaları

| Yapılıyor | Yapılmıyor |
|---|---|
| Görme temelli geri besleme (stereo + looming) | Mutlak konum/açı algısı |
| Efferent kopya (robotun kendi komut geçmişi) | Dışarıdan koordinat ölçümü |
| Öğrenilmiş ileri model (Faz 4) | Güç/tork matrisi |
| Yetişkin yürüme/durma sınırları (boyutlar, yaş, ağırlık) | Değişken yük taşıma dışı multimodal sensör |
| Refleks > hızlı > normal > dikkatli > hassas hız sınıfları | Değişken hedef |
| İnsan gibi hata yapma yetisi (yavaş ama kusursuz değil) | Değirmen gibi sürekli mükemmel davranış |

## 6. Faz 4 — öğrenilmiş feedback (planlanan)

- **Efferent kopya**: robotun kendi motor komutlarını geçmiş zaman serisi olarak sakla (a posteriori).
- **Öğrenilmiş ileri model**: `state(t+1) = F(state, command)` — HLSL + DQN/simülasyon ile dolaylı (indirect motor expectation; güç/sürtünme/atalet mapii).
- **Göz kapalıyken test**: göz kapatıldığında (görsel geri besleme off) robot hedefine yaklaşmaya devam edebiliyor olmalı, hata sadece “biraz sürüklenme”, eksik kapalıyken ilk kötü expected.
- **Göz açılınca düzeltme**: büyük hatalarda görsel doğrulama anlık baskın olur.

## 7. Makaleye eklenecek assumptions paragrafı (öneri, İngilizce)

> **Core thesis (motivation paragraph).** Classical robot reaching is *measurement-driven*: encoders read absolute joint angles, a calibrated stereo rig triangulates the target into metric \((X,Y,Z)\), and an inverse-kinematics solver converts that coordinate into an angle command. Our framework rejects all three premises. First, **the joints are blind**: no encoder exists anywhere in the pipeline; a motor contracts only for the duration of the electrical pulse it receives, so the internal position estimate is the integral of issued pulses, drifting with friction, gravity and payload. Second, **there is no metric distance**: the eyes deliver ordinal, relational descriptors ("hand is far behind and slightly left of the target", "the hand now looks as large as the target"), never centimetres, and no inverse-kinematics step is ever solved. Third, **gaze is intermittent**: the eyes acquire the hand, saccade to the target, then look away while the arm flies ballistically (open loop), and only re-fixate — hand first, target second — when the hand is expected to be close or has drifted, closing the residual relational gap with a short corrective pulse. The resulting motion is therefore *not tuned to be human-like*; it is human-like because the information that would make it non-human has been removed. Imperfection is the mechanism, not a defect: the first ballistic pulse produces an **Initial Approach Error**, and the second fixation produces the **compensation** that corrects it. The simulation is designed so that an observer can see *when* the eyes look at the hand, *when* they look at the target, and *how* a relational gap becomes a pulse duration without a single coordinate being computed.

> **Assumptions.** The proposed robot perceives the scene exclusively through its two upward-and-behind-shoulder stereo cameras (interpupillary distance 64 mm, 520 px focal length, 640 × 480 image), with a 60 ms visual delay and 0.35 px pixel-level noise (depth 2.5× noisier). No absolute position, angle or force sensing is available anywhere in the control pipeline; every estimate is produced internally from past motor commands (efference copy) and a learned forward model. The head rotates within adult-range limits (yaw ±32°, pitch +18°/−28°, ≤ ~150°/s smooth pursuit) and fixates the goal or tracks the moving hand under a **distance-dependent alternating-gaze policy** (\(>20^\circ\) target/hand alternation at 0.26 s holds, 8–20° slow alternation at 0.42 s, \(<8^\circ\) midpoint fixation, <12 mm final target lock), so both the target and the reaching hand enter the same foveal window before capture. Outside of the reflexive urgency tier, the delay-derived speed limit satisfies κ ≤ 1/e, i.e. the motion is guaranteed monotone toward the goal. The reach speed profile is the **HDA human-like dynamic approach**: the peak-speed cap scales with reach amplitude \(v_{cap}(D) = v_{max} \cdot \mathrm{clamp}(\sqrt{D/0.30}, 0.35, 1)\), the descent follows an asymmetric bell (ramp up, wide crest, soft exponential tail) rather than a constant-speed-then-decelerate profile, command noise grows with commanded speed (signal-dependent noise), and joint torques follow a **triphasic proximal-to-distal activation order** (ballistic: trunk/shoulder/elbow dominant; transport: shoulder+elbow; grasp: wrist/hand; capture: global co-contraction), with per-joint velocity caps never exceeding motor speed limits. The arm is mounted at the **left shoulder** (base origin \(y = +0.19\) m) while the trunk and head remain centred at \(y = 0\); the head is therefore not coplanar with the arm, as in human anatomy. The plant is a rigid-body DH arm at 1 kHz joint-servo rate. Every joint is limited to human-anatomical ranges — trunk/shoulder-girdle rotation ±60°, shoulder flexion −35° … +175°, elbow flexion 0° … +140° (hyperextension forbidden), wrist flexion/extension −70° … +75°, wrist radial/ulnar deviation −40° (ulnar) … +20° (radial), hand axial rotation ±30° — and the integration clips at those limits, so no pose outside the human envelope is reachable. Note the model's anatomical scope limits, stated explicitly: true forearm pronation/supination (forearm-axis rotation) is not represented in this DH anchoring, glenohumeral abduction/adduction and isolated shoulder rotation are likewise not modeled (j1 trunk/girdle rotation plus j2 shoulder flexion partially cover the transverse component). Perfection of the capture is not required; the design goal is a robot with human-like motor capability, accepting human-like modest inaccuracy.

## 8. Metin dışı notlar (gelecekteki ek iş)

- `docs/insans_hiz_profilleri.md` — hız sınıfı tablosu (5 tier + 15 alt mod).
- `models/human_arm.json` — insan ölçeklerinde 6R kolu (0.31/0.26/0.12 m, shoulder 0.90 m, ~5 kg).
- `reports/concept_validation.md` — akademik kanıt, kısıtlamalar, istatistikler.
- Makale: assumptions bölümünün taslak bench out of scope notlarını Faz 4 ile genişletilecek.
