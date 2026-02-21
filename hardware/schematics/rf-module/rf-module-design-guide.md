# AWR2243 77 GHz RF Radar Module - PCB Layout Design Guide

## 1. Genel Bakis

| Parametre | Deger |
|-----------|-------|
| Malzeme | Rogers RO4350B (er=3.66, tand=0.0037) |
| Katman | 2 (Top: sinyal+anten, Bottom: GND plane) |
| Kalinlik | 0.254mm (10 mil) core + 35um (1oz) Cu |
| Boyut | 50 x 40 mm |
| Min trace/space | 0.1mm / 0.1mm |
| Yuzey | ENIG (Immersion Gold) |
| Via | 0.25mm drill, filled & capped |

## 2. Anten Yerlestirme Kurallari

### 2.1 Genel Prensipler
- Antenler kart kenarinda (edge), acik havaya bakan tarafta
- Anten altinda (B.Cu) ground plane OLMAYACAK - copper keepout zone tanimla
- Anten uzerinde solder mask OLMAYACAK - F.Mask keepout zone tanimla
- TX ve RX antenler arasi minimum 2mm bosluk
- Anten dizisi dogru yonde: TX ve RX antenler paralel, ayni kenar

### 2.2 TX Anten Dizisi (3 eleman)
```
Kart kenari (ust)
|                                          |
|  [TX1]  [TX2]  [TX3]                     |
|   |      |      |     <- 50 ohm feed     |
|   AWR2243 MMIC                           |
```
- TX antenler arasi mesafe: lambda/2 = 1.95mm (center-to-center)
- TX feed hatlari esit uzunlukta (phase matching)
- TX1-TX3 hatlari AWR2243'e en kisa yoldan

### 2.3 RX Anten Dizisi (4 eleman)
```
|  [RX1]  [RX2]  [RX3]  [RX4]             |
|   |      |      |      |                 |
|   AWR2243 MMIC                           |
```
- RX antenler arasi mesafe: lambda/2 = 1.95mm (center-to-center)
- RX feed hatlari esit uzunlukta (phase matching, max +-0.01mm)
- RX hassasiyeti kritik: minimum crosstalk icin hatlari ayir

### 2.4 Patch Anten Boyutlari (RO4350B, 77 GHz)
- Patch genisligi W = 1.45 mm
- Patch uzunlugu L = 0.93 mm (lambda_eff/2)
- Inset feed derinligi = 0.25 mm (50 ohm eslestirme)
- Inset bosluk = 0.1 mm (her iki taraftan)
- Feed hatti genisligi = 0.15 mm (50 ohm mikroserit)

## 3. AWR2243 MMIC Yerlestirme

### 3.1 Pozisyon
- Antenlere YAKIN yerlestir (feed hatlari < 5mm)
- Kartin merkezi-alt bolgesinde
- BGA alt yuzey termal pad icin 25 termal via (5x5 grid)
- Termal via pitch: 0.6mm, drill: 0.3mm

### 3.2 BGA Fanout Stratejisi
- 0.65mm pitch BGA icin dog-bone fanout
- Via-in-pad (filled & capped) tercih edilir
- RF pinler (TX/RX) icin via KULLANMA - dogrudan trace
- Guc pinleri icin genis trace + birden fazla via

### 3.3 Bypass Kapasitor Yerlestirme
```
Oncelik sirasi (AWR2243'e yakinlik):
1. 100pF C0G (0201) - EN YAKIN, < 0.5mm
2. 1nF X7R (0201) - < 1mm
3. 100nF X5R (0201) - < 2mm
4. 1uF X5R (0402) - < 3mm
5. 10uF X5R (0402) - < 5mm
6. 22uF X5R (0805) - bulk, konum esnek
```

## 4. Guc Bolgesi Tasarimi

### 4.1 LDO Yerlestirme
- LDO'lar kartin dijital tarafinda (antenlerden uzak)
- Her LDO icin ayri guc adas (copper pour)
- LDO giris ve cikis kapasitorleri LDO'ya YAKIN

### 4.2 Ferrit Bead Izolasyonu
```
5V Giris --> FB1 --> TPS7A47 #1 --> 1.0V VDD_RF
             FB2 --> TPS7A47 #2 --> 1.8V VDD_RF + VDD_IO
             FB3 --> (connector)  --> 1.8V VDD_IO (ayri)
             FB4 --> (connector)  --> 3.3V VDD_IO
```
- Her guc domaini ferrit bead ile izole
- Ferrit bead secimi: 600 ohm @ 100MHz (BLM15AG601SN1D)

### 4.3 Guc Hatti Trace Genislikleri
| Guc Hatti | Akim | Min Trace | Onerilen |
|-----------|------|-----------|----------|
| 5V Giris | ~1.5A | 0.5mm | 1.0mm |
| 1.0V RF Core | ~800mA | 0.3mm | 0.5mm |
| 1.8V RF Analog | ~400mA | 0.2mm | 0.3mm |
| 1.8V Digital | ~200mA | 0.15mm | 0.2mm |
| 3.3V I/O | ~100mA | 0.1mm | 0.15mm |

## 5. LVDS Routing Kurallari

### 5.1 Diferansiyel Cift Parametreleri
- Trace genisligi: 0.1mm
- Cift arasi bosluk: 0.1mm
- Diferansiyel empedans: 100 ohm
- Max uzunluk: 30mm
- Cift arasi skew: < 0.05mm (< 0.3ps @ 77GHz)

### 5.2 Routing Prensipler
- Diger sinyallerden min 3x trace genisligi bosluk (0.3mm)
- LVDS hatlari altinda kesintisiz ground plane
- Via gecisi YAPMA - tek katman routing
- 45 derece donus (chamfered bend), 90 derece YASAK
- Cift arasi uzunluk eslestirme: serpentine ile

### 5.3 AC Coupling
- Her LVDS ciftine 100nF AC coupling kapasitoru
- Kapasitor cift arasinda simetrik yerlestir
- 0201 boyut, kisa trace ile baglanti

### 5.4 Terminasyon
- Her LVDS ciftine 100 ohm terminasyon direnci
- Alici tarafinda (konnektor oncesinde)
- 0402 boyut, padin hemen yaninda

## 6. SPI Routing Kurallari

### 6.1 Trace Parametreleri
- SPI saat: max 25MHz
- Trace genisligi: 0.15mm
- Kontrollü empedans gerekli degil (dusuk frekans)
- Max uzunluk: 50mm

### 6.2 Prensipler
- SPI_CLK ve SPI_DATA hatlari arasi min 0.2mm
- SPI_CS pull-up 10K direnc (VDD_1P8)
- NRESET pull-up 10K direnc (VDD_1P8)
- SOP[2:0] = 100 (SPI slave modu): SOP2=VDD, SOP1=GND, SOP0=GND

## 7. Ground Plane Kurallari

### 7.1 Alt Katman (B.Cu)
- TAMAMEN ground plane (kesilmez, slot acilmaz)
- ISTISNA: Anten alanlari (copper keepout)
- ISTISNA: Termal via pad alanlari

### 7.2 Via Stitching
- Kart kenarlarinda via stitching: lambda/20 = 0.2mm aralik
- Anten etrafinda yogun via stitching (anten ground plane siniri)
- RF feed hatlari boyunca via stitching (her iki tarafa)
- Power bolgelerinde via stitching (thermal + EMI)

### 7.3 Via Stitching Parametreleri
- Via drill: 0.2mm
- Via pad: 0.4mm
- Aralik: max 0.2mm (lambda/20 @ 77GHz)
- Kart kenari boyunca: her 0.5mm bir via

## 8. Empedans Hesaplamalari

### 8.1 50 Ohm Mikroserit (RF)
```
Substrat: RO4350B
er = 3.66
h = 0.254 mm (10 mil)
t = 0.035 mm (1 oz Cu)
W = 0.15 mm --> Zo ≈ 50 ohm

Dogrulama formulu:
ere = (er+1)/2 + (er-1)/2 * 1/sqrt(1+12*h/W)
ere = 2.33 + 1.33 * 1/sqrt(1+12*0.254/0.15)
ere = 2.33 + 1.33 * 0.219 = 2.62

Zo = (120*pi) / (sqrt(ere) * (W/h + 1.393 + 0.667*ln(W/h+1.444)))
Zo = 376.7 / (1.618 * (0.591 + 1.393 + 0.667*ln(2.035)))
Zo ≈ 50.2 ohm ✓
```

### 8.2 100 Ohm Diferansiyel (LVDS)
```
Substrat: RO4350B
W = 0.1 mm
S = 0.1 mm (ciftler arasi)
h = 0.254 mm

Zdiff = 2 * Zo * (1 - 0.48 * exp(-0.96 * S/h))
Tek hat Zo (W=0.1mm) ≈ 65 ohm
Zdiff = 2 * 65 * (1 - 0.48 * exp(-0.96 * 0.394))
Zdiff = 130 * (1 - 0.48 * 0.684)
Zdiff = 130 * 0.672 = 87.3 ohm

Ayarlama: S=0.08mm veya W=0.09mm ile 100 ohm elde edilir
Pratik: 2D EM simulasyon (Sonnet/HFSS) ile dogrula
```

## 9. Termal Yonetim

### 9.1 AWR2243 Termal
- Guc tuketimi: ~2.5W (aktif mod)
- Termal pad: 4.5 x 4.5 mm
- 25 termal via (5x5 grid, 0.3mm drill)
- Alt katman termal pad: 6 x 6 mm copper pour
- Theta_JA hedefi: < 30 C/W

### 9.2 LDO Termal
- TPS7A47 #1 (1.0V): Pdiss = (5-1.0)*0.8A = 3.2W --> DIS SOGUTUCU GEREKLI
  - ONEMLI: 5V'dan 1.0V uretmek cok verimsiz!
  - ALTERNATIF: 1.2V/1.5V ara voltaj kullan veya DC-DC kullan
  - Ya da LDO giris voltajini 1.5V'a dusur (DC-DC pre-regulator)
- TPS7A47 #2 (1.8V): Pdiss = (5-1.8)*0.6A = 1.92W
  - Ayni sorun: Pre-regulator onerisi
- Termal via'lar exposed pad altinda

### 9.3 Isil Cozum Onerisi
```
Optimum guc topolojisi:
5V --> MP1584EN (DC-DC) --> 2.0V --> TPS7A47 #1 --> 1.0V (Pdiss=0.8W ✓)
5V --> MP1584EN (DC-DC) --> 2.5V --> TPS7A47 #2 --> 1.8V (Pdiss=0.42W ✓)
```

## 10. Uretim Notlari

### 10.1 PCB Siparisi (JLCPCB)
- Malzeme: Rogers RO4350B (ozel siparis, standart FR4 degil)
- 2 katman, 0.254mm kalinlik
- ENIG yuzey kaplama
- Min trace/space: 0.1mm/0.1mm (ozel yetenek gerekebilir)
- Via: 0.25mm drill, filled & capped (ozel)
- Solder mask: yesil, anten bolgesi acik
- Silkscreen: beyaz

### 10.2 Montaj
- AWR2243 BGA: reflow soldering, profil kontrolu gerekli
  - Peak temp: 260C, ramp rate: 3C/s max
  - X-ray inspection onerisi (BGA dogrulama)
- 0201 bilesenler: hassas pick & place
- Anten bolgesine flux/pasta bulasmamali

### 10.3 Test Proseduru
1. Guc hatlari kontrol (multimetre)
2. SPI iletisim testi (logic analyzer)
3. LVDS sinyal kalitesi (osiloskop, diff probe)
4. Anten S11 parametresi (VNA, 77GHz prob)
5. Radar fonksiyon testi (hedef algilama)

## 11. Onemli Uyarilar

1. **EMPEDANS DOGRULAMA**: Uretim oncesi 2D EM simulasyon (Sonnet veya HFSS) ile
   50 ohm ve 100 ohm hatlari MUTLAKA dogrula
2. **ANTEN SIMULASYONU**: CST veya HFSS ile patch anten performansini simule et
   - Hedef: S11 < -10dB, BW > 4GHz (76-81 GHz)
3. **TERMAL**: LDO guc dagilimini hesapla, gerekirse DC-DC pre-regulator ekle
4. **ESD**: RF pinlerde ESD koruma YOK - montaj sirasinda ESD onlem al
5. **RO4350B TEMINI**: Rogers malzeme teslim suresi 4-6 hafta olabilir
