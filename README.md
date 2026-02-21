# El Tipi 77 GHz SAR Radar

Taşınabilir, el tipi Sentetik Açıklıklı Radar (SAR) görüntüleme cihazı. 76-81 GHz bandında çalışarak yüksek çözünürlüklü radar görüntüleri oluşturur.

## Sistem Mimarisi

```
┌────────────────────────────────────────────────────────────┐
│                   EL TİPİ CİHAZ (~400g)                    │
│                                                            │
│  ┌──────────────┐    ┌──────────────┐   ┌──────────────┐  │
│  │  TI AWR2243  │    │ ICM-42688-P  │   │  ESP32-C3    │  │
│  │  77GHz FMCW  │    │  6-DOF IMU   │   │  WiFi/BT     │  │
│  │  5GHz BW     │    │  32kHz ODR   │   │  Veri Aktarım│  │
│  │  3Tx / 4Rx   │    └──────┬───────┘   └──────┬───────┘  │
│  └──────┬───────┘           │                   │          │
│         │ LVDS/SPI          │ SPI               │ SPI      │
│         ▼                   ▼                   ▼          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │            Xilinx Artix-7 XC7A100T FPGA             │   │
│  │  - Radar veri yakalama (LVDS IF data)               │   │
│  │  - Range FFT (on isleme)                            │   │
│  │  - IMU veri toplama + timestamp                     │   │
│  │  - SD karta ham veri kayit                          │   │
│  │  - WiFi uzerinden PC'ye veri akisi                  │   │
│  └──────────────┬──────────────────────────────────────┘   │
│                 │                                          │
│  ┌──────────┐  │  ┌──────────┐  ┌──────────────────────┐  │
│  │ST7789 TFT│  │  │ microSD  │  │ 2S Li-Po 4000mAh    │  │
│  │ 1.3" LCD │  │  │ 128 GB   │  │ 29.6Wh - USB-C PD   │  │
│  │ Durum    │  │  │ Ham Veri │  │ BQ25792 Sarj IC     │  │
│  └──────────┘  │  └──────────┘  └──────────────────────┘  │
│                │                                           │
│  Butonlar: [Guc] [Tarama Baslat/Durdur] [Mod]            │
│  Boyut: 180 x 90 x 45 mm                                  │
└────────────────────────────────────────────────────────────┘
         │
         │ WiFi (TCP/UDP stream) veya USB
         ▼
┌────────────────────────────────────────┐
│  PC / Tablet                           │
│  - SAR Goruntu Isleme (Python/CUDA)    │
│  - Back-Projection Algorithm (BPA)     │
│  - PGA Autofocus                       │
│  - Gercek zamanli goruntuleme          │
└────────────────────────────────────────┘
```

## Performans Hedefleri

| Parametre | Hedef |
|-----------|-------|
| Frekans | 76-81 GHz |
| Bant genisligi | 5 GHz |
| Menzil cozunurlugu | 3.0 cm |
| Capraz menzil cozunurlugu | 0.6-2 cm |
| Goruntuleme mesafesi | 0.5-5 m |
| Batarya omru | ~6 saat |
| Agirlik | ~400-500g |
| Boyut | 180x90x45 mm |

## Gelistirme Fazlari

### Faz 1: Algoritma Gelistirme (EVM Tabanli)
- TI AWR2243BOOST + DCA1000EVM ile SAR goruntuleme dogrulama
- Back-Projection Algorithm (BPA) implementasyonu
- Phase Gradient Autofocus (PGA)
- IMU tabanli hareket tahmini ve kompanzasyonu

### Faz 2: Ozel PCB Tasarimi
- **RF Modulu**: Rogers RO4350B, AWR2243 + patch anten dizisi
- **Ana Kart**: FR4 6 katman, Artix-7 FPGA + guc + sensorler

### Faz 3: Firmware + Yazilim
- FPGA Verilog: veri yakalama, Range FFT, SD kayit, WiFi bridge
- PC Python: SAR isleyici, CUDA GPU hizlandirma, GUI

### Faz 4: Entegrasyon + Kasa
- 3D baski PETG kasa + HDPE radome
- Batarya entegrasyonu, termal yonetim, saha testleri

## Tahmini Maliyet

| Senaryo | Maliyet |
|---------|---------|
| Sadece Ozel PCB (Faz 2-4) | $280-525 |
| EVM ile basla (Faz 1 dahil) | $950-1,500 |
| Tam proje (EVM + Ozel PCB) | $1,200-2,000 |

## Klasor Yapisi

```
sar-radar/
├── hardware/           # Donanim tasarim dosyalari
│   ├── schematics/     # KiCad sematikler
│   ├── pcb/            # PCB layout
│   ├── bom/            # Malzeme listeleri
│   ├── mechanical/     # 3D kasa tasarimi
│   └── production/     # Uretim dosyalari
├── firmware/           # Gomulu yazilim
│   ├── fpga/           # Verilog FPGA tasarimi
│   └── esp32/          # ESP32-C3 WiFi bridge
├── software/           # PC yazilimi
│   ├── sar_processor/  # Python SAR isleyici
│   ├── gui/            # Goruntuleme uygulamasi
│   └── simulation/     # RadarSimPy simulasyonlar
└── docs/               # Dokumantasyon
```

## Hizli Baslangic

```bash
# Python bagimliliklari
cd software/
pip install -r requirements.txt

# Simulasyon calistir
python simulation/sar_simulation.py

# SAR isleyici
python sar_processor/main.py --input data/raw_capture.bin
```

## Lisans

MIT License
