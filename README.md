# IoT Health Monitor - Malaysia

Proyek IoT berbasis ESP32 untuk monitoring kesehatan, meliputi:

- **Heart Rate & SpO2** — Menggunakan sensor MAX30102
- **Handgrip Strength** — Menggunakan load cell HX711
- **Blood Pressure (Tensimeter)** — Menggunakan sensor tekanan MPX via ADS1115

## Fitur

- LCD 20x4 dengan navigasi menu (tombol Up/Down/Select)
- Indikator LED (Hijau/Kuning/Merah) berdasarkan hasil pengukuran
- Koneksi WiFi (STA mode) dengan fallback ke AP mode
- HTTP Server untuk akses data via REST API
- Penyimpanan lokal (SPIFFS) jika tidak ada koneksi internet
- Upload otomatis data yang tertunda saat koneksi tersedia
- Monitoring level baterai

## Hardware

| Komponen          | Pin / Alamat |
|-------------------|-------------|
| MAX30102          | I2C (SDA 21, SCL 22) |
| LCD 20x4 I2C     | Alamat 0x27 |
| ADS1115           | I2C default |
| HX711 DOUT        | GPIO 4 |
| HX711 SCK         | GPIO 5 |
| Tombol Up          | GPIO 12 |
| Tombol Down        | GPIO 15 |
| Tombol Select      | GPIO 14 |
| LED Hijau          | GPIO 16 |
| LED Kuning         | GPIO 17 |
| LED Merah          | GPIO 2 |
| Pompa              | GPIO 18 |
| Valve              | GPIO 19 |
| Baterai (ADC)      | GPIO 34 |

## API Endpoints

| Endpoint              | Method | Deskripsi |
|-----------------------|--------|-----------|
| `/data`               | GET    | Data sensor terakhir |
| `/status`             | GET    | Status koneksi & sensor |
| `/measure/heart`      | POST   | Mulai pengukuran HR & SpO2 |
| `/measure/grip`       | POST   | Mulai pengukuran handgrip |
| `/measure/bp`         | POST   | Mulai pengukuran tekanan darah |
| `/measure/all`        | POST   | Mulai semua pengukuran |

## Penggunaan

1. Upload `sketch_mar9a.ino` ke board ESP32
2. Ubah `STA_SSID` dan `STA_PASSWORD` sesuai jaringan WiFi Anda
3. Ubah `SERVER_URL` sesuai endpoint server Anda