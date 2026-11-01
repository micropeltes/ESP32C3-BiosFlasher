# SPI ESP32-C3 BIOS FLASHER

Programmer SPI-NOR / chip BIOS (SOIC8, 3.3 V) berbasis **ESP32-C3-DevKitM-1**
memakai **USB native (USB-Serial/JTAG) bawaan ESP32-C3** — **tanpa USB-to-TTL /
CH340 / CP2102**. Kabel USB cukup dicolok ke port USB DevKitM-1.

- Firmware: `src/main.cpp` (Arduino framework, PlatformIO)
- Tool host: `tools/biosflash.py` (Python + pyserial)

## 1. Wiring

| ESP32-C3-DevKitM-1 | Chip SPI-NOR (SOP8) | Keterangan |
|--------------------|---------------------|------------|
| GPIO4  (SCK)       | pin 6  CLK          | clock |
| GPIO5  (MISO)      | pin 2  DO / IO1     | data chip -> ESP |
| GPIO6  (MOSI)      | pin 5  DI / IO0     | data ESP -> chip |
| GPIO7  (CS)        | pin 1  CS#          | chip select |
| 3V3                | pin 8  VCC          | **3.3 V saja**, tambah 100 nF ke GND |
| GND                | pin 4  GND          | ground |

- Pin 3 (`WP#`) dan pin 7 (`HOLD#`/`RESET#`) **harus HIGH**: biasanya sudah ada
  pull-up di modul/clip. Kalau tidak, hubungkan ke 3V3 lewat resistor 10 kΩ
  (atau set `FLASH_WP_PIN` / `FLASH_HOLD_PIN` lewat `build_flags`).
- **GPIO18/GPIO19 dipakai USB native** — jangan dipakai untuk SPI.
- Kalau chip masih menempel di motherboard: matikan motherboard (idealnya tahan
  sinyal reset SoC-nya) dan jangan memberi daya dari dua sumber sekaligus.
- Kabel sebaiknya pendek (< 15 cm) supaya sinyal bersih.

## 2. Build & upload

```bash
pio run                 # compile
pio run -t upload       # upload lewat port USB native (esptool USB-JTAG-Serial)
pio device monitor      # console 115200 (USB CDC)
```

`platformio.ini` sudah menyetel:

```ini
build_flags = -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1
```

artinya `Serial` di firmware = USB CDC internal C3 (port USB DevKitM-1).

Kalau board "hilang" setelah upload: tekan **BOOT** lalu **RESET** untuk masuk
mode download, atau cabut-colok USB.

## 3. Perintah console (ketik `help`)

| Perintah | Fungsi |
|----------|--------|
| `info` / `i` / `id` | JEDEC ID, nama chip, kapasitas, status register, proteksi |
| `status` | baca SR1/SR2/SR3 |
| `read <addr> [len]` | hexdump isi flash |
| `binread <addr> <len>` | dump biner (header `@BIN`, akhir `@END`) |
| `write <addr>` | mulai tulis: kirim baris hex, `.` = selesai |
| `verify <addr>` | verifikasi dengan baris hex, `.` = selesai |
| `erase <addr> [n]` | hapus n sector 4 KB |
| `berase <addr> [n]` | hapus n block 64 KB |
| `chiperase confirm` | hapus seluruh chip |
| `unprotect` | clear block-protect (BP = 000) |
| `wsr <sr1> [sr2]` | tulis status register manual |
| `freq <khz>` | ubah clock SPI (100–40000 kHz, default 8000) |
| `fastread on|off` | pakai perintah 0x0B (perlu untuk clock tinggi) |
| `addr4 on|off` | mode alamat 4 byte (otomatis untuk chip > 16 MB) |
| `reset` | soft reset chip flash |
| `pins` | tampilkan pin yang dipakai |

Balasan: `+` = baris data diterima, `!ERR ...` = gagal, `# ...` = info/progress,
`OK ...` / `DONE ...` / `VERIFY OK ...` = sukses.

## 4. Tool host (lebih praktis)

```bash
pip install pyserial

python tools/biosflash.py id                        # deteksi chip
python tools/biosflash.py read 0 0x400000 -o backup.bin
python tools/biosflash.py write bios.bin            # erase + write + verify
python tools/biosflash.py verify bios.bin
python tools/biosflash.py erase 0 0x400000          # hapus sector 4 KB
python tools/biosflash.py chiperase --yes
python tools/biosflash.py console                   # mode interaktif
```

- `-p COM5` untuk memilih port (default: auto-detect VID `0x303A`).
- Alamat/panjang boleh desimal atau `0x...`.
- `write` default mulai di alamat 0 dan otomatis meng-erase sector yang
  tercakup, lalu memverifikasi.

## 5. Troubleshooting

| Gejala | Penyebab / solusi |
|--------|-------------------|
| `Chip: TIDAK terdeteksi` (ID `00 00 00` / `FF FF FF`) | MISO tidak tersambung, chip tidak dapat 3V3, CS salah pin, atau chip masih di-hold motherboard |
| Erase/program gagal / data acak | kabel terlalu panjang, clock terlalu tinggi (`freq 1000`), atau `WP#`/`HOLD#` tidak HIGH |
| `!ERR gagal program halaman (write-protect?)` | jalankan `unprotect`, cek SR1 BP bits |
| Banyak `MISMATCH` saat verify | turunkan clock (`freq 1000`), perbaiki grounding |
| Chip > 16 MB tidak terbaca benar | pastikan `info` menunjukkan `alamat 4 byte` |
| Upload firmware gagal | pakai port USB native + tekan BOOT/RESET, atau `upload_speed = 115200` |

## 6. Catatan keamanan data

Selalu **backup dulu** (`read`) sebelum `write` / `chiperase`. Untuk chip BIOS
motherboard, simpan dump asli supaya bisa dipulihkan kalau update gagal.
