/* ============================================================================
 *  SPI-NOR / BIOS FLASH PROGRAMMER
 *  Target board : ESP32-C3-DevKitM-1
 *  Interface    : native USB (USB-Serial/JTAG internal ESP32-C3)
 *                 -> kabel USB langsung ke DevKitM-1, TANPA USB-to-TTL
 *
 *  Wiring (SOP8 / SOIC8 clip, chip BIOS motherboard):
 *
 *      ESP32-C3-DevKitM-1          SPI flash chip
 *      ------------------          --------------
 *      GPIO4   (SCK)   ----------  CLK   (pin 6)
 *      GPIO5   (MISO)  <---------  DO    (pin 2, IO1)
 *      GPIO6   (MOSI)  ----------  DI    (pin 5, IO0)
 *      GPIO7   (CS)    ----------  CS#   (pin 1)
 *      3V3             ----------  VCC   (pin 8) + 100nF ke GND
 *      GND             ----------  GND   (pin 4)
 *      (opsional) WP# (pin 3) dan HOLD#/RESET# (pin 7) -> 3V3 lewat 10k
 *
 *  PENTING:
 *   - GPIO18/GPIO19 dipakai USB native. JANGAN dipakai untuk SPI.
 *   - Chip 3.3V saja. Jangan beri 5V.
 *   - Kalau chip masih menempel di motherboard, motherboard HARUS mati
 *     (idealnya SoC di-hold reset) supaya bus SPI tidak bentrok.
 *   - Jangan beri daya dari motherboard DAN DevKit sekaligus.
 *
 *  Protokol console (ASCII lewat USB CDC):
 *      help | h                      daftar perintah
 *      info | i | id                 JEDEC ID + kapasitas + status + proteksi
 *      status | st                   baca status register 1/2/3
 *      read <addr> [len]             hexdump isi flash
 *      binread <addr> <len>          dump biner mentah (header "@BIN")
 *      write <addr>                  mulai tulis (kirim baris hex, "." selesai)
 *      verify <addr>                 bandingkan isi flash dengan baris hex
 *      erase <addr> [jumlah]         hapus sector 4 KB
 *      berase <addr> [jumlah]        hapus block 64 KB
 *      chiperase confirm             hapus seluruh chip
 *      unprotect                     clear block-protect (SR1=0, SR2=0)
 *      wsr <sr1> [sr2]               tulis status register
 *      freq <khz>                    ubah clock SPI (100..40000 kHz)
 *      addr4 <on|off>                paksa mode alamat 3/4 byte
 *      reset                         soft-reset chip flash
 *      pins                          tampilkan konfigurasi pin
 *
 *  Contoh menulis BIOS 4 MB (pakai tools/biosflash.py, lebih praktis):
 *      write 0
 *      <baris hex 32 byte> ...
 *      .
 * ==========================================================================*/

#include <Arduino.h>
#include <SPI.h>
#include <strings.h>

/* ----------------------------- pin / clock -------------------------------- */
#ifndef FLASH_SCK_PIN
#define FLASH_SCK_PIN 4
#endif
#ifndef FLASH_MISO_PIN
#define FLASH_MISO_PIN 5
#endif
#ifndef FLASH_MOSI_PIN
#define FLASH_MOSI_PIN 6
#endif
#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN 7
#endif

/* Opsional: kalau WP#/HOLD# ikut disambung ke GPIO, isi nomor pin-nya.
 * Default -1 = tidak dipakai (chip memakai pull-up internal). */
#ifndef FLASH_WP_PIN
#define FLASH_WP_PIN -1
#endif
#ifndef FLASH_HOLD_PIN
#define FLASH_HOLD_PIN -1
#endif

#ifndef FLASH_SPI_HZ
#define FLASH_SPI_HZ 8000000UL
#endif

/* ------------------------------ konstanta -------------------------------- */
static const uint32_t SPI_HZ_MIN = 100000UL;
static const uint32_t SPI_HZ_MAX = 40000000UL;
static const uint32_t CHIP_SIZE_UNKNOWN = 16UL * 1024UL * 1024UL;
static const uint16_t PAGE_SIZE = 256;
static const uint32_t SECTOR_SIZE = 4096UL;
static const uint32_t BLOCK_SIZE = 65536UL;
static const uint32_t INPUT_TIMEOUT_MS = 15000UL;
static const size_t LINE_BUF_SIZE = 720;

/* SPI-NOR opcodes */
#define CMD_WRITE_ENABLE 0x06
#define CMD_WRITE_DISABLE 0x04
#define CMD_READ_SR1 0x05
#define CMD_WRITE_SR 0x01
#define CMD_READ_SR2 0x35
#define CMD_READ_SR3 0x15
#define CMD_READ_DATA 0x03
#define CMD_FAST_READ 0x0B
#define CMD_PAGE_PROGRAM 0x02
#define CMD_SECTOR_ERASE 0x20
#define CMD_BLOCK_ERASE 0xD8
#define CMD_CHIP_ERASE 0x60
#define CMD_JEDEC_ID 0x9F
#define CMD_RELEASE_PD 0xAB
#define CMD_EN4B 0xB7
#define CMD_EX4B 0xE9
#define CMD_RESET_EN 0x66
#define CMD_RESET 0x99

/* ------------------------------- state ----------------------------------- */
static uint32_t g_spiHz = FLASH_SPI_HZ;
static bool g_addr4B = false;
static bool g_fastRead = false;
static uint32_t g_chipSize = 0;
static uint32_t g_jedecId = 0xFFFFFF;

/* buffer console */
static char g_line[LINE_BUF_SIZE];
static uint8_t g_rx[1024];
static size_t g_rxHead = 0, g_rxTail = 0;


/* ==========================================================================
 *  Bagian 1 - akses SPI level rendah
 * ==========================================================================*/

static inline void csLow() { digitalWrite(FLASH_CS_PIN, LOW); }
static inline void csHigh() { digitalWrite(FLASH_CS_PIN, HIGH); }

static void spiApplyClock() {
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(g_spiHz, MSBFIRST, SPI_MODE0));
}

static inline void selectChip() {
  csLow();
  delayMicroseconds(1);
}

static inline void deselectChip() {
  csHigh();
  delayMicroseconds(2);
}

static inline uint8_t xfer(uint8_t b) { return SPI.transfer(b); }

static void sendAddr(uint32_t addr) {
  if (g_addr4B) {
    xfer((uint8_t)(addr >> 24));
  }
  xfer((uint8_t)(addr >> 16));
  xfer((uint8_t)(addr >> 8));
  xfer((uint8_t)addr);
}

static uint32_t readJedecId() {
  selectChip();
  xfer(CMD_JEDEC_ID);
  uint32_t id = 0;
  for (uint8_t i = 0; i < 3; i++) {
    id = (id << 8) | xfer(0xFF);
  }
  deselectChip();
  return id;
}

static uint8_t readStatus(uint8_t cmd) {
  selectChip();
  xfer(cmd);
  uint8_t v = xfer(0xFF);
  deselectChip();
  return v;
}

static bool waitReady(uint32_t timeoutMs, bool showProgress = false) {
  uint32_t t0 = millis();
  uint32_t lastDot = 0;
  while (readStatus(CMD_READ_SR1) & 0x01) {
    uint32_t elapsed = millis() - t0;
    if (elapsed > timeoutMs) {
      return false;
    }
    if (showProgress && (elapsed - lastDot) >= 2000) {
      lastDot = elapsed;
      Serial.write('.');
    }
    delay(1);
  }
  return true;
}

static bool writeEnable() {
  selectChip();
  xfer(CMD_WRITE_ENABLE);
  deselectChip();
  return (readStatus(CMD_READ_SR1) & 0x02) != 0;
}

static void readData(uint32_t addr, uint8_t *buf, uint32_t len) {
  if (!len) {
    return;
  }
  selectChip();
  if (g_fastRead) {
    xfer(CMD_FAST_READ);
    sendAddr(addr);
    xfer(0xFF); /* dummy byte */
  } else {
    xfer(CMD_READ_DATA);
    sendAddr(addr);
  }
  SPI.transferBytes(nullptr, buf, len);
  deselectChip();
}

/* program satu halaman; TIDAK boleh melewati batas halaman 256 byte */
static bool programPage(uint32_t addr, const uint8_t *data, uint16_t len) {
  if (!len) {
    return true;
  }
  if (!writeEnable()) {
    return false;
  }
  selectChip();
  xfer(CMD_PAGE_PROGRAM);
  sendAddr(addr);
  SPI.transferBytes(data, nullptr, len);
  deselectChip();
  return waitReady(2000);
}

static bool eraseSector(uint32_t addr) {
  if (!writeEnable()) {
    return false;
  }
  selectChip();
  xfer(CMD_SECTOR_ERASE);
  sendAddr(addr);
  deselectChip();
  return waitReady(5000);
}

static bool eraseBlock(uint32_t addr) {
  if (!writeEnable()) {
    return false;
  }
  selectChip();
  xfer(CMD_BLOCK_ERASE);
  sendAddr(addr);
  deselectChip();
  return waitReady(20000);
}

static bool writeStatusRegister(uint8_t sr1, uint8_t sr2) {
  if (!writeEnable()) {
    return false;
  }
  selectChip();
  xfer(CMD_WRITE_SR);
  xfer(sr1);
  xfer(sr2);
  deselectChip();
  return waitReady(2000);
}

static void set4ByteMode(bool enable) {
  selectChip();
  xfer(enable ? CMD_EN4B : CMD_EX4B);
  deselectChip();
  delay(1);
  g_addr4B = enable;
}

static void softResetChip() {
  selectChip();
  xfer(CMD_RESET_EN);
  deselectChip();
  delayMicroseconds(50);
  selectChip();
  xfer(CMD_RESET);
  deselectChip();
  delay(5); /* tRST ~ 30 us */
}

/* ==========================================================================
 *  Bagian 2 - identifikasi chip
 * ==========================================================================*/

struct MfrEntry {
  uint8_t id;
  const char *name;
};

static const MfrEntry MFR_TABLE[] = {
    {0x01, "Spansion/Cypress"}, {0x04, "Fujitsu"},     {0x0B, "XTX"},
    {0x1C, "EON"},              {0x1F, "Adesto/Atmel"}, {0x20, "Micron/ST"},
    {0x37, "AMIC"},             {0x5E, "Zbit"},        {0x68, "Boya"},
    {0x85, "Puya"},             {0x89, "Intel"},       {0x8C, "ESMT"},
    {0x98, "Toshiba"},          {0x9D, "ISSI"},        {0xBF, "SST/Microchip"},
    {0xC2, "Macronix"},         {0xC8, "GigaDevice"},  {0xEF, "Winbond"},
};

static const char *mfrName(uint8_t id) {
  for (size_t i = 0; i < sizeof(MFR_TABLE) / sizeof(MFR_TABLE[0]); i++) {
    if (MFR_TABLE[i].id == id) {
      return MFR_TABLE[i].name;
    }
  }
  return "Unknown";
}

/* nama umum chip berdasarkan JEDEC ID */
static void chipName(uint32_t id, char *out, size_t outLen) {
  uint8_t mfr = (uint8_t)(id >> 16);
  uint8_t cap = (uint8_t)(id & 0xFF);
  uint32_t size = 0;
  if (cap >= 0x10 && cap <= 0x1C) {
    size = 1UL << cap;
  }
  unsigned long mbit = (unsigned long)(size / 1024UL / 1024UL) * 8UL;

  switch (mfr) {
    case 0xEF:
      snprintf(out, outLen, "W25Q%lu", mbit);
      break;
    case 0xC8:
      snprintf(out, outLen, "GD25Q%lu", mbit);
      break;
    case 0xC2:
      snprintf(out, outLen, "MX25L%lu", mbit);
      break;
    case 0x1C:
      snprintf(out, outLen, "EN25Q%lu", mbit);
      break;
    case 0x20:
      snprintf(out, outLen, "M25P%lu", mbit);
      break;
    case 0x9D:
      snprintf(out, outLen, "IS25LQ%lu", mbit);
      break;
    default:
      snprintf(out, outLen, "%lu Mbit", mbit);
      break;
  }
}

static bool chipPresent() {
  return g_jedecId != 0x000000 && g_jedecId != 0xFFFFFF;
}

/* baca ID, hitung kapasitas, aktifkan mode alamat 4 byte bila perlu */
static void detectChip() {
  g_jedecId = readJedecId();
  g_chipSize = 0;
  g_addr4B = false;
  if (!chipPresent()) {
    return;
  }
  uint8_t cap = (uint8_t)(g_jedecId & 0xFF);
  if (cap >= 0x10 && cap <= 0x1C) {
    g_chipSize = 1UL << cap;
  }
  if (g_chipSize > 16UL * 1024UL * 1024UL) {
    set4ByteMode(true);
  }
}

static uint32_t chipLimit() {
  return g_chipSize ? g_chipSize : CHIP_SIZE_UNKNOWN;
}

static void printChipLine() {
  if (!chipPresent()) {
    Serial.print("Chip      : TIDAK terdeteksi - cek wiring, 3V3, dan GND\r\n");
    return;
  }
  char name[40];
  chipName(g_jedecId, name, sizeof(name));
  uint32_t size = chipLimit();
  Serial.printf("JEDEC ID  : %02X %02X %02X\r\n",
                (unsigned)((g_jedecId >> 16) & 0xFF),
                (unsigned)((g_jedecId >> 8) & 0xFF),
                (unsigned)(g_jedecId & 0xFF));
  Serial.printf("Chip      : %s %s (%lu MB / %lu Mbit)\r\n",
                mfrName((uint8_t)(g_jedecId >> 16)), name,
                (unsigned long)(size / 1024UL / 1024UL),
                (unsigned long)(size / 1024UL / 1024UL) * 8UL);
}

static void printStatusLine(const char *label, uint8_t cmd) {
  uint8_t v = readStatus(cmd);
  Serial.printf("%s: 0x%02X", label, v);
  if (cmd == CMD_READ_SR1) {
    Serial.printf("  BUSY=%u WEL=%u BP=%u%u%u TB=%u SEC=%u SRP0=%u", v & 0x01,
                  (unsigned)((v >> 1) & 0x01), (unsigned)((v >> 4) & 0x01),
                  (unsigned)((v >> 3) & 0x01), (unsigned)((v >> 2) & 0x01),
                  (unsigned)((v >> 5) & 0x01), (unsigned)((v >> 6) & 0x01),
                  (unsigned)((v >> 7) & 0x01));
  } else if (cmd == CMD_READ_SR2) {
    Serial.printf("  QE=%u SRP1=%u CMP=%u", (unsigned)((v >> 1) & 0x01),
                  (unsigned)(v & 0x01), (unsigned)((v >> 6) & 0x01));
  }
  Serial.print("\r\n");
}

static void printConfigLine() {
  Serial.printf("SPI clock : %lu kHz, mode 0, alamat %u byte%s\r\n",
                (unsigned long)(g_spiHz / 1000UL), g_addr4B ? 4u : 3u,
                g_fastRead ? ", fast-read 0x0B" : "");
  Serial.printf("Pins      : SCK=%d MISO=%d MOSI=%d CS=%d\r\n", FLASH_SCK_PIN,
                FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);
}


/* ==========================================================================
 *  Bagian 3 - helper console (input baris, hex, output)
 * ==========================================================================*/

static int serialPeek() {
  if (g_rxHead == g_rxTail) {
    int avail = Serial.available();
    if (avail <= 0) {
      return -1;
    }
    if (avail > (int)sizeof(g_rx)) {
      avail = (int)sizeof(g_rx);
    }
    int got = (int)Serial.read(g_rx, (size_t)avail);
    if (got <= 0) {
      return -1;
    }
    g_rxHead = 0;
    g_rxTail = (size_t)got;
  }
  return g_rx[g_rxHead];
}

static int serialReadByte() {
  int c = serialPeek();
  if (c < 0) {
    return -1;
  }
  g_rxHead++;
  return c;
}

/* baca satu baris (tanpa CR/LF). return panjang, -1 kalau timeout. */
static int readLine(char *out, size_t maxLen, uint32_t timeoutMs) {
  size_t len = 0;
  uint32_t t0 = millis();
  while (true) {
    int c = serialReadByte();
    if (c < 0) {
      if (millis() - t0 >= timeoutMs) {
        out[len] = '\0';
        return len ? (int)len : -1;
      }
      delay(1);
      continue;
    }
    t0 = millis();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      out[len] = '\0';
      return (int)len;
    }
    if (len + 1 < maxLen) {
      out[len++] = (char)c;
    }
  }
}

static bool parseU32(const char *s, uint32_t *out) {
  if (!s || !*s) {
    return false;
  }
  char *end = nullptr;
  unsigned long v = strtoul(s, &end, 0);
  if (end == s) {
    return false;
  }
  *out = (uint32_t)v;
  return true;
}

static int hexNibble(int c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return -1;
}

/* decode baris hex (spasi diabaikan, '#' = komentar).
 * return jumlah byte, -1 = format salah, -2 = buffer penuh */
static int parseHexLine(const char *s, uint8_t *out, size_t maxLen) {
  size_t count = 0;
  int hi = -1;
  while (*s) {
    char c = *s++;
    if (c == '#' || c == ';') {
      break;
    }
    if (c == ' ' || c == '\t' || c == ':' || c == ',') {
      continue;
    }
    int n = hexNibble((unsigned char)c);
    if (n < 0) {
      return -1;
    }
    if (hi < 0) {
      hi = n;
    } else {
      if (count >= maxLen) {
        return -2;
      }
      out[count++] = (uint8_t)((hi << 4) | n);
      hi = -1;
    }
  }
  if (hi >= 0) {
    return -1; /* jumlah digit ganjil */
  }
  return (int)count;
}

static bool isDataEnd(const char *s) {
  return strcmp(s, ".") == 0 || strcasecmp(s, "end") == 0 ||
         strcasecmp(s, "eof") == 0 || strcasecmp(s, "done") == 0;
}

static void hexDumpLine(uint32_t addr, const uint8_t *buf, uint32_t n) {
  char line[96];
  int p = snprintf(line, sizeof(line), "%08lX  ", (unsigned long)addr);
  for (uint32_t i = 0; i < 16; i++) {
    if (i < n) {
      p += snprintf(line + p, sizeof(line) - p, "%02X ", buf[i]);
    } else {
      p += snprintf(line + p, sizeof(line) - p, "   ");
    }
    if (i == 7) {
      p += snprintf(line + p, sizeof(line) - p, " ");
    }
  }
  p += snprintf(line + p, sizeof(line) - p, " |");
  for (uint32_t i = 0; i < n; i++) {
    line[p++] = (buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.';
  }
  line[p++] = '|';
  line[p++] = '\n';
  Serial.write(line, (size_t)p);
}



/* ==========================================================================
 *  Bagian 4 - perintah console (info, read, erase, proteksi, clock)
 * ==========================================================================*/

static bool checkRange(uint32_t addr, uint32_t len) {
  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi (cek wiring / 3V3)\r\n");
    return false;
  }
  if ((uint64_t)addr + (uint64_t)len > (uint64_t)chipLimit()) {
    Serial.printf("!ERR range 0x%08lX+%lu di luar chip (%lu byte)\r\n",
                  (unsigned long)addr, (unsigned long)len,
                  (unsigned long)chipLimit());
    return false;
  }
  return true;
}

static void printHelp() {
  Serial.print(
      "\r\n=== BIOS / SPI-NOR FLASHER - ESP32-C3 DevKitM-1 (native USB) ===\r\n"
      " info | i | id              ID chip + kapasitas + status + proteksi\r\n"
      " status | st               status register 1/2/3\r\n"
      " read <addr> [len]         hexdump isi flash\r\n"
      " binread <addr> <len>      dump biner (header @BIN, akhir @END)\r\n"
      " write <addr>              tulis data (kirim baris hex, '.' = selesai)\r\n"
      " verify <addr>             verifikasi (kirim baris hex, '.' = selesai)\r\n"
      " erase <addr> [n]          hapus n sector (4 KB, default 1)\r\n"
      " berase <addr> [n]         hapus n block (64 KB, default 1)\r\n"
      " chiperase confirm         hapus seluruh chip\r\n"
      " unprotect                 clear block protect (BP=000)\r\n"
      " wsr <sr1> [sr2]           tulis status register\r\n"
      " freq <khz>                clock SPI (100..40000 kHz)\r\n"
      " fastread <on|off>         pakai perintah 0x0B\r\n"
      " addr4 <on|off>            mode alamat 4 byte (chip > 16 MB)\r\n"
      " reset                     soft reset chip flash\r\n"
      " pins                      tampilkan pin yang dipakai\r\n"
      " help | h                  menu ini\r\n"
      " addr/len boleh desimal atau hex (0x...)\r\n"
      " Progress: '+' = baris diterima, '!' = error, '#' = info\r\n\r\n");
}

static void cmdInfo() {
  detectChip();
  Serial.print("--- FLASH CHIP ---\r\n");
  printChipLine();
  if (chipPresent()) {
    printStatusLine("Status 1", CMD_READ_SR1);
    printStatusLine("Status 2", CMD_READ_SR2);
    printStatusLine("Status 3", CMD_READ_SR3);
  }
  printConfigLine();
}

static void cmdStatus() {
  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi\r\n");
    return;
  }
  printStatusLine("Status 1", CMD_READ_SR1);
  printStatusLine("Status 2", CMD_READ_SR2);
  printStatusLine("Status 3", CMD_READ_SR3);
}

static void cmdRead(uint32_t addr, uint32_t len) {
  static uint8_t buf[256];
  if (!len) {
    len = 256;
  }
  if (!checkRange(addr, len)) {
    return;
  }
  uint32_t t0 = millis();
  uint32_t remaining = len;
  while (remaining) {
    uint32_t n = remaining > sizeof(buf) ? (uint32_t)sizeof(buf) : remaining;
    readData(addr, buf, n);
    for (uint32_t off = 0; off < n; off += 16) {
      uint32_t m = n - off;
      if (m > 16) {
        m = 16;
      }
      hexDumpLine(addr + off, buf + off, m);
    }
    addr += n;
    remaining -= n;
  }
  Serial.printf("OK read %lu byte dalam %lu ms\r\n", (unsigned long)len,
                (unsigned long)(millis() - t0));
}

static void cmdBinRead(uint32_t addr, uint32_t len) {
  static uint8_t buf[512];
  if (!len) {
    Serial.print("!ERR butuh <len>\r\n");
    return;
  }
  if (!checkRange(addr, len)) {
    return;
  }
  Serial.printf("@BIN %08lX %lu\n", (unsigned long)addr, (unsigned long)len);
  uint32_t t0 = millis();
  uint32_t remaining = len;
  while (remaining) {
    uint32_t n = remaining > sizeof(buf) ? (uint32_t)sizeof(buf) : remaining;
    readData(addr, buf, n);
    Serial.write(buf, n);
    addr += n;
    remaining -= n;
  }
  Serial.printf("\n@END %lu ms\n", (unsigned long)(millis() - t0));
}

static void cmdErase(uint32_t addr, uint32_t count, bool block) {
  const uint32_t step = block ? BLOCK_SIZE : SECTOR_SIZE;
  if (!count) {
    count = 1;
  }
  addr -= addr % step; /* bulatkan ke bawah */
  if (!checkRange(addr, count * step)) {
    return;
  }
  uint32_t t0 = millis();
  for (uint32_t i = 0; i < count; i++) {
    uint32_t a = addr + i * step;
    bool ok = block ? eraseBlock(a) : eraseSector(a);
    if (!ok) {
      Serial.printf("\r\n!ERR hapus gagal @ 0x%08lX (timeout / write-protect)\r\n",
                    (unsigned long)a);
      return;
    }
    if ((i % 64) == 0 || i + 1 == count) {
      Serial.printf("# erase %lu/%lu @ 0x%08lX\r\n", (unsigned long)(i + 1),
                    (unsigned long)count, (unsigned long)a);
    }
  }
  Serial.printf("OK hapus %lu %s (%lu KB) dalam %lu ms\r\n", (unsigned long)count,
                block ? "block" : "sector",
                (unsigned long)(count * step / 1024UL),
                (unsigned long)(millis() - t0));
}

static void cmdChipErase(bool confirmed) {
  if (!confirmed) {
    Serial.print("!ERR konfirmasi diperlukan: 'chiperase confirm'\r\n");
    return;
  }
  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi\r\n");
    return;
  }
  Serial.print("# menghapus seluruh chip, bisa 1-3 menit...\r\n");
  if (!writeEnable()) {
    Serial.print("!ERR write enable gagal (proteksi?)\r\n");
    return;
  }
  selectChip();
  xfer(CMD_CHIP_ERASE);
  deselectChip();
  uint32_t t0 = millis();
  if (!waitReady(400000UL, true)) {
    Serial.print("\r\n!ERR timeout chip erase\r\n");
    return;
  }
  Serial.printf("\r\nOK chip terhapus dalam %lu ms\r\n",
                (unsigned long)(millis() - t0));
}

static void cmdUnprotect() {
  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi\r\n");
    return;
  }
  uint8_t sr1 = readStatus(CMD_READ_SR1);
  uint8_t sr2 = readStatus(CMD_READ_SR2);
  uint8_t newSr2 = sr2 & 0x02; /* pertahankan bit QE */
  Serial.printf("# SR1=0x%02X SR2=0x%02X -> SR1=0x00 SR2=0x%02X\r\n", sr1, sr2,
                newSr2);
  if (!writeStatusRegister(0x00, newSr2)) {
    Serial.print("!ERR gagal menulis status register\r\n");
    return;
  }
  printStatusLine("Status 1", CMD_READ_SR1);
  printStatusLine("Status 2", CMD_READ_SR2);
  Serial.print("OK proteksi dibersihkan\r\n");
}

static void cmdWriteStatus(uint8_t sr1, uint8_t sr2) {
  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi\r\n");
    return;
  }
  if (!writeStatusRegister(sr1, sr2)) {
    Serial.print("!ERR gagal menulis status register\r\n");
    return;
  }
  Serial.printf("OK SR1=0x%02X SR2=0x%02X\r\n", sr1, sr2);
  printStatusLine("Status 1", CMD_READ_SR1);
  printStatusLine("Status 2", CMD_READ_SR2);
}

static void cmdFreq(uint32_t khz) {
  uint32_t hz = khz * 1000UL;
  if (hz < SPI_HZ_MIN) {
    hz = SPI_HZ_MIN;
  }
  if (hz > SPI_HZ_MAX) {
    hz = SPI_HZ_MAX;
  }
  g_spiHz = hz;
  spiApplyClock();
  Serial.printf("OK clock SPI = %lu kHz\r\n", (unsigned long)(g_spiHz / 1000UL));
}

static void cmdAddr4(bool on) {
  set4ByteMode(on);
  Serial.printf("OK mode alamat %s\r\n", on ? "4 byte (EN4B)" : "3 byte (EX4B)");
}



/* ==========================================================================
 *  Bagian 5 - streaming write / verify (data hex lewat USB)
 * ==========================================================================*/

static const size_t MAX_LINE_BYTES = 256;

static void cmdWrite(uint32_t addr) {
  static uint8_t data[MAX_LINE_BYTES];
  static uint8_t pageBuf[PAGE_SIZE];
  uint16_t pageLen = 0;
  uint32_t total = 0;
  uint32_t t0 = millis();
  bool ok = true;

  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi\r\n");
    return;
  }
  Serial.printf("WRITE 0x%08lX - kirim baris hex, '.' untuk selesai\n",
                (unsigned long)addr);

  while (true) {
    int n = readLine(g_line, sizeof(g_line), INPUT_TIMEOUT_MS);
    if (n < 0) {
      Serial.print("\n!ERR timeout menunggu data\n");
      ok = false;
      break;
    }
    if (n == 0) {
      continue;
    }
    if (isDataEnd(g_line)) {
      break;
    }
    int cnt = parseHexLine(g_line, data, sizeof(data));
    if (cnt == -1) {
      Serial.print("\n!ERR format hex salah (harus genap 0-9A-F)\n");
      ok = false;
      break;
    }
    if (cnt == -2) {
      Serial.printf("\n!ERR baris terlalu panjang (max %u byte)\n",
                    (unsigned)sizeof(data));
      ok = false;
      break;
    }
    if ((uint64_t)addr + total + (uint64_t)cnt > (uint64_t)chipLimit()) {
      Serial.print("\n!ERR data melewati akhir chip\n");
      ok = false;
      break;
    }
    for (int i = 0; i < cnt; i++) {
      uint32_t cur = addr + total;             /* alamat byte ini */
      uint32_t pageStart = cur - pageLen;
      uint16_t limit = (uint16_t)(PAGE_SIZE - (pageStart % PAGE_SIZE));
      pageBuf[pageLen++] = data[i];
      total++;
      if (pageLen >= limit) {
        if (!programPage(pageStart, pageBuf, pageLen)) {
          ok = false;
          break;
        }
        pageLen = 0;
      }
    }
    if (!ok) {
      Serial.print("\n!ERR gagal program halaman (write-protect?)\n");
      break;
    }
    Serial.write('+');
  }

  if (ok && pageLen) {
    uint32_t pageStart = addr + total - pageLen;
    if (!programPage(pageStart, pageBuf, pageLen)) {
      Serial.print("\n!ERR gagal program halaman terakhir\n");
      ok = false;
    }
  }
  if (ok) {
    Serial.printf("\nDONE %lu byte dalam %lu ms\n", (unsigned long)total,
                  (unsigned long)(millis() - t0));
  }
}

static void cmdVerify(uint32_t addr) {
  static uint8_t data[MAX_LINE_BYTES];
  static uint8_t chunk[256];
  static uint8_t flashBuf[256];
  uint16_t chunkLen = 0;
  uint32_t total = 0;
  uint32_t errors = 0;
  uint32_t t0 = millis();
  bool ok = true;

  if (!chipPresent()) {
    Serial.print("!ERR chip tidak terdeteksi\r\n");
    return;
  }
  Serial.printf("VERIFY 0x%08lX - kirim baris hex, '.' untuk selesai\n",
                (unsigned long)addr);

  while (true) {
    int n = readLine(g_line, sizeof(g_line), INPUT_TIMEOUT_MS);
    if (n < 0) {
      Serial.print("\n!ERR timeout menunggu data\n");
      ok = false;
      break;
    }
    if (n == 0) {
      continue;
    }
    if (isDataEnd(g_line)) {
      break;
    }
    int cnt = parseHexLine(g_line, data, sizeof(data));
    if (cnt < 0) {
      Serial.print("\n!ERR format hex salah / terlalu panjang\n");
      ok = false;
      break;
    }
    for (int i = 0; i < cnt; i++) {
      chunk[chunkLen++] = data[i];
      total++;
      if (chunkLen == sizeof(chunk)) {
        uint32_t chunkAddr = addr + total - chunkLen;
        readData(chunkAddr, flashBuf, chunkLen);
        for (uint16_t k = 0; k < chunkLen; k++) {
          if (flashBuf[k] != chunk[k]) {
            if (errors < 8) {
              Serial.printf("\nMISMATCH @0x%08lX exp %02X got %02X\n",
                            (unsigned long)(chunkAddr + k), chunk[k],
                            flashBuf[k]);
            }
            errors++;
          }
        }
        chunkLen = 0;
      }
    }
    Serial.write('+');
  }

  if (ok && chunkLen) {
    uint32_t chunkAddr = addr + total - chunkLen;
    readData(chunkAddr, flashBuf, chunkLen);
    for (uint16_t k = 0; k < chunkLen; k++) {
      if (flashBuf[k] != chunk[k]) {
        if (errors < 8) {
          Serial.printf("\nMISMATCH @0x%08lX exp %02X got %02X\n",
                        (unsigned long)(chunkAddr + k), chunk[k], flashBuf[k]);
        }
        errors++;
      }
    }
  }

  if (!ok) {
    return;
  }
  if (errors == 0) {
    Serial.printf("\nVERIFY OK %lu byte dalam %lu ms\n", (unsigned long)total,
                  (unsigned long)(millis() - t0));
  } else {
    Serial.printf("\n!ERR VERIFY GAGAL: %lu byte tidak cocok\n",
                  (unsigned long)errors);
  }
}

/* ==========================================================================
 *  Bagian 6 - dispatch perintah
 * ==========================================================================*/

static bool tokenOnOff(const char *tok, bool *value) {
  if (!tok) {
    return false;
  }
  if (!strcasecmp(tok, "on") || !strcmp(tok, "1") || !strcasecmp(tok, "yes")) {
    *value = true;
    return true;
  }
  if (!strcasecmp(tok, "off") || !strcmp(tok, "0") || !strcasecmp(tok, "no")) {
    *value = false;
    return true;
  }
  return false;
}

static void handleCommand(char *line) {
  char *save = nullptr;
  char *cmd = strtok_r(line, " \t", &save);
  if (!cmd) {
    return;
  }
  char *a1 = strtok_r(nullptr, " \t", &save);
  char *a2 = strtok_r(nullptr, " \t", &save);
  uint32_t v1 = 0, v2 = 0;

  if (!strcasecmp(cmd, "help") || !strcasecmp(cmd, "h") || !strcmp(cmd, "?")) {
    printHelp();
  } else if (!strcasecmp(cmd, "info") || !strcasecmp(cmd, "i") ||
             !strcasecmp(cmd, "id")) {
    cmdInfo();
  } else if (!strcasecmp(cmd, "status") || !strcasecmp(cmd, "st")) {
    cmdStatus();
  } else if (!strcasecmp(cmd, "pins")) {
    printConfigLine();
  } else if (!strcasecmp(cmd, "read")) {
    if (!parseU32(a1, &v1)) {
      Serial.print("!ERR butuh <addr> [len]\r\n");
      return;
    }
    if (a2 && !parseU32(a2, &v2)) {
      Serial.print("!ERR <len> tidak valid\r\n");
      return;
    }
    cmdRead(v1, v2);
  } else if (!strcasecmp(cmd, "binread") || !strcasecmp(cmd, "dump")) {
    if (!parseU32(a1, &v1) || !parseU32(a2, &v2)) {
      Serial.print("!ERR butuh <addr> <len>\r\n");
      return;
    }
    cmdBinRead(v1, v2);
  } else if (!strcasecmp(cmd, "write")) {
    v1 = 0;
    if (a1 && !parseU32(a1, &v1)) {
      Serial.print("!ERR <addr> tidak valid\r\n");
      return;
    }
    cmdWrite(v1);
  } else if (!strcasecmp(cmd, "verify")) {
    v1 = 0;
    if (a1 && !parseU32(a1, &v1)) {
      Serial.print("!ERR <addr> tidak valid\r\n");
      return;
    }
    cmdVerify(v1);
  } else if (!strcasecmp(cmd, "erase")) {
    if (!parseU32(a1, &v1)) {
      Serial.print("!ERR butuh <addr> [jumlah sector]\r\n");
      return;
    }
    if (a2 && !parseU32(a2, &v2)) {
      Serial.print("!ERR <jumlah> tidak valid\r\n");
      return;
    }
    cmdErase(v1, v2, false);
  } else if (!strcasecmp(cmd, "berase")) {
    if (!parseU32(a1, &v1)) {
      Serial.print("!ERR butuh <addr> [jumlah block]\r\n");
      return;
    }
    if (a2 && !parseU32(a2, &v2)) {
      Serial.print("!ERR <jumlah> tidak valid\r\n");
      return;
    }
    cmdErase(v1, v2, true);
  } else if (!strcasecmp(cmd, "chiperase") || !strcasecmp(cmd, "eraseall")) {
    cmdChipErase(a1 && !strcasecmp(a1, "confirm"));
  } else if (!strcasecmp(cmd, "unprotect")) {
    cmdUnprotect();
  } else if (!strcasecmp(cmd, "wsr")) {
    if (!parseU32(a1, &v1)) {
      Serial.print("!ERR butuh <sr1> [sr2]\r\n");
      return;
    }
    if (a2 && !parseU32(a2, &v2)) {
      Serial.print("!ERR <sr2> tidak valid\r\n");
      return;
    }
    cmdWriteStatus((uint8_t)v1, (uint8_t)v2);
  } else if (!strcasecmp(cmd, "freq")) {
    if (!parseU32(a1, &v1)) {
      Serial.print("!ERR butuh <khz>\r\n");
      return;
    }
    cmdFreq(v1);
  } else if (!strcasecmp(cmd, "fastread")) {
    bool on = false;
    if (!tokenOnOff(a1, &on)) {
      Serial.print("!ERR pakai: fastread on|off\r\n");
      return;
    }
    g_fastRead = on;
    Serial.printf("OK fast-read %s\r\n", on ? "aktif (0x0B)" : "nonaktif (0x03)");
  } else if (!strcasecmp(cmd, "addr4")) {
    bool on = false;
    if (!tokenOnOff(a1, &on)) {
      Serial.print("!ERR pakai: addr4 on|off\r\n");
      return;
    }
    cmdAddr4(on);
  } else if (!strcasecmp(cmd, "reset")) {
    softResetChip();
    detectChip();
    Serial.print("OK chip flash di-reset\r\n");
    printChipLine();
  } else {
    Serial.printf("!ERR perintah tidak dikenal: '%s' (coba 'help')\r\n", cmd);
  }
}

/* ==========================================================================
 *  Bagian 7 - setup / loop
 * ==========================================================================*/

void setup() {
  /* CS idle HIGH sebelum apapun */
  pinMode(FLASH_CS_PIN, OUTPUT);
  digitalWrite(FLASH_CS_PIN, HIGH);
#if FLASH_WP_PIN >= 0
  pinMode(FLASH_WP_PIN, OUTPUT);
  digitalWrite(FLASH_WP_PIN, HIGH); /* WP# tidak aktif */
#endif
#if FLASH_HOLD_PIN >= 0
  pinMode(FLASH_HOLD_PIN, OUTPUT);
  digitalWrite(FLASH_HOLD_PIN, HIGH); /* HOLD# tidak aktif */
#endif

  /* USB CDC native: buffer RX/TX diperbesar supaya transfer hex/biner lancar */
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);

  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 5000) {
    delay(10);
  }
  delay(100);

  SPI.begin(FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, -1);
  spiApplyClock();

  /* keluar dari deep power-down (kalau chip sebelumnya di-PD) */
  selectChip();
  xfer(CMD_RELEASE_PD);
  deselectChip();
  delay(1);

  Serial.print("\r\n\r\n");
  Serial.print("=====================================================\r\n");
  Serial.print("  BIOS / SPI-NOR FLASHER - ESP32-C3-DevKitM-1\r\n");
  Serial.print("  native USB (USB-Serial/JTAG), tanpa USB-TTL\r\n");
  Serial.print("=====================================================\r\n");
  cmdInfo();
  Serial.print("\r\nKetik 'help' untuk daftar perintah.\r\n> ");
}

void loop() {
  int n = readLine(g_line, sizeof(g_line), 1000);
  if (n <= 0) {
    return;
  }
  handleCommand(g_line);
  Serial.print("> ");
}

