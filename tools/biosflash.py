#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Host tool for the ESP32-C3 BIOS / SPI-NOR flasher (src/main.cpp).

Firmware runs on an ESP32-C3-DevKitM-1 using its NATIVE USB port
(USB-Serial/JTAG), so no USB-to-TTL adapter is needed.

Install dependency:
    pip install pyserial

Examples:
    python tools/biosflash.py id
    python tools/biosflash.py read 0 0x400000 -o bios_backup.bin
    python tools/biosflash.py write bios.bin            # erase + write + verify
    python tools/biosflash.py verify bios.bin
    python tools/biosflash.py erase 0 0x400000
    python tools/biosflash.py chiperase --yes
    python tools/biosflash.py console

Notes:
    * Address/length accept decimal or 0x... hexadecimal.
    * If --port is omitted the script tries to auto-detect the ESP32-C3
      USB-JTAG-Serial port (USB VID 0x303A).
"""

import argparse
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("pyserial belum terpasang. Jalankan: pip install pyserial")

CHUNK = 256  # byte per baris hex (firmware membatasi 256 byte/baris)
ESP_VID = 0x303A


class DeviceError(RuntimeError):
    pass


def parse_int(text):
    return int(str(text), 0)


def find_port():
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == ESP_VID:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    names = ", ".join(p.device for p in ports) or "(tidak ada)"
    sys.exit("Port tidak ditemukan otomatis. Pakai --port. Tersedia: %s" % names)


class Flasher(object):
    def __init__(self, port, baud=115200, verbose=False):
        self.verbose = verbose
        self.ser = serial.Serial(port, baud, timeout=0.05)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.sync(0.5)

    def close(self):
        self.ser.close()

    def sync(self, timeout=5.0):
        """Drain whatever the device sent until its prompt (best effort)."""
        try:
            self.collect_until_prompt(timeout)
        except DeviceError:
            pass

    # ---------------- low level ----------------
    def _write(self, data):
        if isinstance(data, str):
            data = data.encode("ascii")
        self.ser.write(data)
        self.ser.flush()

    def read_line(self, timeout=10.0):
        """Read one CR/LF terminated line, None on timeout."""
        end = time.time() + timeout
        buf = bytearray()
        while time.time() < end:
            b = self.ser.read(1)
            if not b:
                continue
            if b == b"\n":
                return buf.decode("ascii", "replace").rstrip("\r")
            if b != b"\r":
                buf += b
        return None

    def read_exact(self, count, timeout=30.0):
        end = time.time() + timeout
        buf = bytearray()
        while len(buf) < count and time.time() < end:
            chunk = self.ser.read(min(4096, count - len(buf)))
            if chunk:
                buf += chunk
        if len(buf) != count:
            raise DeviceError("data biner tidak lengkap (%d/%d byte)" % (len(buf), count))
        return bytes(buf)

    def collect_until_prompt(self, timeout=20.0):
        """Read output until the '> ' prompt is seen."""
        lines = []
        cur = bytearray()
        end = time.time() + timeout
        while time.time() < end:
            b = self.ser.read(1)
            if not b:
                continue
            if b == b"\r":
                continue
            if b == b"\n":
                lines.append(cur.decode("ascii", "replace"))
                cur = bytearray()
                continue
            cur += b
            if cur.endswith(b"> "):
                text = cur[:-2].decode("ascii", "replace").strip()
                if text:
                    lines.append(text)
                return lines
        raise DeviceError("timeout menunggu prompt (device tidak merespon?)")

    # ---------------- commands ----------------
    def command(self, cmd, timeout=20.0, echo=True):
        self._write(cmd + "\n")
        lines = self.collect_until_prompt(timeout)
        for ln in lines:
            if ln.startswith("!ERR"):
                raise DeviceError(ln)
        if echo:
            for ln in lines:
                print(ln)
        return lines

    def _read_ack(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            b = self.ser.read(1)
            if not b:
                continue
            if b == b"+":
                return "+"
            if b == b"!":
                rest = self.read_line(5.0) or ""
                return "!" + rest
        return ""

    def _stream(self, cmd, data, label, progress=True):
        """Send hex data lines after `cmd`; returns the final status line."""
        self._write(cmd + "\n")
        first = self.read_line(10.0)
        if first is None:
            raise DeviceError("tidak ada balasan untuk '%s'" % cmd)
        if first.startswith("!ERR"):
            raise DeviceError(first)
        if self.verbose:
            print(first)

        total = len(data)
        done = 0
        for off in range(0, total, CHUNK):
            piece = data[off:off + CHUNK]
            self._write(piece.hex().encode("ascii") + b"\n")
            ack = self._read_ack()
            if ack != "+":
                raise DeviceError(ack or "tidak ada ACK dari device")
            done += len(piece)
            if progress:
                pct = 100.0 * done / total if total else 100.0
                sys.stderr.write("\r%s %6.2f%% (%d/%d byte)" % (label, pct, done, total))
                sys.stderr.flush()
        self._write(".\n")

        while True:
            ln = self.read_line(180.0)
            if ln is None:
                raise DeviceError("timeout menunggu hasil %s" % label)
            if ln.startswith("!"):
                raise DeviceError(ln)
            if ln.startswith("DONE") or ln.startswith("VERIFY OK"):
                if progress:
                    sys.stderr.write("\r%s 100.00%% (%d/%d byte)\n" % (label, total, total))
                    sys.stderr.flush()
                self.sync(5.0)
                return ln

    def write_flash(self, addr, data, progress=True):
        return self._stream("write %d" % addr, data, "WRITE ", progress)

    def verify_flash(self, addr, data, progress=True):
        return self._stream("verify %d" % addr, data, "VERIFY", progress)

    def read_flash(self, addr, length):
        self._write("binread %d %d\n" % (addr, length))
        while True:
            ln = self.read_line(15.0)
            if ln is None:
                raise DeviceError("timeout menunggu header @BIN")
            if ln.startswith("!ERR"):
                raise DeviceError(ln)
            if "@BIN" in ln:
                break
        data = self.read_exact(length)
        while True:
            ln = self.read_line(15.0)
            if ln is None:
                break
            if "@END" in ln:
                break
        self.sync(5.0)
        return data

# --------------------------------------------------------------------------
#  CLI
# --------------------------------------------------------------------------
def do_read(fl, args):
    addr = parse_int(args.addr)
    length = parse_int(args.length)
    total = 0
    t0 = time.time()
    with open(args.output, "wb") as f:
        while total < length:
            n = min(0x10000, length - total)
            f.write(fl.read_flash(addr + total, n))
            total += n
            sys.stderr.write("\rREAD  %6.2f%% (%d/%d byte)" %
                             (100.0 * total / length, total, length))
            sys.stderr.flush()
    sys.stderr.write("\n")
    print("OK %d byte -> %s (%.1f detik)" % (length, args.output, time.time() - t0))


def load_image(path):
    with open(path, "rb") as f:
        data = f.read()
    if not data:
        sys.exit("file kosong: %s" % path)
    if data[:1] == b":":
        sys.exit("File terlihat seperti Intel HEX, bukan biner. Konversi dulu, "
                 "mis: objcopy -I ihex -O binary in.hex out.bin")
    return data


def do_write(fl, args):
    addr = parse_int(args.addr)
    data = load_image(args.file)
    print("File      : %s (%d byte)" % (args.file, len(data)))
    print("Alamat    : 0x%08X" % addr)

    if not args.no_erase:
        start = addr - (addr % 4096)
        count = ((addr + len(data) - start) + 4095) // 4096
        fl.command("erase %d %d" % (start, count), timeout=max(120.0, count * 0.5))

    t0 = time.time()
    print(fl.write_flash(addr, data))
    print("Waktu tulis: %.1f detik" % (time.time() - t0))

    if not args.no_verify:
        print(fl.verify_flash(addr, data))


def do_verify(fl, args):
    addr = parse_int(args.addr)
    data = load_image(args.file)
    print(fl.verify_flash(addr, data))


def do_erase(fl, args, block=False):
    addr = parse_int(args.addr)
    length = parse_int(args.length)
    step = 65536 if block else 4096
    start = addr - (addr % step)
    count = (length + (addr - start) + step - 1) // step
    fl.command("%s %d %d" % ("berase" if block else "erase", start, count),
               timeout=max(120.0, count * 0.5))


def do_chiperase(fl, args):
    if not args.yes:
        sys.exit("chiperase menghapus SELURUH chip. Tambahkan --yes kalau yakin.")
    fl.command("chiperase confirm", timeout=600.0)


def do_console(fl, args):
    print("Ketik perintah firmware ('help' = menu, 'exit' = keluar).")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line.lower() in ("exit", "quit"):
            return
        if not line:
            continue
        try:
            fl.command(line, timeout=900.0, echo=True)
        except DeviceError as exc:
            print(str(exc))




def build_parser():
    ap = argparse.ArgumentParser(
        description="ESP32-C3-DevKitM-1 BIOS/SPI-NOR flasher (native USB)")
    ap.add_argument("-p", "--port", help="serial port (mis. COM5). Kosong = auto-detect")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="baud (USB CDC: tidak berpengaruh)")
    ap.add_argument("-v", "--verbose", action="store_true", help="tampilkan detail")
    sub = ap.add_subparsers(dest="action")

    sub.add_parser("id", help="tampilkan JEDEC ID / kapasitas / status chip")
    sub.add_parser("status", help="baca status register 1/2/3")
    sub.add_parser("unprotect", help="hapus block-protect")
    sub.add_parser("pins", help="tampilkan konfigurasi pin")

    p = sub.add_parser("read", help="dump isi flash ke file biner")
    p.add_argument("addr", help="alamat awal (0x... atau desimal)")
    p.add_argument("length", help="jumlah byte")
    p.add_argument("-o", "--output", required=True, help="file output")

    p = sub.add_parser("write", help="hapus sector + tulis + verifikasi")
    p.add_argument("file", help="file biner yang ditulis")
    p.add_argument("--addr", default="0", help="alamat awal (default 0)")
    p.add_argument("--no-erase", action="store_true", help="lewati erase sector")
    p.add_argument("--no-verify", action="store_true", help="lewati verifikasi")

    p = sub.add_parser("verify", help="bandingkan isi flash dengan file")
    p.add_argument("file")
    p.add_argument("--addr", default="0")

    p = sub.add_parser("erase", help="hapus sector 4 KB yang tercakup range")
    p.add_argument("addr")
    p.add_argument("length")

    p = sub.add_parser("berase", help="hapus block 64 KB yang tercakup range")
    p.add_argument("addr")
    p.add_argument("length")

    p = sub.add_parser("chiperase", help="hapus seluruh chip (butuh --yes)")
    p.add_argument("--yes", action="store_true")

    p = sub.add_parser("freq", help="ubah clock SPI (kHz)")
    p.add_argument("khz")

    sub.add_parser("console", help="mode interaktif (ketik perintah firmware)")
    return ap


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.action:
        parser.print_help()
        return 1

    port = args.port or find_port()
    try:
        fl = Flasher(port, args.baud, args.verbose)
    except serial.SerialException as exc:
        sys.exit("Gagal membuka %s: %s" % (port, exc))

    if args.verbose:
        print("# port %s" % port)

    try:
        if args.action == "id":
            fl.command("info")
        elif args.action == "status":
            fl.command("status")
        elif args.action == "unprotect":
            fl.command("unprotect")
        elif args.action == "pins":
            fl.command("pins")
        elif args.action == "read":
            do_read(fl, args)
        elif args.action == "write":
            do_write(fl, args)
        elif args.action == "verify":
            do_verify(fl, args)
        elif args.action == "erase":
            do_erase(fl, args, block=False)
        elif args.action == "berase":
            do_erase(fl, args, block=True)
        elif args.action == "chiperase":
            do_chiperase(fl, args)
        elif args.action == "freq":
            fl.command("freq %d" % parse_int(args.khz))
        elif args.action == "console":
            do_console(fl, args)
        else:  # pragma: no cover
            sys.exit("aksi tidak dikenal")
    except DeviceError as exc:
        sys.stderr.write("\n")
        sys.exit("ERROR: %s" % exc)
    except KeyboardInterrupt:
        sys.stderr.write("\nDibatalkan.\n")
        return 130
    finally:
        fl.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

