# LittleFS source directory

Isi folder ini di-flash ke partisi LittleFS (`spiffs` @ 0x290000) saat
`pio run -t uploadfs` — atau otomatis setelah `pio run -t upload`
(lihat `scripts/auto_uploadfs.py`).

**Catatan penting:**
- UI (HTML/JS/CSS) TIDAK di sini — itu di `web/`, di-embed ke PROGMEM
  via `scripts/build_ui.py`. Jangan taruh file UI di sini (menghindari
  dua source-of-truth seperti dulu).
- Partisi LittleFS dipakai runtime oleh `edge_snapshot` untuk menyimpan
  scope snapshot. File-file itu dibuat firmware saat jalan.
- File ini cuma placeholder supaya folder `data/` tidak kosong (mklittlefs
  butuh source dir yang ada). Boleh diisi config/aset statis kalau perlu.
- Firmware juga `LittleFS.begin(true)` → auto-format kalau partisi kosong,
  jadi flashing fs ini sebetulnya opsional (belt-and-suspenders).
