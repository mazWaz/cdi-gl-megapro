# Research: Dukungan Motor 2-Tak (Future Project)

> Status: **RESEARCH ONLY — belum diimplementasikan.**
> Dokumen ini menangkap analisa kelayakan + rencana untuk menambah
> dukungan motor 2-tak ke firmware CDI (saat ini 4-tak only:
> Honda Megapro/GL Pro/Tiger dan kerabatnya).
>
> Dibuat: 2026-05-25. Basis firmware: v0.4.0 (commit f96f84c).

---

## TL;DR

Firmware **firing model sudah kompatibel** dengan 2-tak (fire tiap putaran
crank). Yang membedakan: **kurva advance jauh lebih kecil + retard di top
RPM**, **detonasi sangat sensitif** (butuh safety clamp ketat), dan
**isu ignition type** (mayoritas 2-tak pakai CDI capacitif, hardware
saat ini fixed TCI inductif). Effort minimal (~2-3 hari) kalau pakai
pendekatan coil-swap; tinggi (+1 minggu) kalau revive CDI output stage.

---

## 1. Perbedaan fundamental 2T vs 4T (ignition)

| Aspek | 4-Tak (current) | 2-Tak | Impact |
|---|---|---|---|
| Power stroke | 1 per 2 putaran (720°) | 1 per **setiap** putaran (360°) | 🟢 Firing model sama |
| Firing | Wasted-spark (fire tiap rev) | Fire tiap rev (semua berguna) | 🟢 Tidak perlu ubah scheduler |
| Peak advance | 30-35° BTDC | **12-20° BTDC** | 🟡 Data + safety clamp |
| Curve shape | Naik → plateau → sedikit retard | Naik mid → **retard agresif** di top | 🟡 Data saja |
| RPM ceiling | 10-13k | 11-13k (race 2T 14k+) | 🟡 Naikkan RPM_MAX_VALID |
| Ignition type | TCI inductif | **Mayoritas CDI capacitif** | 🔴 Lihat §5 |
| Detonasi | Toleran | **Sangat sensitif → jebol piston** | 🔴 Lihat §4 |

## 2. Yang SUDAH compatible

**Firing model langsung kompatibel.** `onPulseCh1FromIsr` fire sekali per
pulse CH1 = sekali per putaran crank. Di 4-tak jadi wasted-spark; di 2-tak
setiap putaran = power stroke, jadi logika identik. **Tidak perlu ubah
scheduler core.**

Reusable tanpa perubahan:
- Pulser input ISR (single pickup)
- Spark scheduler + HW timer
- WiFi UI, profile system, datalog, telemetry
- Safety stack (no-signal, panic, watchdog)
- Pickup geometry editor (configurable)
- RPM calc + smoothing

## 3. Perubahan kecil (data + struct)

### 3a. Preset baru dengan kurva 2-tak
Contoh karakteristik Yamaha RX-King:
```
Idle (~1500)   : 15-18° BTDC
Mid (4000-6000): 18-20° BTDC   ← PEAK
Top (8000+)    : 12-14° BTDC   ← RETARD anti-detonasi
```
Berbeda dari 4-tak (plateau di top). Map struct support shape apapun;
cuma butuh data akurat per motor.

### 3b. Field baru di `Preset` struct (engine_preset.h)
```cpp
struct Preset {
    ...
    uint8_t stroke_type;       // 2 atau 4 — safety clamp + dokumentasi
    float   advance_hard_cap;  // per-preset max advance (2T lebih rendah)
    ...
};
```

### 3c. RPM ceiling
`RPM_MAX_VALID = 13000` cukup untuk RX-King/Ninja. Race 2T mungkin
14-15k → jadikan per-preset atau configurable.

## 4. SAFETY KRITIS — Detonasi 2-tak

Perbedaan paling berbahaya:
- Piston crown sangat panas (tidak ada exhaust valve, oli campur bensin)
- Over-advance → detonasi → **piston holing/seizure dalam detik**
  (catastrophic, bukan gradual seperti 4-tak)
- Margin error jauh lebih sempit

`validateForSafety` saat ini allow sampai `ADVANCE_MAX_DEG = 45°`.
Untuk 2-tak ini berbahaya. Butuh stroke-aware clamp:
```cpp
if (preset.stroke_type == 2) {
    if (adv > 25.0f) adv = 25.0f;   // typical max safe 22-25° BTDC
}
```
Plus UI warning keras saat user set advance >20° di preset 2-tak.

## 5. KRITIS — Ignition type (TCI vs CDI)

Current build: **fixed TCI (inductif) dengan N-MOSFET**. Mode capacitive
sudah dihapus (`inductive()` return `true` const; `setInductive` no-op stub).

Mayoritas 2-tak pakai **CDI (capacitive discharge)**:
- CDI: rise-time cepat, bagus untuk busi ke-foul oli (umum di 2-tak)
- TCI: spark lebih lembut, butuh saturasi primary ~3-4ms

**Bisa TCI di 2-tak?** Bisa, tapi:
1. **Dwell budget high RPM**: @ 11000 rpm period = 5454µs, dwell cap 40%
   = 2180µs. TCI butuh ~3-4ms saturasi penuh → marginal, spark energy
   turun di top RPM.
2. **Coil mismatch**: koil 2-tak (CDI-spec) induktansi primary beda →
   TCI driver mungkin tidak saturasi benar → spark lemah.

| Opsi | Hardware | Firmware | Effort |
|---|---|---|---|
| **A. Coil swap** | Ganti ke koil TCI 4-tak | Tidak ada | Low (user beli koil) |
| **B. CDI output stage** | Tambah step-up + SCR/cap discharge | Revive `inductive` per-preset, fire di rising edge (no dwell subtract) | High |

Opsi B sudah pernah ada di plan awal (HW step-up + SCR), lalu di-refactor
ke TCI. Sisa kode capacitive path masih ada (`setInductive` stub).

## 6. Feature compatibility matrix

| Fitur | 2-Tak | Catatan |
|---|---|---|
| Programmable curve | ✅ | Data beda, struct sama |
| Multi-profile | ✅ | Reusable |
| Rev limiter | ✅ | Sesuaikan RPM |
| Launch control | ✅ | Drag 2T populer |
| Quickshifter | ⚠️ | Transmisi 2T beda feel, perlu test |
| ALVP | ✅ | Reusable |
| Shift light | ✅ | Reusable |
| Datalog | ✅ | Reusable |
| **Exhaust flame** | ❌ | Skip-fire loads up 2T → detonasi risk, disable |
| **Idle rumble** | ❌ | 2T idle sudah lopey, skip → stall/load-up |
| Dwell compensation | ⚠️ | Relevan hanya kalau tetap TCI |
| **Power valve control** | 🆕 | YPVS/KIPS/RC — fitur baru, butuh servo GPIO |

## 7. Target motor 2-tak Indonesia

| Motor | CC | Redline | Catatan |
|---|---|---|---|
| **Yamaha RX-King** | 135 | ~9500 | Prioritas #1, komunitas besar |
| Yamaha F1ZR / Force1 | 110 | ~10000 | Bebek 2T populer |
| Kawasaki Ninja 150 RR/R | 150 | ~11000 | KIPS power valve |
| Honda NSR 150 SP | 150 | ~11000 | RC valve, sport 2T |
| Suzuki Satria 2T / RGR | 120/150 | ~10500 | Bebek/sport |
| Vespa 2T | 150 | ~6000 | Klasik, beda total |

## 8. Estimasi effort

**Opsi A (coil-swap, TCI dipertahankan):**
- Preset data 2-tak (riset kurva per motor): Medium (riset)
- Field `stroke_type` + safety clamp: Low-Medium
- Disable flame/rumble untuk 2T: Low
- RPM ceiling per-preset: Low
- UI stroke indicator + warning: Low
- **Total: ~2-3 hari**

**Opsi B (revive CDI output):** + High (hardware design + capacitive
firmware path) → +1 minggu.

## 9. Keputusan yang diperlukan sebelum eksekusi

1. **Ignition hardware**: Opsi A (coil swap) atau B (CDI output stage)?
   → menentukan scope.
2. **Target motor pertama**: RX-King saja, atau beberapa sekaligus?
3. **Power valve control** (YPVS/KIPS/RC) — masuk scope atau nanti?
4. **Flame + idle rumble** — confirm disable untuk 2-tak.
5. **Verifikasi pulser 2T**: konfirmasi 1 pulse/rev dengan scope mode di
   motor real (kebanyakan iya, tapi wajib cek).

## Rekomendasi

Mulai dengan **Opsi A (coil swap) + RX-King saja** sebagai proof-of-concept
paling murah. Lalu ekspansi ke motor lain + pertimbangkan Opsi B kalau
ternyata TCI tidak memuaskan di high-RPM 2-tak.

---

## Referensi yang perlu dikumpulkan saat eksekusi

- Data timing factory RX-King / Ninja 150 / NSR 150 (manual servis)
- Spesifikasi pulser/magneto tiap motor (posisi pickup, lebar magnet)
- Studi expansion chamber resonance vs ignition timing 2-tak
- Spesifikasi koil 2-tak (induktansi primary) untuk validasi TCI compat
