# Dual-edge ignition (CH2) — analisis, Tier-1 (crank-assist) & Tier-2 (EMI gate)

Status: **Tier-1 ter-implementasi (opt-in, default OFF) · Tier-2 desain + tervalidasi simulator · BELUM divalidasi hardware.**

Dokumen ini merangkum kenapa start pertama susah pada firmware ini, dan
bagaimana memanfaatkan **edge kedua pickup (CH2)** untuk memperbaikinya —
berikut bukti dari log cranking nyata.

---

## 1. Masalah: start pertama susah (≠ CDI stok)

CDI stok memicu spark **tiap pulsa pickup** pada advance mekanis tetap →
gampang nyala. Firmware ini, demi mencegah kickback, memasang 3 gate di
`spark_scheduler::onPulseCh1FromIsr` yang men-**skip** spark justru saat
periode pulser tidak stabil (cranking):

1. **First-pulse skip** + **delay-primed gate** — ~3 pulsa pertama tiap
   arm/stall mandul.
2. **Period-drift gate ±2×** (`spark_scheduler.cpp`) — menolak fire bila
   periode antar-pulsa berubah >2×.

Akar fisik: delay dihitung dari **periode sebelumnya**, sementara pada
single-silinder periode CH1 berurutan **anti-korelasi** (putaran kompresi
lambat, berikutnya cepat). Delay basi dari fase terbalik → spark mendarat
salah sudut atau di-skip.

### Bukti terukur (`data_cek.csv`, cranking Megapro nyata)
- Periode CH1 ayun ekstrem: `73ms → 1800ms → 15ms → 1288ms` (rasio 24–120×).
- **38% siklus** swing >2× → memicu drift gate.
- Simulasi gating eksak (replikasi firmware):
  - **Spark BERGUNA hanya 56%** (sisanya skip atau mendarat ATDC).
  - Mesin hangat: ~separuh fire mendarat ATDC (percuma).
  - Mesin dingin (kompresi >2×): drift gate bisa tolak ~100% → **0% spark**.

(Log KLX idle ~2450 rpm → gating sekarang 99% fire — masalah **spesifik
cranking/RPM-tidak-stabil**, bukan running.)

---

## 2. Hardware: pickup 2-edge

VRS coil + 2 optocoupler anti-parallel → tiap lewatan magnet menghasilkan
2 edge di sudut crank **tetap secara mekanis**:

| Edge | Sumber | Sudut (Megapro) | API |
|------|--------|-----------------|-----|
| CH1 | leading | 32° BTDC | `pickup::maxAdvanceRef()` |
| CH2 | trailing | 14° BTDC | `pickup::baseAdvanceRef()` = maxRef − magnetWidth |
| gap | magnet width | 18° | `pickup::magnetWidth()` |

**Geometri per-motor — WAJIB auto-cal, haram hardcode.** Log KLX:
gap CH1→CH2 = **43.5°** (vs Megapro 18°), ratio width/period CV 3% (idle).
Kalau konstanta Megapro dipakai di KLX, referensi CH2 meleset ~25°.

Primitif dari CH2: `pulse_width = t_CH2 − t_CH1` → `ω_lokal = magnetWidth /
pulse_width` = kecepatan crank **tepat di zona fire (14–32° BTDC)**,
bukan rata-rata putaran yang terdistorsi kompresi.

---

## 3. Tier-1: crank-assist (CH2 cranking fire) — TER-IMPLEMENTASI

**Konsep:** di bawah `config::CRANK_MODE_RPM` (1500), charge dari CH1, fire
**AT CH2** → spark mendarat di base advance mekanis (period-independent),
seperti CDI stok. Di atas itu, jalur CH1+delay running **tidak disentuh**.

**Skema final tervalidasi simulator (thd `data_cek.csv`):**

| Skema | Spark berguna | Kickback |
|-------|---------------|----------|
| CH1-only (lama) | 56% | 1 |
| **CH2 fire + skip-jika-CH2-hilang** ✅ | **75%** | 1 |
| + fallback prediktif (dari width terakhir) ❌ | 78% | **28** |

> **Pelajaran kunci:** fallback prediktif (ada di desain awal) memakai width
> basi → memunculkan **28 kickback**. Disimulasikan SEBELUM koding →
> ditolak. Skema final: fire AT CH2 saat ada, **skip aman** kalau CH2
> hilang (~17% saat cranking), **tanpa prediksi**.

**Mekanisme (commit `8434f8d`):**
- `pulser_input::isrCh2` (CH2 falling) → `spark::onPulseCh2FromIsr` (no-op
  kalau crank-assist off / tidak armed).
- `onPulseCh1FromIsr`: SETELAH semua gate keamanan (armed/QS/shouldFire/
  ceiling/stall/first-pulse), SEBELUM drift gate → charge dari CH1, set
  `s_ch2FirePending`. Anti-strand: cap fire-off `CRANK_MAX_DWELL_US`
  (10ms) → kalau CH2 hilang, fire near base advance, ≤~20° BTDC (no
  kickback), coil tak ter-strand.
- `live_stats`: `armCrankCycle(r_inst < CRANK_MODE_RPM)` per tick.
- Default OFF. Enable: WS `{cmd:setCrankAssist}` / toggle settings.html.
  Persist NVS. Telemetry `flags4 & 0x04`.

**Batas:** 75% (bukan ~100%) karena CH2-pairing saat cranking ~83% (kualitas
sinyal/hardware), bukan algoritma.

---

## 4. Tier-2: EMI validity gate — DESAIN (tervalidasi simulator)

**Masalah:** noise-gate sekarang cuma cek periode CH1→CH1 <4.6ms. Edge EMI
tunggal menggeser anchor → pulsa real berikutnya ikut korup. `safety.cpp`
mencatat **phantom 11000 rpm di KLX saat coil firing** (EMI HV).

**Prinsip:** lewatan magnet ASLI = 1 CH1 → 1 CH2, berurutan, lebar masuk
akal. EMI melanggar salah satunya:
1. **Urutan** CH1 sebelum CH2.
2. **Jumlah** tepat 1+1 per pass (CH2/CH1 ganda = EMI burst).
3. **Lebar** `width` dalam band relatif periode:
   `w_exp = (magnetWidth/360)×period`, valid bila
   `max(150µs, 0.3·w_exp) ≤ width ≤ 4·w_exp`.
   (Band longgar karena CV width/period saat cranking ~84%; ketat saat
   RPM stabil.)

**Hasil simulator (`crank_validator.py`):**

| Log | VALID | EMI ditolak | phantom>11k: tanpa→dengan gate | false-reject (data bersih) |
|-----|-------|-------------|-------------------------------|----------------------------|
| `data_cek` (crank) | 73% | 33 (7 short-CH1, 11 extra-CH2, 15 bad-width) | **7 → 0** | — |
| `KLX` (idle) | 99% | 5 short-CH1 | **5 → 0** | **0** width/pairing |
| `KCL200ms` (idle) | 99% | 6 short-CH1 | **6 → 0** | **0** width/pairing |

→ **menghapus 100% phantom-RPM, 0 false-reject width/pairing di data bersih.**
(`bad-width=15` di cranking sebagian mungkin pass asli → **fail-open**.)

**Dua titik penerapan:**
- **2A — pre-fire width check (ISR, kecil).** Sebelum crank-assist fire di
  CH2, cek `width` dalam band (band dihitung loop, ISR cuma integer
  compare). Implausible → jangan fire, biar CH2 real / cap-timer yang fire.
  Melindungi Tier-1 dari CH2 palsu.
- **2B — loop validator + gate rpm + counter (sedang).** Hanya pass valid
  meng-update `rpm_calc` → RPM/period bersih. Counter `emi_rejects` ke
  telemetry → indikator kesehatan wiring (shielding pickup).

**Risiko:** false-reject saat cranking (band longgar + fail-open);
"missing CH2" (sinyal lemah) ≠ EMI, jangan dihitung reject; EMI real-firing
> capture sensor → uji dengan log saat mesin nyala.

---

## 5. Status validasi
- **Tervalidasi:** logika gating (replikasi firmware) + algoritma di data
  cranking/idle nyata via simulator (`.temp/crank_*.py`, gitignored).
- **BELUM hardware:** scope GPIO25 (fire@CH2, dwell tak melar), strobe
  (timing mark vs map), cold-crank nyata, EMI saat HV firing.
- Tier-1 **default OFF** → zero-regression sampai tervalidasi.

## 6. Validasi yang masih dibutuhkan
1. Capture **cold-crank murni** (mesin dingin, jangan nyala) → baseline.
2. Capture **saat mesin firing** (HV EMI nyata) → uji Tier-2 phantom-reduction.
3. Bench scope + strobe sebelum crank-assist dipakai jalan.
