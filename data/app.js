// ─────────────────────────────────────────────────────────────
// CDI//MGPRO — Shared client runtime
// Common WebSocket client + telemetry state + nav helpers
// loaded by every page. Page-specific code subscribes to the
// `cdi` bus via window.cdi.on('telemetry', cb) etc.
// ─────────────────────────────────────────────────────────────

const $ = (id) => document.getElementById(id);

const MODE = { 0:'BOOT', 2:'PENGAPIAN', 3:'MODE AMAN' };

// Highlight active nav tab from URL path
function highlightNav(){
  const path = location.pathname.replace(/\/$/, '') || '/';
  document.querySelectorAll('.bottom-nav a').forEach(a => {
    const target = a.getAttribute('href').replace(/\/$/, '') || '/';
    if (target === path) a.classList.add('active');
    else a.classList.remove('active');
  });
}
document.addEventListener('DOMContentLoaded', highlightNav);

// Tiny event bus
const bus = (() => {
  const listeners = {};
  return {
    on(name, fn) { (listeners[name] = listeners[name] || []).push(fn); },
    emit(name, payload) { (listeners[name] || []).forEach(fn => { try { fn(payload); } catch (e) { console.error(e); } }); }
  };
})();

// Latest telemetry snapshot (also stashed on window for inspection)
const state = {
  telemetry: { mode: 0, rpm: 0, rpm_raw: 0, advance_x10: 0, count: 0, uptime: 0, heap: 0 },
  hello: null,
  connected: false,
  map: null
};

// Topbar status update
function updateTopbar(t){
  const mp = $('topMode');
  const ml = $('topLink');
  if (mp){
    const name = MODE[t.mode] || '?';
    mp.querySelector('.txt').textContent = name;
    const dot = mp.querySelector('.dot');
    dot.className = 'dot ' + (t.mode === 2 ? 'ok' : t.mode === 3 ? 'crit' : t.mode === 1 ? 'warn' : '');
  }
  if (ml){
    const dot = ml.querySelector('.dot');
    dot.className = 'dot ' + (state.connected ? 'ok' : 'crit');
    ml.querySelector('.txt').textContent = state.connected ? 'OK' : 'NO';
  }
}
bus.on('telemetry', updateTopbar);
bus.on('connection', updateTopbar);

// ─────── WebSocket client ───────
let ws = null;
function connect(){
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => {
    state.connected = true;
    bus.emit('connection', state.telemetry);
    bus.emit('open');
  };
  ws.onclose = () => {
    state.connected = false;
    bus.emit('connection', state.telemetry);
    setTimeout(connect, 1000);
  };
  ws.onerror = () => { try { ws.close(); } catch (e) {} };
  ws.onmessage = ev => {
    if (typeof ev.data === 'string'){
      try {
        const m = JSON.parse(ev.data);
        if (m.type === 'hello'){ state.hello = m; bus.emit('hello', m); }
        else if (m.type === 'map'){ state.map = m.points; bus.emit('map', m.points); }
        else if (m.type === 'mapApplied'){ bus.emit('mapApplied', m); }
        else if (m.type === 'dwellCurve'){ bus.emit('dwellCurve', m); }
        else if (m.type === 'datalog'){ bus.emit('datalog', m); }
        else if (m.type === 'presetList'){ bus.emit('presetList', m); }
        else if (m.type === 'preset'){ bus.emit('preset', m); }
        else if (m.type === 'mode'){ bus.emit('modeChanged', m.mode); }
        else if (m.type === 'err'){ bus.emit('err', m.msg); }
      } catch (e) { console.warn('bad json', e); }
    } else {
      decodeBinary(ev.data);
    }
  };
}

function decodeBinary(buf){
  const v = new DataView(buf);
  const magic = v.getUint8(0);
  if (magic === 0xB0){
    const t = {
      mode: v.getUint8(1),
      rpm: v.getUint16(2, true),
      rpm_raw: v.getUint16(4, true),
      count: v.getUint32(6, true),
      uptime: v.getUint32(10, true),
      heap: v.getUint32(14, true),
      advance_x10: buf.byteLength >= 20 ? v.getInt16(18, true) : 0,
      armed:       buf.byteLength >= 21 ? v.getUint8(20)        : 0,
      fire_count:  buf.byteLength >= 25 ? v.getUint32(21, true) : 0,
      jitter_us:   buf.byteLength >= 27 ? v.getInt16(25, true)  : 0,
      safety_flags:buf.byteLength >= 28 ? v.getUint8(27)        : 0,
      rev_main:    buf.byteLength >= 30 ? v.getUint16(28, true) : 10500,
      rev_overrev: buf.byteLength >= 32 ? v.getUint16(30, true) : 11500,
      dwell_us:    buf.byteLength >= 34 ? v.getUint16(32, true) : 2500,
      adv_offset_x10: buf.byteLength >= 36 ? v.getInt16(34, true) : 0,
      cut_mode:    buf.byteLength >= 37 ? v.getUint8(36) : 1,
      retard_deg:  buf.byteLength >= 38 ? v.getUint8(37) / 2.0 : 10.0,
      pattern_fire_n: buf.byteLength >= 39 ? v.getUint8(38) : 3,
      pattern_skip_n: buf.byteLength >= 40 ? v.getUint8(39) : 1,
      shift_state: buf.byteLength >= 41 ? v.getUint8(40) : 0,
      flags2:      buf.byteLength >= 42 ? v.getUint8(41) : 0,
      shift_warn:  buf.byteLength >= 44 ? v.getUint16(42, true) : 7500,
      shift_shift: buf.byteLength >= 46 ? v.getUint16(44, true) : 8500,
      launch_rpm:  buf.byteLength >= 48 ? v.getUint16(46, true) : 5000,
      qs_cut_ms:   buf.byteLength >= 50 ? v.getUint16(48, true) : 65,
      qs_count:    buf.byteLength >= 54 ? v.getUint32(50, true) : 0,
      bf_trigger:  buf.byteLength >= 55 ? v.getUint8(54) : 0,
      flags3:      buf.byteLength >= 56 ? v.getUint8(55) : 0,
      bf_rpm_lo:   buf.byteLength >= 58 ? v.getUint16(56, true) : 3000,
      bf_rpm_hi:   buf.byteLength >= 60 ? v.getUint16(58, true) : 7000,
      bf_retard:   buf.byteLength >= 61 ? v.getUint8(60) / 2.0 : 15.0,
      bf_duration: buf.byteLength >= 63 ? v.getUint16(61, true) : 200,
      vbat_mv:     buf.byteLength >= 65 ? v.getUint16(63, true) : 0,
      alvp_state:  buf.byteLength >= 66 ? v.getUint8(65) : 0,
      alvp_derate: buf.byteLength >= 67 ? v.getUint8(66) / 10.0 : 10.5,
      alvp_disarm: buf.byteLength >= 68 ? v.getUint8(67) / 10.0 :  9.0,
      flags4:      buf.byteLength >= 69 ? v.getUint8(68) : 0,
      alvp_derate_rpm: buf.byteLength >= 71 ? v.getUint16(69, true) : 4000
    };
    state.telemetry = t;
    bus.emit('telemetry', t);
  } else if (magic === 0xA7){
    // Edge-event stream — see edge_capture.cpp for layout.
    const seq      = v.getUint32(1, true);
    const firstTs  = v.getUint32(5, true);
    const n        = v.getUint16(9, true);
    const events   = new Array(n);
    let o = 11;
    for (let i = 0; i < n; i++){
      events[i] = {
        ts: firstTs + v.getUint16(o, true),
        ch: v.getUint8(o + 2),
        level: v.getUint8(o + 3)
      };
      o += 4;
    }
    bus.emit('scopeEdge', { seq, firstTs, events });
  }
}

function send(obj){ if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

// expose
window.cdi = { bus, state, send, MODE };

connect();
