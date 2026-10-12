// values の専用表示。RENDERERS に {id, detect, render} を足せばパターンを追加できる。
//   detect(values) -> data | null   … 純関数(DOM に触れない)。当てはまらなければ null
//   render(data)   -> {el, update(data)}  … 描画。update は差分だけ書き換える
// どれにも当てはまらなければ表示しない(フォールバックの key/value 表は常に下に出す)。
// ROS 由来の文字列は textContent でのみ入れる(innerHTML 禁止)。

// ---- 値の解釈(純関数) ----

const NUM_RE = /^\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*(.*?)\s*$/;

/** '47.8%' -> {value: 47.8, unit: '%'}。数値でなければ null(例外は出さない) */
export function parseNumber(s) {
  if (typeof s !== 'string') return null;
  const m = NUM_RE.exec(s);
  if (!m) return null;
  const value = Number(m[1]);
  return Number.isFinite(value) ? { value, unit: m[2] } : null;
}

/** values を key -> value の Map に(同じキーは先勝ち) */
export function valuesMap(values) {
  const m = new Map();
  for (const kv of values || []) if (!m.has(kv.key)) m.set(kv.key, kv.value);
  return m;
}

function num(map, key, unit) {
  const p = parseNumber(map.get(key));
  if (!p) return null;
  if (unit !== undefined && p.unit !== unit) return null;
  return p.value;
}

export function detectUsage(values) {
  const m = valuesMap(values);
  const usage = num(m, 'Usage', '%');
  const warn = num(m, 'Warn threshold', '%');
  const error = num(m, 'Error threshold', '%');
  if (usage === null || warn === null || error === null) return null;
  return { usage, warn, error, raw: m.get('Usage') };
}

export function detectFrequency(values) {
  const m = valuesMap(values);
  const actual = num(m, 'Actual frequency (Hz)');
  const min = num(m, 'Minimum acceptable frequency (Hz)');
  const max = num(m, 'Maximum acceptable frequency (Hz)');
  if (actual === null || min === null || max === null) return null;
  return { actual, min, max, target: num(m, 'Target frequency (Hz)') };
}

const LAT_KEYS = ['Latitude [deg]', 'Latitude'];
const LON_KEYS = ['Longitude [deg]', 'Longitude'];

export function detectLocation(values) {
  const m = valuesMap(values);
  const pick = (keys) => {
    for (const k of keys) {
      if (!m.has(k)) continue;
      const p = parseNumber(m.get(k));
      if (p && p.unit === '') return { v: p.value, s: m.get(k).trim() };
    }
    return null;
  };
  const lat = pick(LAT_KEYS);
  const lon = pick(LON_KEYS);
  if (!lat || !lon) return null;
  if (Math.abs(lat.v) > 90 || Math.abs(lon.v) > 180) return null;
  return { lat: lat.v, lon: lon.v, text: `${lat.s}, ${lon.s}` };
}

export function osmUrl(lat, lon) {
  return `https://www.openstreetmap.org/?mlat=${lat}&mlon=${lon}#map=17/${lat}/${lon}`;
}

/** ゲージの位置(0..100%)。範囲外はクランプ */
export function pct(v, lo, hi) {
  if (!(hi > lo)) return 0;
  return Math.min(100, Math.max(0, ((v - lo) / (hi - lo)) * 100));
}

// ---- 描画 ----

function h(tag, cls, text) {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (text !== undefined) el.textContent = text;
  return el;
}

function setText(el, s) {
  if (el.textContent !== s) el.textContent = s;
}

function fmt(v) {
  return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2);
}

function renderUsage(d) {
  const el = h('div', 'viz viz-usage');
  const bar = h('div', 'gauge');
  const fill = h('div', 'gauge-fill');
  const warn = h('div', 'gauge-mark mark-warn');
  const err = h('div', 'gauge-mark mark-error');
  bar.append(fill, warn, err);
  const label = h('div', 'viz-label');
  const val = h('span', 'mono num');
  const th = h('span', 'viz-sub');
  label.append(val, th);
  el.append(bar, label);
  const update = (d) => {
    fill.style.width = `${pct(d.usage, 0, 100)}%`;
    fill.dataset.zone = d.usage >= d.error ? 'error' : d.usage >= d.warn ? 'warn' : 'ok';
    warn.style.left = `${pct(d.warn, 0, 100)}%`;
    err.style.left = `${pct(d.error, 0, 100)}%`;
    setText(val, d.raw);
    setText(th, `警告 ${d.warn}% / 異常 ${d.error}%`);
  };
  update(d);
  return { el, update };
}

function renderFrequency(d) {
  const el = h('div', 'viz viz-freq');
  const bar = h('div', 'gauge');
  const band = h('div', 'gauge-band');
  const target = h('div', 'gauge-mark mark-target');
  const cur = h('div', 'gauge-cursor');
  bar.append(band, target, cur);
  const label = h('div', 'viz-label');
  const val = h('span', 'mono num');
  const sub = h('span', 'viz-sub');
  label.append(val, sub);
  el.append(bar, label);
  const update = (d) => {
    const hi = Math.max(d.max * 1.25, d.actual * 1.05, 1e-9);
    band.style.left = `${pct(d.min, 0, hi)}%`;
    band.style.width = `${pct(d.max, 0, hi) - pct(d.min, 0, hi)}%`;
    target.hidden = d.target === null;
    if (d.target !== null) target.style.left = `${pct(d.target, 0, hi)}%`;
    cur.style.left = `${pct(d.actual, 0, hi)}%`;
    cur.dataset.zone = d.actual < d.min || d.actual > d.max ? 'error' : 'ok';
    setText(val, `${fmt(d.actual)} Hz`);
    setText(sub, `許容 ${fmt(d.min)}〜${fmt(d.max)} Hz${d.target !== null ? ` / 目標 ${fmt(d.target)} Hz` : ''}`);
  };
  update(d);
  return { el, update };
}

function renderLocation(d, { copyText } = {}) {
  const el = h('div', 'viz viz-loc');
  const text = h('span', 'mono');
  const copy = h('button', 'btn small', 'コピー');
  copy.type = 'button';
  const link = h('a', 'btn small', 'OpenStreetMap');
  link.target = '_blank';
  link.rel = 'noopener noreferrer';
  el.append(text, copy, link);
  let cur = d;
  copy.addEventListener('click', (ev) => {
    ev.stopPropagation();
    if (copyText) copyText(cur.text, copy);
  });
  const update = (d) => {
    cur = d;
    setText(text, d.text);
    // 地図は埋め込まず、クリック時のみ新規タブで開く
    const url = osmUrl(d.lat, d.lon);
    if (link.href !== url) link.href = url;
  };
  update(d);
  return { el, update };
}

export const RENDERERS = [
  { id: 'usage', detect: detectUsage, render: renderUsage },
  { id: 'frequency', detect: detectFrequency, render: renderFrequency },
  { id: 'location', detect: detectLocation, render: renderLocation },
];

/** 当てはまる最初の専用表示を返す。無ければ null */
export function pickRenderer(values) {
  for (const r of RENDERERS) {
    const data = r.detect(values);
    if (data) return { renderer: r, data };
  }
  return null;
}

/** フォールバック: key/value の表。キーごとに行を持ち、変わった値だけ書き換える */
export function renderTable(values) {
  const table = h('table', 'kv');
  const tbody = h('tbody');
  table.append(tbody);
  let rows = new Map();
  const update = (values) => {
    const next = new Map();
    const seen = new Map();
    for (const kv of values || []) {
      // 同じキーが複数あっても行を分ける
      const n = (seen.get(kv.key) || 0) + 1;
      seen.set(kv.key, n);
      const id = `${kv.key}\u0000${n}`;
      let row = rows.get(id);
      if (!row) {
        const tr = h('tr');
        const k = h('th', null, kv.key);
        const v = h('td', 'mono');
        tr.append(k, v);
        row = { tr, v };
      }
      setText(row.v, kv.value);
      row.v.classList.toggle('num', parseNumber(kv.value) !== null);
      next.set(id, row);
    }
    // 順序が変わったときだけ並べ直す
    const order = [...next.values()].map((r) => r.tr);
    const same = order.length === tbody.children.length && order.every((tr, i) => tbody.children[i] === tr);
    if (!same) tbody.replaceChildren(...order);
    rows = next;
  };
  update(values);
  return { el: table, update };
}
