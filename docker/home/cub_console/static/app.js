// WebSocket 接続、状態管理、画面更新。
// 方針: name をキーにした要素マップを持ち、変わった部分だけ書き換える(毎回作り直さない)。
// ROS 由来の文字列は textContent でのみ入れる(innerHTML 禁止)。

import {
  buildTree, computeVisible, countAllLeaves, countLeaves, countsSummary, defaultExpanded,
  formatForCopy, levelName, severity, sortChildren, splitLeafName, squeeze, COUNT_ORDER,
} from './diagtree.js';
import { pickRenderer, renderTable } from './renderers.js';

const DIAG_TIMEOUT_SEC = 5; // これより長く診断が来なければ「止まっています」
const HEARTBEAT_TIMEOUT_SEC = 3.5; // これより長く heartbeat が来なければ「接続が切れています」
const LS = {
  expanded: 'cub_console.expanded',
  hidden: 'cub_console.hidden',
  problemsOnly: 'cub_console.problemsOnly',
};
const ICON = { OK: '✔', WARN: '▲', ERROR: '✖', STALE: '◷', UNKNOWN: '?' };

// ---- localStorage(使えなくても動く) ----
function lsGet(key, fallback) {
  try {
    const v = localStorage.getItem(key);
    return v === null ? fallback : JSON.parse(v);
  } catch {
    return fallback;
  }
}
function lsSet(key, value) {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch {
    /* 保存できない環境(プライベートモード等)では保持しないだけ */
  }
}

// ---- 状態 ----
const state = {
  tree: null,
  ws: null,
  wsState: 'connecting', // connecting | open | closed
  lastHeartbeatAt: null, // performance.now() 基準(ms)
  lastDiagAt: null,
  backendDiagAge: null, // heartbeat の diag_age_sec
  levelFilter: null, // 'ERROR' 等 or null
  query: '',
  problemsOnly: !!lsGet(LS.problemsOnly, false),
  hidden: new Set(lsGet(LS.hidden, [])),
  expandedSaved: lsGet(LS.expanded, {}), // ユーザー操作で決まった展開状態
  expandedDefault: new Map(), // 初めて見たときの既定(このページの間だけ)
  openLeaves: new Set(),
};

// ---- DOM ヘルパ ----
const $ = (id) => document.getElementById(id);
function h(tag, cls, text) {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (text !== undefined) el.textContent = text;
  return el;
}
function setText(el, s) {
  if (el.textContent !== s) el.textContent = s;
}
function setAttr(el, k, v) {
  if (el.getAttribute(k) !== v) el.setAttribute(k, v);
}
function setBadge(el, ln) {
  setText(el, `${ICON[ln]} ${ln}`);
  setAttr(el, 'data-level', ln);
}
function button(text, cls, onClick) {
  const b = h('button', cls, text);
  b.type = 'button';
  b.addEventListener('click', (ev) => {
    ev.stopPropagation();
    onClick(ev, b);
  });
  return b;
}

async function copyText(text, btn) {
  let ok = false;
  try {
    await navigator.clipboard.writeText(text);
    ok = true;
  } catch {
    // http でのアクセス等で Clipboard API が使えない場合
    const ta = h('textarea');
    ta.value = text;
    ta.style.position = 'fixed';
    ta.style.opacity = '0';
    document.body.append(ta);
    ta.select();
    try {
      ok = document.execCommand('copy');
    } catch {
      ok = false;
    }
    ta.remove();
  }
  if (btn) {
    const orig = btn.dataset.label || btn.textContent;
    btn.dataset.label = orig;
    btn.textContent = ok ? 'コピー済' : 'コピー失敗';
    setTimeout(() => (btn.textContent = orig), 1500);
  }
}

// ---- 展開状態 ----
function isExpanded(name) {
  if (Object.prototype.hasOwnProperty.call(state.expandedSaved, name)) return !!state.expandedSaved[name];
  return !!state.expandedDefault.get(name);
}
function toggleGroup(name) {
  state.expandedSaved[name] = !isExpanded(name);
  lsSet(LS.expanded, state.expandedSaved);
  applyView();
}

// ---- 非表示 ----
function hide(name) {
  state.hidden.add(name);
  lsSet(LS.hidden, [...state.hidden]);
  applyView();
  renderHiddenList();
}
function unhide(name) {
  state.hidden.delete(name);
  lsSet(LS.hidden, [...state.hidden]);
  applyView();
  renderHiddenList();
}

// ---- ツリーの要素 ----
/** name -> 要素のレコード */
const elements = new Map();

function createGroupEl(name) {
  const wrap = h('div', 'node group');
  wrap.dataset.name = name;
  const row = h('div', 'row');
  row.setAttribute('role', 'button');
  row.tabIndex = 0;
  const caret = h('span', 'caret', '▸');
  const badge = h('span', 'badge');
  const title = h('span', 'title');
  const counts = h('span', 'counts');
  const actions = h('span', 'actions');
  actions.append(button('非表示', 'btn small ghost', () => hide(name)));
  row.append(caret, badge, title, counts, actions);
  const children = h('div', 'children');
  wrap.append(row, children);
  const toggle = () => {
    if (hasSelectionIn(row)) return;
    toggleGroup(name);
  };
  row.addEventListener('click', toggle);
  row.addEventListener('keydown', (ev) => {
    if (ev.key === 'Enter' || ev.key === ' ') {
      ev.preventDefault();
      toggleGroup(name);
    }
  });
  return { kind: 'group', wrap, row, caret, badge, title, counts, children };
}

function createLeafEl(name) {
  const wrap = h('div', 'node leaf');
  wrap.dataset.name = name;
  const row = h('div', 'row');
  row.setAttribute('role', 'button');
  row.tabIndex = 0;
  const badge = h('span', 'badge');
  const names = h('span', 'names');
  const check = h('span', 'title');
  const node = h('span', 'sub');
  names.append(check, node);
  const msgWrap = h('span', 'msg');
  const staleTag = h('span', 'stale-tag', '古い値');
  const msg = h('span', 'msg-text');
  msgWrap.append(staleTag, msg);
  const hw = h('span', 'hw sub');
  const actions = h('span', 'actions');
  const rec = { kind: 'leaf', wrap, row, badge, check, node, msg, staleTag, hw, detail: null, status: null };
  actions.append(
    button('コピー', 'btn small', (_ev, b) => rec.status && copyText(formatForCopy(rec.status), b)),
    button('非表示', 'btn small ghost', () => hide(name)),
  );
  row.append(badge, names, msgWrap, hw, actions);
  const detailEl = h('div', 'detail');
  detailEl.hidden = true;
  wrap.append(row, detailEl);
  rec.detailEl = detailEl;
  const toggle = () => {
    // テキスト選択中のクリックでは開閉しない(コピー操作を邪魔しない)
    if (hasSelectionIn(row)) return;
    if (state.openLeaves.has(name)) state.openLeaves.delete(name);
    else state.openLeaves.add(name);
    updateLeafDetail(rec);
  };
  row.addEventListener('click', toggle);
  row.addEventListener('keydown', (ev) => {
    if (ev.target === row && (ev.key === 'Enter' || ev.key === ' ')) {
      ev.preventDefault();
      toggle();
    }
  });
  return rec;
}

function hasSelectionIn(el) {
  const sel = window.getSelection();
  return !!sel && !sel.isCollapsed && sel.toString() !== '' && el.contains(sel.anchorNode);
}

function updateGroup(rec, tnode) {
  const s = tnode.status;
  const ln = levelName(s.level);
  setBadge(rec.badge, ln);
  setAttr(rec.wrap, 'data-level', ln);
  setText(rec.title, squeeze(splitPathLast(s.name)));
  setAttr(rec.title, 'title', s.name);
  // 子孫の葉の件数(0 は出さない)。OK だけのときは OK 件数を薄く出す
  const c = countLeaves(state.tree, s.name);
  const parts = countsSummary(c);
  const text = parts.length ? parts.map((p) => `${p.level} ${p.n}`).join(' / ') : `OK ${c.OK}`;
  setText(rec.counts, text);
  setAttr(rec.counts, 'data-level', parts.length ? parts[0].level : 'OK');
}

function splitPathLast(name) {
  const parts = name.split('/').filter((x) => x !== '');
  return parts.length ? parts[parts.length - 1] : name;
}

function updateLeaf(rec, tnode) {
  const s = tnode.status;
  rec.status = s;
  const ln = levelName(s.level);
  setBadge(rec.badge, ln);
  setAttr(rec.wrap, 'data-level', ln);
  const { check, node } = splitLeafName(s.name);
  setText(rec.check, check);
  setAttr(rec.check, 'title', s.name);
  setText(rec.node, node);
  setText(rec.msg, s.message);
  setAttr(rec.msg, 'title', s.message);
  rec.staleTag.hidden = ln !== 'STALE';
  setText(rec.hw, s.hardware_id || '');
  updateLeafDetail(rec);
}

function updateLeafDetail(rec) {
  const name = rec.wrap.dataset.name;
  const open = state.openLeaves.has(name);
  rec.wrap.classList.toggle('open', open);
  rec.detailEl.hidden = !open;
  setAttr(rec.row, 'aria-expanded', String(open));
  if (!open || !rec.status) return;
  const s = rec.status;
  if (!rec.detail) {
    const note = h('div', 'stale-note', 'STALE: 更新が止まっています。以下は最後に受信した値です。');
    const special = h('div', 'special');
    const table = renderTable(s.values);
    rec.detailEl.append(note, special, table.el);
    rec.detail = { note, special, table, viz: null };
  } else {
    rec.detail.table.update(s.values);
  }
  rec.detail.note.hidden = levelName(s.level) !== 'STALE';
  const picked = pickRenderer(s.values);
  const d = rec.detail;
  if (!picked) {
    if (d.viz) d.special.replaceChildren();
    d.viz = null;
  } else if (d.viz && d.viz.id === picked.renderer.id) {
    d.viz.inst.update(picked.data);
  } else {
    const inst = picked.renderer.render(picked.data, { copyText });
    d.special.replaceChildren(inst.el);
    d.viz = { id: picked.renderer.id, inst };
  }
}

function ensureEl(tnode) {
  const kind = tnode.isGroup ? 'group' : 'leaf';
  let rec = elements.get(tnode.name);
  if (rec && rec.kind !== kind) {
    // 葉 ⇄ グループが入れ替わった(まれ)。作り直す
    rec.wrap.remove();
    rec = null;
  }
  if (!rec) {
    rec = kind === 'group' ? createGroupEl(tnode.name) : createLeafEl(tnode.name);
    elements.set(tnode.name, rec);
    if (kind === 'group' && !state.expandedDefault.has(tnode.name)) {
      state.expandedDefault.set(tnode.name, defaultExpanded(state.tree, tnode.name));
    }
  }
  return rec;
}

/** container の子要素を names の順に。順序が同じなら触らない */
function reorder(container, names) {
  const want = names.map((n) => elements.get(n).wrap);
  const have = container.children;
  if (want.length === have.length && want.every((el, i) => have[i] === el)) return;
  for (const el of want) container.append(el); // append は移動になる
}

// ---- 診断の受信 → 画面更新 ----
function onDiagnostics(msg) {
  state.tree = buildTree(msg.status || []);
  state.lastDiagAt = performance.now();
  const tree = state.tree;

  for (const tnode of tree.nodes.values()) ensureEl(tnode);
  for (const [name, rec] of elements) {
    if (!tree.nodes.has(name)) {
      rec.wrap.remove();
      elements.delete(name);
      state.openLeaves.delete(name);
    }
  }
  for (const tnode of tree.nodes.values()) {
    const rec = elements.get(tnode.name);
    if (rec.kind === 'group') updateGroup(rec, tnode);
    else updateLeaf(rec, tnode);
  }
  // 並び: 重大度 → 名前。重大度が変わらなければ結果も変わらないので DOM は動かない
  reorder($('tree'), sortChildren(tree, tree.roots));
  for (const tnode of tree.nodes.values()) {
    if (tnode.isGroup) reorder(elements.get(tnode.name).children, sortChildren(tree, tnode.children));
  }
  updateHeader();
  applyView();
}

function filtering() {
  return !!(state.levelFilter || state.problemsOnly || squeeze(state.query));
}

/** 絞り込み・非表示・展開状態を反映(要素の hidden とクラスだけ) */
function applyView() {
  const tree = state.tree;
  if (!tree) return;
  const visible = computeVisible(tree, {
    levels: state.levelFilter ? new Set([state.levelFilter]) : null,
    problemsOnly: state.problemsOnly,
    query: state.query,
    hidden: state.hidden,
  });
  const forceOpen = filtering();
  for (const [name, rec] of elements) {
    rec.wrap.hidden = !visible.has(name);
    if (rec.kind === 'group') {
      const open = forceOpen || isExpanded(name);
      rec.wrap.classList.toggle('collapsed', !open);
      setText(rec.caret, open ? '▾' : '▸');
      setAttr(rec.row, 'aria-expanded', String(open));
    }
  }
  const empty = $('empty');
  if (visible.size === 0) {
    empty.hidden = false;
    setText(empty, tree.nodes.size === 0 ? '診断に項目がありません。' : '条件に一致する項目はありません。');
  } else {
    empty.hidden = true;
  }
  updateChips();
}

// ---- ヘッダ ----
function updateHeader() {
  const tree = state.tree;
  const roots = $('roots');
  const groups = tree.roots.filter((r) => tree.nodes.get(r).isGroup);
  const items = groups.length
    ? groups.map((r) => ({ key: r, label: squeeze(splitPathLast(r)), level: tree.nodes.get(r).status.level }))
    : [{ key: '', label: '全体', level: worstLeafLevel(tree) }];
  const want = items.map((it) => it.key);
  const have = [...roots.children].map((el) => el.dataset.key);
  if (want.join('\n') !== have.join('\n')) {
    roots.replaceChildren(
      ...items.map((it) => {
        const el = h('span', 'root-badge');
        el.dataset.key = it.key;
        el.append(h('span', 'root-name'), h('span', 'badge big'));
        return el;
      }),
    );
  }
  items.forEach((it, i) => {
    const el = roots.children[i];
    setText(el.children[0], it.label);
    setAttr(el, 'title', it.key || '全体(最悪値)');
    setBadge(el.children[1], levelName(it.level));
  });
}

function worstLeafLevel(tree) {
  let worst = null;
  for (const n of tree.nodes.values()) {
    if (n.isGroup) continue;
    if (worst === null || severity(n.status.level) > severity(worst)) worst = n.status.level;
  }
  return worst ?? -1;
}

function updateChips() {
  const counts = state.tree ? countAllLeaves(state.tree) : null;
  for (const btn of $('chips').children) {
    const ln = btn.dataset.level;
    setText(btn.querySelector('.n'), counts ? String(counts[ln]) : '–');
    setAttr(btn, 'aria-pressed', String(state.levelFilter === ln));
    btn.classList.toggle('zero', !counts || counts[ln] === 0);
  }
}

function initChips() {
  const chips = $('chips');
  for (const ln of COUNT_ORDER) {
    const b = h('button', 'chip');
    b.type = 'button';
    b.dataset.level = ln;
    b.append(h('span', 'chip-label', `${ICON[ln]} ${ln}`), h('span', 'n', '–'));
    b.addEventListener('click', () => {
      state.levelFilter = state.levelFilter === ln ? null : ln;
      applyView();
      updateChips();
    });
    chips.append(b);
  }
}

// ---- 鮮度と途絶の表示 ----
function tick() {
  const now = performance.now();
  const fresh = $('fresh');
  setText(fresh, state.lastDiagAt === null ? '未受信' : `更新 ${((now - state.lastDiagAt) / 1000).toFixed(1)} 秒前`);

  const hbLate = state.lastHeartbeatAt === null || (now - state.lastHeartbeatAt) / 1000 > HEARTBEAT_TIMEOUT_SEC;
  const connected = state.wsState === 'open' && !hbLate;
  const conn = $('conn');
  setText(conn, connected ? '接続中' : state.wsState === 'connecting' ? '接続待ち' : '切断');
  setAttr(conn, 'data-state', connected ? 'ok' : 'bad');

  let banner = null;
  let kind = '';
  if (!connected) {
    kind = 'disconnected';
    banner = '接続が切れています(本アプリに届きません)。表示中の値は古い可能性があります。再接続を試みています…';
  } else if (state.backendDiagAge === null) {
    kind = 'nodiag';
    banner = '診断をまだ受信していません(本アプリは動作中。ROS 側から /diagnostics_agg が届いていません)。';
  } else if (state.backendDiagAge > DIAG_TIMEOUT_SEC) {
    kind = 'stale';
    banner = `診断の更新が止まっています(本アプリは動作中。ROS 側の診断が ${state.backendDiagAge.toFixed(0)} 秒届いていません)。表示中の値は古い値です。`;
  }
  const el = $('banner');
  el.hidden = banner === null;
  if (banner !== null) {
    setText(el, banner);
    setAttr(el, 'data-kind', kind);
  }
  document.body.classList.toggle('is-stale', banner !== null && state.lastDiagAt !== null);
}

// ---- WebSocket ----
let retryMs = 1000;
function connect() {
  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
  state.wsState = 'connecting';
  const ws = new WebSocket(url);
  state.ws = ws;
  ws.addEventListener('open', () => {
    state.wsState = 'open';
    state.lastHeartbeatAt = performance.now();
    retryMs = 1000;
  });
  ws.addEventListener('message', (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch (e) {
      console.error('不正なメッセージ', e);
      return;
    }
    state.lastHeartbeatAt = performance.now();
    if (msg.type === 'diagnostics') {
      state.backendDiagAge = 0;
      onDiagnostics(msg);
    } else if (msg.type === 'heartbeat') {
      state.backendDiagAge = msg.diag_age_sec;
    }
  });
  ws.addEventListener('close', () => {
    state.wsState = 'closed';
    state.ws = null;
    setTimeout(connect, retryMs);
    retryMs = Math.min(retryMs * 2, 5000);
  });
}

// ---- 非表示一覧 ----
function renderHiddenList() {
  setText($('hidden-count'), String(state.hidden.size));
  const ul = $('hidden-list');
  const items = [...state.hidden].sort();
  ul.replaceChildren(
    ...items.map((name) => {
      const li = h('li');
      li.append(h('span', 'mono', name), button('復元', 'btn small', () => unhide(name)));
      return li;
    }),
  );
  $('hidden-empty').hidden = items.length > 0;
}

// ---- 初期化 ----
function init() {
  initChips();
  const q = $('search');
  q.addEventListener('input', () => {
    state.query = q.value;
    applyView();
  });
  const po = $('problems-only');
  po.checked = state.problemsOnly;
  po.addEventListener('change', () => {
    state.problemsOnly = po.checked;
    lsSet(LS.problemsOnly, state.problemsOnly);
    applyView();
  });
  const dlg = $('hidden-dlg');
  $('hidden-open').addEventListener('click', () => {
    renderHiddenList();
    dlg.showModal();
  });
  $('hidden-close').addEventListener('click', () => dlg.close());
  $('hidden-restore-all').addEventListener('click', () => {
    state.hidden.clear();
    lsSet(LS.hidden, []);
    applyView();
    renderHiddenList();
  });
  renderHiddenList();
  updateChips();
  connect();
  setInterval(tick, 250);
  tick();
}

init();
