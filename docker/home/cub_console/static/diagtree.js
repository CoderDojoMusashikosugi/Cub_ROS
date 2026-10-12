// 診断の name(/ 区切りの階層パス)からツリーを作る純関数群。DOM には触れない。

export const LEVEL = Object.freeze({ OK: 0, WARN: 1, ERROR: 2, STALE: 3 });
export const LEVEL_NAMES = ['OK', 'WARN', 'ERROR', 'STALE'];
// ヘッダの件数チップ・件数表示の並び
export const COUNT_ORDER = ['ERROR', 'WARN', 'STALE', 'UNKNOWN', 'OK'];

/** level(int)→ 'OK' | 'WARN' | 'ERROR' | 'STALE' | 'UNKNOWN' */
export function levelName(level) {
  return Number.isInteger(level) && level >= 0 && level < LEVEL_NAMES.length ? LEVEL_NAMES[level] : 'UNKNOWN';
}

// 並び替え用の重大度(大きいほど上)
const SEVERITY = { ERROR: 4, WARN: 3, STALE: 2, UNKNOWN: 1, OK: 0 };
export function severity(level) {
  return SEVERITY[levelName(level)];
}

/** OK 以外(WARN / ERROR / STALE / UNKNOWN)を「問題」とみなす */
export function isProblem(level) {
  return levelName(level) !== 'OK';
}

/** name をパスの要素に分ける。先頭の '/' は無視。'/' を含まない名前は 1 要素。 */
export function splitPath(name) {
  return name.split('/').filter((s) => s !== '');
}

/** 表示用: 連続空白(元は '/')を 1 つに詰める */
export function squeeze(s) {
  return s.replace(/\s+/g, ' ').trim();
}

/**
 * 葉の表示名。最後のパス要素を最初の ": " で分け、チェック名を主、ノード名を補助にする。
 * @returns {{check: string, node: string}}
 */
export function splitLeafName(name) {
  const parts = splitPath(name);
  const last = parts.length ? parts[parts.length - 1] : name;
  const i = last.indexOf(': ');
  if (i < 0) return { check: squeeze(last), node: '' };
  return { check: squeeze(last.slice(i + 2)), node: squeeze(last.slice(0, i)) };
}

/**
 * status 配列からツリーを作る。配列の順序には依存しない。
 * グループ = 他の status の name が `name + "/"` で始まるもの(values の有無では判断しない)。
 * 親 = 自分の name の祖先パスのうち、実在する最も近いもの。無ければトップ階層。
 * @returns {{roots: string[], nodes: Map<string, TreeNode>}}
 *   TreeNode = {name, status, parent: string|null, children: string[], isGroup: boolean, depth: number}
 */
export function buildTree(statusList) {
  const nodes = new Map();
  for (const s of statusList) {
    // 同名が重複したら後勝ち(aggregator では通常起きない)
    nodes.set(s.name, { name: s.name, status: s, parent: null, children: [], isGroup: false, depth: 0 });
  }
  const roots = [];
  for (const node of nodes.values()) {
    const parts = splitPath(node.name);
    const lead = node.name.startsWith('/') ? '/' : '';
    let parent = null;
    for (let k = parts.length - 1; k > 0; k--) {
      const cand = lead + parts.slice(0, k).join('/');
      if (cand !== node.name && nodes.has(cand)) {
        parent = cand;
        break;
      }
    }
    node.parent = parent;
    if (parent === null) roots.push(node.name);
    else nodes.get(parent).children.push(node.name);
  }
  for (const node of nodes.values()) {
    node.isGroup = node.children.length > 0;
  }
  // 深さ(インデント用)
  const setDepth = (name, d) => {
    const n = nodes.get(name);
    n.depth = d;
    for (const c of n.children) setDepth(c, d + 1);
  };
  for (const r of roots) setDepth(r, 0);
  return { roots, nodes };
}

function emptyCounts() {
  return { ERROR: 0, WARN: 0, STALE: 0, UNKNOWN: 0, OK: 0 };
}

/** 子孫の葉のレベル別件数(葉自身なら自分 1 件)。 */
export function countLeaves(tree, name, out = emptyCounts()) {
  const n = tree.nodes.get(name);
  if (!n) return out;
  if (!n.isGroup) {
    out[levelName(n.status.level)] += 1;
    return out;
  }
  for (const c of n.children) countLeaves(tree, c, out);
  return out;
}

/** ツリー全体の葉のレベル別件数 */
export function countAllLeaves(tree) {
  const out = emptyCounts();
  for (const r of tree.roots) countLeaves(tree, r, out);
  return out;
}

/** 件数を `ERROR 3 / STALE 2` 形式の配列にする(0 は出さない) */
export function countsSummary(counts, { includeOk = false } = {}) {
  return COUNT_ORDER.filter((k) => counts[k] > 0 && (includeOk || k !== 'OK')).map((k) => ({ level: k, n: counts[k] }));
}

/** 子の並び: 重大度の高い順 → name 順 */
export function sortChildren(tree, names) {
  return [...names].sort((a, b) => {
    const sa = severity(tree.nodes.get(a).status.level);
    const sb = severity(tree.nodes.get(b).status.level);
    if (sa !== sb) return sb - sa;
    return a < b ? -1 : a > b ? 1 : 0;
  });
}

/** 検索: 名前・message・values のキーと値に部分一致(大文字小文字無視、空白は詰めて比較) */
export function matchesQuery(status, query) {
  const q = squeeze(query).toLowerCase();
  if (!q) return true;
  const hay = [status.name, status.message, status.hardware_id || ''];
  for (const kv of status.values || []) hay.push(kv.key, kv.value);
  return hay.some((h) => squeeze(String(h)).toLowerCase().includes(q));
}

/** name 自身か祖先が hidden に含まれるか */
export function isHiddenByUser(tree, name, hidden) {
  for (let n = name; n !== null; n = tree.nodes.get(n)?.parent ?? null) {
    if (hidden.has(n)) return true;
  }
  return false;
}

/**
 * 表示する name の集合を求める。
 * - ユーザーが隠したもの(と配下)は出さない
 * - 葉は level 絞り込み / 問題のみ / 検索 をすべて満たすとき表示
 * - グループは、表示される子孫があるとき表示。絞り込みが無い場合は常に表示
 *   (検索でグループ名自体が一致した場合は配下を全部見せる)
 * @param {{levels?: Set<string>|null, problemsOnly?: boolean, query?: string, hidden?: Set<string>}} opts
 * @returns {Set<string>}
 */
export function computeVisible(tree, opts = {}) {
  const levels = opts.levels && opts.levels.size ? opts.levels : null;
  const problemsOnly = !!opts.problemsOnly;
  const query = opts.query || '';
  const hidden = opts.hidden || new Set();
  const filtering = !!(levels || problemsOnly || squeeze(query));
  const visible = new Set();

  const leafPasses = (s, queryOk) => {
    const ln = levelName(s.level);
    if (levels && !levels.has(ln)) return false;
    if (problemsOnly && !isProblem(s.level)) return false;
    return queryOk || matchesQuery(s, query);
  };

  // 戻り値: 自分(または子孫)が表示されるか
  const walk = (name, ancestorMatched) => {
    if (hidden.has(name)) return false;
    const n = tree.nodes.get(name);
    if (!n.isGroup) {
      const ok = leafPasses(n.status, ancestorMatched);
      if (ok) visible.add(name);
      return ok;
    }
    const selfMatched = ancestorMatched || (!!squeeze(query) && matchesQuery(n.status, query));
    let any = false;
    for (const c of n.children) any = walk(c, selfMatched) || any;
    if (any || !filtering) visible.add(name);
    return any || !filtering;
  };
  for (const r of tree.roots) walk(r, false);
  return visible;
}

/** 「コピー」用のプレーンテキスト(Slack 貼り付け用) */
export function formatForCopy(status) {
  const lines = [`[${levelName(status.level)}] ${status.name}`, `message: ${status.message}`];
  if (status.hardware_id) lines.push(`hardware_id: ${status.hardware_id}`);
  for (const kv of status.values || []) lines.push(`  ${kv.key}: ${kv.value}`);
  return lines.join('\n');
}

/** グループの初期展開: 自分か子孫に問題があれば展開 */
export function defaultExpanded(tree, name) {
  const n = tree.nodes.get(name);
  if (!n) return false;
  if (isProblem(n.status.level)) return true;
  const c = countLeaves(tree, name);
  return c.ERROR + c.WARN + c.STALE + c.UNKNOWN > 0;
}
