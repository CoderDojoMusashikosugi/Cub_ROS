// node --test tests/  で実行(依存なし)。フィクスチャは tools/fixture_to_json.py で作った JSON を読む。
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  buildTree, computeVisible, countAllLeaves, countLeaves, countsSummary, defaultExpanded,
  formatForCopy, levelName, sortChildren, splitLeafName,
} from '../static/diagtree.js';
import { detectFrequency, detectLocation, detectUsage, parseNumber, pickRenderer } from '../static/renderers.js';

const load = (name) => JSON.parse(readFileSync(new URL(`./fixtures/${name}.json`, import.meta.url), 'utf8')).status;
const sample = load('diag_agg_sample');
const local = load('diag_agg_local_example');

const leaves = (tree) => [...tree.nodes.values()].filter((n) => !n.isGroup);

test('サンプル: ルート 1 つ、グループ 2 つ、葉約 20、STALE 3', () => {
  const t = buildTree(sample);
  assert.deepEqual(t.roots, ['/Robot']);
  const groups = [...t.nodes.values()].filter((n) => n.isGroup).map((n) => n.name).sort();
  assert.deepEqual(groups, ['/Robot', '/Robot/Linux', '/Robot/Sensors']);
  assert.deepEqual(t.nodes.get('/Robot').children.sort(), ['/Robot/Linux', '/Robot/Sensors']);
  const n = leaves(t).length;
  assert.ok(n >= 15 && n <= 25, `葉 ${n} 件`);
  assert.equal(countAllLeaves(t).STALE, 3);
  assert.equal(sample[sample.length - 1].name, '/Robot'); // ルートが最後に来る配列
});

test('配列の順序に依存しない', () => {
  const a = buildTree(sample);
  const b = buildTree([...sample].reverse());
  const shuffled = [...sample].sort((x, y) => (x.message + x.name).localeCompare(y.message + y.name));
  const c = buildTree(shuffled);
  for (const t of [b, c]) {
    assert.deepEqual(t.roots, a.roots);
    for (const [name, node] of a.nodes) {
      assert.equal(t.nodes.get(name).parent, node.parent, name);
      assert.deepEqual([...t.nodes.get(name).children].sort(), [...node.children].sort(), name);
    }
  }
});

test('グループ判定は name のパスのみ(values の有無で判断しない)', () => {
  const t = buildTree([
    { name: '/R', level: 0, message: 'OK', values: [] },
    { name: '/R/g', level: 0, message: 'OK', values: [] }, // values 無しのグループ
    { name: '/R/g/n: c', level: 0, message: 'x', values: [{ key: 'a', value: '1' }] },
    { name: '/R/leaf: no values', level: 0, message: 'x', values: [] }, // values 無しの葉
    { name: '/R/gx', level: 0, message: 'x', values: [{ key: 'k', value: 'v' }] }, // /R/g の前方一致だが別物
  ]);
  assert.equal(t.nodes.get('/R/g').isGroup, true);
  assert.equal(t.nodes.get('/R/leaf: no values').isGroup, false);
  assert.equal(t.nodes.get('/R/gx').isGroup, false);
  assert.equal(t.nodes.get('/R/gx').parent, '/R');
});

test('"/" を含まない名前はトップ直下の葉', () => {
  const t = buildTree([
    { name: 'jetson_state: Jetson Thermal State', level: 0, message: 'ok', values: [] },
    { name: 'linux_state: Linux CPU Usage', level: 1, message: 'w', values: [] },
  ]);
  assert.equal(t.roots.length, 2);
  assert.ok(leaves(t).every((n) => n.parent === null));
  assert.deepEqual(splitLeafName('jetson_state: Jetson Thermal State'), { check: 'Jetson Thermal State', node: 'jetson_state' });
});

test('葉の名前: 最初の ": " で分割、連続空白は詰める', () => {
  assert.deepEqual(splitLeafName('/Robot/Sensors/topic_hz_checker: TopicMonitor  fix'), { check: 'TopicMonitor fix', node: 'topic_hz_checker' });
  assert.deepEqual(splitLeafName('/Robot/Sensors/a: b: c'), { check: 'b: c', node: 'a' });
  assert.deepEqual(splitLeafName('/Robot/Sensors/no colon'), { check: 'no colon', node: '' });
});

test('途中の階層が無いときは最も近い実在の祖先につなぐ', () => {
  const t = buildTree([
    { name: '/A', level: 2, message: 'Error', values: [] },
    { name: '/A/B/C/x: y', level: 2, message: 'e', values: [] },
  ]);
  assert.equal(t.nodes.get('/A/B/C/x: y').parent, '/A');
});

test('level の名前(範囲外は UNKNOWN)', () => {
  assert.deepEqual([0, 1, 2, 3, 4, -1, 255].map(levelName), ['OK', 'WARN', 'ERROR', 'STALE', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN']);
});

test('グループの level は再計算しない・件数は子孫の葉を数える', () => {
  const t = buildTree(sample);
  assert.equal(levelName(t.nodes.get('/Robot/Sensors').status.level), 'ERROR');
  const c = countLeaves(t, '/Robot/Sensors');
  assert.equal(c.STALE, 3);
  assert.ok(c.ERROR >= 1);
  assert.deepEqual(countsSummary({ ERROR: 3, WARN: 0, STALE: 2, UNKNOWN: 0, OK: 9 }), [
    { level: 'ERROR', n: 3 },
    { level: 'STALE', n: 2 },
  ]);
});

test('初期展開: 問題を含むグループは展開、OK だけのグループは閉じる', () => {
  const t = buildTree(sample);
  assert.equal(defaultExpanded(t, '/Robot'), true);
  assert.equal(defaultExpanded(t, '/Robot/Sensors'), true);
  assert.equal(defaultExpanded(t, '/Robot/Linux'), false);
});

test('並び順: 重大度 → 名前', () => {
  const t = buildTree(sample);
  const order = sortChildren(t, t.nodes.get('/Robot/Sensors').children).map((n) => levelName(t.nodes.get(n).status.level));
  const rank = { ERROR: 4, WARN: 3, STALE: 2, UNKNOWN: 1, OK: 0 };
  for (let i = 1; i < order.length; i++) assert.ok(rank[order[i - 1]] >= rank[order[i]], order.join(','));
  assert.deepEqual(sortChildren(t, t.nodes.get('/Robot').children), ['/Robot/Sensors', '/Robot/Linux']);
});

test('絞り込み: 問題のみ・レベル・検索・非表示', () => {
  const t = buildTree(sample);
  const all = computeVisible(t, {});
  assert.equal(all.size, t.nodes.size);

  const prob = computeVisible(t, { problemsOnly: true });
  assert.ok(!prob.has('/Robot/Linux'), 'OK だけのグループは消える');
  assert.ok(prob.has('/Robot/Sensors'));
  assert.ok([...prob].every((n) => t.nodes.get(n).isGroup || t.nodes.get(n).status.level !== 0));

  const stale = computeVisible(t, { levels: new Set(['STALE']) });
  assert.equal([...stale].filter((n) => !t.nodes.get(n).isGroup).length, 3);

  // values の値にも一致(元の name の "/" は空白になっている)
  const q = computeVisible(t, { query: '/livox/lidar' });
  assert.ok(q.has('/Robot/Sensors/topic_hz_checker: TopicMonitor  livox  lidar'));
  assert.ok(!q.has('/Robot/Linux'));
  // グループ名に一致したら配下を全部
  const g = computeVisible(t, { query: 'linux' });
  assert.ok(leaves(t).filter((n) => n.parent === '/Robot/Linux').every((n) => g.has(n.name)));

  const hid = computeVisible(t, { hidden: new Set(['/Robot/Sensors']) });
  assert.ok(!hid.has('/Robot/Sensors'));
  assert.ok(leaves(t).filter((n) => n.parent === '/Robot/Sensors').every((n) => !hid.has(n.name)), 'グループを隠したら配下も隠れる');
});

test('コピー用テキスト', () => {
  const s = { name: '/R/n: c', level: 3, message: '3D fix', hardware_id: 'cub4', values: [{ key: 'k', value: 'v' }] };
  assert.equal(formatForCopy(s), '[STALE] /R/n: c\nmessage: 3D fix\nhardware_id: cub4\n  k: v');
});

test('数値の解釈は失敗しても例外を出さない', () => {
  assert.deepEqual(parseNumber('47.8%'), { value: 47.8, unit: '%' });
  assert.deepEqual(parseNumber('7.37 GiB'), { value: 7.37, unit: 'GiB' });
  assert.deepEqual(parseNumber('9.499818'), { value: 9.499818, unit: '' });
  assert.equal(parseNumber('n/a'), null);
  assert.equal(parseNumber(''), null);
  assert.equal(parseNumber(undefined), null);
});

test('専用表示の判定', () => {
  const t = buildTree(sample);
  const v = (name) => t.nodes.get(name).status.values;
  assert.deepEqual(detectUsage(v('/Robot/Linux/linux_state: Linux CPU Usage')), { usage: 47.8, warn: 95, error: 98, raw: '47.8%' });
  assert.equal(detectUsage(v('/Robot/Linux/linux_state: Linux Storage Usage')), null);
  const f = detectFrequency(v('/Robot/Sensors/topic_hz_checker: TopicMonitor  scan'));
  assert.equal(f.actual, 9.499818);
  assert.equal(f.target, 10);
  assert.deepEqual(detectLocation(v('/Robot/Sensors/ublox_gps_node: fix')), { lat: 35, lon: 139, text: '35.000000, 139.000000' });
  assert.equal(pickRenderer([{ key: 'a', value: 'b' }]), null);
  assert.equal(pickRenderer([{ key: 'Usage', value: 'n/a' }, { key: 'Warn threshold', value: '1%' }, { key: 'Error threshold', value: '2%' }]), null);
});

test('実データ(ローカル取得)でもツリーが作れる', () => {
  const t = buildTree(local);
  assert.deepEqual(t.roots, ['/Robot']);
  assert.ok(t.nodes.get('/Robot/Linux').isGroup);
  assert.ok(t.nodes.get('/Robot/Sensors').isGroup);
  assert.ok(detectFrequency(t.nodes.get('/Robot/Sensors/topic_hz_checker: TopicMonitor  odom').status.values));
});
