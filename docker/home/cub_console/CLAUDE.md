# cub_console

> **現状メモ:** プロセス管理は枠組み検討中のため、いったんアプリ(API・画面)から外している。`app/processes.py` と設定・単体テストは残してある。接続元制限(`CUB_CONSOLE_ALLOW`)を追加済み。詳細は README.md。

ロボット `cub`(Jetson Orin 上の Docker コンテナ)向けの Web コンソール。
ブラウザから ROS 2 の診断(`/diagnostics_agg`)を見やすく表示し、ROS ノードのプロセスを起動/停止/再起動する。
最初は「診断の閲覧 + プロセス操作」だけ。将来はロボット制御の中核に育てる予定なので、拡張しやすさと安全性を優先する。

## 最重要の制約

1. **Jetson の計算資源を最小限にする。** 常駐するのは本アプリ1プロセスだけ。nginx・rosbridge・DB などは足さない。重い描画や整形は全てブラウザ側で行う。
2. **ROS が落ちても操作系が生きること。** プロセスの起動/停止は ROS 通信に載せず、OS(subprocess / systemd)を直接操作する。
3. **任意コマンド実行 API を作らない。** 操作できるのは `config/managed_processes.yaml` に書いた id だけ(ホワイトリスト)。
4. **認証を入れるまでは `127.0.0.1` のみ待受。** 操作系 API を LAN に露出させない。
5. **既存資産を壊さない。** `/home/cub/colcon_ws` 配下は変更しない(`cub_diagnostics` などはそちら)。本アプリは `/home/cub/cub_console` だけで完結させ、colcon ビルドの対象にしない。
6. **外部ネットワークに依存しない。** ロボットはインターネットに繋がらない前提。CDN、Web フォント、地図タイルなどを読み込まない。JS/CSS は全てローカルに置く。

## アーキテクチャ

```
[ブラウザ] ──WebSocket(状態配信)/ REST(命令)── [cub_console (FastAPI, 1プロセス)]
                                                  ├─ ros_bridge : rclpy で /diagnostics_agg を購読(購読のみ)
                                                  └─ ProcessBackend : subprocess(開発) / systemd(本番)
```

- ブラウザは DDS を話せないので、本アプリが翻訳役になる。ブラウザが直接つながるのは本アプリだけ。
- 状態の配信は WebSocket、命令は REST。命令を WebSocket や ROS topic に混ぜない(成否・認証・監査を HTTP で扱うため)。
- フロントは素の HTML + ES modules(ビルド工程なし、外部ライブラリなし)。診断のツリー構築と表示は全てクライアント側。
- バックエンドは診断を **加工せず**(level の int 化と JSON 化のみ)配る。見せ方の工夫はフロントに閉じ込める。

## 診断データの実態(実機サンプルより)

`/diagnostics_agg` の実データから分かっている性質。**これを前提に実装する**(推測で単純化しない)。

```yaml
# 抜粋(値は差し替え済み)。実物は tests/fixtures/diag_agg_sample.yaml
header: {stamp: {sec: 1791013863, nanosec: 79736957}, frame_id: ''}
status:
- level: "\0"                         # OK。bytes で届く
  name: /Robot/Linux/linux_state: Linux CPU Usage     # 葉: "<ノード名>: <チェック名>"
  message: CPU usage 47.8%
  hardware_id: cub4
  values:
  - {key: Usage, value: 47.8%}
  - {key: Warn threshold, value: 95.0%}
  - {key: Error threshold, value: 98.0%}
- level: "\x03"                       # STALE。最後の値を保持したまま
  name: '/Robot/Sensors/ublox_gps_node: fix'
  message: 3D fix
  values: [{key: 'Latitude [deg]', value: '35.0000'}, ...]
- level: "\x02"                       # グループ(子の最悪値)
  name: /Robot/Sensors
  message: Error
  values:                             # 子の要約。ツリー構築には使わない
  - {key: 'topic_hz_checker: TopicMonitor /fix', value: No events recorded.}
- level: "\x02"
  name: /Robot                        # ルートは配列の最後に来ることがある
  message: Error
```

1. **名前は `/` 区切りの階層パス。** `Robot` は aggregator の設定(`path: Robot`)由来なのでハードコードせず、トップ階層は名前から動的に求める。`/` を含まない名前(`/diagnostics` を直接見た場合の `jetson_state: Jetson Thermal State` など)も、トップ直下の葉として扱えること。
2. **葉の名前は `<ノード名>: <チェック名>`。** 最初の `": "` で分割し、チェック名を主表示、ノード名を補助表示にする。`": "` が無い名前はそのまま主表示。
3. **名前の中の `/` は空白に置換されている**(`TopicMonitor  fix` のように空白が連続する。元は `/fix`)。表示では連続空白を詰めてよいが、元の `name` は内部キーと tooltip に残す。
4. **グループは「他の status の name が `この name + "/"` で始まる」ものとして判定する。** 葉かグループかを `values` の有無で判断しない。
5. **グループの `values`(子の要約)は、キーが子の `name` と一致しない**(`/` の有無など)。ツリー構築には使わず、`name` のパスだけで親子を作る。
6. **グループの level は aggregator が付けた「子の最悪値」。再計算しない。** message は `OK` / `Error` のような一語。画面に出す子孫の件数(ERROR 3 件など)はクライアントで数える。
7. **配列の順序は保証されない**(実測では子が先、ルートが最後)。順序に依存しない。
8. **level は `bytes` で届く**(`ros2 topic echo` では `"\x03"` と見える)。0:OK 1:WARN 2:ERROR 3:STALE。範囲外は `UNKNOWN` 扱い。
9. **STALE の葉は最後の値を保持している**(message が `3D fix` のままなど)。「正常」と誤認させないよう、灰色 + `STALE` の文字ラベルで「古い値」と分かる表示にする。
10. **`values` は文字列だけ。** 数値も `'47.8%'` `'7.37 GiB'` `'9.499818'` の形。数値化は表示(ゲージ)のためだけに行い、失敗したら素の文字列で表示する(例外を出さない)。
11. **規模:** 1 メッセージに約 30 件の status、葉ごとに最大 16 個程度の `values`。JSON で数十 KB。1 Hz なので全量送信でよい(実測して重ければ差分化を検討)。

### values の定番パターン(専用表示の対象)

| パターン | 判定に使うキー | 表示 |
|---|---|---|
| 使用率 | `Usage`, `Warn threshold`, `Error threshold`(すべて `%`) | 横棒 + 警告/異常のしきい線 |
| 周波数 | `Actual frequency (Hz)`, `Minimum acceptable frequency (Hz)`, `Maximum acceptable frequency (Hz)`(あれば `Target frequency (Hz)`) | 許容範囲の帯 + 現在値の印 |
| 位置 | `Latitude [deg]`, `Longitude [deg]` | 「緯度, 経度」のコピーボタンと OpenStreetMap へのリンク(クリック時のみ新規タブ。地図は埋め込まない) |
| その他 | 任意 | key/value の表(フォールバック) |

専用表示は `static/renderers.js` に「判定関数 + 描画関数」の配列として登録する作りにし、後からパターンを足せるようにする。どれにも当てはまらなければフォールバック。

## UI 仕様

目的は「ロボットを動かしながら、問題が起きた箇所を一目で見つける」こと。正常な項目は邪魔にならず、異常な項目が目立つ。

**ヘッダ(常に見える)**
- 接続状態(WebSocket)と、診断の鮮度(「更新 0.8 秒前」)。
- ルートの総合レベルを大きなバッジで表示(ルートが複数ならそれぞれ)。
- レベル別の件数チップ(ERROR / WARN / STALE / OK、**葉のみ**を数える)。クリックでそのレベルだけに絞り込む。
- 検索ボックス(名前・メッセージ・`values` のキーと値に部分一致)、「問題のみ表示」トグル。

**ツリー**
- グループは折りたたみ可能。行にレベルバッジ、名前、子孫のレベル別件数(`ERROR 3 / STALE 2` のように 0 のものは出さない)。
- 初回表示は、WARN 以上を含むグループを展開し、OK だけのグループは折りたたむ。以降はユーザー操作を優先し、展開状態をパス(`name`)をキーに `localStorage` へ保存する。折りたたまれたグループ内で新しく問題が出ても、件数表示で気付けること。
- 同じ階層内の並び順は「重大度の高い順 → 名前順」。更新のたびに並びが入れ替わって読みにくくならないよう、**並び替えは重大度が変わったときだけ**行う。

**葉の行**
- レベルバッジ、チェック名(主)、ノード名(補助、薄く)、message(長ければ省略して tooltip)、`hardware_id`(薄く)。
- クリックで展開し、`values` を専用表示(上の表)またはフォールバック表で見せる。展開は既定で閉じる。
- 「コピー」ボタン: その葉の name / level / message / values をプレーンテキストにして、クリップボードへ(Slack に貼る用途)。
- 「非表示」ボタン: パスを `localStorage` に保存して隠す。ヘッダから非表示一覧を開いて復元できる。グループを隠したら配下も隠れる。

**更新の扱い(重要)**
- **毎回 DOM を全部作り直さない。** `name` をキーにした要素のマップを持ち、変わった部分(バッジ、message、値)だけ更新する。展開状態・スクロール位置・テキスト選択(コピー操作)が更新で消えないこと。
- ROS 由来の文字列は必ず `textContent` / DOM API で入れる(`innerHTML` 禁止)。
- 診断が一定時間(既定 5 秒)届かない場合は、画面全体に「診断の更新が止まっています」を出し、表示中の値が古いことを明示する。**WebSocket 切断(本アプリに届かない)** と **ROS 側の診断停止(本アプリは生きている)** を区別して表示する(後者はバックエンドの heartbeat で判定)。

**見た目**
- レベルは色だけに頼らず、文字ラベルまたはアイコンを必ず併記する。OK = 緑、WARN = 黄、ERROR = 赤、STALE = 灰、UNKNOWN = 白抜き。
- ダークテーマを既定にする(屋外・暗所での利用を想定)。`prefers-color-scheme` に追従してもよい。
- 等幅フォントは値の欄だけ。数値は右揃え。画面幅が狭い(スマホ)場合でも崩れない(横スクロールはしない)。

## ディレクトリ構成

```
/home/cub/cub_console/
├─ CLAUDE.md
├─ README.md                  起動手順と実測結果(短く)
├─ requirements.txt / requirements-dev.txt
├─ app/
│  ├─ __init__.py
│  ├─ main.py                 FastAPI 本体(REST + WebSocket + 静的配信)
│  ├─ ros_bridge.py           rclpy スレッド → asyncio への橋渡し、Hub
│  └─ processes.py            ProcessBackend(Protocol)+ SubprocessBackend
├─ config/managed_processes.yaml
├─ static/
│  ├─ index.html
│  ├─ style.css
│  ├─ app.js                  WebSocket 接続、状態管理、画面更新
│  ├─ diagtree.js             name のパスからツリーを作る純関数(DOM 非依存)
│  └─ renderers.js            values の専用表示(判定関数 + 描画関数)
├─ tools/fake_diag_pub.py     サンプルを /diagnostics_agg に流すテスト用 publisher
└─ tests/
   ├─ fixtures/diag_agg_sample.yaml   実機の `ros2 topic echo --once` 出力
   ├─ test_processes.py
   └─ diagtree.test.mjs       node があれば。diagtree.js を node:test で検証
```

## テスト用データ(作業前に確認)

- `tests/fixtures/diag_agg_sample.yaml` は、ユーザーが実機で `ros2 topic echo /diagnostics_agg --once > tests/fixtures/diag_agg_sample.yaml` として保存する。**無ければ作業前にユーザーへ依頼する。** 実機の GPS 座標などが含まれるので、リポジトリを公開する場合は値を差し替えること。
- `tools/fake_diag_pub.py` はこれを `yaml.safe_load_all` で読み込み、`DiagnosticArray` に詰めて 1 Hz で publish する。
  - `level` は YAML 上では 1 文字の文字列(`"\x03"`)なので、`bytes([ord(ch)])` で `DiagnosticStatus.level` に設定する。整数で来た場合は `bytes([n])`。
  - `header.stamp` は現在時刻に更新する。
  - オプション: `--hz`(既定 1)、`--jitter`(`Usage` や周波数を少し揺らして画面の動きを確認する)、`--stop-after N`(N 秒で publish を止め、診断途絶の表示を確認する)。

## 通信仕様

### API

- `GET  /api/health` → `{"ok": true, "ros": <rclpy が使えるか>, "last_diag_age_sec": <最後の診断受信からの秒数 or null>}`
- `GET  /api/processes` → `[{"id","label","state"}]`(`state` は `active` / `inactive` / `failed(rc=N)`)
- `POST /api/processes/{id}/{start|stop|restart}` → `{"ok": true}`。未知の id・未知の action は 404
- `WS   /ws` → 接続直後に最新の診断(あれば)を送り、以降は更新ごとに送る。診断とは別に 1 秒ごとに heartbeat を送る

### WebSocket メッセージ(JSON 文字列。`type` で種別を区別)

```json
{"type":"diagnostics","stamp":1791013863.0797,
 "status":[{"name":"/Robot/Linux","level":0,"message":"OK","hardware_id":"",
            "values":[{"key":"Usage","value":"47.8%"}]}]}
{"type":"heartbeat","diag_age_sec":0.8}
```

- `stamp` は `header.stamp` の `sec + nanosec * 1e-9`。画面の「更新 N 秒前」は、ブラウザ側で最後に受信した時刻から数える(Jetson とブラウザの時計のずれに影響されない)。
- `diag_age_sec` は、バックエンドが最後に診断を受け取ってからの秒数(未受信なら `null`)。

## 実装仕様(バックエンド)

### 設定(環境変数)

| 変数 | 既定 | 意味 |
|---|---|---|
| `CUB_CONSOLE_CONFIG` | `config/managed_processes.yaml` | 管理対象プロセスの定義 |
| `CUB_CONSOLE_HOST` | `127.0.0.1` | 待受アドレス |
| `CUB_CONSOLE_PORT` | `8080` | 待受ポート |

### ros_bridge.py

- `rclpy` が import できない環境でも起動できる(診断購読だけ無効にして警告ログ)。
- rclpy は別スレッドの `SingleThreadedExecutor` で回す。コールバックから asyncio へは `loop.call_soon_threadsafe` で渡す。
- **罠:** rclpy では `byte` 型フィールド(`DiagnosticStatus.level`)は `bytes` で届く。`v if isinstance(v, int) else v[0]` で int にする。
- JSON 化は **1回だけ**行い、文字列を全クライアントに `send_text` で配る(クライアントごとに `send_json` しない)。
- `Hub`: 最新値を保持し、接続ごとに `asyncio.Queue(maxsize=2)`。満杯なら古い値を捨てる(遅いクライアントに引きずられない)。
- 受信時刻(`time.monotonic()`)を保持し、`/api/health` と heartbeat に使う。
- ROS ノード名は `cub_console`。起動時に `ros2 node list` で重複がないこと。

### processes.py

- `ProcessBackend` は `typing.Protocol`:`list()` / `start(id)` / `stop(id)` / `restart(id)` / `shutdown()`。
- `SubprocessBackend`(開発用)の要件:
  - `asyncio.create_subprocess_exec(*cmd, start_new_session=True)` で起動(`ros2 launch` の子ごと止めるため)。**shell=True は使わない。**
  - 停止は `os.killpg(pid, SIGINT)` → 8 秒待つ → `SIGTERM` → 3 秒 → `SIGKILL`。ROS 2 は SIGINT で正常終了するため最初は SIGINT。
  - id ごとに `asyncio.Lock` で start/stop/restart を直列化する。`restart` は同じロックの中で stop → start を行う。
  - アプリ終了時(`lifespan` の終了処理)に管理下のプロセスを停止する(開発中の道連れ動作)。
- 将来 `SystemdBackend`(D-Bus 経由で `ActiveState` 取得、`PropertiesChanged` 購読、`StartUnit` / `StopUnit` / `RestartUnit`)を同じ Protocol で追加する。**今回は実装しない**が、差し替えやすいよう `main.py` は Protocol にだけ依存させる。

### config/managed_processes.yaml

```yaml
backend: subprocess
processes:
  - id: demo_talker
    label: デモ talker
    cmd: ["ros2", "run", "demo_nodes_cpp", "talker"]
  - id: jetson_state
    label: Jetson 温度監視
    cmd: ["ros2", "run", "cub_diagnostics", "jetson_state"]
```

`cmd` は配列のみ(文字列は受け付けない)。`cwd` は任意。

## 環境(Docker コンテナ内)

- ROS 2 は **起動前に source 済みであること**。ディストリビューションは `echo $ROS_DISTRO` で確認し、決め打ちしない。
- Python: `python3 -m venv --system-site-packages .venv` で作る(`rclpy` をシステム側から見せるため)。
- 依存は `requirements.txt` に: `fastapi`, `uvicorn[standard]`, `pyyaml`(テスト用に `pytest`, `httpx` は `requirements-dev.txt`)。
- 実行ユーザは `cub`。root 実行が前提のコードは書かない。
- **Docker の注意:** `127.0.0.1` 待受はコンテナのネットワークが `host` モードのときだけホストから見える。ブリッジ接続の場合は `CUB_CONSOLE_HOST=0.0.0.0` にしたうえで、`docker run -p 127.0.0.1:8080:8080` のようにホスト側を localhost に限定する。**どちらの構成かは作業前にユーザーへ確認する。**

## 進め方

1. **土台:** 上記の構成でバックエンドとファイルを作る。まず ROS なし(`HAVE_ROS=False`)で起動し、`/api/health` を確認する。
2. **プロセス操作:** `tests/test_processes.py` を書く。`["sleep","60"]` のようなコマンドで次を検証する。
   - start → `active`、stop → `inactive`、restart で PID が変わる
   - 子プロセスを持つコマンド(`["sh","-c","sleep 60 & wait"]` など)でも、stop 後に子が残らない
   - 未知の id が `UnknownProcess` になる
3. **診断ツリー:** `diagtree.js` を純関数で作り、フィクスチャで検証する(上の「診断データの実態」の 1〜9 を満たすこと)。`node` があれば `diagtree.test.mjs` を書く。
   - 期待: ルート 1 つ、`/Robot/Linux` と `/Robot/Sensors` の 2 グループ、葉が約 20 件、STALE の葉が 3 件、`/Robot` が最後に来る配列順でも正しいツリーになる。
4. **UI:** `fake_diag_pub.py` で流して、ヘッダ・ツリー・葉の展開・検索・「問題のみ」・コピー・非表示を実装する。その後に専用表示(使用率・周波数・位置)を足す。
5. **途絶の確認:** `--stop-after` で診断を止め、「診断の更新が止まっています」が出ること、本アプリを止めたときは「接続が切れています」が出ることを確認する。
6. **実機測定:** `top -H -p $(pgrep -f "app.main")` で、ブラウザ接続あり/なしの CPU とメモリを測り、結果を README に数値で記録する。

## 完了の定義

- `python -m py_compile app/*.py` と `pytest` が通る(node があれば `node --test tests/` も)。
- `curl http://127.0.0.1:8080/api/health` が応答する。
- フィクスチャを流すと、問題のあるグループが展開され、OK だけのグループは折りたたまれて表示される。STALE の葉が「古い値」と分かる。
- 更新が来ても、展開状態・スクロール位置・テキスト選択が保たれる。
- `demo_talker` を UI から start / stop / restart でき、`ros2 node list` と `ps` で挙動(子が残らないこと)を確認できる。

## やらないこと(今回)

- 認証・HTTPS・nginx の導入(必要になった時点で別途相談)。
- ROS 側への publish や service 呼び出し(`cmd_vel` など)。本アプリは購読のみ。
- `SystemdBackend` の実装、ログ配信、パラメータ編集などの機能追加。
- 外部ネットワークからの取得(CDN、Web フォント、地図タイル)と、ビルド工程を要するフロントエンド構成。
- `colcon_ws` 配下の変更。

## コーディング規約

- Python 3、型ヒントあり、標準ライブラリ優先。コメントは日本語でよい。
- JavaScript は ES modules、依存なし。状態とDOM更新を分け、ツリー構築・値の解釈は DOM に触れない純関数にする。
- 例外を握りつぶさない。ユーザー操作の失敗は HTTP ステータスで返す。
- 新しい依存を足すときは理由を README に1行書く。
