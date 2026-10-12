# cub_console

ROS 2 の診断(`/diagnostics_agg`)をブラウザで見やすく表示する Web コンソール。仕様は [CLAUDE.md](CLAUDE.md)。

**プロセス管理(起動/停止/再起動)は枠組みを検討中のため、いったんアプリから外している**(下の「プロセス管理(休止中)」参照)。

ホストの `docker/home/cub_console/` が、コンテナ内の `/home/cub/cub_console/` になる(colcon ビルドの対象外)。

## セットアップ(初回のみ・要インターネット)

```bash
# ホストのリポジトリ直下から
./docker/run_in_container.sh 'cd ~/cub_console && ./setup.sh'        # 実行用
./docker/run_in_container.sh 'cd ~/cub_console && ./setup.sh --dev'  # テストも行う場合
```

`.venv` は `docker/home` 側に作られるので、コンテナを作り直しても残る。
今のイメージには `python3-venv`(ensurepip)が無いため、`setup.sh` は pip 無しで venv を作ってから get-pip.py で pip を入れる。

## 起動(launch)

ロボット本体とは別に起動する:

```bash
ros2 launch cub_bringup common.launch.py      # ロボット本体
ros2 launch cub_bringup cub_console.launch.py # cub_console(別端末で)
```

`cub_bringup/launch/cub_console.launch.py` が `~/cub_console/run.sh` を起動する(アプリ本体は colcon ビルドの対象外。launch ファイルを追加・変更したときだけ `cbs cub_bringup` が必要)。

| 引数 | 既定 | 意味 |
|---|---|---|
| `host` | `0.0.0.0` | 待受アドレス。`127.0.0.1` にするとこのマシンからのみ |
| `allow` | `auto` | 接続を許可する CIDR。`auto` は起動時点の LAN インタフェース(Docker のブリッジ等を除く)のサブネット |
| `port` | `8080` | 待受ポート |
| `console_dir` | `~/cub_console` | アプリの場所 |

- Ctrl+C で止まる。アプリは `setsid` で別セッションにしてあり、launch からの SIGINT 1 回だけで正常終了する(端末の SIGINT と二重に届くと uvicorn が後始末を飛ばして強制終了するため)。
- `allow:=auto` は起動時点のアドレスで決まる。Wi-Fi がつながる前に起動すると LAN が見つからず起動に失敗する(その場合は起動し直すか、`allow:=192.168.63.0/24` のように明示する)。Jetson では USB 接続用の `l4tbr0`(192.168.55.0/24)も対象になる。

## 起動(直接)

```bash
./docker/run_in_container.sh 'cd ~/cub_console && ./run.sh'
```

ブラウザで <http://127.0.0.1:8080/> を開く。コンテナは `network_mode: host` なので、`127.0.0.1` で待ち受ければホストからそのまま見える(LAN には出ない)。

| 環境変数 | 既定 | 意味 |
|---|---|---|
| `CUB_CONSOLE_HOST` | `127.0.0.1` | 待受アドレス |
| `CUB_CONSOLE_PORT` | `8080` | 待受ポート |
| `CUB_CONSOLE_ALLOW` | (空) | localhost 以外に接続を許す CIDR(空白/カンマ区切り)。`auto` で LAN のサブネット。`127.0.0.1` 以外で待ち受けるときは必須 |

### スマホ・別 PC から見る(同じサブネットだけ許可)

```bash
./docker/run_in_container.sh 'cd ~/cub_console && CUB_CONSOLE_HOST=0.0.0.0 CUB_CONSOLE_ALLOW=192.168.63.0/24 ./run.sh'
```

`http://<ロボットの IP>:8080/` を開く。**認証は無い**ので、許可したサブネットにいる人は誰でも画面を見られる。信頼できるネットワークでだけ使うこと。

- 許可していない接続元は 403(WebSocket は切断)。localhost は常に許可。
- POST と WebSocket は、ブラウザの `Origin` が `Host` と一致するときだけ通す(他のサイトを開いたブラウザ経由で操作 API を叩かれないように)。
- `X-Forwarded-For` は信用しない(接続元の偽装を防ぐ)。
- `CUB_CONSOLE_ALLOW` なしで `127.0.0.1` 以外を指定すると起動しない。

LAN に出したくないときは、待受は変えずに SSH ポートフォワードを使う: `ssh -L 8080:127.0.0.1:8080 <robot>`

## 動作確認(実機なし)

```bash
# 端末 1: フィクスチャを /diagnostics_agg に 1 Hz で流す(--jitter で値を揺らす、--stop-after N で途絶を再現)
./docker/run_in_container.sh 'cd ~/cub_console && .venv/bin/python tools/fake_diag_pub.py --jitter'
# 端末 2
./docker/run_in_container.sh 'cd ~/cub_console && ./run.sh'
```

ロボット本体と同じ LAN で試すときは、両方に `export ROS_DOMAIN_ID=<空いている番号>` を付けて本物の診断と混ざらないようにする。

## テスト

```bash
./docker/run_in_container.sh 'cd ~/cub_console && .venv/bin/python -m py_compile app/*.py && .venv/bin/python -m pytest -q'
# node があれば(コンテナには無いので、ホストで docker を使う例)
docker run --rm --network none -v "$PWD/docker/home/cub_console":/w:ro -w /w node:22-alpine node --test tests/*.test.mjs
```

Node 22 では `node --test tests/`(ディレクトリ指定)は動かないので、ファイルを glob で渡す。

## フィクスチャ

- `tests/fixtures/diag_agg_sample.yaml` … **今は合成データ**(仕様の「診断データの実態」に合わせて作ったもの。葉 19 件、STALE 3 件)。実機で `ros2 topic echo /diagnostics_agg --once > tests/fixtures/diag_agg_sample.yaml` を保存して置き換えること(GPS 座標は公開前に差し替える)。
- `tests/fixtures/diag_agg_local_example.yaml` … 開発機で `cub_diagnostics example.launch.py` を動かして取った実データ。
- node のテストは JSON を読む。YAML を変えたら `python3 tools/fixture_to_json.py <yaml> > <json>` で再生成する(pytest が不一致を検出する)。
- 置き換えると `diagtree.test.mjs` の件数の期待値(葉の数・STALE 3 件・グループ名)が合わなくなる場合がある。その場合は実データに合わせて直す。

## プロセス管理(休止中)

API(`/api/processes`)と画面のパネルは外してある。`app/processes.py`(`ProcessBackend` / `SubprocessBackend`)と `config/managed_processes.yaml`、`tests/test_processes.py` は再開時のために残しており、バックエンド単体のテストは引き続き走る。

設計メモ: `config/managed_processes.yaml` の id だけが操作できる。`cmd` は配列のみ(シェルを通さない)。相対パスの `cwd` と、`cmd` 中の相対パスは本ディレクトリ基準。

- `demo_talker` … コンテナに `demo_nodes_cpp` が無いので、同梱の `tools/demo_talker.py`(ノード名 `/demo_talker`)を使う。
- `diag_example` … `ros2 launch cub_diagnostics example.launch.py`。子プロセスを持つので、停止で子まで消えることの確認に使える。
- 停止は プロセスグループへ SIGINT → 8 秒 → SIGTERM → 3 秒 → SIGKILL。本アプリを止めると管理下のプロセスも止まる(開発用の道連れ動作)。

## 実測

2026-10-12、**Raspberry Pi 5(4 コア, 8 GB)上の開発用コンテナ**で計測(Jetson Orin では未計測)。フィクスチャ(status 22 件)を 1 Hz で流した状態で `top -b -d 30 -n 2 -p <pid>` の 2 回目の値。

| 状態 | CPU | RSS |
|---|---|---|
| クライアント接続なし | 0.4 % | 96 MiB |
| WebSocket クライアント 1 つ | 0.4 % | 96 MiB |

1 メッセージの JSON は約 12.4 KB。クライアントには診断 1 Hz + heartbeat 1 Hz が届く。

## 仕様(CLAUDE.md)からの変更

- 制約 4「認証を入れるまでは 127.0.0.1 のみ」に対し、スマホから使うために `CUB_CONSOLE_ALLOW` による接続元の許可リストと Origin チェックを追加した(既定は従来どおり 127.0.0.1 のみ)。

## 依存を足した理由

- `fastapi` / `uvicorn[standard]` … REST + WebSocket + 静的配信を 1 プロセスで行うため(`[standard]` は WebSocket 実装と uvloop)。
- `pyyaml` … 設定ファイルとフィクスチャの読み込み。
- `pytest` / `httpx`(開発用)… テストと FastAPI の TestClient。
