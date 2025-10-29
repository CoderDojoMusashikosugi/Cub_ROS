README — PLY to PGM Converter Tool
概要

本ツールは 大規模な屋外 PLY 点群 を 2D Occupancy Grid (PGM + YAML) に変換するための GUI アプリケーションです。
Open3D の GUI 機能をベースにしており、矩形範囲選択・平面推定・バンド除去などをインタラクティブに行えます。

主な用途は ROS2 (Nav2) に入力可能な地図生成です。

主な機能
1. 領域選択

Legacy pick (VisualizerWithEditing)
サブプロセスで矩形を指定し、2点を返す方式。
→ 大規模点群に対しても安定。

Plane pick (Scene 内クリック)
平面を推定後、任意の位置をクリックして平面との交点を 2 点取得。
→ クリックで矩形を確定でき、斜面の道路でも正確にバンドを選択可能。

2. 平面推定

領域選択後、その範囲内点群から平面を推定。

RANSAC による平面分割、失敗/低インライア時は SVD でフォールバック。

法線はカメラから見て表向きになるよう自動反転。

3. バンド選択 & 点群除去

推定平面を基準に、上下にオフセットを持つ「バンド」を設定。

Min/Max Z で高さ制限も可能。

「Apply Remove」でバンド内の点群を削除。

4. ラスタライズ & PGM 生成

バンド内点群を 2D Occupancy Grid に変換。

解像度（m/pixel）、膨張半径を調整可能。

「Apply Rasterize」でグリッドに反映。

Undo 機能で直前の操作を戻せます。

5. PGM プレビュー

「Open PGM Preview」で別ウィンドウに生成結果を表示。

2回目以降も安定して開閉可能。

「Save PGM+YAML」でファイル保存。

UI の構成

左パネル: パラメータ入力

Voxel size (表示用)

Resolution (PGM 解像度)

Inflate radius (セル膨張半径)

Band lower/upper

MinZ / MaxZ

ボタン群:

Pick 2 corners (legacy)

Estimate Plane from Region

Pick 2 corners on plane

Apply Remove

Apply Rasterize

Undo Last

Save PGM+YAML

Open PGM Preview

操作フロー（推奨）

Pick 2 corners (legacy)
→ 点群内の矩形領域を指定

Estimate Plane from Region
→ 平面を推定し表示（法線・距離付きログ出力）

Pick 2 corners on plane
→ 平面上で任意の矩形を選択（必要に応じて）

Apply Remove または Apply Rasterize
→ 領域点群を除去 or Occupancy Grid に変換

Open PGM Preview
→ 生成された PGM を確認

Save PGM+YAML
→ 出力ファイルを保存

エラーメッセージ・注意事項

plane is behind the camera.
→ クリック点と平面の交差がカメラ後方にある場合。数値誤差も含む。

Select region first.
→ 領域が未確定の状態で処理を実行。必ず 2 点を指定してから実行すること。

[entity=.. missing required attributes]
→ Open3D の描画エンジン警告。動作に支障はない。

[Open3D WARNING] 'defaultUnlitLine' is not a valid shader
→ Open3D 0.19 では defaultUnlit を使う必要あり。

動作環境

Python 3.8+

Open3D 0.19 (推奨)

NumPy

既知の制約

Open3D 0.19 では set_on_point_picked が未実装 → legacy pick を利用。

点群が非常に大きい場合は初期読み込み・描画が数分かかる。

Shader 警告は表示されるが無害。