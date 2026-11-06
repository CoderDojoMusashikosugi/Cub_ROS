# Cub4の表示モデル

ホストが操作する主ロボットだけに使います。クライアントから見た主ロボットも同じCub4を表示し、クライアント自身のロボット・通行人・コーンは従来の簡易モデルを使います。

元データは以下の3ファイルです。元データの変更は行わず、Godotプロジェクト内に生成済みの`cub4.glb`を同梱しています。通常の起動にPython・Blender・ROSは不要です。

- [cub4.urdf](../../../ros/cub_description/urdf/cub4.urdf)：表示用メッシュ、縮尺、原点と回転
- [cub4-comp.obj](../../../ros/cub_description/meshes/cub4-comp.obj)：形状と法線
- [cub4-comp.mtl](../../../ros/cub_description/meshes/cub4-comp.mtl)：拡散色

URDFの`base_link`のvisualにある縮尺`0.1 0.1 0.1`、回転`1.57 0 1.57`、原点`0 0 0`を反映し、ROS座標`(x, y, z)`をGodot座標`(-y, z, -x)`に変換しています。Godotの前方は`-Z`です。外形寸法は幅約0.254 m・高さ約0.594 m・長さ約0.378 mで、底面は`base_link`から約6 mm上です。

元の183,889三角形を保持し、色を頂点色へまとめた1メッシュ・1サーフェスのGLB（約5.2 MiB）にしています。外部テクスチャはありません。GodotのインポートでLODを生成し、[import_cub4.gd](import_cub4.gd)で頂点色の表示を有効にして影を無効にします。接地・段差乗り越えは既存の簡易カプセルを使います。車輪の回転や実機の衝突形状の再現はこの表示モデルに含みません。

元データを更新した場合は、リポジトリのルートで次を実行してください。Pythonの標準ライブラリだけを使います。

```bash
python3 simulator/tools/build_cub4_model.py
```

続いてGodotエディタで再インポートするか、`--headless --editor --path simulator --import --quit`を実行します。生成される`cub4.json`には元データのSHA-256・変換条件・外形を記録します。`cub4.glb`・`cub4.json`・`cub4.glb.import`とインポートスクリプトを一緒に管理し、`.godot/`のキャッシュは含めません。この変換スクリプトは現在のCub4用で、任意のURDFを読み込む機能はありません。
