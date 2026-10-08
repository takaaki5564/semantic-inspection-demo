# Day 1 — 2視点のRGB撮影

## 確認済みベースライン

2026-10-08、ユーザーが既存 `semantic_nav_demo/src/main.py` を実行し、
GUI表示・青い立方体の移動・`MISSION COMPLETE` を確認した。
既存デモと `/home/ishii/work/isaac/env_isaac` は変更していない。

- Ubuntu 24.04、GeForce RTX 4080、ドライバ595.91.07。
- Python 3.12.3、Isaac Simパッケージ6.1.0.0。
- インストール内VERSION: `6.1.0-rc.26+release.49347.2d230af4.gl`。
- 依存追加なし。既存のNumPy・Pillowを利用する。

## 実行

既存デモのウィンドウを閉じ、Ubuntuデスクトップ上の端末で実行する。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/day1_capture.py
```

板・低いリブ・緑のR1マーカー・橙のR2マーカーが表示される。
最初の撮影後、カメラが約3秒で反対側へ移動し、2回目の撮影を行う。
Playボタンの操作は不要。端末に次のメッセージが出る。

```text
DAY1 SCENE READY | R1=green, R2=orange | simulator markers
CAPTURED view_01 | position_m=...
MOVING CAMERA TO VIEW 02
CAPTURED view_02 | position_m=...
DAY1 CAPTURE COMPLETE | .../outputs
```

完了後もGUIを維持する。ウィンドウを閉じて終了する。
出力先が既に存在する場合は、起動前に停止し、上書きしない。
再実行には新しい保存先を指定する。

```bash
python -u src/day1_capture.py --output-dir outputs/run_02
```

撮影後の自動終了とGUIなしでの確認も可能。

```bash
python -u src/day1_capture.py --exit-after-capture --output-dir outputs/run_auto
python -u src/day1_capture.py --headless --output-dir outputs/run_headless
```

## 成果物の確認

```text
outputs/images/view_01.png
outputs/images/view_02.png
outputs/camera_poses.json
```

2枚を画像ビューアで開き、同じ部品が異なる方向から撮れていることを確認する。
保存データの自動確認にはIsaac Simの起動は不要。

```bash
python src/verify_day1.py
python src/verify_day1.py outputs/run_02
python -m unittest discover -s tests -v
```

受入条件は、2枚の異なる640×480 RGB画像、異なる実カメラ姿勢、
進行する描画参照時刻、座標変換の整合性、同じ起動コマンドによる再実行。
自動確認は画素の違いを測るものであり、構図の妥当性は目視で確認する。

## 座標・時刻・撮影の対応

- 世界・部品座標は右手系、Z-up、単位はメートル。
- 部品原点は板の底面中心。世界への配置は `(0, 0, 0.25)`。
- カメラはUSD光学系: Xは右、Yは上、視線方向は-Z。
- JSONは行ごとに配列化した4×4行列。計算は列ベクトル。
  `T_world_camera = T_world_part @ T_part_camera`。
- USD/Gfの行ベクトル行列を読み戻す際に転置する。
- カメラ姿勢はUSDの実変換を読み戻し、行列とwxyzクォータニオンを記録。
- 描画は `rep.orchestrator.step(wait_for_render=True)` で同期する。
  同梱テストに従って撮影前にtimelineのplayをcommitし、撮影後は一時停止する。
  各撮影中は部品・カメラを固定し、前後の姿勢が一致することを確認する。
- `render_reference_time` は同じRenderProductに接続した `ReferenceTime`
  annotatorによる有理数。UTCや独自のフレーム番号として扱わない。
- `readback_utc` はRGB取得後のホスト時刻。露光時刻ではない。
- `timeline_seconds_at_readback` と `app_update_count` は補助情報。
  後者はadapterが呼んだupdate回数で、内部描画回数ではない。
- 最大10回の同期撮影を試し、有効で新しいRGBが得られなければエラーにする。

## API選定と範囲

インストール済みの `isaacsim.sensors.experimental.rtx` の実装・テストを確認し、
`RtxCamera`、`CameraSensor` とRGB annotatorを利用する。
拡張機能はこのプロセスの起動引数で有効にする。
640×480の撮影とRaytracedLightingを使用し、外部USDアセットは取得しない。

領域の色はシミュレータ上の識別マーカーで、画像認識結果ではない。
撮影状態は `captured`。可視性・検査完了・欠陥検出・プランナ・アーム制御は
未実装で、Day 2以降の承認を得て進める。

GPUに接続できない実行セッションでは、仮想環境を有効にしてもGPUアクセス権は
変わらない。デスクトップ端末での `nvidia-smi` と比較し、
正常に動く既存環境を再インストールしない。

## 実装時の確認結果（2026-10-08）

- `python -m unittest discover -s tests -v`: 14テスト成功。
  座標の向き、回転・平行移動した部品、古い描画時刻、同一画像、変換の不整合、
  上書き防止、保存PNGの検証、失敗時の非ゼロ終了コードを確認。
- GPUでheadless撮影とGUI撮影（`--exit-after-capture`）がともに終了コード0。
  それぞれ2枚のPNGとJSONを保存し、`verify_day1.py` がPASS。
- headless確認先: `/tmp/nedo-day1-validation-20261008-03`。
- GUI確認先: `/tmp/nedo-day1-validation-20261008-gui-01`。
- 画像を読み、同じ板・リブ・領域マーカーが異なる方向から写ることを確認。
- 実カメラ位置は世界座標で `(0.85, -1.05, 1.0)` と `(0.85, 1.05, 1.0)` m。
- GUI撮影の平均絶対画素差は約24.460（0–255画素値）。検査精度を表す値ではない。
- 既存 `main.py`、`robot_controller.py`、`pyvenv.cfg` のSHA-256は作業前後で一致。
- 手動でGUIを閉じる終了経路の目視確認はユーザー側で行う。

エラー時は `DAY1 CAPTURE FAILED` と例外を表示し、
`SimulationApp.close(exit_code=1)` で終了コードを保持する。
インストール済み版のfast shutdownでは、引数なしのcloseが例外を隠して
終了コード0にするため、プロセス終了コードだけで撮影成功と判定しない。
失敗した保存先には部分的な画像が残ることがある。再実行は新しい保存先を使用する。
