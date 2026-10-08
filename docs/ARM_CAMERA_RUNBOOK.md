# UR10e手先カメラ：Day 4.5a — 固定取付と2地点のRGB撮影

UR10eのフランジにRGBカメラを固定し、アームでカメラを移動して2枚撮影します。今回はカメラ取付・座標変換・撮影の独立確認です。撮影対象には色付き確認ボードを使い、A/B部品の検査ループへの接続は次の工程で行います。

## 実行手順

前のIsaac Simウィンドウを通常の操作で閉じてから実行します。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/arm_camera_check.py --output-dir outputs/arm_camera_01
```

既存の出力先には上書きしません。再実行時は `arm_camera_02` など、新しい名前にしてください。

入力のStep 3レポートは次の順に探し、実際に使ったパスを `STEP 3 EVIDENCE` に表示します。通常出力が存在して失敗している場合、開発時の成功結果に切り替えて進行することはありません。

1. `outputs/arm_step3_01/arm_motion_report.json`
2. 上が存在しない場合、前工程で確認済みの `outputs/arm_step3_codex_gui_01/arm_motion_report.json`

別のStep 3出力を使う場合は明示します。両方の既定レポートがない環境では、Step 3を実行するか、成功レポートのパスを指定してください。

```bash
python -u src/arm_camera_check.py --step3-report outputs/arm_step3_02/arm_motion_report.json --output-dir outputs/arm_camera_02
```

## 画面とターミナルで確認すること

1. UR10e、手先のカメラ表示形状、色付きボードが表示されます。標準の画面はロボット全体を見る外部カメラです。
2. アームが1地点目へ動き、静止後に手先カメラで撮影します。
3. 続いて2地点目へ動いて撮影します。
4. 結果保存後は最後の姿勢で画面が残ります。確認後にウィンドウを閉じてください。

**Play操作は不要です。** 移動中はStep 3と同じ手動物理ステップを使い、撮影中は物理を進めずに描画します。タイムラインの表示時刻は、撮影時に物理スナップショットの時刻へ合わせるため更新されます。

```text
STEP 3 EVIDENCE | .../arm_motion_report.json
WRIST MOVE | view_01
WRIST CAPTURED | view_01 | position_m=... | physics_s=...
WRIST MOVE | view_02
WRIST CAPTURED | view_02 | position_m=... | physics_s=...
REPORT | .../arm_camera_01/arm_camera_report.json
WRIST CAMERA PASS | captures=2
```

手先映像は保存したPNGを開いて確認します。GUIで `WristCamera` を操作用カメラに選ぶと、ビューポートのナビゲーションが取付姿勢を書き換える場合があるため、今回の実行中は外部カメラの表示を使ってください。取付姿勢が変わった場合はエラーを検出して停止します。

GUIを使わずに撮影する場合は次のコマンドです。

```bash
python -u src/arm_camera_check.py --headless --output-dir outputs/arm_camera_headless_01
```

結果保存後にGUIを自動終了する場合は `--exit-after-capture` を追加します。

## 色付きボードと画像の意味

赤・緑・青・黄のタイルは、カメラが違う位置から同じ固定ボードを撮ったことを目で確認するための目印です。R1/R2の検査領域、可視判定の色、OK/NG判定ではありません。画像への色の上書き処理も行っていません。

`images/view_01.png` には黄・青のタイルが大きく写り、`images/view_02.png` には緑・赤が大きく写る想定です。ボードの端の見え方・向きも変わります。影にはロボットの影が含まれます。色や影の違いは検査の合否を示す情報ではありません。

## 出力ファイルの読み方

| ファイル | 内容 |
|---|---|
| `images/view_01.png` / `view_02.png` | 実際の手先RGBカメラから取得した640×480画像 |
| `arm_camera_report.json` | 全体の成否、入力レポート、使用モデル、制御設定、各移動の結果、確認項目 |
| `camera_poses.json` | 各画像に対応する目標・実カメラ姿勢、手先姿勢、取付変換、物理時刻、画像参照時刻 |
| `arm_camera_events.jsonl` | 取付、IK、各物理ステップ、カメラ目標到達、撮影の履歴。1行が1イベント |

まず `arm_camera_report.json` の `run_status: passed` と、`checks` がすべて `true` であることを確認します。

| 項目 | 平易な説明 |
|---|---|
| `step3_report` | 入力に使った成功レポートのパスとハッシュ |
| `camera.prim_path` / `render_camera_paths` | 手先カメラの場所と、RGBを描画するカメラの接続先。外部カメラの画像と取り違えないための確認 |
| `movements[]` | 各カメラ目標をフランジ目標へ変換し、IK・関節制御で実行した記録 |
| `checks.two_controlled_camera_goals_reached` | 両方のカメラ目標に到達したか |
| `checks.two_fresh_distinct_images` | 新しい参照時刻を持つ、異なる2画像を保存したか |
| `checks.fixed_mount_and_measured_camera_pose` | 取付変換と実カメラ姿勢の照合を通ったか |
| `checks.physics_held_during_both_captures` | 撮影中に物理時間・ステップ数が変わらなかったか |
| `image_comparison` | 2画像の平均画素差と変化した画素の割合。検査カバレッジではない |
| `scene.inspection_evaluated` | 今回は `false`。撮影できたことと検査完了を区別する記録 |

`camera_poses.json` の `captures[]` は、1要素が1枚の画像に対応します。

| 項目 | 平易な説明 |
|---|---|
| `image` | 対応する画像ファイル |
| `T_world_part` | ボード上面中心の基準座標。今回はこの確認ボードを `part` 座標として使用 |
| `T_part_camera` | ボードを基準とした実撮影姿勢 |
| `T_world_camera_target` / `T_world_flange_target` | 要求したカメラ姿勢と、そこから変換した手先姿勢 |
| `T_world_camera` | USDの手先カメラから読み戻した実撮影姿勢 |
| `T_world_flange` | PhysXから読み戻した実際の手先姿勢 |
| `T_flange_camera_nominal` / `T_flange_camera` | 設計した取付変換と、カメラのローカル座標から読み戻した値 |
| `T_world_camera_from_physics` | 実手先姿勢と実取付変換を合成した姿勢。USDの実カメラ姿勢と照合する |
| `camera_pose_error` | 要求したカメラ位置・向きと実撮影姿勢の差 |
| `mount_error` / `physical_attachment_error` | 取付変換の差と、物理手先から求めた姿勢・USDカメラ姿勢の差 |
| `physics_time_seconds` / `physics_step_index` | 撮影時に保持した物理の時刻・ステップ番号 |
| `render_reference_time` | 画像と一緒に取得したレンダラー参照時刻。分子÷分母で秒になる |
| `render_step_index` | 同期描画を要求した回数 |
| `readback_utc` | 画像を読み取ったホストのUTC時刻。露光開始時刻ではない |

## 取付と撮影の仕組み

カメラは `/World/UR10e/wrist_3_link/flange/WristCamera` に作成します。フランジの子要素として、開始時に次のローカル変換を設定します。

```text
T_flange_camera =
[[0, -1, 0, 0.18],
 [1,  0, 0, 0.00],
 [0,  0, 1, 0.08],
 [0,  0, 0, 1.00]]
```

フランジからX方向へ18 cm、Z方向へ8 cm離し、Z軸まわりに90度回しています。カメラはUSD光学座標（+Xが画像右、+Yが画像上、-Zが見る方向）です。位置はメートル、角度誤差・関節角はラジアン、行列は列ベクトルに作用します。

`T_world_camera_target = T_world_part @ T_part_camera_target` とし、`T_world_flange_target = T_world_camera_target @ inverse(T_flange_camera)` を計算します。さらに実ベース座標へ変換してIKへ渡します。カメラ自体をワールド座標で移動する操作や、関節を目標へ瞬間移動する操作は行いません。

カメラ目標の位置5 mm・向き1度以内への到達と、Step 3の関節・静止条件を確認した後、物理を止めたまま `rep.orchestrator.step(delta_time=0.0, wait_for_render=True)` で描画します。画像参照時刻が保持した物理時刻に一致し、前回より新しいことを確認します。撮影中の手先・カメラのずれも確認します。

取付変換の数値照合は位置 `1e-6 m`・向き `1e-6 rad`、物理手先とUSDカメラの照合は `1e-5 m`・`1e-5 rad` 以下です。RTX APIの数値丸めを含めた確認閾値で、実ロボットの取付精度を保証する値ではありません。実行時の値はJSONの `evidence_tolerances` に保存します。

## 完了条件・失敗時・今回の制限

- 2地点への移動後、それぞれの手先RGB画像が保存され、最後に `WRIST CAMERA PASS | captures=2` が出る。
- `run_status` が `passed`、`checks` がすべて `true`。画像・実姿勢・時刻の対応を目で確認する。
- 目標へ到達しない場合は `blocked` で停止し、撮影・次の移動へ進みません。静止していない手先、取付の変更、画像時刻のずれ、保存エラーも成功扱いしません。
- `Step 3 must pass` や入力・モデル変更のエラーでは、使用レポート・バージョン・ハッシュを確認します。`File exists` では新しい出力先を指定します。終了コードは成功0、目標未完了2、実行・保存エラー1、中断130です。
- 2地点は確認済みStep 3フランジ目標から作ったカメラ目標で、検査プランナーの判断結果ではありません。
- カメラ・ブラケット・ボードは表示用形状です。カメラ搭載質量、取付部やボードとの衝突、障害物回避は未検証です。XRDFの衝突球・障害物モデルも後続工程で扱います。
- `FreeCameraExecutor` と `ArmMountedCameraExecutor` の共通インターフェースを追加しました。既存Day 1–4の入口は変更しておらず、A/B・X/Yの比較と観測更新への接続は未完了です。

ここでユーザーの画像・画面・JSON確認を待ちます。次はA/B検査シーンへの接続、カメラ候補の到達性、アーム形状を含む遮蔽評価を進め、既存仮想カメラ方式と比較する工程です。

## 開発時の確認（2026-10-08）

- ヘッドレス：`outputs/arm_camera_codex_02/arm_camera_report.json`。最終GUI確認：`outputs/arm_camera_codex_gui_02/arm_camera_report.json`。両方とも2画像保存と全確認項目に成功しました。
- 実カメラの位置誤差は約0.88 mm／0.70 mm、向きの誤差は約0.000864 rad／0.000598 rad。撮影時の物理時刻は約3.258秒／6.508秒で、各撮影中に変化していません。
- 両画像で取付変換の差は約 `1.21e-7 m`、`3.42e-8 rad`。2地点とも同じ値で、数値丸めの範囲内です。
- 画像の違いとボードの写り方を確認済みです。GUIとヘッドレスでは描画の画素差があり、画像の完全なビット一致は保証しません。
- 手先カメラをGUI操作用の主カメラに指定する試行では、取付位置の変更を検出して成功扱いせず停止しました。この経路は今回のCLIから除き、外部視点でアームを表示する構成を最終GUI実行で確認しています。
- 既存を含む119テスト成功。座標変換、実手先と実カメラの照合、取付変更、撮影中の移動、未静止、古い画像時刻、入力失敗、出力上書き防止、保存失敗を確認しました。
- 使用APIはインストール済みIsaac Sim 6.1の `RtxCamera` / `CameraSensor`、USD `Camera` / `Xformable`、Replicator `orchestrator.step()` / `ReferenceTime` の公開実装と同梱テストで確認しました。
- 自動生成物はGitに含めず、コード・手順のみローカルコミットします。pushはユーザーが行います。
