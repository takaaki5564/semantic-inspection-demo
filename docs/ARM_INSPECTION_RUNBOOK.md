# UR10e A/B検査シーン：Day 4.5b — 到達性とアームを含む可視性

色付き確認ボードを、Day 2–4と同じ低リブA／高リブB、検査領域R1／R2に差し替えます。アームで手先カメラを前方・後方へ動かし、実際の画像・撮影姿勢・その時点のアーム形状から観測状態を更新します。

今回は**明示した2方向を撮る独立確認**です。自動プランナーによる撮影方向の選択と仮想カメラ方式との比較は、出力を確認した後の工程です。既存Day 1–4のコマンドと既定値は変更していません。

## 最初にB・仕様Yを実行

前のIsaac Simウィンドウを通常操作で閉じてから実行してください。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/arm_inspection_check.py --part B --spec Y --output-dir outputs/arm_inspection_B_Y_01
```

Play操作は不要です。アームが前方へ移動して静止・撮影し、続いて後方へ移動して撮影します。GUIはアーム全体を見る外部カメラです。**操作用カメラにWristCameraを選ばず**、手先画像は保存PNGを開いて確認してください。保存後は最後の姿勢で画面が残るので、確認後に閉じます。

```text
CANDIDATE | front | feasible | bounded_ik_solution
CANDIDATE | rear | feasible | bounded_ik_solution
CANDIDATE | high | blocked | ik_failed
ARM INSPECTION MOVE | B/Y/front
ARM INSPECTION CAPTURED | front | R1=63/63, R2=3/63 | arm_occlusion={'R1': 0, 'R2': 0}
ARM INSPECTION MOVE | B/Y/rear
ARM INSPECTION CAPTURED | rear | R1=0/63, R2=63/63 | arm_occlusion={'R1': 0, 'R2': 0}
REPORT | .../arm_inspection_B_Y_01/arm_inspection_report.json
ARM INSPECTION PASS | captures=2 | observation=inspection_observation_satisfied
```

上は開発時の実測例です。観測数はシーンと実カメラ姿勢から計算し、成功値として埋め込んでいません。`high`の除外は全体の失敗ではなく、実行しない候補を記録したものです。`ik_failed`は今回の有限個の初期値で解を確認できなかったという意味で、数学的な到達不能の証明ではありません。

最初に次を確認します。

1. `images/front.png`：Bの高いリブが奥のR2を隠している。
2. `images/rear.png`：後方からR2が見える。
3. `arm_inspection_report.json`：`run_status`が`passed`、`checks`がすべて`true`。
4. `final_state.regions`：R1とR2の累積観測数。後方画像でR1が見えなくても、前方で観測した記録は残る。

出力先は上書きしません。再実行には末尾を`_02`などへ変えます。GUIを自動終了するには`--exit-after-capture`、GUIを使わない場合は`--headless`を追加します。関節移動の時間上限は各方向20秒のシミュレーション時間で、静止確認を含みます。

## 同じ工程でAを確認する場合

Bの出力を確認してから、同じ配置・同じカメラ候補でAを実行できます。

```bash
python -u src/arm_inspection_check.py --part A --spec Y --output-dir outputs/arm_inspection_A_Y_01
```

Aは低いリブなので、開発時は前方からR1・R2とも63/63点見えました。今回の診断スクリプトはAでも2方向撮影します。`--spec X`はR1だけを評価しますが、こちらも2方向撮影します。必要な観測が揃った時点で追加撮影を止める制御は次の閉ループ接続で確認します。

## 画像と色の意味

| 画像 | 意味 |
|---|---|
| `front.png`／`rear.png` | 手先RGBカメラの画像。可視性の描画を加えていない原画像 |
| `front_visibility.png`／`rear_visibility.png` | 原画像に、各領域の表面サンプル点を重ねた説明画像 |

可視性画像の**緑点は今回の画像で観測可能、赤点は今回の画像で観測できない点**です。赤点は遮蔽物の向こうにある本来の検査点を画像へ投影するため、リブなどの上に重なって表示されます。元の緑色／橙色マーカーとは別の情報です。画像上部の数値はその1枚で見える点数で、累積観測数はJSONの`final_state`を読みます。

この点の判定はRGB画像からの認識ではなく、シミュレーターの既知形状・光学系からのレイ判定です。緑点は傷なしや検査合格を意味しません。

## 出力ファイルの読み方

| ファイル | 内容 |
|---|---|
| `arm_inspection_report.json` | 入力・配置・候補の到達性、各移動、画像と実姿勢、可視性、累積観測、確認項目 |
| `arm_inspection_events.jsonl` | 到達性確認、IK、関節制御、各物理ステップ、撮影・観測更新の履歴。1行が1イベント |
| `geometry/front.json`／`rear.json` | 各撮影時点の実シーンの三角形、領域サンプル、実カメラ光学系、時刻・ハッシュ。可視性の再計算用 |
| `images/*.png` | 原画像2枚と可視性説明画像2枚 |

`arm_inspection_report.json`の主な項目は次の意味です。この工程では画像の姿勢情報も同じレポートに含め、別の`camera_poses.json`は作りません。

| 項目 | 平易な説明 |
|---|---|
| `part`／`spec`／`required_region_ids` | 部品の形状と検査仕様。XはR1、YはR1＋R2 |
| `cell`／`viewpoints` | 使った配置と部品基準のカメラ候補。その内容・ファイルのハッシュ |
| `reachability[]` | 移動前のIK確認。解・関節範囲・姿勢残差を確認できた候補だけ`feasible`。この時点では移動していない |
| `movements[]` | 実行時に再度IKを解き、実手先姿勢・関節角・速度から到達と静止を確認した記録 |
| `captures[].capture` | 対応する画像名、目標／実カメラ姿勢、実フランジ姿勢、取付変換、誤差、画像時刻・物理ステップ |
| `captures[].capture.rgb_array_sha256` | 取得したRGB画素配列のハッシュ |
| `captures[].visibility.regions` | 各画像で見える点・不足する点と、最初に当たった面の位置・パス |
| `captures[].visibility.robot_occluded_point_ids` | アーム、ブラケット、カメラ表示形状に隠された点。理由は`scene_occlusion`、具体的な相手は`hit.prim_path` |
| `captures[].visibility.geometry_snapshot` | 可視性に使ったシーンと撮影時点を結び付ける情報 |
| `part_geometry_version` | 部品だけの形状識別情報。アームが動いても変わらず、A/Bを切り替えると変わる |
| `static_scene_sha256` | 固定セルと部品の形状・配置。アーム移動前後で変わらないことを確認 |
| `full_scene_sha256` | アームを含む全形状の配置。撮影中は固定、2方向の撮影間では変わることを確認 |
| `physics_step_index`／`physics_time_seconds` | この形状と画像を取得した同じ物理時点 |
| `final_state` | 撮影を通じて観測した点の累積。`inspection_observation_satisfied`は必要な表面サンプルが観測できたという意味 |
| `planner_executed`／`defect_decision` | 今回は`false`／`not_evaluated`。自動撮影方向選択と傷の合否判定は行っていない |

## 配置・IK・形状判定の方針

`config/arm_inspection_cell.json`で部品基準をワールド`(-0.6, 0, 0.25) m`に固定します。元の位置では既存の撮影候補のIKが解けなかったための、A/B共通のセル配置です。部品形状、領域、`config/camera_views.json`の部品基準カメラ候補は変更していません。アームの基準位置は前工程と同じ`(0, 0, 0.4) m`です。

現在関節角だけでは局所的なIK失敗があったため、設定に記載したA/B共通の数値初期値も試します。得られた解は参照関節姿勢への角度距離、現在関節角への距離の順で選びます。これは部品名や検査仕様による分岐ではなく、ロボット側の解の選び方です。初期値を物理関節へ設定する操作は行いません。各撮影前に実カメラ目標への到達・静止・固定取付を確認します。

形状は撮影ごとにUSDから読み直します。UR10eの表示用Meshをインスタンス内部まで読み、`guide`／`proxy`や非表示の形状を除外します。現在のアセットは細分化なしの三角形Meshで、7個・8,638三角形です。床・台・部品・取付表示形状を合わせて8,746三角形をレイ判定に使います。未対応の表示形状、非三角形Mesh、細分化、穴、退化面を検出した場合は形状を無視せずエラーで停止します。

撮影の前後で同じ物理ステップ・時刻、カメラ姿勢、全三角形・シーンハッシュを確認してから観測へ反映します。アームが動いた後は新しいシーンを取得するため、前の姿勢の遮蔽情報を流用しません。

## 停止確認と制限

到達性で除外された`high`を撮影対象に指定する確認コマンドです。

```bash
python -u src/arm_inspection_check.py --headless --views front high --output-dir outputs/arm_inspection_blocked_01
```

`run_status: blocked`、`blocked_reason: requested_probe_view_infeasible`、`movements: []`、`captures: []`となり、関節移動・撮影へ進まず終了コード2で停止します。撮影候補のうち1つが除外されたことと、選んだ撮影対象が実行できず全体停止したことを区別します。

既存出力先、前工程の失敗、モデル／バージョンの変更では開始しません。移動失敗・未静止・画像時刻のずれ・撮影中の形状変更・保存エラーも成功扱いしません。終了コードは成功0、実行できない目標2、実行・保存エラー1、中断130です。

入力レポートは、Step 3では通常の`outputs/arm_step3_01/arm_motion_report.json`、手先カメラでは`outputs/arm_camera_01/arm_camera_report.json`を優先します。通常ファイルが存在しない場合だけ、確認済み開発結果`outputs/arm_step3_codex_gui_01/arm_motion_report.json`／`outputs/arm_camera_codex_gui_02/arm_camera_report.json`を使います。存在する失敗レポートを成功レポートへ自動で切り替えません。別の保存先は`--step3-report`／`--camera-report`で指定できます。実際に使ったパスとハッシュは出力JSONに記録します。

この工程で確認するのは可視性と制御結果です。XRDFに衝突球・障害物モデルはなく、経路の衝突回避、搭載カメラの質量、実ロボットの安全性は未検証です。部品・台・カメラ取付形状は表示用です。また、2方向の診断成功は閉ループ選択や仮想カメラとの比較の完了を意味しません。

## 開発時の確認（2026-10-08）

- B/Yヘッドレス：`outputs/arm_inspection_codex_B_Y_02/arm_inspection_report.json`。前方R1=63/63、R2=3/63、後方R2=63/63。必要126点を累積観測。実カメラ位置誤差は約0.58 mm／0.62 mm。
- A/Y GUI：`outputs/arm_inspection_codex_A_Y_gui_01/arm_inspection_report.json`。前方からR1・R2とも63/63。2方向撮影と全確認項目に成功。
- 実際の前腕が撮影点を隠す初期試行では、RGBにも遮蔽が写り、レイ判定もその前腕Meshを記録しました。最終構成の2方向ではアーム遮蔽0点で、Bの不足はリブによる`self_occlusion`でした。
- `front high`指定：`outputs/arm_inspection_codex_blocked_01/arm_inspection_report.json`。IK除外により移動・撮影0回、終了コード2を確認。
- B/Xヘッドレス：`outputs/arm_inspection_codex_B_X_01/arm_inspection_report.json`。R1だけが評価対象となり、前方63/63、後方0/63、累積63/63。診断用の2方向撮影に成功。
- 前工程の色付きボード撮影：`outputs/arm_camera_codex_regression_01/arm_camera_report.json`。2画像と全確認項目に成功し、対象差し替え後も固定手先カメラが動くことを確認。
- 既存を含む137テスト成功。Mesh変換・インスタンス内部・guide除外・動くアームの遮蔽、IK初期値／解選択、候補除外、撮影時点の不一致、静的シーン変更、古い画像、入力失敗、出力上書き防止、保存失敗を確認。
- 保存したA/Y・B/Y・B/Xの全シーン・光学系を読み直して可視性を再計算し、全点の判定・レイの最初の当たり先が保存レポートと一致することを確認。B/Xでは保存PNGの画素ハッシュも取得時RGBと一致。

ここで画像・アーム表示・JSONのユーザー確認を待ちます。次工程は、到達性とアーム実行を閉ループプランナーへつなぎ、同じセル条件で仮想カメラ方式と比較する実装です。
