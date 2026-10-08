# Day 4.5c — UR10eと仮想カメラの閉ループ検査

初回画像で必要な観測が足りなければ、プランナーが追加視点を選び、カメラを移動・撮影して、実際の観測結果から次の判断をします。A/B、検査仕様X/Y、部品配置、光学系、カメラ目標を共通にして、`arm`と`free`を比較できます。既存Day 1–4の起動コマンドと既定値は変更していません。

## まずB・仕様Yをアームで確認

前のIsaac Simウィンドウを閉じてから実行します。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/camera_closed_loop.py --executor arm --part B --spec Y --output-dir outputs/arm_loop_B_Y_01
```

Play操作は不要です。アームが前方の初期視点へ移動・静止・撮影します。B/YではR2の観測不足から、プランナーが後方を選びます。もう一度移動・撮影し、必要な表面点が揃うと停止します。GUIは外部のOverviewCameraです。**WristCameraを操作用ビューに選ばず**、手先画像は保存PNGで確認してください。

開発時の表示例です。点数は実形状・実撮影姿勢から計算しています。

```text
CANDIDATE | front | feasible | bounded_ik_solution
MOVEMENT_STARTED | front |
OBSERVED | capture_00 | R1=63/63, R2=3/63 | new=66
CANDIDATE | rear | feasible | bounded_ik_solution
PLAN | ready | view=rear predicted_new=60
MOVEMENT_STARTED | rear |
OBSERVED | capture_01 | R1=63/63, R2=63/63 | new=60
PLAN | inspection_observation_satisfied | required_samples_already_observed
SESSION_STOPPED |  | required_samples_already_observed
CAMERA LOOP PASSED | executor=arm | captures=2 | additional_actions=1
```

最初は次の3点を確認してください。

1. `outputs/arm_loop_B_Y_01/images/capture_00.png`で、高いリブの奥にあるR2が隠れている。
2. `images/capture_01.png`で、後方からR2が見える。
3. `camera_loop_report.json`の`run_status`が`passed`、`session.selected_view_ids`が`["rear"]`、`session.final_state`でR1・R2が各63/63点になっている。

保存後もGUIは最後の姿勢で残ります。確認後に閉じます。`--exit-after-run`で保存後に自動終了、`--headless`でGUIなしにできます。出力先は上書きしないので、再実行時は`_02`など別名にします。

## 仮想カメラと比較

アームの出力確認後、同じ条件で実行します。こちらにも同じUR10e・取付表示形状・部品・台が存在しますが、ロボットは初期関節目標を保ち、独立した仮想カメラだけが動きます。両方式とも初期視点は`front`、追加候補は同じ設定の`front`／`rear`です。

```bash
python -u src/camera_closed_loop.py --executor free --part B --spec Y --output-dir outputs/free_loop_B_Y_01
```

両方のウィンドウを閉じた後、保存済み画像・形状・計画を再計算して比較します。このコマンドはIsaac Simを起動しません。

```bash
python -u src/compare_camera_executors.py \
  --arm-report outputs/arm_loop_B_Y_01/camera_loop_report.json \
  --free-report outputs/free_loop_B_Y_01/camera_loop_report.json \
  --output outputs/camera_comparison_B_Y_01.json
```

比較では部品・仕様・セル・カメラ光学系・候補・能力・追加動作上限・モデルの一致を確認します。PNGの画素ハッシュ、描画側カメラ行列、実形状からの可視性・累積状態、候補ごとの予測形状からの計画を再検証します。条件違いや証拠の不一致は比較失敗です。

ロボットの姿勢と影は方式間で異なります。比較対象は同じカメラ目標での観測数・追加動作数・停止理由であり、画像の完全一致や衝突回避の性能ではありません。初回撮影に成功した後の`blocked`も比較に載せられます。初期視点から撮影できなかった実行は、初回観測との比較ができないため拒否します。

## A・YとB・Xの停止を確認

以下はこの工程内の追加確認です。各実行の出力を見てから、次のコマンドへ進めてください。

```bash
python -u src/camera_closed_loop.py --executor arm --part A --spec Y --output-dir outputs/arm_loop_A_Y_01
python -u src/camera_closed_loop.py --executor free --part A --spec Y --output-dir outputs/free_loop_A_Y_01
python -u src/camera_closed_loop.py --executor arm --part B --spec X --output-dir outputs/arm_loop_B_X_01
python -u src/camera_closed_loop.py --executor free --part B --spec X --output-dir outputs/free_loop_B_X_01
```

| 条件 | 初回の必要点の観測 | 最終累積 | 撮影回数 | 追加動作 |
|---|---:|---:|---:|---:|
| A・Y | 126/126 | 126/126 | 1 | 0 |
| B・X | R1の63/63 | 63/63 | 1 | 0 |
| B・Y | 66/126 | 126/126 | 2 | 後方へ1回 |

上は検証時の実測です。Aだから停止、Bだから後方へ移動、という分岐はありません。仕様が要求する点と、実際に観測済みの点が揃ったかで判断します。A/YとB/Xは初回撮影で終了するため、2枚目の画像は生成されません。別ケースの比較も同じ比較コマンドの入力パス・出力名を替えて実行できます。

## 出力の意味

| ファイル | 内容 |
|---|---|
| `camera_loop_report.json` | 入力条件、実行方式、移動・撮影・観測・計画・終了理由をまとめたレポート |
| `events.jsonl` | 初回確認、移動、撮影、観測更新、候補確認、計画、停止を順番に記録。1行が1イベント |
| `motion_events.jsonl` | アームIK、関節指令、物理ステップなどの制御記録。仮想カメラではアーム移動イベントはない |
| `images/capture_00.png`／`capture_01.png` | 実RGB原画像。番号は撮影順で、視点名はレポートの`view_id`に記録 |
| `images/capture_XX_visibility.png` | 原画像に表面サンプル点を重ねた説明画像 |
| `geometry/capture_XX.json` | その撮影時点の実三角形・検査点・カメラ・物理時刻。観測の再計算用 |
| `geometry/prediction_XX.json` | 候補を評価するときの予測形状とIK情報。撮影画像や実観測ではない |
| 比較コマンドのJSON | 共通条件と、両方式の初回／最終観測数、撮影回数、追加動作数、停止理由 |

可視性画像の**緑点はその1枚で見える点、赤点はその1枚で見えない点**です。元の緑色R1／橙色R2マーカーとは別の情報です。赤点は遮蔽された検査点の投影位置なので、リブやロボットの上に重なる場合があります。画像上部の点数は1枚ごとの可視点数です。累積観測はJSONを見ます。緑点は傷なしや検査合格を意味しません。

`camera_loop_report.json`の主な項目です。

| 項目 | 平易な説明 |
|---|---|
| `run_status` | 必要な観測が揃うと`passed`、実行できる次の動作がなければ`blocked`、証拠や保存の異常は`failed` |
| `comparison_context` | 比較に必要な共通条件。検査知識のスナップショット、セル・候補のハッシュ、光学系など |
| `session.initial_positioning_count` | 最初の視点へ配置する回数。通常1回で、追加動作には数えない |
| `session.actions_executed`／`selected_view_ids` | 観測不足から計画して実行した追加動作の回数と視点名 |
| `session.movements[]` | 目標カメラ姿勢、変換後のフランジ姿勢、実到達姿勢、誤差、到達／静止の結果 |
| `session.captures[].capture` | 画像名、実撮影姿勢・時刻・物理ステップ、RGB画素ハッシュ。アームでは固定取付も確認 |
| `session.captures[].capture.renderer_camera` | RGBと同じ描画先の`CameraParams`。描画姿勢・光学系とUSDの実カメラが一致した証拠 |
| `session.captures[].visibility` | 実際の撮影形状に対する各点の見え方と、レイが最初に当たった面。アームによる遮蔽も含む |
| `state_before_capture`／`state_after_visibility` | その撮影の前後の累積観測状態。新たに観測した点も記録 |
| `session.events[].plan` | 候補が増やせると予測した必要点数、移動コスト、選択した動作と理由 |
| `predictions[]` | 候補ごとの予測形状ファイル、適用した追加動作番号と形状ハッシュ |
| `session.final_state`／`stop_reason` | 最終的に観測できた点と、処理を止めた理由 |

`inspection_observation_satisfied`は必要な表面サンプルを観測できた状態です。画像から傷を見つけた／製品が良品だったという意味ではありません。形状と検査点はシミュレーターの既知情報です。

## 計画と実観測を分ける仕組み

プランナーは引き続き部品基準のカメラ姿勢を出します。アーム側でカメラ目標をフランジ・ロボットベース座標へ変換し、IK、関節範囲、FK残差を確認してから候補を渡します。選択後も再度IKを解いて関節目標で動かし、PhysXから読んだ実関節・手先姿勢・速度で到達と静止を確認します。

アームが動くと遮蔽形状も変わるため、候補評価には**候補IKの関節角で予測したロボット形状**を使います。検証済みUSD関節チェーンのFKで実表示Meshを各リンクに付けたまま変換します。固定部品・台を変えず、シミュレーターも動かしません。仮想カメラ側は同じロボットの駐機関節目標で予測します。

予測だけでは観測済み点を増やしません。移動後に実形状を読み直し、静止した同じ物理時点のRGBと実カメラ姿勢で観測を更新します。静的セルのハッシュは維持し、アームを含む全形状のハッシュは撮影ごとに保存します。描画側カメラ行列・光学系も照合します。

仮想カメラでは各移動後に、駐機関節目標を変えず物理を1ステップ進め、画像参照時刻を更新します。撮影中は物理を止め、画像参照時刻が保持した物理時刻に一致することを確認します。初期駐機姿勢の静止待ちにも有限の時間上限があります。実験版CameraSensorの描画切替不整合を避けるため、切替元の手先センサー／描画テクスチャは終了まで保持します。

## 停止条件の確認

追加動作を禁止しても、最初の視点への配置と初回撮影は行います。

```bash
python -u src/camera_closed_loop.py --executor arm --part B --spec Y --disable-camera-motion --output-dir outputs/arm_loop_disabled_01
python -u src/camera_closed_loop.py --executor arm --part B --spec Y --max-actions 0 --output-dir outputs/arm_loop_limit_01
```

B/Yは初回の66/126点で止まります。前者は`required_capability_unavailable`、後者は`max_actions_reached`です。A/YやB/Xは必要点が初回で揃うので、追加動作禁止でも成功です。

現在の共通比較は`front`／`rear`を候補にしています。既存の`high`だけを追加候補にすると、アーム側は初回撮影後に候補のIKを確認できず停止します。

```bash
python -u src/camera_closed_loop.py --executor arm --part B --spec Y --candidate-views high --output-dir outputs/arm_loop_unreachable_01
```

`session.stop_reason`は`no_reachable_candidate_views`、初回撮影1回、追加動作0回です。`ik_failed`は今回の有限な数値探索で解が見つからないという意味です。到達不能の数学的証明ではありません。`--initial-view high`で初期視点自体が確認できなければ、移動・撮影前に`initial_view_infeasible`で止まります。

動作上限は`--max-actions`（追加動作のみ、既定3回）と`--max-motion-seconds`（各アーム移動、静止確認込み、既定20シミュレーション秒）です。実時間にも有限の上限があります。終了コードは成功0、実行不可2、実行・保存異常1、中断130です。古い画像、取付変更、撮影中の物理／形状変更、描画カメラ不一致は成功扱いにしません。

## 前工程と制限

通常の前工程レポートを優先します。ファイルが存在しない場合だけ、確認済みの開発出力へ切り替えます。存在する失敗レポートから成功結果への自動切替は行いません。

| 工程 | 通常の入力 | 不在時の開発入力 |
|---|---|---|
| 制御動作 | `outputs/arm_step3_01/arm_motion_report.json` | `outputs/arm_step3_codex_gui_01/arm_motion_report.json` |
| 固定手先カメラ | `outputs/arm_camera_01/arm_camera_report.json` | `outputs/arm_camera_codex_gui_02/arm_camera_report.json` |
| A/B検査セル | `outputs/arm_inspection_B_Y_01/arm_inspection_report.json` | `outputs/arm_inspection_codex_B_Y_02/arm_inspection_report.json` |

別パスは`--step3-report`、`--camera-report`、`--inspection-report`で指定できます。Isaac Sim 6.1.0.0、cuMotion、ロボットモデル・アセット、セル・視点設定と証拠の整合性を確認します。入力・モデル・設定を変えた場合は対応する前工程を再確認してください。

XRDFに衝突球や障害物モデルはなく、経路の衝突回避は未実装です。部品・台・カメラ取付形状は表示用で、搭載質量や実ロボットの安全性も未検証です。撮影・可視性・閉ループ動作のシミュレーション結果として扱います。UI操作、レシピ候補、傷の画像認識はこの工程には含めません。

## 開発時の確認（2026-10-08）

採用した確認結果は以下です。パスはいずれも`outputs/`からの相対パスです。

| 条件 | アームのレポート | 仮想カメラのレポート | 再計算した比較結果 |
|---|---|---|---|
| B/Y | `camera_loop_codex_arm_B_Y_gui_02/camera_loop_report.json` | `camera_loop_codex_free_B_Y_05/camera_loop_report.json` | `camera_comparison_codex_B_Y_02.json` |
| A/Y | `camera_loop_codex_arm_A_Y_gui_02/camera_loop_report.json` | `camera_loop_codex_free_A_Y_02/camera_loop_report.json` | `camera_comparison_codex_A_Y_02.json` |
| B/X | `camera_loop_codex_arm_B_X_02/camera_loop_report.json` | `camera_loop_codex_free_B_X_02/camera_loop_report.json` | `camera_comparison_codex_B_X_01.json` |

- B/Yは両方式とも初回66点、後方の追加撮影後126点。A/Yは両方式とも初回126点、B/Xは両方式とも初回R1の63点で停止しました。アームのB/Y・A/YはGUIでも動作確認しました。
- 上記6実行は、全PNGと実撮影シーン、累積状態、候補形状からの計画の再計算に成功しました。保存原画像を目視し、対象部品と撮影方向も確認しました。
- 仮想カメラのB/YもGUIで成功：`camera_loop_codex_free_B_Y_gui_01/camera_loop_report.json`。原画像2枚を目視し、アームGUIとの比較`camera_comparison_codex_B_Y_gui_01.json`も成功しました。
- `camera_loop_codex_arm_high_01/camera_loop_report.json`：初回撮影後に`no_reachable_candidate_views`、追加0回、終了コード2。`camera_loop_codex_arm_disabled_01/camera_loop_report.json`：初回撮影後に`required_capability_unavailable`、追加0回、終了コード2。どちらも66/126点のまま停止し、保存証拠の再計算に成功しました。
- 既存を含む166テスト成功。候補別アーム形状による遮蔽、仕様による停止、到達候補除外、動作禁止・上限、実観測が増えない場合の停止、古い画像・時刻不一致、セル変更、保存／入力異常、画像・予測・実行履歴の不整合、描画カメラの姿勢／光学系不一致、センサー切替時の寿命保持を確認しました。
- 従来の`src/day4_closed_loop.py --headless --part B --spec Y`も実行し、`day4_codex_regression_arm_loop_01`で初回66点から追加1回で126点を観測して成功しました。
- 初期の仮想カメラ試行では、数値上の姿勢とRGB描画が一致しない不具合がありました。修正後の採用パスを上表に限定しています。B/Yの`free_B_Y_01`〜`free_B_Y_04`、A/Y・B/Xの`free_*_01`、`free_debug_*`を含む以前の試行は成果・比較結果に使用しません。

再計算コマンドは次のとおりです。

```bash
python -B -m unittest discover -s tests -v
python -B src/compare_camera_executors.py \
  --arm-report outputs/camera_loop_codex_arm_B_Y_gui_02/camera_loop_report.json \
  --free-report outputs/camera_loop_codex_free_B_Y_05/camera_loop_report.json \
  --output outputs/camera_comparison_review_B_Y_01.json
```

ここでユーザーの画像・動作・JSON確認を待ちます。次工程のDay 5（簡易画面・レシピ候補）には進みません。
