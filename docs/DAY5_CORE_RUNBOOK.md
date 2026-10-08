# Day 5 コア：1ステップ制御・部位状態・カメラ取付の確認

GUI案は承認済みです。今回は画面を支える共通制御と部位状態を実装しました。黒い操作パネル、RGB履歴、レシピ候補の画面への組込みは、この工程の出力確認後に行います。黒基調の[画面案](GUI_IMPROVEMENT_PROPOSAL.svg)は実装済みGUIのスクリーンショットではありません。

## まず1回だけ撮影する

既存のIsaac Simウィンドウを通常の方法で閉じ、ターミナルで実行します。

```bash
cd /home/ishii/work/nobique/nedo-demo
source /home/ishii/work/isaac/env_isaac/bin/activate
python -u src/camera_closed_loop.py --executor arm --part B --spec Y --single-step --output-dir outputs/day5_step_B_Y_01
```

出力先が既に存在する場合は末尾を`_02`などへ変更してください。既存の結果は上書きしません。

初期視点frontへ移動して静止し、1枚を撮影・保存します。その実観測から次の計画を作り、rearへ移動する前に止まります。最後は`CAMERA LOOP PAUSED`で、完了やblockedではありません。Isaac Sim画面は確認のため残ります。

期待する初回の状態は次のとおりです。

| 部位 | 累積観測 | 状態 | 次の撮影対象 |
|---|---|---|---|
| R1：手前の検査面 | 63/63 | confirmed：観測条件充足 | false |
| R2：リブ奥の検査面 | 3/63 | partial：一部観測 | true |

`outputs/day5_step_B_Y_01/`で確認するファイル：

- `images/capture_00.png`：保存済みの原画像。初回なので2枚目はありません。
- `images/capture_00_visibility.png`：可視性評価点を重ねた説明画像。原画像とは別です。
- `step_01.json`：初回サイクル終了時の状態。`status=paused`、`capture_count=1`、`next_plan.action.view_id=rear`、`region_states`のR2だけ`next_capture_target=true`を確認します。
- `camera_loop_report.json`：実移動・撮影・観測・計画・イベントの全記録。`run_status=paused`、`control_mode=single_step`です。
- `events.jsonl`：移動、撮影検証、保存成功、観測反映、計画、ステップ終了の順番です。
- `geometry/`：実撮影時と候補視点の形状。予測形状の観測点は実観測へ加算しません。

この確認用CLIはウィンドウを閉じると終了します。保存JSONから物理状態を復元する再開機能はありません。次工程のGUIでは、同じコントローラを保持し、次のボタン操作で2回目の`step()`を呼びます。

## 既存の連続実行も確認する場合

1ステップ確認用ウィンドウを閉じてから実行します。

```bash
python -u src/camera_closed_loop.py --executor arm --part B --spec Y --output-dir outputs/day5_run_B_Y_01
python -u src/camera_closed_loop.py --executor free --part B --spec Y --output-dir outputs/day5_free_B_Y_01
```

両方とも初回66/126点から、後方の実撮影で126/126点となり、2枚・追加動作1回で終了します。`step_01.json`には一部観測と次計画、`step_02.json`にはR1/R2のconfirmedと次計画なしを保存します。コマンドは順に実行し、それぞれ確認後にウィンドウを閉じます。

仕様・部品を変える確認には、新しい出力先で`--part A --spec Y`または`--part B --spec X`を指定します。A/Yは初回で両部位の観測条件を満たします。B/XはR1のみを評価し、R2は`not_required`、coverageは`null`です。表示では「—／対象外」とし、0%や100%にはしません。

## 部位状態と失敗時の扱い

`region_states`はGUIが独自に作る値ではなく、有効な実観測を保持するコアから得られます。

| 状態 | 意味 |
|---|---|
| unconfirmed | 有効な評価履歴がまだない。撮影しただけでは確認済みにならない |
| needs_recheck | 有効な評価はあるが、その部位の累積観測点が0 |
| partial | 一部の必要点を累積観測済み |
| confirmed | 必要点すべてを累積観測済み。製品の良否は未評価 |
| not_required | 今回の仕様の対象外。coverageは数値を持たない |

重複撮影で点数を二重に増やしません。後の視点で隠れた点も、同じセッションで有効に観測した履歴は保持します。blockedになっても部位全体を赤へ書き換えません。次の撮影対象は計画の予測新規点に基づきますが、予測ではcoverageを増やしません。

保存は原画像・説明画像・形状証拠の処理を終えてから観測へ反映します。保存失敗時は`session_failed`の`failure_phase=saving`となり、直前の有効な観測を保持します。途中まで書けたファイルが残る場合も、成功した撮影履歴には追加しません。撮影検証失敗、到達不可、保存失敗の理由を区別します。

1ステップの二重実行を拒否し、待機中に部品・形状・カメラが変わった場合は次の移動前に失敗させます。停止予約は現在の観測サイクルの境界で止める機能です。resetや条件変更は停止した境界で新しいセッションを作り、旧画像・旧計画・旧観測を持ち越しません。

## カメラが浮いて見えた原因と修正

実アセットの`wrist_3_link`の可視メッシュは存在し、表示も有効でした。`flange`自体は座標系のXformです。Eye-in-Handなので把持用のグリッパーは追加していません。

以前の横支柱はフランジ座標のZ=0.10mにあり、下端は0.0875mでした。手首から横支柱へつなぐ部材がなく、浮いて見える形状でした。UR10eのフランジ面は+X方向なので、その面へ台座を置き、縦支柱→横支柱→カメラ筐体を接続しました。レンズの表示も追加し、光学原点の前方は空けています。

`T_flange_camera`、IK、関節制御は維持します。追加部材は表示形状で、質量・衝突モデルはありません。ただし通常のRGBとレイ評価には含まれるため、形状変更後の実撮影で回帰確認しました。

実RTXによる[取付確認画像](../outputs/camera_loop_codex_day5_arm_01/mount_review.png)と[確認メタデータ](../outputs/camera_loop_codex_day5_arm_01/mount_review.json)を保存しています。これは検査画像ではなく、撮影終了後に外部カメラから取得した診断画像です。取得中に物理状態・評価形状が変化しないことを確認しています。検査用カメラを画面操作で動かさず、OverviewCameraから取付を確認してください。

## 変更ファイルの役割

- `src/executor_closed_loop.py`：共通の1ステップ／連続実行、停止境界、保存後の観測確定。
- `src/inspection_region_state.py`・`config/inspection_regions.json`：部位状態の導出と名称・Prim対応。
- `src/camera_closed_loop.py`：既存入口を維持し、`--single-step`とステップJSON保存を追加。
- `src/camera_mount_geometry.py`・`src/arm_camera_adapter.py`：フランジと筐体の接続表示。
- `tests/test_inspection_step_session.py`・`tests/test_camera_mount_geometry.py`・`tests/test_camera_closed_loop.py`：新しいコア動作の検証。
- GUI改善案・SVG、プロジェクト文脈、開発計画、部位可視化設計：黒基調・軽量更新と今回の進捗を反映。

## 自動確認

```bash
python -B -m unittest discover -s tests -v
```

共通コアについて、1ステップ／連続実行の同一結果、B/Yの遷移、A/Y・B/X、対象外coverage、0点、累積保持、保存失敗、待機中の形状変更、二重実行、停止予約、セッション作り直しを確認します。取付形状の連結と光学原点前方の空間も確認します。

2026-10-09、Isaac Sim 6.1.0.0／既存RTX 4080環境で確認しました。コアを含む全186テストが通っています（ログ：`/tmp/nedo_day5_core_tests_final.log`）。

| 条件 | 採用した出力先（`outputs/`以下） | 実測結果 |
|---|---|---|
| B/Y・アーム | `camera_loop_codex_day5_arm_01` | 66→126/126点、2枚、rear追加1回 |
| B/Y・仮想 | `camera_loop_codex_day5_free_01` | 66→126/126点、2枚、rear追加1回 |
| B/Y・アーム・1ステップ | `camera_step_codex_day5_arm_01` | 66/126点、1枚、rear計画でpaused |
| A/Y・アーム | `camera_loop_codex_day5_A_Y_arm_01` | 初回126/126点、1枚、追加0回 |
| A/Y・仮想 | `camera_loop_codex_day5_A_Y_free_01` | 初回126/126点、1枚、追加0回 |
| B/X・アーム | `camera_loop_codex_day5_B_X_arm_01` | 初回63/63点、R2対象外、1枚、追加0回 |
| B/X・仮想 | `camera_loop_codex_day5_B_X_free_01` | 初回63/63点、R2対象外、1枚、追加0回 |

保存RGB・実形状・状態・候補計画を再計算した両方式の比較は、`camera_comparison_codex_day5_01.json`、`camera_comparison_codex_day5_A_Y_01.json`、`camera_comparison_codex_day5_B_X_01.json`です。

取付修正の確認画像とB/Yの原画像を目視確認しています。今回のランタイム検証はheadlessです。黒基調のGUIの応答性・負荷測定、状態色の3D/RGB分離、レシピ候補は次工程の確認対象です。
