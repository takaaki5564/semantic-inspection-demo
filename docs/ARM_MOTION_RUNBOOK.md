# UR10e導入：Step 3 — IKと2地点への制御移動

Step 2で照合したUR10eモデルを使い、物理シミュレーション上のロボットを2地点へ順番に動かします。IKは「手先をこの位置・向きにするための関節角を求める計算」です。IKの成功後に関節の目標角を少しずつ変え、実際に手先が到達・静止したかを別に判定します。

今回はアーム駆動の独立確認です。検査部品・プランナー・手先RGBカメラは接続しません。既存のDay 1–4、Step 1–2の起動スクリプト、元USD、URDF・XRDFは変更していません。

## 実行手順

前のIsaac Simウィンドウを通常の操作で閉じてから実行します。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/arm_motion_check.py --output-dir outputs/arm_step3_01
```

入力は既定で `outputs/arm_step2_01/arm_model_report.json` です。Step 2の出力先が違う場合は指定します。

```bash
python -u src/arm_motion_check.py --step2-report outputs/arm_step2_02/arm_model_report.json --output-dir outputs/arm_step3_02
```

出力先は新しいフォルダにします。再実行時は末尾の番号を変えてください。既存の出力には上書きしません。

## 画面で確認すること

1. 外部カメラから見たUR10eと台座が表示されます。初期姿勢で手先が床に近づかないよう、物理開始前にロボット全体を上へ `0.4 m` 配置します。台座は表示用で、ベースの固定にはUSDの固定ジョイントを使います。
2. 自動的に `goal_01` へ動き、到達・静止を確認します。
3. 続いて `goal_02` へ動き、もう一度到達・静止を確認します。各移動は少なくともシミュレーション時間で3秒かけます。描画負荷によって実時間は長くなります。
4. 結果保存後、最後の姿勢で画面が残ります。確認後にウィンドウを閉じてください。

**Playボタンの操作は不要です。** タイムラインを停止したまま、公開APIで物理を `1/240秒` ずつ進めます。途中でPlay・Stop・関節編集をすると確認条件が変わります。物理時間はレポートの `simulation_time_s` で確認します。タイムライン表示の0秒は、アームが動いていないという意味ではありません。

画面のカメラはロボット全体を見る外部カメラです。手先カメラの映像ではありません。

## 期待する出力と判定条件

```text
MOTION START | goal_01
MOTION REACHED | goal_01 | position_m=... | angle_rad=... | settled_s=...
MOTION START | goal_02
MOTION REACHED | goal_02 | position_m=... | angle_rad=... | settled_s=...
REPORT | .../outputs/arm_step3_01/arm_motion_report.json
UR10E MOTION PASS | goals=2
GUI remains open at the final pose. Close the window to finish.
```

両地点で次の条件を連続して0.25秒以上満たす必要があります。

| 条件 | 許容値 |
|---|---|
| 実フランジ位置と目標の差 | 5 mm以下 |
| 実フランジの向きと目標の差 | 1度以下 |
| 各関節の実角度とIK目標角の差 | 0.01 rad以下 |
| 各関節の実速度の絶対値 | 0.02 rad/s以下 |
| 移動の証拠 | 物理時間が進み、いずれかの関節が開始時から0.03 rad以上変化 |

これらはシミュレーション確認用の設定です。実ロボットの精度や安定性を保証する値ではありません。

## 保存ファイルの読み方

| ファイル | 内容 |
|---|---|
| `arm_motion_report.json` | モデル、制御設定、2地点の目標・実姿勢、誤差、到達・停止理由のまとめ |
| `arm_motion_events.jsonl` | IK結果と、物理ステップごとの指令・関節角・速度・実手先姿勢。1行が1イベント |

まずJSONの次の項目を確認します。

| 項目 | 平易な説明 |
|---|---|
| `run_status` | `passed` は2地点とも到達・静止に成功。`blocked` は目標を完了できず停止。`failed` は設定や実行のエラー |
| `checks` | モデルの照合、両地点への到達、実際の関節移動の確認結果。成功時はすべて `true` |
| `step2_report` / `model_manifest` | 元のStep 2の結果と、アセット・モデルの変更検出に使う情報 |
| `controller` | 物理の時間刻み、重力、PD制御ゲイン、関節順、実姿勢の読出し元 |
| `motion_limits` | 使用モデルに明示したデモ用の角度・指令速度・加速度・ジャーク制限 |
| `goals[].status` / `reason` | 各地点の `reached` / `blocked` と、その判定理由 |
| `goals[].T_base_flange_target` | ロボットの `base_link` を基準とする、要求したフランジの位置・向き |
| `goals[].T_world_flange_target` | 実際のベース位置を使い、要求姿勢をシーン全体の座標へ変換したもの |
| `goals[].ik` | 試した初期値とIKの結果。解が複数なら現在の関節角に近い解を選ぶ |
| `goals[].ik_model_residual` | IKの関節角から再計算した手先姿勢と、要求した姿勢の差 |
| `goals[].trajectory` | 関節目標を滑らかに変える時間と、指令の速度・加速度・ジャーク上限 |
| `goals[].initial_state` / `final_state` | 移動前・後に物理側から読み戻した関節角、速度、ベース・フランジ姿勢、物理時間 |
| `goals[].final_error` | 実手先の位置・向きの誤差、最大関節誤差・速度 |
| `goals[].physics_steps` | 目標に対して物理を進めた回数 |
| `goals[].max_actual_joint_displacement_rad` | 開始時からどれだけ関節が実際に動いたか |
| `goals[].settled_seconds` | 到達・静止条件を連続して満たした時間 |

実フランジ姿勢は、PhysXのRigidPrimから読み出した `wrist_3_link` の姿勢に、USDから確認した固定変換 `T_wrist_flange` を合成します。指令角やIKモデルのFKから実手先姿勢を作る処理ではありません。ベースも物理側から読み取ります。

4×4行列は列ベクトルに作用し、`T_world_flange = T_world_base @ T_base_flange` です。位置はメートル、関節角・角度誤差はラジアンです。

イベントは `controller_initialized` → `goal_started` → `ik_result` → 複数の `physics_sample` → `goal_finished` の順です。連番と記録UTC時刻を持ち、指令と実際の動きを比較できます。

## 制御の内容と今回の制限

- 2地点は `config/robots/ur10e/motion_probe_goals.json` に保存しています。照合済みモデルから作った駆動確認用の位置・向きで、検査用のカメラ視点は後続工程で扱います。`reference_fk_joints_rad` は目標作成の由来で、関節指令として直接使いません。
- cuMotionのIKを、現在の関節角と、そこから決まった2通りのずらし方、計3初期値で試します。IKの数値確認は位置 `1e-5 m`、向き `1e-3 rad` 以下です。物理側の到達条件とは区別して記録します。
- モデルの制限に基づく5次補間で関節目標を変え、`Articulation.set_dof_position_targets()` で動かします。目標への関節の瞬間移動は行いません。速度 `0.5 rad/s`、加速度 `1 rad/s²`、ジャーク `10 rad/s³` は**指令軌道**の制限で、実際の速度は別途読み出して記録します。
- 重力は有効です。PDゲインとソルバー反復回数をこのシーン内で明示し、元USDやモデル設定は書き換えません。
- XRDFに衝突球・障害物モデルがまだなく、今回のIK・軌道生成は衝突回避を行いません。床の物理衝突は有効ですが、安全な経路や自己衝突回避を保証する検証ではありません。実ロボットへ転用する制御ではありません。
- 1目標あたり物理時間12秒、制御ループの実時間60秒の上限を設けます。未到達時は `blocked` にし、次の目標へ進みません。起動と最終表示の待ち時間は上限に含みません。

## 失敗したとき

| 表示・停止理由 | 確認すること |
|---|---|
| `Step 2 must pass` | 指定したStep 2レポートが成功しているか |
| バージョン・モデル・元USDの変更 | Step 1–2の証拠と現在環境が一致するか。変更を無視して続行しない |
| `ik_failed` / `ik_joint_limit_violation` / `ik_pose_residual` | `goals[].ik` の初期値・結果・残差。実移動は開始しない |
| `insufficient_motion_time_budget` | 軌道と静止待ちの必要時間が上限内に入らない |
| `motion_timeout` | 手先誤差・関節誤差・速度のどれが残ったか |
| `wall_clock_timeout` | 描画停止や極端な処理遅延がなかったか |
| `physics_clock_did_not_advance` / `fixed_base_moved` | 物理や固定ベースの条件が変わっていないか |
| `File exists` | 新しい出力フォルダを指定 |

終了コードは成功 `0`、目標未完了 `2`、実行・保存エラー `1`、中断 `130` です。レポート保存に失敗した場合は成功メッセージを出しません。

GUI確認後に自動終了する場合と、GUIを使わない場合のコマンドです。

```bash
python -u src/arm_motion_check.py --exit-after-run --output-dir outputs/arm_step3_gui_auto_01
python -u src/arm_motion_check.py --headless --output-dir outputs/arm_step3_headless_01
```

## 完了条件と次の工程

1. 画面でUR10eが2地点へ順番に動く。
2. `MOTION REACHED` が2回、最後に `UR10E MOTION PASS | goals=2` が出る。
3. JSONの `run_status` が `passed`、`checks` がすべて `true`。
4. 実姿勢、物理時間、関節変化、到達誤差、静止時間が保存される。

ここでユーザーの画面・出力確認を待ちます。次のDay 4.5では承認後、フランジとカメラの固定変換を定義して手先カメラを追加し、実撮影姿勢と新しい画像を確認する工程へ進みます。

## 開発時の確認結果（2026-10-08）

- ヘッドレス：`outputs/arm_step3_codex_03/arm_motion_report.json`。2地点とも到達・静止成功。
- GUIの自動実行：`outputs/arm_step3_codex_gui_01/arm_motion_report.json`。2地点とも成功し、結果保存後に自動終了。ユーザーの目視確認は未完了です。
- 実フランジの位置誤差は約 `0.80 mm` と `0.65 mm`、向きの誤差は約 `0.000864 rad` と `0.000600 rad`。両地点で0.25秒の静止を確認しました。
- 到達不能な最初の目標を使った実行では `blocked / ik_failed`、関節目標指令・当該目標の物理ステップとも0で停止し、2地点目には進みませんでした。結果：`outputs/arm_step3_codex_blocked_01/arm_motion_report.json`、終了コード2。
- 新規テストは、指令制限、座標変換、IK失敗・範囲外・残差、実手先のずれ、未移動、未静止、時計停止、時間上限、固定ベース移動、モデル改変、出力上書き防止、保存失敗を確認します。既存と合わせて103テスト成功。
- 生成物はGitに含めません。コードと手順をローカルコミットし、pushはユーザーが行います。

```bash
python -B -m unittest discover -s tests
```

## 参照したAPI

- インストール済みIsaac Sim 6.1の `Articulation`、`RigidPrim`、`SimulationManager.initialize_physics()` / `step()` / `get_simulation_time()` の公開実装とテストを確認しました。
- [公式Isaac Simの関節目標制御の例](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup_tutorials/tutorial_pickplace_example.html)
- [公式cuMotion IK API](https://nvidia-isaac.github.io/cumotion/api/python_api.html#inverse-kinematics-collision-unaware)
