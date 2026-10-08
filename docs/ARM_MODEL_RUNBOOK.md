# UR10e導入：Step 2 — モデルと座標の照合

Step 1で表示できたUR10e USDと、cuMotionが使うURDF・XRDFの運動学が一致するかを確認します。順運動学（FK）は「関節角から手先の位置と向きを計算すること」です。

## 今回の確認範囲

- 同梱の `ur10` 用設定を流用せず、Step 1と同じUR10e USDから生成したモデルを使います。
- ゼロ関節角の計算結果を、USDに保存されたフランジ姿勢と照合します。
- ゼロ姿勢、6関節それぞれの正負回転、複合姿勢2つの計15条件で照合します。
- 関節名・角度範囲、ロボット基準座標、フランジ座標を確認します。
- USD・モデルのハッシュと数値結果をJSONへ保存します。

これはモデル変換・単位・座標・回転方向の整合性確認です。非ゼロ関節角の比較対象はUSDの関節定義から計算した姿勢で、物理シミュレーションの実測姿勢ではありません。モデルも同じUSDから生成しており、実ロボットの校正精度を示す検証ではありません。IK、物理Articulationの駆動、手先カメラは後続ステップです。

## 実行手順

Step 1のウィンドウを通常の操作で閉じてから実行します。Step 2は数値照合のためヘッドレスで起動し、確認後に自動終了します。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/arm_model_check.py --output-dir outputs/arm_step2_01
```

既定では `outputs/arm_step1_01/arm_preflight_report.json` から使用アセットのパスとIsaac Simバージョンを読み取ります。Step 1の出力先が違う場合は指定します。

```bash
python -u src/arm_model_check.py --step1-report outputs/arm_step1_02/arm_preflight_report.json --output-dir outputs/arm_step2_02
```

既存の出力先には上書きしません。再実行時は出力先の末尾を変えてください。

## 期待する出力

Isaac Simの起動ログに続いて、15個の `FK PASS` と最後の `UR10E MODEL PASS` が出ます。

```text
FK PASS | zero | position_m=... | angle_rad=...
FK PASS | shoulder_pan_joint_positive | position_m=... | angle_rad=...
...
FK PASS | mixed_02 | position_m=... | angle_rad=...
REPORT | .../outputs/arm_step2_01/arm_model_report.json
UR10E MODEL PASS | cases=15 | base=base_link | tool=flange
```

判定の許容誤差は位置 `1e-5 m`、角度 `1e-5 rad` です。これは数値モデル照合のための閾値で、アーム駆動時の到達許容誤差とは別です。

## 保存したモデル

| ファイル | 内容 |
|---|---|
| `config/robots/ur10e/robot.urdf` | インストール済みのUSD→URDF変換APIで生成したUR10eの関節・リンク・フランジ定義 |
| `config/robots/ur10e/robot.xrdf` | cuMotionの関節順、初期角度、手先フレームなどの設定。JSON表記の有効なXRDF |
| `config/robots/ur10e/model_manifest.json` | 使用元USDレイヤーとモデルのハッシュ、対応版、今回の設定値の出所 |

このモデルの基準は `base_link`、手先は `flange` です。`wrist_3_link` や、別モデルの `tool0` と同じ座標だとは扱いません。物理・描画用のロボット本体は引き続きUSDを使用します。

URDFから描画・衝突形状を除き、XRDFにも衝突球をまだ定義していません。速度 `0.5 rad/s`、加速度 `1 rad/s²`、ジャーク `10 rad/s³` は明示したデモ用設定値で、メーカーの仕様値や検証済みの動作設定ではありません。今回のFK検証はこれらの動的制約や衝突回避を検証しません。

## 出力JSONの読み方

`outputs/arm_step2_01/arm_model_report.json` を開きます。

| 項目 | 平易な説明 |
|---|---|
| `run_status` | `passed` は今回のモデル照合に成功したという意味 |
| `verification_scope` | 検証した範囲。物理駆動やIKの成功と区別するための記録 |
| `step1_report` | 元にしたStep 1の結果ファイルとハッシュ |
| `model_manifest` | 使用したモデルの由来と、ファイルの変更確認に使うハッシュ |
| `usd_reference.joints` | USDから読んだ関節の接続先、回転軸、取付座標、角度範囲 |
| `usd_reference.T_base_flange_authored` | USDに保存されている、基準座標から見たフランジのゼロ姿勢 |
| `comparison.model_joint_order` | cuMotionへ関節角を渡す順番。USDパスの並び順とは別に名前で対応付ける |
| `comparison.cases` | 15条件それぞれの関節角、2通りの計算姿勢、差、合否 |
| `comparison.cases[].T_base_flange_usd_joint_fk` | USDの関節定義から計算したフランジ姿勢 |
| `comparison.cases[].T_base_flange_model_fk` | URDF・XRDFを読み込んだcuMotionが計算したフランジ姿勢 |
| `position_error_m` / `orientation_error_rad` | 2通りの計算の位置差・向きの差。小さいほど一致している |
| `comparison.checks` | 関節名、角度範囲、保存されたゼロ姿勢、全FK条件の確認結果 |
| `timeline_seconds` | 今回の確認では0秒。アーム駆動は行っていない |

位置の単位はメートル、関節角と姿勢誤差の角度はラジアンです。4×4行列は列ベクトルに作用し、`p_base = T_base_flange @ p_flange` で基準座標へ変換します。

## 失敗したとき

- `Step 1 must pass`：Step 1の出力ファイルと成功状態を確認します。
- `Isaac Sim version changed`：更新された環境でStep 1から再確認します。
- `Model files changed` / `USD source layers ... changed`：使用元やモデルが保存時と異なります。変更を無視して成功扱いにはしません。
- `FK FAIL` / `Model consistency checks failed`：JSONの該当条件と差を確認します。
- `File exists`：新しい出力先を指定します。

モデル再生成が必要な場合は、明示的に次を実行すると、新しい出力先の `model/` にモデルが生成され、そのモデルを照合します。Git内のモデルは書き換えません。標準の変換処理が書き出すMeshも、この出力フォルダ内に保存されます。

```bash
python -u src/arm_model_check.py --export-model --output-dir outputs/arm_step2_reexport_01
```

## 完了条件

1. 最後に `UR10E MODEL PASS | cases=15` が出る。
2. JSONの `run_status` が `passed`。
3. `comparison.checks` がすべて `true`。

ここで出力を確認して停止します。次のStep 3では物理Articulationを初期化し、IKと関節目標による2地点への移動、実際の関節・フランジ姿勢、収束とタイムアウトを確認します。

## 開発時の確認結果（2026-10-08）

- 既存環境でのモデル生成・cuMotion読込、および保存モデルを使う通常コマンドの両方が成功しました。
- 15条件すべて成功。最大位置誤差は約 `7.04e-8 m`、最大角度誤差は約 `2.19e-7 rad`。保存されたUSDゼロ姿勢との照合、関節名・角度範囲も成功しています。
- 通常コマンドの確認結果：`outputs/arm_step2_codex_01/arm_model_report.json`。自動生成物なのでGitには含めません。
- 既存テストと、閉形式の2関節モデル・軸の反転・位置ずれ・関節順変更・モデル改変・出力上書き・結果保存失敗を扱うテストを確認しています。
- ユーザーによるStep 2の出力確認が完了し、Step 3への進行が承認されました。Step 3の実装・開発時の動作確認は完了し、ユーザーのGUI・出力確認を待っています。手順は [ARM_MOTION_RUNBOOK.md](ARM_MOTION_RUNBOOK.md) を参照してください。

```bash
python -B -m unittest discover -s tests
```

## 参照したインストール済みAPI・公式資料

- `isaacsim.asset.exporter.urdf.UsdToUrdfConverter`：インストール済み6.1の公開APIと同梱テストを確認。[公式URDF出力手順](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/importer_exporter/export_urdf.html)
- `isaacsim.robot_motion.cumotion.load_cumotion_robot()`：[公式モデル読込手順](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/cumotion/tutorial_robot_configuration.html)
- `cumotion.Kinematics.pose()`：[公式cuMotion Python API](https://nvidia-isaac.github.io/cumotion/api/python_api.html#kinematics)
