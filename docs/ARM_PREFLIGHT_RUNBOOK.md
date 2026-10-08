# UR10e導入：Step 1 — 読み込み・表示確認

Day4までの仮想カメラ方式を残し、独立したプロセスでUR10eを確認します。各ステップの出力を確認してから次へ進めます。

## 今回の確認範囲

- 既存のPython環境からIsaac Simを起動する。
- ローカルのUR10e USDを読み込み、GUIで形状を確認する。
- 6個の回転関節、関節可動範囲、ArticulationRoot、フランジ、MeshをUSDから読み取る。
- 結果をJSONに保存する。

Step 1ではタイムラインを停止したままにします。物理Articulationの初期化、IK、関節駆動、手先カメラ取付、検査は後続ステップです。画面のカメラはロボット全体を見るための外部カメラです。

## 実行手順

既存のIsaac Simウィンドウを通常の操作で閉じてから、ターミナルで次を順番に実行します。

```bash
cd /home/ishii/work/isaac
source env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/arm_preflight.py --output-dir outputs/arm_step1_01
```

Isaac Simの起動とアセットの描画を待ちます。GUIにUR10eと床が表示されます。初期姿勢はUSDに保存された姿勢で、腕は横方向に伸びています。Playボタンは押さず、表示を確認してください。確認後はウィンドウを閉じて終了できます。

出力先が既にあるときは上書きせず終了します。再実行では `outputs/arm_step1_02` など新しい出力先を指定してください。

## 期待するターミナル出力

Isaac Simの起動ログに続いて、次のような行が表示されます。パスは環境に依存します。

```text
UR10E LOAD START | Isaac Sim 6.1.0.0 | .../ur10e/ur10e.usd
UR10E LOAD PASS | joints=6 | meshes=14
FLANGE | /World/UR10e/wrist_3_link/flange
REPORT | .../outputs/arm_step1_01/arm_preflight_report.json
Step 1: static display only. Close the window to exit; leave the timeline stopped.
```

`meshes=14` は現在の同梱アセットの値です。合格判定では固定数ではなく、Meshが存在し、すべてに頂点があることを確認します。

## 出力ファイルの意味

`outputs/arm_step1_01/arm_preflight_report.json` を開きます。

| 項目 | 意味 |
|---|---|
| `demo_step` / `verification_scope` | 今回は静止したUSDの読み込み確認であること |
| `run_status` | `passed` は読み込み確認の成功。アーム動作成功という意味ではない |
| `runtime` | 実際に使用したPython、Isaac Sim、GUI/ヘッドレスの区別 |
| `asset.path` / `asset.source` | 使用したUSDのパスと出所 |
| `robot.variant_selections` | PhysXを選び、グリッパーと内蔵センサーを無効にした設定 |
| `robot.joints` | 6関節の名前、USDパス、下限・上限。単位は度で、現在角度の実測値ではない |
| `robot.articulation_root_paths` | ロボットのArticulationRoot設定が付いている場所。物理初期化の成功を示すものではない |
| `robot.flange_prim_path` | 後続ステップでカメラ取付先として使うフランジの場所 |
| `robot.T_world_robot` / `robot.T_world_flange` | 現在のUSDに保存されたロボット・フランジ座標を世界座標へ変換する4×4行列 |
| `robot.meshes` | インスタンス内部も含むMeshのパスと頂点数 |
| `robot.checks` | 関節、フランジ、形状、単位・軸の確認結果。すべて `true` が必要 |

行列は列ベクトルに作用し、位置の単位はメートル、世界座標はZ上向きです。ここで記録する姿勢は物理シミュレーションによる移動後の姿勢ではありません。

## アセットと失敗時の確認

既定では、インストール済みIsaac Simパッケージ内の `exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd` を使用します。テスト用同梱アセットであり、ファイルのダウンロードやパッケージのインストールは行いません。別のローカルUR10e USDを試す場合は、次のように指定できます。

```bash
python -u src/arm_preflight.py --robot-usd /absolute/path/to/ur10e.usd --output-dir outputs/arm_step1_custom_01
```

- `UR10E LOAD FAILED` が出た場合は、その行と末尾の例外を確認します。起動後の失敗は、可能な場合に同じJSONへ `run_status: failed` とエラー内容を保存します。
- `Local UR10e USD not found`：環境とアセットパスを確認します。
- `File exists`：新しい出力ディレクトリを指定します。
- `USD checks failed` / `flange missing`：別モデルを成功扱いにせず、使用したUSDと構造を確認します。
- `LOAD PASS` があってもロボットが画面に見えない場合は、表示確認が未完了です。次のステップには進みません。

## 自動確認用

GUI表示の確認とは別に、ヘッドレスで読み込み・JSON保存を確認できます。

```bash
python -u src/arm_preflight.py --headless --output-dir outputs/arm_step1_headless_01
```

## Step 1の完了条件と次のステップ

1. GUIにUR10eが表示される。
2. `UR10E LOAD PASS` が出力される。
3. JSONの `run_status` が `passed`、`robot.checks` がすべて `true`。

ここで結果を確認して停止します。次のStep 2では、UR10e用モデルとUSDの座標・順運動学を照合します。同梱のcuMotion `ur10` は、このUR10eと寸法が異なるため、そのままIKに使いません。

## 開発時の確認結果（2026-10-08）

- 既存環境でのヘッドレス起動：終了コード0、6関節、14 Mesh、期待するフランジ、全チェック成功、タイムライン0秒。
- 確認結果：`outputs/arm_step1_codex_01/arm_preflight_report.json`。自動生成物なのでGitには含めません。
- テスト：既存75件と新規4件、計79件が成功。新規テストでは、不完全なUSDの拒否、インスタンス内部のMesh取得、失敗時の終了コード・JSON、出力の上書き防止を確認。
- ユーザーによるGUI表示確認は未完了。Step 2は未着手。

```bash
python -B -m unittest discover -s tests -v
```
