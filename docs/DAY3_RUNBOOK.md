# Day 3 — 検査仕様・知識・視点プランナ

## 実行

Day 2で保存した形状・実カメラ姿勢・RGBの証拠を検証し、同じ初期撮影を仕様X/Yで再評価する。
Isaac Simは起動せず、追加パッケージも不要。Day 1/2の起動コマンドは変更していない。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python src/day3_plan.py --input-dir outputs/day2-run-01b
```

入力は自分のDay 2保存先を指定する。`outputs/day2_run_01` も使用可能。
結果は `outputs/day3_run_01/plan_report.json` に保存する。入力画像・JSONは読み取りのみ。
既存の保存先は上書きしない。再実行時は新しい保存先を指定する。

```bash
python src/day3_plan.py --input-dir outputs/day2-run-01b --output-dir outputs/day3_run_02
python -m unittest discover -s tests -v
```

正常終了時は次の結果と `DAY3 COMPLETE: 11 checks PASS` を表示する。

| ケース | 要求領域 | 初期撮影による観測 | 計画結果 |
|---|---|---:|---|
| A・X | R1 | 63/63点 | 追加視点不要 |
| A・Y | R1、R2 | 126/126点 | 追加視点不要 |
| B・X | R1 | 63/63点 | 追加視点不要 |
| B・Y | R1、R2 | 66/126点 | `ready`、`rear`を選択、追加60点を予測 |
| B・Y・移動能力なし | R1、R2 | 66/126点 | `blocked: required_capability_unavailable` |
| B・Y・現在視点のみ | R1、R2 | 66/126点 | `blocked: no_candidate_improves_required_observation` |

表の点数は保存済みDay 2データを計算した結果で、固定の成功率ではない。
**`ready`は実行可能な計画を得た状態。まだカメラは動かさず、画像も追加しない。**
B・Yの実際の観測は計画後もR2=3/63点のまま。予測値を観測済みに加算しない。
既存の `B_alternate` 撮影を成功の証拠として適用せず、初期撮影と候補の幾何評価だけで選択する。

## 知識と計画の仕組み

`knowledge/inspection_knowledge.json` に次の関係を定義した。

- 仕様X → R1、仕様Y → R1とR2。
- 未観測原因 `self_occlusion` → 適用候補 `change_viewpoint`。
- `change_viewpoint` → 必要能力 `camera_pose_change`。
- 観測状態 `not_captured` → `captured` → `missing_observation` または `inspection_observation_satisfied`。

既存の観測状態から未観測点だけを取り、その最後の評価理由・撮影IDを保持する。
撮影しただけでは観測済みにせず、以前に観測済みの点が次の画像で隠れても累積観測からは除かない。
部品形状または仕様が変われば新しい状態を作り、以前の状態を引き継がない。
異なる形状・仕様・点IDをプランナに渡した場合はエラーにする。

知識は追加依存なしの最小JSON。手作業で記述したデモ用の関係であり、
`source=manually_authored_demo_knowledge`、`approval_status=prototype_not_approved` を明記する。
RDF/Turtleの推論器や専門家承認ワークフローは導入していない。
現在の知識で扱わない原因、例えば `surface_not_hit` しかなければ、対策を捏造せずblockedにする。

`config/camera_views.json` の3候補は全部品・仕様で共通。
部品座標の位置・注視点を、Day 1と同じUSDカメラ軸（+X右、+Y上、-Z前方）で姿勢に変換する。
`T_world_camera = T_world_part @ T_part_camera` でワールド姿勢を求め、
入力から読み戻したカメラの画角・クリッピングを維持し、既存の可視性計算で評価する。
プランナにはA/Bなどの部品名や欠陥ラベルを渡さない。

順位は以下の順で決める。

1. 必要かつ未観測の点について、新たに可視になる点数が多い候補。
2. 同点なら移動コストが小さい候補。
3. コストも同じなら候補IDの辞書順。

移動コストは並進距離[m] + 0.1[m/rad] × 姿勢変化角[rad]。
数値誤差による同点の揺れを避けるため、コスト比較は小数12桁に丸める。
このコストは比較用の値であり、ロボットの経路・所要時間・衝突安全性を表さない。
候補順を入れ替えても同じ結果を返す。

今回のB・Yでは `front` が追加0点、`high` が追加51点、`rear` が追加60点。
必要点が増える候補がなければblocked。候補なし、能力なし、適用方法なしも理由を保存する。
必要点が既に観測済みなら、移動能力がなくても追加視点は要求しない。

## 保存内容と実装

`plan_report.json` には入力JSONのパス・SHA-256、Day 2再検証の結果、知識と候補設定の
スナップショット、既存撮影の姿勢・描画参照時刻、仕様ごとの初期可視性、観測状態、
未観測理由、方法と必要能力の照合、全候補の追加点ID・点数・移動コスト・姿勢、選択理由を保存する。
`prediction_source=predicted_from_simulator_geometry`、`execution_status=not_executed`、
`defect_decision=not_evaluated` とし、予測・実行・欠陥判定を区別する。

| ファイル | 役割 |
|---|---|
| `src/inspection_knowledge.py` | 仕様・原因・方法・能力の読み込み、関係の整合性検証 |
| `src/viewpoint_planner.py` | 部品相対候補の予測、順位付け、理由付きblocked |
| `src/inspection_state.py` | 既存の観測状態に未観測理由の保持を追加。既存summary形式は維持 |
| `src/day3_plan.py` | 保存済みDay 2データの検証、6ケースの比較、計画保存 |
| `tests/test_inspection_knowledge.py` | 仕様と関係の参照、不正な定義の拒否 |
| `tests/test_viewpoint_planner.py` | 仕様差、能力、候補、決定性、座標変換、予測による状態変更の防止 |
| `tests/test_day3_runner.py` | 一連の比較、入力検証失敗、保存結果の上書き防止 |
| `tests/test_inspection_state.py` | 未観測理由の由来・累積・リセットの追加検証 |

## 検証結果（2026-10-08）と制限

既存Python環境で59ユニットテスト、保存済み `outputs/run` のDay 1再検証、
`outputs/day2-run-01b` のDay 2再検証、同データを使うDay 3受入条件11項目を確認した。
Day 3確認先: `/tmp/nedo-day3-validation-20261008-02/plan_report.json`。

Day 3は保存済みデータによる計画まで。Isaac SimのGUI起動・自動カメラ移動は今回の検証対象外。
幾何形状と領域はシミュレータ既知情報であり、画像認識・欠陥検出の精度は示さない。
有限の候補で改善しなければblockedになり、連続空間の最適視点がないことまでは意味しない。
実行→新しい撮影→再評価→次の計画、鮮度確認、最大操作回数は承認後のDay 4で接続する。
