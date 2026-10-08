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

## 出力ファイルの読み方

`outputs/day3_run_01/plan_report.json` は、**「何を見る必要があり、何がまだ見えておらず、
次にどこから撮ればよさそうか」を記録した検討結果**。
Day 3で新しい画像を撮った記録ではなく、Day 2の保存データを使った計画の記録である。
画像そのものはこのJSONに入っておらず、元のDay 2保存先に残っている。

以下の点数・候補名は、確認した `outputs/day3_run_01/plan_report.json` の例。
形状や候補設定を変えて再実行すれば結果も変わる。
項目の場所は `cases.B_Y.plan.status` のように書く。
これはJSONの `cases` → `B_Y` → `plan` → `status` と順に開く意味。

### まずB・Yの結論を読む

エディタで `"B_Y"` を検索し、その中の次の項目を確認する。

| 読む場所 | 今回の値 | 平易な意味 |
|---|---|---|
| `state_before_planning.regions.R1.observed_count` | `63` | R1の必要な63点は全部観測できた |
| `state_before_planning.regions.R2.observed_count` | `3` | R2は63点のうち3点しか観測できていない |
| `plan.missing_reason_counts` | `{"self_occlusion": 60}` | 残り60点は部品自身のリブに隠れている |
| `plan.status` | `"ready"` | 追加撮影の案が決まった |
| `plan.action.view_id` | `"rear"` | 反対側の候補視点を選んだ |
| `plan.action.predicted_new_point_count` | `60` | そこから撮れば、未観測だった60点が新たに見えると予測した |
| `plan.execution_status` | `"not_executed"` | この計画はまだ実行していない |
| `state_after_planning.regions.R2.observed_count` | `3` | 計画しただけなので、観測済み点は増えていない |

つまり「R2はまだ3点しか観測していない。反対側へ移れば残り60点も見えそうなので、
その移動を提案した」という結果。`ready` と `missing_observation` が同時に出ても矛盾ではない。
前者は計画ができたこと、後者は必要な観測がまだ足りないことを表す。
実際の移動・再撮影による確認は [Day 4手順](DAY4_RUNBOOK.md) で行う。

### ファイル全体に書かれていること

JSONの一番外側には、比較全体の条件と結果が入る。

| 項目名 | 意味 |
|---|---|
| `schema_version` | このJSONの書式の版。`1` はDay番号や検査結果ではない |
| `mode` | 保存済みのシミュレータデータを使い、カメラを動かさず計画したこと |
| `execution_status` | `not_executed`＝Day 3では提案した移動を実行していない |
| `approval_status` | `prototype_not_approved`＝デモ用の試作結果で、承認済みの検査手順ではない |
| `defect_decision` | `not_evaluated`＝傷や欠陥の有無は判定していない |
| `coordinate_conventions` | 位置や向きをどう記録するかの約束。距離はメートル、上下方向やカメラの軸もここに記載 |
| `input_evidence` | どのDay 2データを使ったか、そのデータの確認結果 |
| `knowledge_snapshot` | 実行時に使った検査仕様・原因・対策・必要能力の定義を、そのまま保存したもの |
| `viewpoint_configuration` | 実行時に比較したカメラ位置・向きの候補と、移動コストの設定 |
| `cases` | 条件を変えた6ケースの詳しい結果 |
| `acceptance_checks` | プログラムが期待どおり判断したかを確かめる11項目の確認結果 |

`input_evidence.path` は元の `visibility_report.json` の場所。
`sha256` は元ファイルを識別するための「指紋」のような文字列で、点数や成功率ではない。
`day2_replay_verification: "passed"` は、Day 2の保存された形状・姿勢から計算し直して
保存結果と一致したこと。`additional_captures: 0` は、Day 3で撮影を増やしていないことを表す。

`knowledge_snapshot` の中の `specifications` は何を見る必要があるか、
`missing_observation_causes` は見えない原因と対策の関係、`methods` は対策に必要な能力、
`capabilities` はその能力の説明、`observation_states` は使う観測状態の名前。
`viewpoint_configuration.views` の `eye_part_m` は部品を基準にしたカメラ位置、
`target_part_m` はカメラを向ける先。これらは今回の計算に使った設定の控えである。

### 6ケースの名前

| `cases` 内の名前 | 比較している条件 |
|---|---|
| `A_X` | 低いリブのAを、R1だけ見る仕様Xで評価 |
| `A_Y` | 同じAを、R1とR2を見る仕様Yで評価 |
| `B_X` | 高いリブのBを、R1だけ見る仕様Xで評価 |
| `B_Y` | 同じBを、R1とR2を見る仕様Yで評価 |
| `B_Y_motion_disabled` | B・Yで、カメラを動かす能力がないと仮定した比較 |
| `B_Y_current_view_only` | B・Yで、今と同じ視点しか候補がないと仮定した比較 |

6回の新しい撮影ではない。Aのケースは既存の `A_initial`、Bのケースは既存の
`B_initial` を使い、検査仕様や使える能力・候補を変えて比較している。

### 各ケースにある撮影情報と観測状態

| 項目名 | 意味 |
|---|---|
| `source_case_id` | 元にしたDay 2の撮影名。例：`B_initial` |
| `source_capture` | その既存撮影の画像ファイル名、実カメラ位置・向き、時刻など |
| `state_before_capture` | 既存撮影をこのケースの状態に登録する前。観測済み点は0から始める |
| `state_after_capture` | 撮影済みの証拠を登録した直後。まだ何点見えたかの評価は適用していない |
| `baseline_visibility` | 既存の初期視点から、必要な点のどれが見えるかを計算した結果 |
| `state_before_planning` | その可視性評価を適用した状態。これを見て次の撮影を考える |
| `plan` | 次の行動案、選んだ理由、候補の比較結果 |
| `state_after_planning` | 計画後の観測状態。Day 3では計画前と同じで、観測済み点を増やさない |

`source_capture.image` の `images/B_initial.png` は、**Day 2の保存先からの相対パス**。
今回なら `outputs/day2-run-01b/images/B_initial.png` を指し、Day 3保存先の画像ではない。
`world_position_m` はシーン全体を基準にした実カメラ位置。
`T_world_part` は部品、`T_world_camera` はカメラの位置・向き、
`T_part_camera` は部品を基準にしたカメラの位置・向きを表す4×4の数値表。
結論だけを読むなら、この数値表を手で解釈する必要はない。
`source_capture` の姿勢は撮影時の実測記録、`plan.action` の姿勢は提案先という違いがある。

`render_reference_time` は描画側の時刻を分子・分母で記録したもの。
`readback_utc` は画像を読み出したときのPCのUTC時刻で、日本時間はUTCに9時間を加える。
どちらも元のDay 2撮影の記録で、Day 3に追加撮影した時刻ではない。
`world_orientation_wxyz` はカメラの向きを4個の数値（w、x、y、zの順）で表したもの。
`timeline_seconds_at_readback` は画像読み出し時のシミュレータ内の時刻、
`app_update_count` はadapterが画面更新を呼んだ回数で、撮影枚数ではない。

各観測状態の `regions` にある数字は次の意味。

| 項目名 | 意味 |
|---|---|
| `observed_count` | これまでに観測できた必要点の数 |
| `total_count` | その領域で観測する必要がある点の総数。今回は各63点 |
| `coverage` | 観測済み点数÷必要点数。B・YのR2は `3/63 ≒ 0.0476`、約4.76% |
| `missing_point_ids` | まだ観測していない点の番号一覧。`R2:000` などは点を識別する名前 |
| `capture_ids`（状態全体） | その観測状態に登録した撮影名の一覧 |
| `inspection_observation_satisfied`（状態全体） | 必要点を全部観測できたか。`true`＝はい、`false`＝まだ足りない |

`coverage` はサンプル点の割合。画像の面積比、傷の検出率、良品率ではない。
`status` は `not_captured`＝撮影未登録、`captured`＝撮影登録済み・観測評価未適用、
`missing_observation`＝観測不足、`inspection_observation_satisfied`＝必要点の観測が充足、という段階を表す。

`baseline_visibility.regions.R2` の `visible_count` は初期視点だけで見える点数。
`target_prim_path` は対象領域のシーン内の名前、`target_face_id: "z+"` はその上面を指す。
`reason_counts` は見える／見えない理由ごとの点数で、今回のB・Yは `visible: 3`、`self_occlusion: 60`。
`points` には点ごとの位置、画像上の投影位置、可視性と理由が入る。
`point_part_m` は部品を基準にした点の位置、`point_world_m` はシーン全体を基準にした点の位置。
`pixel_uv` はその点を画像へ重ねる位置、`visible: false` はその視点で見えないこと。
`hit` はカメラから対象点へ向けた線が最初に当たった面で、例えば `hit.prim_path` が
`/World/Part/Rib` なら途中でリブに遮られたことが分かる。`hit: null` はヒット記録がないことを表す。
これらはシミュレータの形状から計算した説明で、画像から傷を検出した記録ではない。

### `plan` にある行動案と理由

| 項目名 | 意味 |
|---|---|
| `status` | `ready`＝追加撮影案あり、`inspection_observation_satisfied`＝追加撮影不要、`blocked`＝現在の条件で進められない |
| `reason` | その結論になった理由。下表のコードで記録 |
| `required_region_ids` | 今の仕様で見る必要がある領域。XはR1、YはR1とR2 |
| `inspection_spec_id` / `inspection_spec_version` | 使った検査仕様の名前と版 |
| `part_geometry_version` | どの部品形状についての計画かを識別する文字列。品種名や成功率ではない |
| `observation_state` | 判断の根拠にした観測状態。`state_before_planning` と同じ内容 |
| `missing_evidence` | 未観測点ごとの理由と、その判断に使った撮影名 |
| `missing_reason_counts` | 未観測理由ごとの点数。B・Yでは自己遮蔽が60点 |
| `available_capability_ids` | 使える能力。`camera_pose_change` はカメラ移動能力あり、`[]` は使える能力なし |
| `method_lookup` | 見えない原因に使える対策と、必要な能力を知識から調べた結果 |
| `candidate_evaluations` | 各候補について、追加で見えそうな点数と移動コストを計算した結果 |
| `action` | 選んだ行動案。`null` なら今回の追加行動案はない。不要なのか実行不能なのかは `status` と `reason` で分かる |
| `ranking` | 候補を比べる順番。追加点数を優先し、同点なら移動コスト、その次に候補IDを比べる |
| `visibility_parameters` | 可視性計算の条件。表面を見る角度の上限と、線のヒット位置の許容誤差 |
| `prediction_source` | `predicted_from_simulator_geometry`＝既知のシミュレータ形状からの予測 |

`method_lookup` の `applicable_causes: ["self_occlusion"]` は「自己遮蔽に使える対策」、
`method_id: "change_viewpoint"` は「視点を変える」、
`required_capability_ids: ["camera_pose_change"]` は「そのためにはカメラを動かす能力が必要」という意味。
`missing_capability_ids: []` なら必要能力に不足なし、`["camera_pose_change"]` なら移動能力が足りない。
追加撮影不要や能力不足で候補評価に進まなかった場合、`candidate_evaluations` は空の `[]` になる。

| `reason` の値 | 平易な意味 |
|---|---|
| `required_samples_already_observed` | 必要な点は既に全部観測できている |
| `eligible_method_and_capability_matched` | 見えない原因に合う対策があり、必要な能力も使える |
| `required_capability_unavailable` | 対策に必要な能力が使えない |
| `no_candidate_improves_required_observation` | どの候補でも未観測点を追加で見られない |

今回のB・Yの `candidate_evaluations` は次の比較になっている。コストは小数第3位までの表示。

| `view_id` | 位置の意味 | `predicted_new_point_count`（追加点数の予測） | `motion_cost_m_equivalent`（比較用コスト） |
|---|---|---:|---:|
| `front` | 今と同じ初期視点 | 0 | 0.000 |
| `high` | 初期視点より高い位置 | 51 | 0.681 |
| `rear` | 反対側 | 60 | 2.278 |

`rear` は移動コストが大きくても、追加点数が最も多いので選ばれた。
`predicted_new_point_ids` は新たに見えると予測した未観測点の番号一覧。
既に見えた点を重複して追加点数へ数えてはいない。
`translation_distance_m` は現在位置から候補位置への直線距離[m]、
`rotation_distance_rad` はカメラの向きの変化量[rad]。
`motion_cost_m_equivalent` はこの距離と角度を合わせた比較用の値で、撮影時間やロボットの実移動経路ではない。
選ばれた候補の内容が `action` に入り、`method_id: "change_viewpoint"` とともに保存される。

### 最後の `acceptance_checks` を読む

ここはデモの動作確認。`true` はその確認項目を満たしたという意味で、
すべてのケースで検査を完了した、あるいは部品に欠陥がないという意味ではない。
能力なしの条件で正しくblockedになることも、期待どおりなので `true` になる。

| 項目名 | 確認していること |
|---|---|
| `spec_X_requires_only_R1` | XではR1だけを要求した |
| `spec_Y_requires_R1_and_R2` | YではR1とR2を要求した |
| `A_needs_no_additional_view` | Aは初期撮影だけで必要点を満たした |
| `B_X_needs_no_additional_view` | BでもXなら追加視点は不要と判断した |
| `B_Y_selects_improving_view` | B・Yでは追加点が見えると予測した視点を選んだ |
| `no_camera_motion_is_blocked` | 移動能力なしならblockedにした |
| `no_improving_view_is_blocked` | 改善する視点がなければblockedにした |
| `part_and_spec_start_with_empty_observations` | 部品・仕様の異なるケースへ以前の観測済み状態を持ち越さなかった |
| `planning_does_not_change_observations` | 計画しただけでは観測済み点を増やさなかった |
| `no_predicted_motion_claimed_as_execution` | 提案した移動を実行済みと表示しなかった |
| `no_defect_decision_claimed` | 欠陥の判定結果を出さなかった |

## 実装ファイル

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
実行→新しい撮影→再評価→次の計画、鮮度確認、最大操作回数は [Day 4](DAY4_RUNBOOK.md) で接続済み。
Day 3のコマンドは引き続き保存データからの計画だけを行う。
