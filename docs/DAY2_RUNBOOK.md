# Day 2 — 部品A/Bと可視性

## 実行

既存デモを閉じ、Ubuntuデスクトップ上の端末で実行する。
Day 1の起動スクリプト・adapter・保存結果・仮想環境のパッケージは変更していない。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/day2_visibility.py
```

Playボタンの操作は不要。次の3ケースを順に撮影する。

1. A: 低いリブ、初期視点。
2. B: 高いリブ、Aと同じ初期視点。
3. B: カメラを約3秒で反対側へ動かした別視点。

端末で各ケースのR1/R2可視点数と理由を表示する。
`DAY2 COMPLETE` が出たら成功。完了後もGUIは残り、ウィンドウを閉じて終了する。
再実行時は新しい保存先を指定する。既存ディレクトリは上書きしない。

```bash
python -u src/day2_visibility.py --output-dir outputs/day2_run_02
```

自動終了・GUIなしの実行も可能。

```bash
python -u src/day2_visibility.py --exit-after-capture --output-dir outputs/day2_auto
python -u src/day2_visibility.py --headless --output-dir outputs/day2_headless
```

## 保存結果と確認

```text
outputs/day2_run_01/visibility_report.json
outputs/day2_run_01/images/A_initial.png
outputs/day2_run_01/images/B_initial.png
outputs/day2_run_01/images/B_alternate.png
outputs/day2_run_01/images/A_initial_visibility.png
outputs/day2_run_01/images/B_initial_visibility.png
outputs/day2_run_01/images/B_alternate_visibility.png
```

`*_visibility.png` はシミュレータ形状から計算した点をRGB画像に重ねた説明用画像。
元のRGBは同名の `*_visibility` が付かないPNG。形状の色と、後から重ねた点の色は別の意味を持つ。

| 表示 | 意味 |
|---|---|
| 部品上の緑色の長方形 | 検査領域R1の位置を示すマーカー |
| 部品上のオレンジ色の長方形 | 検査領域R2の位置を示すマーカー |
| 小さい緑の点 | その視点で可視と計算された対象面のサンプル点 |
| 小さい赤の点 | その視点で不可視と計算された対象面のサンプル点 |

点をRGBの上に描くため、点のあるピクセルは別の色で上書きされる。
マーカーの材質や形状を変更した表示ではない。隠れた対象点も投影位置に赤で表示するため、
赤点は手前のリブ上に重なって見える。そのリブ上に欠陥がある意味ではない。
画像認識で抽出した点でもない。現在視点の赤点は、過去の別視点で既に観測済みの場合もあるため、
累積の未観測点はJSONの `state_after_visibility` を確認する。

Aは最初に表示・撮影した後にBへ切り替わり、完了後のGUIにはBが残る。
Aの結果は `A_initial.png` と `A_initial_visibility.png` で確認できる。

検証コマンドはIsaac Simを起動しない。

```bash
python src/verify_day2.py
python src/verify_day2.py outputs/day2_run_02
python -m unittest discover -s tests -v
```

JSONには各撮影の実カメラ姿勢・描画参照時刻、実USD形状の三角形スナップショット、
点ごとの可視性・理由・最初のヒットPrim/面/距離、撮影前後と評価後の観測状態を保存する。
検証では保存された形状とカメラから可視性を再計算し、状態遷移も再実行して一致を確認する。

## 実装と前提

- `part_geometry.py`: 部品パラメータ、USD立方体の三角形化、領域サンプル、形状バージョン。
- `visibility.py`: カメラ投影、画角・クリッピング・入射角、CPUの三角形レイキャスト。
- `inspection_state.py`: 撮影と観測評価の分離、点IDの累積、形状・仕様変更時のリセット。
- `day2_adapter.py`: Day 1 adapterを継承し、リブ高さの変更と実USD形状・カメラ設定の読み戻しを追加。
- `day2_visibility.py`: 手動指定した3ケースの実行、撮影、オーバーレイ、結果保存。
- `verify_day2.py`: 保存済みの証拠・可視性・状態・受入条件を再検証。

Aのリブ高さは0.025 m、Bは0.12 m。領域R1/R2の形状・位置・点群は共通。
初期カメラの部品相対位置は `(0.85, -1.05, 0.75)` m、別視点は `(0.85, 1.05, 0.75)` m。
カメラは共通の `(0, 0, 0.08)` mを向く。座標規約はDay 1と同じ。
視点は手動であらかじめ指定した比較実験であり、部品名による自動視点選択ではない。

対象面はそれぞれのマーカー立方体の `z+` 面。9×7のセル中心に各63点を置く。
要件は明示的な `day2_all_surface_samples_v1` 仕様からR1/R2を取得する。
部品名から要求領域を決めず、仕様X/Yと知識層・プランナはDay 3で追加する。

各点は以下を満たしたときだけ可視とする。

1. 読み戻したカメラの透視画角内・クリッピング範囲内にある。
2. 表面法線と対象点からカメラへ向かう方向の角度が75度以内。
3. カメラからのレイの最初のヒットが、対象のPrim・対象面・対象位置に一致する。

位置一致の許容値は `1e-5` m。Primだけが同じ別面のヒットは成功にしない。
遮蔽物が部品の子Primなら `self_occlusion`、外部なら `scene_occlusion` と記録する。
割合は点ごとの計算結果から求め、成功率を固定値として埋め込まない。

撮影しただけの状態は `captured`。可視性を適用して必要な点がすべて観測された場合に
`inspection_observation_satisfied` となる。これは幾何的な観測要件の充足であり、
欠陥なしの意味ではない。`defect_decision` は常に `not_evaluated`。
形状がAからBへ変わったときは観測済み点・撮影履歴をリセットする。

## 検証結果（2026-10-08）

RTX 4080／Isaac Sim 6.1.0.0のheadless実行結果:

| ケース | R1 | R2 | 状態 |
|---|---:|---:|---|
| A・初期視点 | 63/63 | 63/63 | 観測要件充足 |
| B・同じ初期視点 | 63/63 | 3/63 | R2の60点が自己遮蔽 |
| B・別視点の単独評価 | 0/63 | 63/63 | R2の可視点増加 |
| B・2視点の累積 | 63/63 | 63/63 | 観測要件充足 |

確認先: `/tmp/nedo-day2-validation-20261008-01`。
`verify_day2.py` で形状の再計算と状態遷移が一致し、受入条件12項目がPASS。
RGBとオーバーレイを読み、形状差・遮蔽・別視点を確認した。

GUI実行（`--exit-after-capture`）も終了コード0で完了し、
`/tmp/nedo-day2-validation-20261008-gui-01` の保存結果の再検証がPASS。
36ユニットテストが成功し、既存Day 1の `outputs/run` も再検証PASS。
Day 1の4ソースファイル、元のナビゲーションデモ、仮想環境設定のSHA-256は作業前後で一致。
手動でウィンドウを閉じる終了経路の目視確認はユーザー側で行う。

## 制限

今回のレイキャストadapterは、このシーンで使うUSD Cubeを三角形化する。
任意のUSD Mesh・曲面・透明材質・レンズ歪みには対応していない。
サンプル点の可視性は表面全体の連続的な可視性や実画像の検査品質を保証しない。
点群はマーカー上面に定義したシミュレータ既知情報で、部品認識・姿勢推定は行わない。
Day 2の実行は手動指定視点の比較。知識とプランナの実装は [Day 3手順](DAY3_RUNBOOK.md) を参照。
ロボットアーム、衝突安全性、欠陥検出は未実装。

エラー時は非ゼロ終了コードと理由を出し、完了を装わない。
途中の画像が残った場合も上書きせず、再実行には新しい保存先を使う。
