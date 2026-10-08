# Day 4 — 計画・移動・撮影・観測更新の閉ループ

## 最初に実行するコマンド

既存のIsaac Simウィンドウを閉じ、Ubuntuデスクトップの端末で実行する。
Day 1～3の起動方法、adapter、知識・候補設定、既存保存データは変更していない。
追加パッケージのインストールも不要。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
cd /home/ishii/work/nobique/nedo-demo
python -u src/day4_closed_loop.py --part B --spec Y
```

Play操作は不要。次の順に自動実行する。

1. Bを共通の初期視点 `front` で撮影。R1=63/63、R2=3/63点を観測する。
2. 知識から自己遮蔽→視点変更→必要な移動能力を照合し、全候補を評価する。
3. プランナが選んだ `rear` へカメラを約3秒で移動する。
4. 到達姿勢を読み戻し、同期描画後に新しいRGBを撮影する。
5. 実際の撮影姿勢・形状から再評価し、R2の追加60点を観測済みに加える。
6. 次の計画で観測要件の充足を確認し、操作を停止する。

正常終了は `DAY4 COMPLETE | status=inspection_observation_satisfied | actions=1`。
処理後もGUIは残り、ウィンドウを閉じて終了する。
最後の視点ではオレンジ色のR2が手前に見える。緑色のR1は隠れるが、初期撮影で観測済みなので累積状態は充足する。
各段階の計画と累積観測点数は端末に表示する。GUI内の操作パネルはDay 5で追加する。

## 通常条件・仕様差・停止条件の確認

各コマンドは前のGUIを閉じてから実行する。保存先は毎回新しくする。
通常のB・Yも一覧に含め、比較用の保存先を指定する。移動は有効、追加操作上限は既定の3回。

```bash
python -u src/day4_closed_loop.py --part A --spec Y --output-dir outputs/day4_A_Y_01
python -u src/day4_closed_loop.py --part B --spec X --output-dir outputs/day4_B_X_01
python -u src/day4_closed_loop.py --part B --spec Y --output-dir outputs/day4_B_Y_01
python -u src/day4_closed_loop.py --part B --spec Y --disable-camera-motion --output-dir outputs/day4_blocked_01
python -u src/day4_closed_loop.py --part B --spec Y --max-actions 0 --output-dir outputs/day4_limit_01
```

| 条件 | 期待する結果 | 撮影数／追加移動数 |
|---|---|---|
| A・Y | 初期撮影でR1/R2充足 | 1／0 |
| B・X | 要求はR1だけなので初期撮影で充足 | 1／0 |
| B・Y | 追加撮影でR1/R2の累積観測を充足 | 2／1 |
| B・Y・移動能力なし | `blocked: required_capability_unavailable` | 1／0 |
| B・Y・上限0 | `blocked: max_actions_reached` | 1／0 |

`blocked` は未観測を残して理由付きで停止した正常な制御結果。`DAY4 COMPLETE` のstatusもblockedとなる。
観測充足と混同しない。撮影・姿勢・鮮度などの検証エラーは `DAY4 FAILED` と非ゼロ終了コードになる。
デフォルトの追加操作上限は3回。候補の再実行を避け、改善なし・適用方法なし・候補なしでも停止する。

GUIの自動終了、headless実行、再実行の例:

```bash
python -u src/day4_closed_loop.py --exit-after-run --output-dir outputs/day4_auto_01
python -u src/day4_closed_loop.py --headless --output-dir outputs/day4_headless_01
python -u src/day4_closed_loop.py --output-dir outputs/day4_run_02
```

## 保存結果

デフォルトのB・Yでは次を保存する。初期撮影だけで停止した場合は `capture_00` のみ。
既存ディレクトリは上書きしない。

```text
outputs/day4_run_01/closed_loop_report.json
outputs/day4_run_01/events.jsonl
outputs/day4_run_01/images/B_Y_capture_00.png
outputs/day4_run_01/images/B_Y_capture_00_visibility.png
outputs/day4_run_01/images/B_Y_capture_01.png
outputs/day4_run_01/images/B_Y_capture_01_visibility.png
```

通常のPNGがRGB、`*_visibility.png` がその視点の可視性を重ねた説明画像。
マーカーと点の色の意味は [Day 2手順](DAY2_RUNBOOK.md#保存結果と確認) を参照。
赤い点は現在視点で不可視という意味で、欠陥や累積未観測を示す色ではない。
累積の観測／未観測は `state_after_visibility` または `final_state` を読む。

`events.jsonl` は一行一イベントで、イベントごとに追記・flushする。
セッション開始、撮影開始・完了、観測更新、計画、移動開始・完了、停止を記録する。
計画の `not_executed` と、実姿勢を検証した移動完了の `executed_pose_verified` は別の記録になる。
操作IDと撮影IDで、どの計画を実行し、どの画像で観測が増えたかを追跡できる。
途中失敗でも記録済みのイベント・画像を残し、失敗ステータスを保存する。

`closed_loop_report.json` は知識・候補設定のスナップショット、実カメラ姿勢・描画参照時刻、
形状・領域点群、予測、実際の点ごとの可視性、累積状態、停止理由、受入条件をまとめる。
RGB配列のSHA-256も保存し、画像の変更を再検証時に検出する。
全結果は `prototype_not_approved`、欠陥判定は `not_evaluated`。

## 鮮度と停止の確認

撮影は既存adapterの同期手順（timeline play/commit、Replicator `rt_subframes=4`、
`wait_for_render=True`、同じRenderProductのRGBとReferenceTime）を使う。
次の条件を確認してから観測を適用する。

- 描画参照時刻が前の撮影より新しい。古いフレームは観測に加えない。
- 実行後のカメラ姿勢が選択した姿勢と一致する。部品相対→ワールド変換も検証する。
- 撮影前後の形状・部品姿勢・対象点、カメラ姿勢・光学設定が一致する。
- 撮影メタデータの姿勢が読み戻した姿勢と一致し、RGBの形式・解像度が正しい。
- 本デモでは異なる視点でRGBが完全一致した場合も、鮮度を確認できないため停止する。

予測点数だけでは状態を更新しない。移動しても実際の追加観測が0点なら
`blocked: executed_view_did_not_improve_observation` で停止する。
形状・仕様の不整合はセッション再開始を要求するエラーとし、以前の観測を流用しない。

## 検証コマンドと確認結果（2026-10-08）

保存結果の再検証はIsaac Simを起動しない。

```bash
python src/verify_day4.py outputs/day4_run_01
python -m unittest discover -s tests -v
```

再検証では保存RGB・形状・実姿勢を読み、知識→計画→移動記録→撮影→観測更新のループを
再実行して、全イベント・状態・RGBハッシュが一致することを確認する。
JSONLとレポートが不一致の場合や、計画・移動姿勢・観測点数が変更された場合も失敗する。

RTX 4080／Isaac Sim 6.1.0.0でB・YのheadlessとGUI（自動終了）を実行し、双方が終了コード0。
初期R1=63/63・R2=3/63、実移動1回・撮影2回で累積R1=63/63・R2=63/63。
保存RGBでも、初期視点で隠れたR2が追加視点で見えることを確認した。
各保存結果の再検証8項目がPASS。

- headless: `/tmp/nedo-day4-validation-20261008-01`
- GUI: `/tmp/nedo-day4-validation-20261008-gui-01`

75ユニットテストがPASS。A・Y、B・X、能力なし、上限、複数操作の再計画、
古い時刻・同一RGB・誤った姿勢・形状変更の拒否、実観測が改善しない場合、ログ変更をテストした。
A・Yや能力なし等の分岐はコア／runnerテストで確認し、今回の実GPU実行はB・Yを対象とした。
Day 1・Day 2の既存保存結果、Day 3の保存データからの計画比較も再検証PASS。
ユーザーがGUIを手動で閉じる操作と見やすさの確認は、上記コマンドで行う。

## 実装と制限

| ファイル | 役割 |
|---|---|
| `src/closed_loop.py` | Isaac非依存の制御、鮮度・姿勢・形状の検証、状態更新、停止条件 |
| `src/day4_adapter.py` | 既存adapterを継承。部品相対姿勢設定、並進と姿勢の補間移動 |
| `src/day4_closed_loop.py` | 実環境起動、A/B・X/Y指定、RGB・イベント・結果保存 |
| `src/verify_day4.py` | 保存済み実行の再検証 |
| `tests/test_closed_loop.py` | 実行順序・制御分岐・不正な観測の拒否 |
| `tests/test_verify_day4.py` | 状態・姿勢・画像・イベントの不一致検出 |
| `tests/test_day4_runner.py` | 起動失敗の終了コード、保存と検証、上書き防止 |
| `tests/loop_fixtures.py` | テスト専用の合成adapter。実デモでは使用しない |

カメラを直接移動するデモで、ロボットアームの軌道・衝突・安全性は評価しない。
形状・領域・姿勢はシミュレータ既知情報。RGBからの形状認識・欠陥検出は行っていない。
有限の候補と現在の知識で解決できない場合はblockedとなる。
画面内のA/B・X/Y切替、step/run/reset、レシピ候補の保存は承認後のDay 5で追加する。
