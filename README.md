# Semantic Inspection Demo

NEDO応募用の動的3D検査デモ。品種変更による自己遮蔽を示し、
検査仕様と未観測情報から追加視点を選ぶループを7日間で段階的に実装する。

現時点ではDay 1の2視点RGB撮影とDay 2の部品A/B・幾何可視性比較を実装。
形状・領域はシミュレータ既知情報。欠陥検出、画像ベースの形状・姿勢推定、
知識層、プランナ、実機制御は未実装。

## 確認済み環境

Ubuntu 24.04、GeForce RTX 4080、NVIDIAドライバ595.91.07。
既存仮想環境のPython 3.12.3、Isaac Sim 6.1.0.0、NumPy 2.3.1、Pillow 12.3.0を利用。
Isaac Simの更新・追加パッケージのインストールは行っていない。

## 実行

既存環境の例。リポジトリのルートで実行する。

```bash
source /home/ishii/work/isaac/env_isaac/bin/activate
python -u src/day1_capture.py --output-dir outputs/day1_run_01
```

Day 1のGUIを閉じてからDay 2を実行する。

```bash
python -u src/day2_visibility.py
```

Play操作は不要。撮影後もGUIを維持する。`--headless` または
`--exit-after-capture` で自動終了する。既存の保存先は上書きしないため、
再実行時は `--output-dir outputs/day2_run_02` など新しい保存先を指定する。

## 検証

以下はIsaac Simを起動せずに実行できる。

```bash
python -m unittest discover -s tests -v
python src/verify_day1.py outputs/day1_run_01
python src/verify_day2.py outputs/day2_run_01
```

Day 2では同じ初期視点でAのR2が63/63点、BのR2が3/63点可視となり、
手動の別視点でBのR2が63/63点に増えた。割合は実USD形状から計算した結果で、
画像認識精度や実世界の検査精度ではない。保存した形状・姿勢・状態は再検証できる。

## ドキュメント

- [プロジェクトの目的と範囲](docs/PROJECT_CONTEXT.md)
- [7日間の開発計画](docs/DEVELOPMENT_PLAN.md)
- [Day 1の手順・座標規約](docs/DAY1_RUNBOOK.md)
- [Day 2の手順・可視性・制限](docs/DAY2_RUNBOOK.md)
- [開発ルール](AGENTS.md)

撮影画像・ログ・実行時キャッシュはGit管理対象から除外する。
