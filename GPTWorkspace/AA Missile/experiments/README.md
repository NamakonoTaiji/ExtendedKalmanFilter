# 実験記録

Stormworks内で行った試験を、再現できる形で保存する場所です。

## 運用方法

1. `000-baseline-beam-manoeuvre.md` を複製する
2. 連番と短い名前を付ける（例：`001-baseline-left-beam.md`）
3. 試験前に設定値と変更点を記入する
4. 試験後に結果とログ・画像・動画の保存場所を記入する
5. 結論では、観測事実と推測を分ける

## ファイル名

```text
NNN-short-description.md
```

同じ条件を複数回試す場合も、試行ごとに結果を残すか、同じ記録内で試行番号を分けます。

## 記録一覧

- `000-baseline-beam-manoeuvre.md` — 初期の基準試験テンプレート
- `001-self-position-delay-compensation.md` — 自機位置遅延補償のA/B試験と正式採用根拠
- `002-aspect-and-logic-delay-regression.md` — 全接近角度と `LOGIC_DELAY=3～5` の退行試験
- `003-hgv-terminal-association.md` — HGV失敗時の終末トラック割当切れと方向ペナルティ除外A/B計画
- `004-hgv-guidance-delay-gain.md` — HGV迎撃時の`LOGIC_DELAY`とPNフィン係数の比較
