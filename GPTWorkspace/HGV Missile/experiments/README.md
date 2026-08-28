# HGV Missile 実験記録

Stormworks内で行った単体飛行試験と、将来の防空ミサイル連携試験を保存する。

## 運用方法

1. `000-hgv-baseline.md` を複製する
2. `NNN-short-description.md` の形式で連番を付ける
3. 試験前に機体版、Lua版、プロパティ、初期条件を記入する
4. 試験後にログ、結果、異常を記録する
5. 観測事実と原因の推測を分ける
6. 同一条件の反復回数と成功数を記録する

1回の比較で変更する要因は原則1つとする。

## 記録一覧

- `000-hgv-baseline.md` — 機動なし基準飛行のテンプレート
- `001-sam-beam-no-detection.md` — SAM連携時にレーダー観測0でEVADEへ遷移しなかった試験
- `002-sam-detected-shallow-beam.md` — SAM探知後、接近時に進路交差角が浅くなった試験と速度基準への変更
