# 開発・検証環境

## ワークスペース

親ワークスペース `GPTWorkspace` のLua 5.3、Stormworks API、lint設定を使用する。ゲーム仕様の参照元はワークスペースルートの `Stormworks 仕様書 (v7).docx`。

仕様書で確認したレーダー要点：

- 指向方位・仰角入力はturns
- 指向を変えても観測角はビーム中心ではなくレーダー筐体正面基準
- EffectiveRange 1900 mでは探知間隔は1 tick
- レーダーの真値更新時刻は全目標で同期する
- 生出力は1目標あたり距離・方位・仰角・経過tickの4 numberチャンネル

## 利用方針

- 読みやすいLuaソースを開発上の正本とする
- Lua 5.3ランタイムで構文とオフライン状態遷移を確認する
- Lua Language Serverで重大診断を確認する
- LifeBoat既定設定で各Luaブロックが8192文字以内か確認する
- ゲーム固有の飛行結果はStormworks内で確認する

## 再現可能なオフライン確認

- `tests/HGV Offline Test.lua`
  - 3点探索
  - 水平1500 mのTERMINALラッチ
  - 300 m/s接近目標のSAM判定
  - 3 tick失探
  - 最接近点通過
  - 90 tickごとの上下反転
  - CANDIDATE中の蛇行停止
  - SAM推定水平速度に直交し、通常HGVでは90 tick反転の上下蛇行を同時実行するビーム機動
  - Radar Bool入力2からGuidance Bool入力5へのシースキミング信号パススルー
  - シースキミング高度、プロパティ指定ポップアップ、DIVE距離条件と、EVADE中の非蛇行
- `tools/LifeBoat Size.lua`
  - VS Code拡張同梱のLifeBoatAPIを使って2ブロックを既定設定で圧縮する
  - 生成物は `_build/lifeboat/` に置く

## 現在のLifeBoat圧縮後文字数

| Luaブロック | 最小化本体 | 出力ファイル全体 | 8192文字以内 |
|---|---:|---:|---|
| `HGV Radar.lua` | 5854 | 5854 | はい |
| `HGV Guidance.lua` | 5094 | 5094 | はい |

2026-08-27にLifeBoat既定設定で再計測。圧縮後LuaもLua 5.3でロードし、HGVのEVADE上下蛇行を含む変更後の構文を確認した。

## Lua Language Server

2026-08-27に以下を `Error` レベルで確認し、重大診断なし。

- `HGV Radar.lua`
- `HGV Guidance.lua`
- `tests/HGV Offline Test.lua`

## デバッグログ

- 接頭辞：`[HGV_DBG]`
- 出力先：`debug.log`からDebugView++
- 文字連結：`..`
- Compositeのデバッグ専用output：使用しない
- 定期ログ：原則1秒ごと（60 tick）
- レーダー定期ログ：フェーズ、敵艦水平距離、`sea_skimming`、指向方位・仰角、当該tick観測数、保持トラック数、生距離 `r1_m`～`r6_m`
- 誘導定期ログ：フェーズ、高度、Yaw/Pitch指令、`beam_aspect_deg`、`sea_skimming`、`popup`
- `KINEMATICS`：有効SAMが500 m以内にいる間は毎tick。Physics Sensor世界位置の1 tick差分による実速度、速度差分による実加速度、SAM LOSに平行な成分を除いた`actual_lateral_accel_mps2`を記録する
- `KINEMATICS`の時刻照合にはHGV/AAそれぞれのtickではなく、DebugView++左端の共通時刻を使用する

主なイベント：

- `LAUNCH`
- `PHASE_CHANGE`
- `THREAT_CANDIDATE`
- `CANDIDATE_REJECTED`
- `THREAT_CONFIRMED`
- `THREAT_LOST`
- `THREAT_PASSED`
- `GUIDANCE_PHASE_CHANGE`
- `KINEMATICS`
- `STATUS` / `GUIDANCE_STATUS`
- `SEA_SKIM_POPUP`
- `SEA_SKIM_DIVE`

## Lua変更後の確認項目

- Lua 5.3構文
- Lua Language Server重大診断
- オフライン状態遷移試験
- LifeBoat圧縮後文字数と圧縮版ロード
- 入出力チャンネルとプロパティ
- 座標軸、角度符号、turns/rad
- 60 Hzに対するtick/秒変換
- 機体配線のロジック遅延
- 機動指令と実際の機体応答
- 発射前・SEARCH・CANDIDATE・EVADE・TERMINALの遷移
