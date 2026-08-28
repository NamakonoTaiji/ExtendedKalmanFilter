# HGV Missile インターフェース台帳

## 現在の状態

`HGV Radar.lua` と `HGV Guidance.lua` 間の論理インターフェースは採用済み。チャンネル配置は流用元 `ADSV3 MissileKF.lua` / `ADSV3 Missile.lua` との互換を優先した。

Stormworks機体側のComposite配線は未照合である。特に生レーダー出力は1目標4 numberチャンネルだが、本Luaは流用元と同じ1目標3 numberチャンネルを受けるため、探知経過tickを除く既存の再配置配線が必要。実配線確認までは「コード採用・機体配線TBD」とする。

## 時間・座標・角度

- 1 tick：1/60秒
- 世界座標：X東、Y上、Z北、左手系
- 想定機体ローカル座標：X右、Y上、Z前（実機確認TBD）
- レーダー指向入力：turns
- レーダー観測方位・仰角：turns、レーダー筐体正面基準
- SEARCH時の指向仰角：クランプ後の敵艦仰角の1/2 turns。敵艦発射位置と水平線を垂直75.6度ビーム内へ収める
- 内部角度・角速度：rad / rad/s
- 位置：m、速度：m/s、加速度：m/s²

## HGV Radar.lua 入力

### number

| ch | 内容 | 単位 | 状態 |
|---:|---|---|---|
| 1-18 | 目標1-6。各3chは距離、方位、仰角 | m, turns, turns | コード採用・再配置配線TBD |
| 19-21 | データリンク敵艦速度 Vx,Vy,Vz | m/s | 採用 |
| 22-24 | データリンク敵艦位置 X,Y,Z | m | 採用 |
| 25-27 | 自機位置 X,Y,Z | m | 採用 |
| 28-30 | 自機姿勢 Pitch,Yaw,Roll | rad | 流用採用・符号TBD |
| 31 | データリンク経過時間 | tick | 予約・未使用 |
| 32 | データリンク目標ID | 整数 | 採用 |

### bool

| ch | 内容 | 状態 |
|---:|---|---|
| 1 | 既存レーダー検出信号 | bool 32へパススルー |
| 2 | シースキミングモード | Bool出力5へパススルー |

### データリンクID手順

- 発射前フレーム：`100000 + 90000 + 生目標ID`
- `90000`加算は対水上モードを表す流用元互換値
- 発射後フレーム：同じ生目標ID
- 発射前に非ゼロ座標を伴う完全フレームを受信してから、同じ生目標IDの発射後フレームを受け入れる

送信側がこの手順を使用しているかは実配線試験で確認する。

## HGV Radar.lua 出力 / HGV Guidance.lua 入力

### number

| ch | 内容 | 単位 | 状態 |
|---:|---|---|---|
| 1-3 | 敵艦位置 X,Y,Z | m | 採用 |
| 4-6 | 敵艦速度 Vx,Vy,Vz | m/s | 採用 |
| 7-9 | 有効SAM位置 X,Y,Z。無効時0 | m | 採用 |
| 10-11 | レーダー指向方位・仰角 | turns | 採用 |
| 12 | 有効SAMトラックID。無効時0 | 整数 | 採用 |
| 13-15 | 自機位置 X,Y,Zパススルー | m | 採用 |
| 16-19 | 自機クォータニオン w,x,y,z | 無次元 | 流用採用・実機確認TBD |
| 20-22 | 有効SAM速度 Vx,Vy,Vz。無効時0 | m/s | 採用。EVADEの水平直交基準に使用 |
| 23 | 0待機、1 SEARCH、2 CANDIDATE、3 EVADE、4 TERMINAL | 整数 | 採用 |
| 24-31 | 予約 | - | 未使用 |
| 32 | 予約 | - | 0固定 |

### bool

| ch | 内容 | 状態 |
|---:|---|---|
| 1 | 有効SAMあり | 採用 |
| 2 | 対水上モード | true固定 |
| 3 | 発射済み | 採用 |
| 4 | TERMINALラッチ | 採用 |
| 5 | シースキミングモード | Radar入力bool 2のパススルー。Guidanceで使用 |
| 32 | 入力bool 1パススルー | 流用互換 |
Radar出力 / Guidance入力のBool ch5と、下記Guidance出力のBool ch5は別のComposite接続面である。

## HGV Guidance.lua 出力

### number

| ch | 内容 | 単位 | 状態 |
|---:|---|---|---|
| 1 | Yawフィン指令 | 正規化値または機体依存 | コード採用・配線/上限TBD |
| 2 | Pitchフィン指令 | 正規化値または機体依存 | コード採用・配線/上限TBD |
| 3 | 敵艦3D距離の1 tick差分 | m/tick | 流用互換 |

### bool

| ch | 内容 | 状態 |
|---:|---|---|
| 1 | 予約 | false固定 |
| 2 | 対水上モード | true固定 |
| 3 | 主レーダーON | 発射後true |
| 4 | 自爆 | 未採用、false固定 |
| 5 | 衝撃信管 | TERMINALかつ敵艦3D距離500 m未満の暫定流用。実配線TBD |

診断専用Composite出力は追加しない。診断は `[HGV_DBG]` の `debug.log` を使用する。

`HGV Guidance.lua`は入力number 13-15のPhysics Sensor世界位置を1 tick差分して実速度、さらに差分して実加速度を診断専用に算出する。有効SAMが500 m以内の間、SAM LOSに平行な加速度成分を除いた実横加速度を`[HGV_DBG] KINEMATICS`へ毎tick記録する。誘導計算・Composite出力・プロパティは変更しない。

## 固定設計値

| 名称 | 値 | 単位 |
|---|---:|---|
| `RADAR_EFFECTIVE_RANGE` | 1900 | m |
| `TERMINAL_HORIZONTAL_RANGE` | 通常モード1500（RadarとGuidanceへ同値設定） | m |
| `SAM_MIN_SPEED` | 250 | m/s |
| `SCAN_OFFSETS` | -20, 0, +20 | degree |
| `CRUISE_ALTITUDE_M` | 2000 | m |
| `VERTICAL_REVERSAL_TICKS` | 90 | tick |
| `SEA_SKIM_ALTITUDE_M` | 現行10、初期合意20（採用確認TBD） | m |
| `SEA_SKIM_POPUP_RANGE_M` | `SEA_SKIM_POPUP_RANGE`プロパティ値（オフライン1000） | m |
| `SEA_SKIM_POPUP_ALTITUDE_M` | `SEA_SKIM_POPUP_ALTITUDE`プロパティ値（オフライン400） | m |
| `SEA_SKIM_DIVE_RANGE_M` | 現行700未満、初期合意500未満（採用確認TBD） | m |

## HGV Radar.lua プロパティ

| プロパティ | コード内フォールバック | 状態 |
|---|---:|---|
| `TERMINAL_HORIZONTAL_RANGE` | なし（機体プロパティ必須） | 通常モード1500 mを設定 |
| `SEA_SKIM_POPUP_RANGE` | なし（機体プロパティ必須） | Guidanceと同じポップアップ水平距離を設定 |
| `TRACK_GATE_M` | 150 m | オフライン初期値、Stormworks調整TBD |
| `DL_GATE` | 200 m | オフライン初期値、Stormworks調整TBD |
| `TRACK_DELETE_TICKS` | 60 tick | オフライン初期値 |
| `THREAT_LOST_TICKS` | 3 tick | 採用 |
| `CANDIDATE_MIN_HITS` | 12 hit | オフライン初期値、ノイズ試験TBD |
| `CANDIDATE_MAX_TICKS` | 45 tick | オフライン初期値、ノイズ試験TBD |

## HGV Guidance.lua プロパティ

| プロパティ | 単位 | 状態 |
|---|---|---|
| `TERMINAL_HORIZONTAL_RANGE` | m | Radarと同じ通常モード値1500を設定 |
| `SEA_SKIM_POPUP_RANGE` | m | Radarと同じポップアップ水平距離を設定 |
| `SEA_SKIM_POPUP_ALTITUDE` | m | ポップアップ世界Y高度を設定 |
| `PN_FIN_STRENGTH` | 機体依存 | TBD |
| `PPN_FIN_STRENGTH` | 機体依存 | TBD |
| `VERTICAL_MANEUVER_AMPLITUDE_M` | m | TBD。0なら蛇行無効。通常HGVのSEARCH/EVADEで使用 |
| `LOGIC_DELAY` | tick | 配線確認TBD |
| `GRAVITY_COMP_ENABLE` | bool | TBD |
| `GRAVITY_COMP_GAIN` | 機体依存 | TBD |
| `MAX_FIN_COMMAND` | 機体依存 | TBD。0以下ならクランプ無効 |

## 機体接続時の確認項目

- 生レーダー4ch/目標からLua用3ch/目標への再配置
- RCS順で入れ替わる目標の追跡継続
- boolとnumberの同一チャンネル併用
- turnsとradの変換
- Physics Sensor姿勢の順序・符号
- Yaw/Pitchフィンの正負
- レーダー入力からLua観測までのtick遅延
- Luaからフィンまでのtick遅延
- 発射前後のデータリンクID手順
