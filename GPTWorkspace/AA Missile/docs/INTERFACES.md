# 現行インターフェース

この文書は、現在のLua実装を基準にしたチャンネル・プロパティ一覧です。配線図そのものではないため、Stormworks側のマイコン配線と照合して使用してください。

## ADSV3 MissileKF.lua

### 数値入力

| チャンネル | 内容 | 単位・備考 |
|---:|---|---|
| 1–18 | レーダー目標1–6 | 1目標につき距離、方位角、仰角の3ch。角度入力はturns |
| 19–21 | データリンク目標速度 Vx, Vy, Vz | m/s |
| 22–24 | データリンク目標位置 X, Y, Z | Physics Sensor世界座標 |
| 25–27 | 自機位置 X, Y, Z | Physics Sensor世界座標 |
| 28–30 | 自機姿勢 Pitch, Yaw, Roll | rad |
| 31 | データリンク目標の経過時間 | 現行コードでは未使用 |
| 32 | データリンク目標ID | 発射前フレームは100000加算、対水上識別も符号化 |

注記：ファイル先頭コメントには19–21を「レーダー目標7」と読める記載もありますが、現在の実装は `MAX_RADAR_TARGETS = 6` であり、19–21をデータリンク速度として読み取ります。本表は実装を優先しています。

### 出力

| 種別・チャンネル | 内容 |
|---|---|
| bool 1 | 有効な追跡目標を出力中 |
| bool 2 | 対水上モード |
| bool 3 | 発射済み状態 |
| num 1–3 | 推定目標位置 X, Y, Z。有効トラックは `LOGIC_DELAY` tick先 |
| num 4–6 | 推定目標速度 Vx, Vy, Vz |
| num 10–11 | レーダー指向方位角・仰角（turns） |
| num 12 | プライマリトラックID |
| num 13–15 | 自機位置パススルー |
| num 16–19 | 自機姿勢クォータニオン w, x, y, z |
| num 32 | 最新のマハラノビス距離 epsilon |
| bool 32 | ミサイル出力用レーダーの検出中信号。KF入力bool 1をそのままパススルー |

有効トラックのレーダービーム指向では、Physics Sensor位置の1階差分と2階差分から自機の速度・加速度を求め、目標と同じ `LOGIC_DELAY` tick先へ予測した自機位置を使用する。姿勢も現在と1 tick前のクォータニオン差分から同じ時刻へ正規化外挿し、予測目標をミサイルローカル座標へ変換する。`q` と `-q` が同じ姿勢を表すことによる符号反転は、前回値との内積で最短側へそろえる。レーダー生観測の世界座標化とEKF更新は現在位置・現在姿勢を使用し、num 13–19も従来どおり現在の位置・姿勢を出力する。

### プロパティ

- `D_ASOC_EPS`
- `T_LOST`
- `PRED_UNCERTAINTY_FACT`
- `RADAR_EFFECTIVE_RANGE`
- `DL_GATE`
- `NOISE_TARGET_EPS`
- `NOISE_I_GAIN`
- `LOGIC_DELAY`

データアソシエーションでは通常、マハラノビス距離epsilonに加えて、観測残差が推定速度方向から外れるほどepsilonを増幅する。発射後に固定中の主トラックだけはこの方向ペナルティを適用せず、`D_ASOC_EPS`による通常ゲートは維持する。他トラックと固定前の候補には従来どおり方向ペナルティを適用する。

## ADSV3 MissileFOVDebug.lua

FOV計測専用の別Luaブロック。既存信号を次のチャンネルへ再配置して入力する。外部出力は持たない。

### 入力

| 種別・チャンネル | 内容 | 接続元・備考 |
|---|---|---|
| num 1–12 | 終末レーダー目標1–4 | KFへ入る目標1–4と同じ距離・方位角・仰角。角度はturns |
| num 13–15 | KF推定目標位置 X, Y, Z | `ADSV3 MissileKF.lua`出力1–3を分岐。`LOGIC_DELAY` tick先の世界座標 |
| num 16–18 | KF推定目標速度 Vx, Vy, Vz | `ADSV3 MissileKF.lua`出力4–6を分岐。m/s |
| num 19–21 | データリンク目標位置 X, Y, Z | `ADSV3 MissileKF.lua`入力22–24と同じ世界座標を再配置 |
| num 22–24 | 自機位置 X, Y, Z | `ADSV3 MissileKF.lua`出力13–15を再配置 |
| num 25–28 | 自機姿勢クォータニオン w, x, y, z | `ADSV3 MissileKF.lua`出力16–19を再配置 |
| num 29–30 | レーダー指向方位角・仰角 | `ADSV3 MissileKF.lua`出力10–11を再配置。turns |
| num 31 | データリンクが現在送信中の目標ID | `ADSV3 MissileKF.lua`入力32と同じ信号。発射前は100000加算 |
| num 32 | KFプライマリトラックID | `ADSV3 MissileKF.lua`出力12を分岐 |
| bool 1 | 終末レーダー有効 | `ADSV3 Missile.lua`出力bool 1 (`missileRadarIO`) |
| bool 2 | KF有効トラックあり | `ADSV3 MissileKF.lua`出力bool 1を分岐 |

### 出力

なし。`debug.log`からDebugView++へ直接記録する。

### コード内デバッグ定数

- `DEBUG_LOG_ENABLED`
- `DEBUG_HORIZONTAL_FOV_DEG`
- `DEBUG_VERTICAL_FOV_DEG`
- `DEBUG_VERTICAL_FOV_OFFSET_DEG`
- `DEBUG_RADAR_COMMAND_DELAY_TICKS`
- `DEBUG_DELAY_SCAN_MAX_TICKS`
- `DEBUG_KF_STATE_LOG_ENABLED`

ロガーは発射前のIDを記憶し、時分割データリンクのうち一致するIDの座標だけを採用する。一致座標の受信間隔から速度を推定し、次の一致データまで位置を補間する。

現在の配線実測は、終末レーダー有効化命令3 tick、KF指向角2 tick、終末レーダー観測1 tickの遅延。主FOV比較では指向角経路と観測経路の差である `1 tick` 前の指向角を使用する。遅延較正用ログでは、同一の生レーダー観測をロガー内の0～3 tick前の指向角とそれぞれ比較する。

`KF_STATE`は、前tickのKF推定位置へ前tickのKF推定速度×`1/60 s`を加えた1 tick予測位置と、今回のKF推定位置との差を記録する。位置残差・速度変化は3軸値、全量、現在のKF LOSに直交する成分を併記する。さらにnum 22–24の自機位置を差分し、AAミサイルの実速度と実加速度を記録する。速度は全量・3軸値、加速度は全量・3軸値・現在LOS直交成分を含む。KF目標速度とAA実速度から、推定閉速度、相対横速度、速度ベクトル交差角を計算し、KF LOSの世界座標仰角も記録する。`kf_track_valid`と`track_changed`により、EKF更新中の変化とデータリンク代替・トラック切替を区別する。この診断入力のため、生レーダー観測枠は6目標から4目標へ減るが、KF本体と誘導の入力・出力には影響しない。

`est_closing_speed_mps`、`est_relative_lateral_speed_mps`、`est_velocity_cross_angle_deg`は、`LOGIC_DELAY`補償済みのKF目標速度と、パススルー自機位置の差分速度を組み合わせた推定値である。正の閉速度は接近、交差角は0°が同方向、90°が直交、180°が正面反航を表す。厳密な実速度同士の交差角は、同じDebugView時刻のHGV側`KINEMATICS`と`missile_velocity_x/y/z_mps`を照合して求める。

## ADSV3 Missile.lua

### 入力

| 種別・チャンネル | 内容 |
|---|---|
| bool 1 | 有効な追跡目標を出力中。現行誘導Luaでは未使用 |
| bool 2 | 対水上モード |
| bool 3 | 発射済み |
| bool 32 | ミサイル出力用レーダーが目標を検出中。検出中だけ中間誘導フィンを抑止 |
| num 1–3 | 推定目標位置 X, Y, Z |
| num 4–6 | 推定目標速度 Vx, Vy, Vz |
| num 10–11 | レーダー指向方位角・仰角。現行誘導Luaでは未使用 |
| num 12 | 追跡目標ID。現行誘導Luaでは未使用 |
| num 13–15 | 現在の自機位置 X, Y, Z |
| num 16–19 | 自機姿勢クォータニオン w, x, y, z |
| num 32 | epsilon。現行誘導Luaでは未使用 |

誘導Luaはnum 13–15の現在位置を履歴化し、1階差分の速度と速度差分の加速度から `LOGIC_DELAY` tick先の自機位置を計算する。num 16–19の現在姿勢も1 tick分履歴化し、クォータニオン差分から同じ時刻へ正規化外挿する。目標相対距離とLOSには予測位置を、pursuitの世界→ローカル変換とPNのLOS角速度の世界→ローカル変換には予測姿勢を使う。実速度のローカル変換と重力補償には現在姿勢を維持する。初回tickは姿勢履歴がないため現在姿勢を使用する。

### 出力

| 種別・チャンネル | 内容 |
|---|---|
| bool 1 | 終末誘導用ミサイルレーダー有効化 (`missileRadarIO`)。距離条件が初めて成立した時点で、目標速度方向と目標からミサイルへの方向がなす3次元アスペクト角が許容値以上なら、その発射中は以後有効化しない |
| bool 2 | 対水上用出力 |
| bool 3 | 主レーダー有効化 (`mainRadarIO`) |
| bool 4 | 自爆指令 |
| bool 5 | 近接信管指令 (`fuseIO`) |
| num 1 | yawフィン指令 |
| num 2 | pitchフィン指令 |
| num 3 | 1 tickあたりの距離変化。負値が接近 |

yaw・pitch出力の接続先であるミサイルフィンの有効入力範囲は±10。`ADSV3 Missile.lua`は値をクランプせず、そのまま出力する。

`PN_GAIN_SCHEDULE_ENABLE=true`かつ対空PNの場合、距離300～100 mの範囲だけ限定ゲインスケジュールを適用する。誘導Lua内部の`距離 × LOS角速度ベクトルの大きさ`を相対横速度相当値とし、80 m/s以下では追加補正なし、160 m/s以上では距離側の上限まで補正する。距離側は300～250 mで立ち上がり、250～150 mで最大、150～100 mで元へ戻る。実効ゲインは`PN_FIN_STRENGTH`から1.5へ向けて線形に変化し、基準値が1.5以上なら追加増幅しない。100 m以下、300 m以上、対水上モード、またはプロパティfalseでは従来値を使う。入力・出力チャンネルは追加しない。

### プロパティ

- `PN_FIN_STRENGTH`
- `PN_GAIN_SCHEDULE_ENABLE`
- `PPN_FIN_STRENGTH`
- `MISSILE_FIN_DISTANCE_THRESHOLD`
- `MISSILE_RADAR_MAX_ANGLE`（度。目標がミサイルへ真っすぐ向かう状態を0°とする。未設定または0以下の場合は40°。しきい値以上で起動を禁止）
- `SKIMMING_ALT`
- `GUIDANCE_START_ALTITUDE`
- `LOGIC_DELAY`
- `GRAVITY_COMP_ENABLE`
- `GRAVITY_COMP_GAIN`
- `HEAD_CORRIDOR_RADIUS`
- `HEAD_UP_OFFSET`
- `HEAD_RELEASE_DISTANCE`
- `BALLISTIC_MIN_SPEED`
- `HEAD_LEAD_MIN_SCALE`
- `HEAD_LEAD_MAX_SCALE`

## 座標・時間の約束

- 1 tick = 1/60 s
- Physics Sensor世界座標：X東、Y上、Z北、左手系
- ミサイルローカル座標：X右、Y上、Z前
- レーダー角度入出力：turns
- Lua内部の三角関数・姿勢計算：rad
- 現行EKF状態：`x, vx, ax, y, vy, ay, z, vz, az`

## インターフェース変更時の確認項目

- Stormworks側のComposite Read/Writeチャンネル
- Lua先頭コメント
- 本文書
- ロジック遅延
- turns/rad変換
- 座標軸と符号
- 発射前・発射後のID符号化
