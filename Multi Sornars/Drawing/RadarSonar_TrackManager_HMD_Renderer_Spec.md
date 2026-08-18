# RadarSonar Track Manager / HMD Renderer 仕様書

更新日: 2026-08-17

この文書は、`RadarSonar_TrackManager.lua` と `RadarSonar_HMD_Renderer.lua` の現行仕様をまとめたものです。  
**会話上で後から変更された仕様を優先**して記載しています。

---

# 1. システム全体構成

```text
Sonar KF ─┐
          ├─> RadarSonar_TrackManager.lua
Radar KF ─┘              │
                         ├─> Number 1: 下流装置向けTarget ID
                         │
                         └─> Composite
                              │
                              v
                    RadarSonar_HMD_Renderer.lua
                              │
                              v
                             HMD
```

役割は以下のように分離する。

## Track Manager

- Radar / Sonar KF Trackの受信
- Sensor Trackのキャッシュ
- Radar / Sonar間のAssociation
- System Trackの生成・維持
- RS目標の再捕捉
- Radar / Sonar / Fusion座標の選択
- System Track単位でのLock維持
- 下流装置向けSensor Track IDの出力
- HMD Renderer向けSystem Track packetの時分割送信

## HMD Renderer

- Track ManagerのSystem Track packet受信
- System Trackのキャッシュ
- packet間の速度補間
- 自機姿勢・HMD視線による座標変換
- HMD投影
- R / S / RS表示
- Lock表示
- 画面外Lock方向表示
- 操作モード表示
- ラベル衝突回避

---

# 2. IDの種類

システムではIDを2種類に分ける。

## 2.1 Sensor Track ID

Radar KF / Sonar KFがそれぞれ管理するTrack ID。

例:

```text
Radar Track ID = 4
Sonar Track ID = 7
```

RadarとSonarで同じ整数IDでも、別Trackとして扱う。

## 2.2 System Track ID

Track ManagerがHMD内部用に生成するID。

例:

```text
System Track 12
├─ Radar Track 4
└─ Sonar Track 7
```

HMD LockはSensor Track IDではなく、**System Track IDに対して保持**する。

Radar / Sonar側のTrack IDが再捕捉によって変化しても、同一目標と判断できればSystem Track IDは維持する。

例:

```text
System 12
├─ Radar 4
└─ Sonar 7

Radar 4 lost
Radar 15 reacquired

System 12
├─ Radar 15
└─ Sonar 7
```

---

# 3. 下流装置向けTarget ID

下流装置へ送るTarget IDはNumber 1chのみを使用する。

整数パッキング方式:

```lua
encodedID = sensorTrackID * 10 + source
```

source:

```text
0 = Radar
1 = Sonar
```

例:

```text
Radar ID 4  -> 40
Sonar ID 4  -> 41

Radar ID 17 -> 170
Sonar ID 17 -> 171
```

復号:

```lua
source = encodedID % 10
sensorTrackID = math.floor(encodedID / 10)
```

System Track IDそのものは下流装置へ出力しない。

---

# 4. RadarSonar_TrackManager.lua

# 4.1 Number入力

## Sonar Track

| ch | 内容 |
|---:|---|
| 1 | X |
| 2 | Y |
| 3 | Z |
| 4 | Vx |
| 5 | Vy |
| 6 | Vz |
| 7 | Sensor Track ID |
| 8 | epsilon |
| 9 | Detect (`1 = valid`) |
| 10 | lastSeenTick |

## Radar Track

| ch | 内容 |
|---:|---|
| 11 | X |
| 12 | Y |
| 13 | Z |
| 14 | Vx |
| 15 | Vy |
| 16 | Vz |
| 17 | Sensor Track ID |
| 18 | epsilon |
| 19 | Detect (`1 = valid`) |
| 20 | lastSeenTick |

## HMD / 自機情報

| ch | 内容 |
|---:|---|
| 25 | HMD View X |
| 26 | HMD View Y |
| 27 | Own X |
| 28 | Own Y |
| 29 | Own Z |
| 30 | Pitch |
| 31 | Yaw |
| 32 | Roll |

---

# 4.2 Bool入力

**Track Manager入力側の現行仕様。**

コックピットの座席Compositeを直接Track Managerへ入力するため、Bool 31をLock Triggerとして使用する。

| Bool ch | 内容 |
|---:|---|
| 1 | Sonar position mode |
| 2 | Fusion position mode |
| 3 | Fusion時の外部出力ID選択 |
| 4 | Auto Fallback |
| 31 | Lock Trigger |

位置モード:

```lua
positionMode =
    input.getBool(2) and MODE_FUSION or
    (input.getBool(1) and MODE_SONAR or MODE_RADAR)

fusionOutputSonar = input.getBool(3)
autoFallback = input.getBool(4)
lockInput = input.getBool(31)
```

結果:

| B1 | B2 | Position Mode |
|---|---|---|
| OFF | OFF | Radar |
| ON | OFF | Sonar |
| 任意 | ON | Fusion |

Fusion時:

```text
B3 OFF -> 外部出力IDはRadar
B3 ON  -> 外部出力IDはSonar
```

Auto Fallback:

```text
B4 OFF -> 選択中Sensorが失探した場合、別Sensorへ自動切替しない
B4 ON  -> 生存している別Sensorへのfallbackを許可
```

---

# 4.3 Sensor Trackキャッシュ

Sensor Trackは配列で保持する。

```text
[1] X
[2] Y
[3] Z
[4] Vx
[5] Vy
[6] Vz
[7] epsilon
[8] lastSeenTick
[9] Track Manager receive tick
```

Track Manager内での補間は、`lastSeenTick` ではなく **receive tick [9]** を使用する。

```lua
dt = (currentTick - receiveTick) / 60
```

これはRadar / Sonar KF側ですでに観測時刻から出力時刻までの予測が行われているためであり、Track Managerでは**時分割通信によって古くなった分だけ**を補間する。

---

# 4.4 System Track

System Trackは概ね以下の配列構造を使用する。

```text
[1]  System ID
[2]  Radar Track ID
[3]  Sonar Track ID
[4]  Association連続不一致回数
[5]  最後に比較したRadar receive tick
[6]  最後に比較したSonar receive tick
[7]  System生成tick
[8]  最終active tick
[9]  X
[10] Y
[11] Z
[12] Vx
[13] Vy
[14] Vz
[15] epsilon
[16] Display Source
```

Display Source:

```text
0 = Radar
1 = Sonar
2 = Fusion
```

---

# 4.5 Association

Radar / Sonar Trackの同一目標判定には以下を使用する。

- 現在tickまで補間した位置
- 位置差
- 速度差

Gate条件:

```text
position difference < position gate
AND
velocity difference < velocity gate
```

正規化score:

```text
position_difference² / position_gate²
+
velocity_difference² / velocity_gate²
```

候補の中からscoreの小さい組を優先し、1対1で対応付ける。

## デフォルト値

```text
ASSOC_POS_GATE = 250 m
ASSOC_VEL_GATE = 60 m/s

KEEP_GATE_MULTIPLIER = 2
LOCK_GATE_MULTIPLIER = 3

ASSOC_CONFIRM = 2
ASSOC_BREAK = 3
LOCK_BREAK = 6
```

したがってデフォルトGateは概ね:

```text
NEW:
  Position 250 m
  Velocity 60 m/s

KEEP:
  Position 500 m
  Velocity 120 m/s

LOCK:
  Position 750 m
  Velocity 180 m/s
```

---

# 4.6 新規RS成立

R-only SystemとS-only SystemがGate内に入った場合、即座にRSにはしない。

同じRadar / Sonar候補について、新しい時分割データ同士で複数回一致した場合にRSへ統合する。

デフォルト:

```text
2回一致 -> RS成立
```

同じキャッシュを毎tick比較してhit数を増やさない。

---

# 4.7 RS維持

一度RSになったTrackは、新規Associationより広いKEEP Gateを使用する。

通常:

```text
3回連続不一致 -> Association解除
```

Lock中:

```text
6回連続不一致 -> Association解除
```

これにより、KF位置が一時的にずれてもR/S Associationが頻繁に切り替わらないようにする。

---

# 4.8 片側失探と再捕捉

RS成立済みSystem Trackは、一方のSensor Trackを失ってもSystem Trackを維持する。

例:

```text
System 12 [RS]
Radar 4 active
Sonar 7 lost
```

Sonarが別IDで再出現した場合、残っているRadar Trackとの位置・速度整合性を確認して再Associationする。

成功時:

```text
System 12
Radar 4
Sonar 19
```

System IDは維持する。

Lock中System Trackは再捕捉判定でも優先される。

---

# 4.9 Fusion

Radar / Sonar両方が現在有効なRS TrackでFusion modeの場合:

```text
Xf  = (Xr  + Xs)  / 2
Yf  = (Yr  + Ys)  / 2
Zf  = (Zr  + Zs)  / 2

Vxf = (Vxr + Vxs) / 2
Vyf = (Vyr + Vys) / 2
Vzf = (Vzr + Vzs) / 2
```

現時点では単純平均。

`epsilon` はRadar / SonarのKFでそれぞれマハラノビス距離として計算されており、両KF間で直接Fusion weightとして比較しない。

Fusion位置の場合:

```text
epsilon = 0
```

としてRendererへ送る。

---

# 4.10 Lock

LockはSystem Track IDに対して保持する。

Lock Trigger:

```text
Bool 31
```

HMD視線から約30°以内にあるSystem Trackのうち、角度差が最も小さい目標を取得する。

Lock中はSensor Track IDが変わってもSystem Track IDが維持される限りLockを維持する。

Lock対象Systemが完全消失した場合、Triggerを押しっぱなしでも別目標へ自動乗り換えしない。

再Lockには一度Triggerを解除する。

---

# 4.11 Track Manager Number出力

| ch | 内容 |
|---:|---|
| 1 | 下流装置向けencoded Sensor Track ID |
| 2 | System ID |
| 3 | X |
| 4 | Y |
| 5 | Z |
| 6 | Vx |
| 7 | Vy |
| 8 | Vz |
| 9 | epsilon |
| 10 | Association State |
| 11 | Display Source |
| 12 | Locked System ID |
| 13 | Radar Track ID |
| 14 | Sonar Track ID |
| 25 | HMD View X |
| 26 | HMD View Y |
| 27 | Own X |
| 28 | Own Y |
| 29 | Own Z |
| 30 | Pitch |
| 31 | Yaw |
| 32 | Roll |

Association State:

```text
0 = Radar only
1 = Sonar only
2 = RS
```

Display Source:

```text
0 = Radar position
1 = Sonar position
2 = Fusion position
```

Number 25-32はRenderer用にpass-throughする。

---

# 4.12 Track Manager Bool出力

**Track Manager入力Boolとは番号が異なる点に注意。**

Bool 1はRenderer通信のPacket Validとして使用するため、操作入力は1chずつ後ろへ配置して出力する。

| Track Manager Bool出力 | Rendererでの意味 |
|---:|---|
| 1 | System Track Packet Valid |
| 2 | Sonar position mode |
| 3 | Fusion position mode |
| 4 | Fusion output Sonar |
| 5 | Auto Fallback |
| 31 | Lock input |

推奨マッピング:

```lua
output.setBool(1, packetIsValid)

output.setBool(2, input.getBool(1))
output.setBool(3, input.getBool(2))
output.setBool(4, input.getBool(3))
output.setBool(5, input.getBool(4))

output.setBool(31, lockInput)
```

`packetIsValid` は説明上の名称であり、実装では例えば:

```lua
local valid = system ~= nil
output.setBool(1, valid)
```

のように扱う。

---

# 4.13 System Track packetの時分割

1tickにつき1 System TrackをRendererへ送る。

System ID順で循環する。

例:

```text
tick 1 -> System 2
tick 2 -> System 5
tick 3 -> System 8
tick 4 -> System 2
...
```

有効packet送信中:

```text
Bool 1 = ON
Number 2-14 = System Track data
```

System Trackが存在しないtick:

```text
Bool 1 = OFF
```

RendererはBool 1がOFFの場合、Number 2-14を新規Track packetとして取り込まない。

---

# 5. RadarSonar_HMD_Renderer.lua

Track ManagerのComposite出力をそのままRendererのComposite入力へ接続する。

したがって、**Rendererの入力ch番号はTrack Manager出力ch番号と同一**。

---

# 5.1 Renderer Number入力

| ch | 内容 |
|---:|---|
| 1 | encoded Sensor Track ID（Rendererでは現在未使用） |
| 2 | System ID |
| 3 | X |
| 4 | Y |
| 5 | Z |
| 6 | Vx |
| 7 | Vy |
| 8 | Vz |
| 9 | epsilon |
| 10 | Association State |
| 11 | Display Source |
| 12 | Locked System ID |
| 13 | Radar Track ID |
| 14 | Sonar Track ID |
| 25 | HMD View X |
| 26 | HMD View Y |
| 27 | Own X |
| 28 | Own Y |
| 29 | Own Z |
| 30 | Pitch |
| 31 | Yaw |
| 32 | Roll |

---

# 5.2 Renderer Bool入力

| ch | 内容 |
|---:|---|
| 1 | System Track Packet Valid |
| 2 | Sonar position mode |
| 3 | Fusion position mode |
| 4 | Fusion output Sonar |
| 5 | Auto Fallback |
| 31 | Lock input |

Renderer側のPosition Mode:

```lua
positionMode =
    input.getBool(3) and FUSION or
    (input.getBool(2) and SONAR or RADAR)
```

---

# 5.3 Renderer Trackキャッシュ

System IDをkeyにしてpacketを保存する。

```text
[1]  X
[2]  Y
[3]  Z
[4]  Vx
[5]  Vy
[6]  Vz
[7]  epsilon
[8]  Association State
[9]  Display Source
[10] Radar Track ID
[11] Sonar Track ID
[12] packet receive tick
```

Bool 1がONのtickだけNumber 2-14をTrack packetとして読み込む。

---

# 5.4 Renderer補間

Renderer側でもSystem Track packetは時分割になるため、受信時点から現在tickまで等速補間する。

```lua
dt = (currentTick - packetReceiveTick) / 60

X = Xpacket + Vx * dt
Y = Ypacket + Vy * dt
Z = Zpacket + Vz * dt
```

Track Managerと同様に、これはKFの観測遅延補正ではなく**通信packet間の古さだけを補正する**。

---

# 5.5 Renderer cache timeout

System Track packetが一定時間届かない場合、Renderer cacheから削除する。

Property:

```text
RENDER_TRACK_TIMEOUT
```

デフォルト:

```text
90 tick
```

Track Manager側でSystem Trackが削除されると、そのIDのpacketが来なくなるため、Rendererはtimeoutで削除を検知する。

---

# 5.6 座標変換

Rendererは以下の順序で変換する。

```text
System Track Global XYZ
        ↓
Own vehicle positionを減算
        ↓
Own vehicle姿勢Quaternionでglobal -> local
        ↓
offsetX / offsetY / offsetZ
        ↓
HMD view Quaternionで逆回転
        ↓
Perspective projection
        ↓
Screen X/Y
```

`offsetX / offsetY / offsetZ` はTrack ManagerのLock判定側とRendererで同じ値を設定する。

---

# 5.7 HMDマーカー

Association Stateによって形状を変える。

```text
Radar only -> Circle
Sonar only -> Diamond
RS         -> Square
```

RSの場合、Display Sourceを内部線で表す。

```text
Radar position  -> 横線
Sonar position  -> 縦線
Fusion position -> 十字
```

概念:

```text
R only     S only      RS/R        RS/S        RS/F
   ○          ◇          □─           □│          □+
```

---

# 5.8 epsilon表示

`epsilon` は位置誤差半径ではなく、KFのinnovationに対するマハラノビス距離。

現在は参考表示として:

```lua
screen.drawCircle(x, y, epsilon * 10)
```

のような円を描画する。

Fusion位置ではTrack Managerから`epsilon = 0`が送られるため表示しない。

---

# 5.9 Lock表示

Number 12の`Locked System ID`と描画対象のSystem IDが一致した場合、Lock中として描画する。

通常マーカーの周囲に四隅Bracketを表示する。

Lock管理そのものはRendererではなくTrack Managerが担当する。

---

# 5.10 画面外Lock表示

Lock中System TrackがHMD画面外に出た場合、画面端に方向三角形を表示する。

目標Azimuth / Elevationと現在HMD Viewとの差を求め、FOVに対して正規化して画面端位置を決める。

三角形は:

```lua
screen.drawTriangleF(
    x1, y1,
    x2, y2,
    x3, y3
)
```

で目標方向へ向けて描画する。

---

# 5.11 操作モード表示

画面左上に現在状態を短縮表示する。

例:

```text
* R
* S A
* F>R
* F>S A
L R
- R
```

Lock表示:

```text
* = System Trackを実際にLock中
L = Lock Trigger ONだが未Lock
- = Lock Trigger OFF
```

Position Mode:

```text
R   = Radar
S   = Sonar
F>R = Fusion位置 / 外部IDはRadar
F>S = Fusion位置 / 外部IDはSonar
```

Auto Fallback:

```text
A = Auto Fallback ON
```

---

# 5.12 ラベル

基本ラベル:

```text
ID <System ID> <R/S/RS>
Distance
```

例:

```text
ID 12 RS
1250m
```

10km以上では短縮表示:

```text
12.3k
```

---

# 5.13 LifeBoatAPI文字列連結上の注意

以下のように**数値式の直後に文字列連結を書くと、圧縮時に不正なLuaへ変換される場合がある**。

避ける:

```lua
distanceText =
    math.floor(distance / 100) / 10 .. "k"
```

実際に確認された例では、圧縮後に:

```lua
10.."k"
```

のような形へつながり、エラー原因となった。

そのため数値計算と文字列連結を分離する。

推奨:

```lua
distanceText =
    math.floor(distance / 100) / 10

distanceText =
    distanceText .. "k"
```

今後もLifeBoatAPIで圧縮するコードでは、この形式を使用する。

---

# 5.14 ラベル重複回避

Renderer側には描画専用Luaとして文字数余裕があるため、ラベル位置を複数候補から選択する。

候補例:

```text
右上
右下
左上
左下
```

既に配置済みのラベル矩形との重なり面積を計算し、最も重なりの少ない位置へ配置する。

Lock目標を先に処理するため、Lock目標のラベル配置を優先する。

---

# 6. Property

# 6.1 Track Manager

| Property | 用途 | デフォルト |
|---|---|---:|
| `offsetX` | HMD/視点位置補正 X | 0 |
| `offsetY` | HMD/視点位置補正 Y | 0 |
| `offsetZ` | HMD/視点位置補正 Z | 0 |
| `TARGET_LOST_THRESHOLD_TICKS` | Sensor Track cache timeout | 120 |
| `ASSOC_POS_GATE` | 新規Association位置Gate | 250 m |
| `ASSOC_VEL_GATE` | 新規Association速度Gate | 60 m/s |
| `ASSOC_CONFIRM` | 新規Association成立回数 | 2 |

コード内固定値:

```text
KEEP multiplier = 2
LOCK multiplier = 3
Association break = 3
Lock association break = 6
Candidate timeout = 120 tick
System lost timeout = Sensor lost timeout * 2
```

# 6.2 HMD Renderer

| Property | 用途 | デフォルト |
|---|---|---:|
| `offsetX` | 視点位置補正 X | 0 |
| `offsetY` | 視点位置補正 Y | 0 |
| `offsetZ` | 視点位置補正 Z | 0 |
| `RGBAlpha` | 描画Alpha | 255 |
| `HMD_FOV_HEIGHT` | 垂直FOV | 58 deg |
| `RENDER_TRACK_TIMEOUT` | Renderer Track timeout | 90 tick |

`offsetX/Y/Z` はTrack ManagerとRendererで同じ値を使用する。

---

# 7. epsilonの扱い

Radar / Sonar KFから送られる`epsilon`は、両方とも基本的に:

```text
epsilon = Y^T S^-1 Y
```

で計算されるinnovationのマハラノビス距離。

したがって:

- 単位は無次元
- 「位置誤差 ±Xm」を直接表す値ではない
- RadarとSonar間で単純に大小比較してFusion weightには使わない
- Sensor Track内での観測と予測の整合度の参考値として扱う

---

# 8. 現在の重要な設計原則

1. **Sensor TrackとSystem Trackを分離する。**
2. **LockはSystem Track IDに対して保持する。**
3. **Radar / Sonar IDが変わってもSystem Trackをできるだけ維持する。**
4. **一度RSになったTrackは新規Associationより強く維持する。**
5. **Lock中RSはさらに強く維持する。**
6. **RSは「現在両Sensorが見えている」だけでなく、R/Sが同一目標として同定済みであることを表す。**
7. **表示座標の選択と外部へ渡すSensor IDを分離する。**
8. **Fusion座標は現時点では単純平均。**
9. **epsilonはFusion weightには使用しない。**
10. **Track Managerは追跡処理、Rendererは描画処理に役割分担する。**
11. **Manager→Renderer間はSystem Trackを時分割通信する。**
12. **Rendererはpacket受信後の古さだけVxyzで補間する。**
13. **LifeBoatAPI圧縮を前提に、数値式と文字列連結の境界に注意する。**

---

# 9. Boolチャンネル対応まとめ

最も混同しやすいため、入力側とManager→Renderer側をまとめる。

## Cockpit / 操作系 → Track Manager

```text
B1  Sonar Mode
B2  Fusion Mode
B3  Fusion output ID source
B4  Auto Fallback
B31 Lock
```

## Track Manager → HMD Renderer

```text
B1  System Track Packet Valid
B2  Sonar Mode
B3  Fusion Mode
B4  Fusion output ID source
B5  Auto Fallback
B31 Lock
```

したがって、Track Managerでは操作BoolをRenderer出力時に1chシフトして中継する。

```text
Manager Input B1 -> Manager Output B2
Manager Input B2 -> Manager Output B3
Manager Input B3 -> Manager Output B4
Manager Input B4 -> Manager Output B5
Manager Input B31 -> Manager Output B31

Manager Output B1 = Packet Valid
```

---

# 10. 接続概要

```text
Cockpit Composite
  B1,B2,B3,B4,B31
  N25-N32
        |
        v
+---------------------------+
| RadarSonar_TrackManager   |
|                           |
| N1 encoded Target ID -----+----> downstream device
|                           |
| System packet / modes     |
+-------------+-------------+
              |
              | Composite
              | N2-N14
              | N25-N32
              | B1-B5
              | B31
              v
+---------------------------+
| RadarSonar_HMD_Renderer   |
|                           |
| Track cache               |
| Projection                |
| HMD drawing               |
+-------------+-------------+
              |
              v
             HMD
```
