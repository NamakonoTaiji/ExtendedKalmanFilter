--[[
アクティブソナー距離測定・同一Tickクラスタリング

機能:
- 推定目標距離に応じた間隔でアクティブソナーをPingする。
- Ping発射位置、自機移動量、ローカル速度、受信角度からニュートン法で距離を推定する。
- 同一Tickに複数のエコーが入った場合、方向の近い観測を1目標へ統合する。
- 統合時は視線単位ベクトルを平均し、方位角・仰角の±π境界を正しく扱う。
- 統合した距離と反射時刻はクラスタ内観測の算術平均を使用する。
- クラスタリング後の目標をFIFOキューへ格納し、1tickにつき1目標だけ4ch形式で出力する。
- 同一tickに複数目標が成立した場合、あふれた目標は次tick以降へ時分割して順次出力する。
- 遅延出力されても targetReachedTick は観測時に算出した値を保持する。

入力 On/Off:
- ch 1-8: ソナー目標1-8のエコー受信パルス（受信TickのみOn）

入力 Number:
- ch 1-16: ソナー角度。目標nは ch(2n-1)=方位角、ch(2n)=仰角（回転単位）
- ch 20: ピンガー周期
- ch 21-23: 検証用固定標的のグローバル座標 X,Y,Z
- ch 24-26: Physics Sensorローカル速度 X(右),Y(上),Z(前) [m/s]
- ch 26-29: 自機グローバル座標 X(東),Y(上),Z(北)
- ch 29-32: 自機姿勢 クォータニオン w, x, y, z

出力 On/Off:
- ch 1: ソナーActive/Ping信号（ソナー入力ch2へ）

出力 Number:
- ch 1: 推定距離 [m]
- ch 2: ローカル方位角 [rad]
- ch 3: ローカル仰角 [rad]
- ch 4: 音波が目標へ到達した推定絶対Tick (targetReachedTick)
- 1tickにつき最大1目標のみ出力。未出力目標はFIFOで次tick以降へ送る。

プロパティ:
- PING_INTERVAL_TICKS: Ping周期の最低値
- SEND_LOGIC_DELAY: Ping送信側のロジック遅延 [tick]
- RECEIVE_LOGIC_DELAY: エコー受信側のロジック遅延 [tick]
- CLUSTER_ANGLE_TURNS: 同一目標とみなす最大角度差 [turn]。0以下なら0.004 turn
- CLUSTER_DISTANCE_RATIO: 距離差ゲート比率。0以下なら0.03
- CLUSTER_DISTANCE_BASE: 距離差ゲート基礎値 [m]。0以下なら20m
--]]

-- 定数
local SOUND_SPEED = 1480
local TICKS_PER_SECOND = 60
local MAX_TARGETS = 8
local PI = math.pi
local PI2 = PI * 2
local NEWTON_ITERATIONS = 2

-- プロパティ読み込み
local SORNAR_OFFSETS_VEC = {
    x = property.getNumber("X_RIGHT_OFFSET"),
    y = property.getNumber("Y_UP_OFFSET"),
    z = property.getNumber("Z_FRONT_FFSET")
} -- 物理センサーからみたソーナーのローカル座標
local SEND_LOGIC_DELAY = property.getNumber("SEND_LOGIC_DELAY")
local RECEIVE_LOGIC_DELAY = property.getNumber("RECEIVE_LOGIC_DELAY")
local CLUSTER_ANGLE_TURNS = property.getNumber("CLUSTER_ANGLE_TURNS")
local CLUSTER_DISTANCE_RATIO = property.getNumber("CLUSTER_DISTANCE_RATIO")
local CLUSTER_DISTANCE_BASE = property.getNumber("CLUSTER_DISTANCE_BASE")
local DISTANCE_STEP = property.getNumber("DIST_STEP")
if CLUSTER_ANGLE_TURNS <= 0 then
    CLUSTER_ANGLE_TURNS = 0.004
end
if CLUSTER_DISTANCE_RATIO <= 0 then
    CLUSTER_DISTANCE_RATIO = 0.03
end
if CLUSTER_DISTANCE_BASE <= 0 then
    CLUSTER_DISTANCE_BASE = 30
end
local CLUSTER_COS = math.cos(CLUSTER_ANGLE_TURNS * PI2)

-- グローバル変数
local pingSentTick = 0
local isPinging = false
local currentTick = 0
local pingGlobalPosition = { x = 0, y = 0, z = 0 }

-- 時分割出力用FIFOキュー
-- queueHead/queueTailを使い、table.remove(1)による全要素シフトを避ける。
local outputQueue = {}
local queueHead = 1
local queueTail = 0

function enqueueDetection(d)
    queueTail = queueTail + 1
    outputQueue[queueTail] = d
end

function dequeueDetection()
    if queueHead > queueTail then
        return nil
    end

    local d = outputQueue[queueHead]
    outputQueue[queueHead] = nil
    queueHead = queueHead + 1

    -- 空になったら添字をリセットして長時間運用時の増大を防ぐ。
    if queueHead > queueTail then
        queueHead = 1
        queueTail = 0
    end

    return d
end

-- === ヘルパー関数 (ベクトル, クォータニオン, ニュートン法, 座標変換) ===
function vectorMagnitude(v)
    local x = v[1] or v.x or 0
    local y = v[2] or v.y or 0
    local z = v[3] or v.z or 0
    return math.sqrt(x ^ 2 + y ^ 2 +
        z ^ 2)
end

function vectorSub(v1, v2)
    local x1 = v1[1] or v1.x or 0
    local y1 = v1[2] or v1.y or 0
    local z1 = v1[3] or v1.z or 0
    local x2 = v2[1] or v2
        .x or 0
    local y2 = v2[2] or v2.y or 0
    local z2 = v2[3] or v2.z or 0
    return { x1 - x2, y1 - y2, z1 - z2 }
end

-- ★ 内積関数を追加 ★
function vectorDot(v1, v2)
    local x1 = v1[1] or v1.x or 0
    local y1 = v1[2] or v1.y or 0
    local z1 = v1[3] or v1.z or 0
    local x2 = v2[1] or v2
        .x or 0
    local y2 = v2[2] or v2.y or 0
    local z2 = v2[3] or v2.z or 0
    return x1 * x2 + y1 * y2 + z1 * z2
end

function multiplyQuaternions(q_a, q_b)
    local w1 = q_a[1]
    local x1 = q_a[2]
    local y1 = q_a[3]
    local z1 = q_a[4]
    local w2 = q_b[1]
    local x2 = q_b[2]
    local y2 =
        q_b[3]
    local z2 = q_b[4]
    local w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    local x = w1 * x2 + x1 * w2 + y1 * z2 -
        z1 * y2
    local y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    local z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return {
        w,
        x, y, z }
end

function eulerZYX_to_quaternion(r, y, p)
    local hr = r * 0.5
    local hy = y * 0.5
    local hp = p * 0.5
    local cr = math.cos(hr)
    local sr = math.sin(hr)
    local cy =
        math.cos(hy)
    local sy = math.sin(hy)
    local cp = math.cos(hp)
    local sp = math.sin(hp)
    local w = cr * cy * cp +
        sr * sy * sp
    local x = cr * cy * sp - sr * sy * cp
    local y = cr * sy * cp + sr * cy * sp
    local z = sr * cy *
        cp -
        cr * sy * sp
    return { w, x, y, z }
end

function rotateVectorByInverseQuaternion(v, q)
    local px = v[1] or v.x or 0
    local py = v[2] or v.y or 0
    local pz = v[3] or v.z or 0
    local p_vec = { 0, px, py, pz }
    local q_conj = {
        q[1], -q[2], -q[3], -q[4] }
    local temp = multiplyQuaternions(q_conj, p_vec)
    local p_prime = multiplyQuaternions(
        temp, q)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local relVecGlob = vectorSub(globalTargetPos, ownGlobalPos)
    local locVec = rotateVectorByInverseQuaternion(
        relVecGlob, ownOrientationQuat)
    return { x = locVec[1], y = locVec[2], z = locVec[3] }
end

function f(d, E, A, dx, dy, dz, st, pt)
    local ce = math.cos(E)
    local se = math.sin(E)
    local ca = math.cos(A)
    local sa = math.sin(A)
    local t1 = (d * ce * sa + dx) ^
        2
    local t2 = (d * se + dy) ^ 2
    local t3 = (d * ce * ca + dz) ^ 2
    return math.sqrt(t1 + t2 + t3) + d -
        (st * pt)
end

function f_prime(d, E, A, dx, dy, dz)
    local ce = math.cos(E)
    local se = math.sin(E)
    local ca = math.cos(A)
    local sa = math.sin(A)
    local rxh = ce * sa
    local ryh =
        se
    local rzh = ce * ca
    local vx = d * rxh + dx
    local vy = d * ryh + dy
    local vz = d * rzh + dz
    local vm =
        math
        .sqrt(vx ^ 2 + vy ^ 2 + vz ^ 2)
    if vm < 1e-6 then return 1 end
    local dot = vx * rxh + vy * ryh + vz * rzh
    return
        dot / vm + 1
end

function NewtonMethod(di, E, A, dx, dy, dz, st, pt)
    local d = di
    local fv = 0
    for i = 1, NEWTON_ITERATIONS do
        fv = f(d, E, A, dx, dy, dz, st, pt)
        local fpv = f_prime(d, E, A, dx, dy, dz)
        if math.abs(fpv or 0) < 1e-6 then break end
        local dn =
            d - fv / fpv
        d = dn
    end
    if d < 0 then
        d = 0
    end
    return d
end

---3次元ベクトルをクォータニオンで回転させる関数（軽量化版）
---@param vector table {x, y, z} または {1, 2, 3} 形式の回転対象ベクトル
---@param quaternion table {w, x, y, z} (インデックス 1, 2, 3, 4) 形式のクォータニオン
---@param isInverse? boolean true を渡すと逆回転を実行（省略時は false/順回転）
---@return number[] @回転後の 3次元ベクトル {x, y, z}
function rotateVectorByQuaternion(vector, quaternion, isInverse)
    local w, x, y, z = quaternion[1], quaternion[2], quaternion[3], quaternion[4]
    local vx, vy, vz = vector[1] or vector.x, vector[2] or vector.y, vector[3] or vector.z

    -- 外積演算の共通部分（軽量化計算）
    local tx = 2 * (y * vz - z * vy)
    local ty = 2 * (z * vx - x * vz)
    local tz = 2 * (x * vy - y * vx)

    -- 逆回転なら w の符号を反転
    if isInverse then
        w = -w
    end

    return {
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx)
    }
end

-- ローカル極座標からローカル直交座標へ
---@param dist number 距離
---@param localAziRad number 方位角(ラジアン)
---@param localEleRad number 仰角(ラジアン)
---@return Vector3 ローカル座標(x右方向, y上方向, z前方向)
function localAngleDistToLocalCoords(dist, localAziRad, localEleRad)
    local localX, localY, localZ
    localX = dist * math.cos(localEleRad) * math.sin(localAziRad)
    localY = dist * math.sin(localEleRad)
    localZ = dist * math.cos(localEleRad) * math.cos(localAziRad)
    return { x = localX, y = localY, z = localZ }
end

--- 乗り物などのローカル座標を、ワールド座標系のグローバル座標に変換します。
---@description 基準となるオブジェクトのグローバル位置と姿勢（クォータニオン）を用いて、
--              オブジェクト上の相対的な位置（ローカル座標）を絶対的な位置（グローバル座標）に変換します。
---@param localPosition Vector3 {x: number, y: number, z: number} 変換したいオブジェクト上のローカル座標。
---@param objectGlobalPos Vector3 {x: number, y: number, z: number} 基準オブジェクト自体のグローバル座標。
---@param objectOrientationQuat Quaternion {w: number, x: number, y: number, z: number} 基準オブジェクトの姿勢を表すクォータニオン。
---@return Vector3 {x: number, y: number, z: number} 変換後のグローバル座標。
function localToGlobal(localPosition, objectGlobalPos, objectOrientationQuat)
    -- 1. 入力テーブルからローカル座標の各成分を取得
    --    {x,y,z} 形式と {1,2,3} 形式の両方に対応します。
    local lx = localPosition.x or localPosition[1] or 0
    local ly = localPosition.y or localPosition[2] or 0
    local lz = localPosition.z or localPosition[3] or 0
    local localVec = { lx, ly, lz }

    -- 2. クォータニオンでローカル座標ベクトルを回転させ、グローバル座標系での相対ベクトルを計算
    local relativeGlobalVec = rotateVectorByQuaternion(localVec, objectOrientationQuat)

    -- 3. オブジェクトのグローバル座標に相対ベクトルを加算し、最終的なグローバル座標を算出
    local gx = relativeGlobalVec[1] + objectGlobalPos.x
    local gy = relativeGlobalVec[2] + objectGlobalPos.y
    local gz = relativeGlobalVec[3] + objectGlobalPos.z

    return { x = gx, y = gy, z = gz }
end

-- 同一Tick内の観測を方向と距離で統合する。
-- 角度は単位ベクトルの内積で比較し、平均もベクトル合成で行う。
function clusterDetections(detections)
    if #detections < 2 then return detections end
    local clusters = {}

    for _, d in ipairs(detections) do
        local ce = math.cos(d.elevation)
        local ux = ce * math.sin(d.azimuth)
        local uy = math.sin(d.elevation)
        local uz = ce * math.cos(d.azimuth)
        local best, bestDot = nil, CLUSTER_COS

        for _, c in ipairs(clusters) do
            local m = math.sqrt(c.sx * c.sx + c.sy * c.sy + c.sz * c.sz)
            local dot = (ux * c.sx + uy * c.sy + uz * c.sz) / m
            local meanDistance = c.sd / c.n
            local distanceGate = CLUSTER_DISTANCE_BASE +
                math.max(meanDistance, d.distance) * CLUSTER_DISTANCE_RATIO

            if dot >= bestDot and
                math.abs(d.distance - meanDistance) <= distanceGate then
                best = c
                bestDot = dot
            end
        end

        if best then
            best.sx = best.sx + ux
            best.sy = best.sy + uy
            best.sz = best.sz + uz
            best.sd = best.sd + d.distance
            best.st = best.st + d.targetReachedTick
            best.n = best.n + 1
        else
            clusters[#clusters + 1] = {
                sx = ux,
                sy = uy,
                sz = uz,
                sd = d.distance,
                st = d.targetReachedTick,
                n = 1
            }
        end
    end

    local result = {}
    for _, c in ipairs(clusters) do
        local horizontal = math.sqrt(c.sx * c.sx + c.sz * c.sz)
        result[#result + 1] = {
            distance = c.sd / c.n,
            azimuth = math.atan(c.sx, c.sz),
            elevation = math.atan(c.sy, horizontal),
            targetReachedTick = c.st / c.n,
            clusterSize = c.n
        }
    end
    return result
end

function packTarget(
    distance,
    azimuth,
    elevation,
    targetReachedTick
)
    local pi = math.pi
    local pi2 = pi * 2

    ----------------------------------------------------------
    -- 距離
    -- 12bit / 2m刻み
    -- 0 ～ 8190m
    ----------------------------------------------------------
    local d = math.floor(distance / DISTANCE_STEP  + 0.5)

    if d < 0 then
        d = 0
    elseif d > 4095 then
        d = 4095
    end


    ----------------------------------------------------------
    -- 観測の古さ
    -- 11bit / 1tick刻み
    -- 0 ～ 2047tick
    ----------------------------------------------------------
    local age = math.floor(
        currentTick - targetReachedTick + 0.5
    )

    if age < 0 then
        age = 0
    elseif age > 2047 then
        age = 2047
    end


    ----------------------------------------------------------
    -- 方位角
    -- 13bit
    -- -pi ～ +pi を 0 ～ 8191 に変換
    ----------------------------------------------------------
    local a = math.floor(
        ((azimuth + pi) % pi2)
        / pi2
        * 8192
        + 0.5
    ) % 8192


    ----------------------------------------------------------
    -- 仰角
    -- 12bit
    -- -pi/2 ～ +pi/2 を 0 ～ 4095 に変換
    ----------------------------------------------------------
    if elevation < -pi / 2 then
        elevation = -pi / 2
    elseif elevation > pi / 2 then
        elevation = pi / 2
    end

    local e = math.floor(
        (elevation + pi / 2)
        / pi
        * 4095
        + 0.5
    )


    ----------------------------------------------------------
    -- 方位角13bitを
    -- 上位1bit + 下位12bit に分割
    ----------------------------------------------------------
    local aLow = a % 4096
    local aHigh = math.floor(a / 4096)


    ----------------------------------------------------------
    -- 24bit整数 × 2ch
    ----------------------------------------------------------
    local packed1 =
        d
        + age * 4096
        + aHigh * 8388608

    local packed2 =
        aLow
        + e * 4096


    return packed1, packed2
end

-- === メイン処理 ===
function onTick()
    -- 出力ノード初期化
    -- Bool ch1 はPing制御、Number ch1-4 は時分割された単一目標。
    for i = 1, 32 do
        output.setBool(i, false)
        output.setNumber(i, 0)
    end

    currentTick = currentTick + 1
    local ownGlobalPos, pitch, yaw, roll, ownOrientation, localVel
    -- 自機状態読み取り
    ownGlobalPos = { x = input.getNumber(27), y = input.getNumber(28), z = input.getNumber(29) }
    pitch = input.getNumber(30)
    yaw = input.getNumber(31)
    roll = input.getNumber(32)
    localVel = { input.getNumber(24), input.getNumber(25), input.getNumber(26) }
    ownOrientation = eulerZYX_to_quaternion(roll, yaw, pitch)

    output.setNumber(26, ownGlobalPos.x)
    output.setNumber(27, ownGlobalPos.y)
    output.setNumber(28, ownGlobalPos.z)
    output.setNumber(29, ownOrientation[1])
    output.setNumber(30, ownOrientation[2])
    output.setNumber(31, ownOrientation[3])
    output.setNumber(32, ownOrientation[4])

    local isPingerRequest = input.getBool(1)
    -- --- Ping 開始/終了処理 ---

    if not isPingerRequest then
        if not isPinging then
            pingSentTick = currentTick + SEND_LOGIC_DELAY
        end
        isPinging = true
        pingGlobalPosition.x = ownGlobalPos.x
        pingGlobalPosition.y = ownGlobalPos.y
        pingGlobalPosition.z = ownGlobalPos.z
    else
        isPinging = false
    end

    local detectionsThisTick = {}

    -- --- エコー受信 & 距離計算 & 角度比較処理 ---
    if isPinging then
        -- 各ソナー入力チャンネル(1-8)をチェック
        for i = 1, MAX_TARGETS do
            local echoDetected = input.getNumber(i * 2) ~= 0

            if echoDetected then
                local echoReceivedTick_actual = currentTick - RECEIVE_LOGIC_DELAY
                local pingTimeTick = echoReceivedTick_actual - pingSentTick
                if pingTimeTick > 0 then -- 至近距離は出力しない
                    local measuredAziTurns = input.getNumber((i - 1) * 2 + 1)
                    local measuredEleTurns = input.getNumber((i - 1) * 2 + 2)
                    -- 角度をラジアンに変換
                    local A = measuredAziTurns * PI2
                    local E = measuredEleTurns * PI2

                    -- ★ 視線(LOS)方向の速度成分 v_los を計算 ★
                    -- 1. 目標方向のローカル単位ベクトル L_hat
                    local cosE = math.cos(E)
                    local sinE = math.sin(E)
                    local cosA = math.cos(A)
                    local sinA = math.sin(A)
                    local losX = cosE * sinA
                    local losY = sinE
                    local losZ = cosE * cosA
                    local localLOS_hat = { losX, losY, losZ } -- 正規化されているはず
                    -- 2. ローカル速度ベクトルと目標方向単位ベクトルの内積
                    local v_los = vectorDot(localVel, localLOS_hat)

                    -- ★ 実効的な音速(Tickあたり)を v_los を使って計算 ★
                    local effectiveSoundSpeed = SOUND_SPEED - v_los
                    if effectiveSoundSpeed < 1 then
                        effectiveSoundSpeed = 1
                    end
                    local sSpeed_tick = effectiveSoundSpeed / TICKS_PER_SECOND

                    -- 移動ベクトル計算 (位置の差分から、ローカル座標系へ)
                    local globalDisplacement = vectorSub(ownGlobalPos, pingGlobalPosition)
                    local localDisplacement = rotateVectorByInverseQuaternion(globalDisplacement, ownOrientation)
                    local dispLocalX = localDisplacement[1]
                    local dispLocalY = localDisplacement[2]
                    local dispLocalZ = localDisplacement[3]

                    -- 初期値 & ニュートン法 (補正後の sSpeed_tick を使用)
                    local deltaTimeSec = pingTimeTick / TICKS_PER_SECOND
                    local d_initial = math.max(0, (SOUND_SPEED * deltaTimeSec) / 2) -- 初期値は元の音速で計算
                    local calculated_distance = NewtonMethod(
                        d_initial, E, A,
                        dispLocalX, dispLocalY, dispLocalZ,
                        sSpeed_tick, pingTimeTick
                    )

                    local targetReachedTick =
                        echoReceivedTick_actual -
                        calculated_distance / sSpeed_tick

                    -- ソナー原点基準の目標座標
                    local v = localAngleDistToLocalCoords(
                        calculated_distance,
                        A,
                        E
                    )

                    -- ソナー設置位置 → 共通機体原点へ補正
                    v.x = v.x + SORNAR_OFFSETS_VEC.x
                    v.y = v.y + SORNAR_OFFSETS_VEC.y
                    v.z = v.z + SORNAR_OFFSETS_VEC.z

                    -- 共通原点からの極座標に変換し直す
                    local horizontal =
                        math.sqrt(v.x * v.x + v.z * v.z)

                    local correctedDistance =
                        math.sqrt(
                            v.x * v.x +
                            v.y * v.y +
                            v.z * v.z
                        )

                    local correctedAzimuth =
                        math.atan(v.x, v.z)

                    local correctedElevation =
                        math.atan(v.y, horizontal)

                    table.insert(detectionsThisTick, {
                        distance = correctedDistance,
                        azimuth = correctedAzimuth,
                        elevation = correctedElevation,

                        -- 音響計算で求めた時刻なのでこれはそのまま
                        targetReachedTick = targetReachedTick
                    })

                    local localTargetCoordsVec = localAngleDistToLocalCoords(calculated_distance, A, E)
                    localTargetCoordsVec.x = localTargetCoordsVec.x + SORNAR_OFFSETS_VEC.x
                    localTargetCoordsVec.y = localTargetCoordsVec.y + SORNAR_OFFSETS_VEC.y
                    localTargetCoordsVec.z = localTargetCoordsVec.z + SORNAR_OFFSETS_VEC.z
                    --[[local globalTargetCoords = localToGlobal(localTargetCoordsVec, ownGlobalPos, ownOrientation)

                    diff = vectorMagnitude(vectorSub(dataLinkCoordsVec, globalTargetCoords))
                    distFromDataLinkAvg, distAvg, distanceDiffAvg = 0, 0, 0
                                         if diff < 100 then
                        diffArry[#diffArry + 1] = diff
                        distFromDataLinkArry[#distFromDataLinkArry + 1] = distanceFromDataLinkCoords
                        distanceDiffArry[#distanceDiffArry + 1] = math.abs(calculated_distance-distanceFromDataLinkCoords)
                        local diffSum = 0
                        local distFromDataLinkSum = 0
                        local distDiffSum = 0
                        for i = 1, #diffArry do
                            diffSum = diffSum + diffArry[i]
                            distFromDataLinkSum = distFromDataLinkSum + distFromDataLinkArry[i]
                            distDiffSum = distDiffSum + distanceDiffArry[i]
                            if i == #diffArry then
                                distFromDataLinkAvg = diffSum / i
                                distAvg = distFromDataLinkSum / i
                                distanceDiffAvg = distDiffSum / i
                            end
                        end
                    end ]]
                end
            end
        end
    end

    -- 同一Tick内の近接観測を統合する。
    detectionsThisTick = clusterDetections(detectionsThisTick)

    -- クラスタリング済み目標をFIFOへ積む。
    -- backlogがある場合も新規観測は末尾へ追加され、観測順を維持する。
    for i = 1, #detectionsThisTick do
        enqueueDetection(detectionsThisTick[i])
    end

    output.setBool(1, isPinging)

    -- 1tickにつき1目標だけ出力する。
    -- targetReachedTickはキュー投入時の値を保持するため、
    -- 時分割による出力遅延が発生しても元の音波到達時刻を復元できる。
    local d = dequeueDetection()
    if d then
        pack1, pack2 = packTarget(d.distance, d.azimuth, d.elevation, d.targetReachedTick)
        output.setNumber(1, pack1)
        output.setNumber(2, pack2)
    end
end
