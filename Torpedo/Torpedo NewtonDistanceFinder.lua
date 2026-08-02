--[[
アクティブソナー距離測定スクリプト (ニュートン法 + v_los補正 + ローカル速度入力)

機能:
- 定期的にソナーのアクティブモードをON/OFF制御。
- Ping発射時の自機位置を記憶。
- 各Tickでソナー入力ch1-8を監視し、ONになっているチャンネルのエコーを処理。
- Physics Sensorからローカル速度ベクトル(ch23-25)を読み取る。
- 目標への視線(LOS)方向の速度成分(v_los)を計算。
- 実効的な音速を (SOUND_SPEED - v_los) として補正。
- エコー受信時の自機位置と発射時の位置の差分から移動ベクトルを計算。
- クォータニオンで移動ベクトルをローカル座標系に変換。
- ニュートン法で距離計算(補正後の実効音速を使用)。
- そのTickで検出・計算された結果を出力。

入力チャンネル設定:
- Composite On/Off 1-8: Sonar Echo Detect Target 1-8 (1 TickのみON)
- Composite On/Off 10: KalmanFilter Init Request(パススルー)
- Composite Number 1-16: Sonar Angles Target 1-8 (1:T1 Azi, 2:T1 Ele, ...) - 回転単位(Turns)
- Composite Number 17-19: Physics Sensor Global Pos X, Y, Z
- Composite Number 30-32: Physics Sensor Euler Pitch, Yaw, Roll (Radian)
- Composite Number 23-25: Physics Sensor Local Vel X, Y, Z (Local)
- Composite Number 26-28: True Target Global Pos X, Y, Z (検証用)

出力チャンネル設定:
- Composite On/Off 1: Sonar Active Mode Trigger (to Sonar Input Ch2)
- Composite On/Off 8: KalmanFilter Init Request (パススルー)
- Composite Number 1-24: Calculated Target Data (3chごとの組: Dist, Azi, Ele)
- Composite Number 19-21: Own Global Pos X, Y, Z
- Composite Number 22-24: Own Euler Pitch, Yaw, Roll
- Composite Number 25-27: True Target Global Pos X, Y, Z
--]]

-- === 定数 ===
local SOUND_SPEED = 1480
local TICKS_PER_SECOND = 60
local SOUND_SPEED_PER_TICK = SOUND_SPEED / 60
local MAX_TARGETS = 8
local PI2 = math.pi * 2
local NEWTON_ITERATIONS = 1

-- === プロパティ設定 ===
local PING_INTERVAL_TICKS = property.getNumber("PING_INTERVAL_TICKS") or 60
local SEND_LOGIC_DELAY = property.getNumber("SEND_LOGIC_DELAY") or 0
local RECEIVE_LOGIC_DELAY = property.getNumber("RECEIVE_LOGIC_DELAY") or 0

-- === 状態保持変数 ===
local pingSentTick = 0
local isPinging = false
local currentTick = 0
local pingGlobalPosition = { x = 0, y = 0, z = 0 }

-- === ベクトル・数値処理ヘルパー関数 ===
local function vectorMagnitude(v)
    local x, y, z
    x = v[1] or v.x
    y = v[2] or v.y
    z = v[3] or v.z
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

local function vectorSub(v1, v2)
    return {
        x = (v1.x or v1[1] or 0) - (v2.x or v2[1] or 0),
        y = (v1.y or v1[2] or 0) - (v2.y or v2[2] or 0),
        z = (v1.z or v1[3] or 0) - (v2.z or v2[3] or 0)
    }
end

local function vectorDot(v1, v2)
    local x1, y1, z1 = v1.x or v1[1] or 0, v1.y or v1[2] or 0, v1.z or v1[3] or 0
    local x2, y2, z2 = v2.x or v2[1] or 0, v2.y or v2[2] or 0, v2.z or v2[3] or 0
    return x1 * x2 + y1 * y2 + z1 * z2
end

-- === クォータニオン演算 ===

local function multiplyQuaternions(q1, q2)
    local w1, x1, y1, z1 = q1[1], q1[2], q1[3], q1[4]
    local w2, x2, y2, z2 = q2[1], q2[2], q2[3], q2[4]
    return {
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    }
end

local function eulerZYX_to_quaternion(r, y, p)
    local hr, hy, hp = r * 0.5, y * 0.5, p * 0.5
    local cr, sr = math.cos(hr), math.sin(hr)
    local cy, sy = math.cos(hy), math.sin(hy)
    local cp, sp = math.cos(hp), math.sin(hp)
    return {
        cr * cy * cp + sr * sy * sp,
        cr * cy * sp - sr * sy * cp,
        cr * sy * cp + sr * cy * sp,
        sr * cy * cp - cr * sy * sp
    }
end

local function rotateVectorByInverseQuaternion(v, q)
    local px, py, pz = v.x or v[1] or 0, v.y or v[2] or 0, v.z or v[3] or 0
    local p_vec = { 0, px, py, pz }
    local q_conj = { q[1], -q[2], -q[3], -q[4] }
    local temp = multiplyQuaternions(q_conj, p_vec)
    local p_prime = multiplyQuaternions(temp, q)
    return { x = p_prime[2], y = p_prime[3], z = p_prime[4] }
end


function rotateVectorByQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime
    -- nilチェックは原則削除
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    -- nilチェックは原則削除
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    -- nilチェックは原則削除
    temp = multiplyQuaternions(q, p)
    -- nilチェックは原則削除
    p_prime = multiplyQuaternions(temp, q_conj)
    -- nilチェックは原則削除
    return { p_prime[2], p_prime[3], p_prime[4] }
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

-- === ニュートン法 (実効音速補正距離計算) ===

local function calcF(d, E, A, dx, dy, dz, st, pt)
    local ce, se = math.cos(E), math.sin(E)
    local ca, sa = math.cos(A), math.sin(A)
    local t1 = (d * ce * sa + dx) ^ 2
    local t2 = (d * se + dy) ^ 2
    local t3 = (d * ce * ca + dz) ^ 2
    return math.sqrt(t1 + t2 + t3) + d - (st * pt)
end

local function calcFPrime(d, E, A, dx, dy, dz)
    local ce, se = math.cos(E), math.sin(E)
    local ca, sa = math.cos(A), math.sin(A)
    local rxh, ryh, rzh = ce * sa, se, ce * ca
    local vx, vy, vz = d * rxh + dx, d * ryh + dy, d * rzh + dz
    local vm = math.sqrt(vx ^ 2 + vy ^ 2 + vz ^ 2)
    if vm < 1e-6 then return 1 end
    return (vx * rxh + vy * ryh + vz * rzh) / vm + 1
end

local function newtonMethod(dInitial, E, A, dx, dy, dz, st, pt)
    local d = dInitial
    for _ = 1, NEWTON_ITERATIONS do
        local fv = calcF(d, E, A, dx, dy, dz, st, pt)
        local fpv = calcFPrime(d, E, A, dx, dy, dz)
        if math.abs(fpv) < 1e-6 then break end
        d = d - fv / fpv
    end
    return math.max(0, d)
end

-- === メイン処理 ===

function onTick()
    currentTick = currentTick + 1
    -- --- 自機状態・センサー値読み込み ---
    local ownGlobalPos = {
        x = input.getNumber(27),
        y = input.getNumber(28),
        z = input.getNumber(29)
    }
    local pitch = input.getNumber(30)
    local yaw = input.getNumber(31)
    local roll = input.getNumber(32)
    local ownOrientation = eulerZYX_to_quaternion(roll, yaw, pitch)

    local localVel = {
        x = input.getNumber(24),
        y = input.getNumber(25),
        z = input.getNumber(26)
    }

    local dataLinkCoords = { x = input.getNumber(21), y = input.getNumber(22), z = input.getNumber(23) }
    local detectionsThisTick = {}

    local distanceToDataLink = vectorMagnitude(vectorSub(ownGlobalPos, dataLinkCoords))

    local dataLinkIntervalTick = (distanceToDataLink * 2) / SOUND_SPEED_PER_TICK + 10

    -- --- Ping 制御 ---
    if not isPinging and currentTick >= pingSentTick + dataLinkIntervalTick then
        pingSentTick = currentTick + SEND_LOGIC_DELAY
        isPinging = true
        pingGlobalPosition.x = ownGlobalPos.x
        pingGlobalPosition.y = ownGlobalPos.y
        pingGlobalPosition.z = ownGlobalPos.z
    elseif isPinging and currentTick >= pingSentTick + dataLinkIntervalTick then
        isPinging = false
    end

    -- --- エコー受信 & 計算処理 ---
    if isPinging then
        for i = 1, MAX_TARGETS do
            if input.getBool(i) then
                -- ピンガーを受信した時刻
                local echoReceivedTick = currentTick - RECEIVE_LOGIC_DELAY
                -- ピンガーを送信してから戻ってくるまでにかかった時間
                local pingTimeTick = echoReceivedTick - pingSentTick

                if pingTimeTick > 2 then
                    local measuredAziTurns = input.getNumber((i - 1) * 2 + 1)
                    local measuredEleTurns = input.getNumber((i - 1) * 2 + 2)

                    -- FOV 範囲内チェック

                    local A = measuredAziTurns * PI2
                    local E = measuredEleTurns * PI2

                    -- 視線(LOS)方向単位ベクトルの計算
                    local cosE, sinE = math.cos(E), math.sin(E)
                    local cosA, sinA = math.cos(A), math.sin(A)
                    local localLOS = { x = cosE * sinA, y = sinE, z = cosE * cosA }

                    -- LOS方向速度成分 v_los の算出
                    local v_los = vectorDot(localVel, localLOS)

                    -- 補正後の実効音速 (Tick単位)
                    local effectiveSoundSpeed = math.max(1, SOUND_SPEED - v_los)
                    local sSpeed_tick = effectiveSoundSpeed / TICKS_PER_SECOND

                    -- Ping発射時からの移動ベクトル (ローカル座標系)
                    local globalDisplacement = vectorSub(ownGlobalPos, pingGlobalPosition)
                    local localDisplacement = rotateVectorByInverseQuaternion(globalDisplacement, ownOrientation)

                    -- ニュートン法による距離計算
                    local deltaTimeSec = pingTimeTick / TICKS_PER_SECOND
                    local d_initial = math.max(0, (SOUND_SPEED * deltaTimeSec) / 2)
                    local calculatedDistance = newtonMethod(
                        d_initial, E, A,
                        localDisplacement.x, localDisplacement.y, localDisplacement.z,
                        sSpeed_tick, pingTimeTick
                    )

                    local forwardTimeTick = calculatedDistance / SOUND_SPEED_PER_TICK
                    local targetReachedTick = pingSentTick + forwardTimeTick -- 目標の位置が特定された時刻
                    table.insert(detectionsThisTick, {
                        distance = calculatedDistance,
                        azimuth = A,
                        elevation = E,
                        targetReachedTick = targetReachedTick
                    })
                end
            end
        end
    end

    -- --- 出力ノード初期化 ---
    for i = 1, 32 do
        output.setBool(i, false)
        output.setNumber(i, 0)
    end

    -- --- 制御信号・フラグ出力 ---
    output.setBool(1, isPinging)         -- Active Mode Trigger (to Sonar Ch2)
    output.setBool(8, input.getBool(10)) -- Kalman Filter Init Request (パススルー)

    -- --- 検出結果出力 (4チャンネル周期: Dist, Azi, Ele) ---
    for i = 1, #detectionsThisTick do
        local det = detectionsThisTick[i]
        local baseChannel = 4
        output.setNumber(baseChannel * i - 3, det.distance)
        output.setNumber(baseChannel * i - 2, det.azimuth)
        output.setNumber(baseChannel * i - 1, det.elevation)
        output.setNumber(baseChannel * i, det.targetReachedTick)

        local localTargetCoords = localAngleDistToLocalCoords(det.distance, det.azimuth, det.elevation)
        local globalTargetCoords = localToGlobal(localTargetCoords, ownGlobalPos, ownOrientation)

        local vecSub = vectorSub(globalTargetCoords, dataLinkCoords)
        local diff = vectorMagnitude(vecSub)
        if diff < 200 then
            debug.log("CLOSE: "..diff)
            debug.log("gX: " .. globalTargetCoords.x .. "gY: " .. globalTargetCoords.y .. "gZ: " .. globalTargetCoords.z)
            debug.log("tX: " .. dataLinkCoords.x .. "tY: " .. dataLinkCoords.y .. "tZ: " .. dataLinkCoords.z)
        end
    end

    -- --- 補助データ・デバッグ用座標出力 ---
    output.setNumber(25, ownGlobalPos.x)
    output.setNumber(26, ownGlobalPos.y)
    output.setNumber(27, ownGlobalPos.z)
    output.setNumber(28, input.getNumber(30))
    output.setNumber(29, input.getNumber(31))
    output.setNumber(30, input.getNumber(32))
end
