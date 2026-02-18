--[[
アクティブソナー距離測定スクリプト (ニュートン法 + v_los補正 + ローカル速度入力)

機能:
- 定期的にソナーのアクティブモードをON/OFF制御。
- Ping発射時の自機位置を記憶。
- 各Tickでソナー入力ch1-8を監視し、ONになっているチャンネルのエコーを処理。
- ★Physics Sensorからローカル速度ベクトル(ch23-25)を直接読み取る。
   (注:仕様書[cite: 26]のグローバル基準という記述は誤りとのユーザー指摘に基づく)
- ★目標への視線(LOS)方向の速度成分(v_los)を計算。
- ★実効的な音速を (SOUND_SPEED - v_los) として補正。
- エコー受信時の自機位置と発射時の位置の差分から移動ベクトルを計算。
- クォータニオンで移動ベクトルをローカル座標系に変換。
- ニュートン法で距離計算(補正後の実効音速を使用)。
- そのTickで検出・計算された結果をリストに一時保存し、順次出力。

入力チャンネル設定:
- Composite On/Off 1-8: Sonar Echo Detect Target 1-8 (1 TickのみON)
- Composite On/Off 9: isLaunch
- Composite On/Off 10: KalmanFilter Init Request(パススルー)
- Composite Number 1-16: Sonar Angles Target 1-8 (1:T1 Azi, 2:T1 Ele, ...) - 回転単位(Turns)
- Composite Number 17: Physics Sensor Global Pos X (East)
- Composite Number 18: Physics Sensor Global Pos Y (Up)
- Composite Number 19: Physics Sensor Global Pos Z (North)
- Composite Number 20: Physics Sensor Euler Pitch (X rot) (Radian)
- Composite Number 21: Physics Sensor Euler Yaw (Y rot) (Radian)
- Composite Number 22: Physics Sensor Euler Roll (Z rot) (Radian)
- Composite Number 23: Physics Sensor Local Vel X (Right) <-- ローカル基準！
- Composite Number 24: Physics Sensor Local Vel Y (Up)    <-- ローカル基準！
- Composite Number 25: Physics Sensor Local Vel Z (Fwd)   <-- ローカル基準！
- Composite Number 26: True Target Global Pos X (East) (ログ比較用)
- Composite Number 27: True Target Global Pos Y (Up)   (ログ比較用)
- Composite Number 28: True Target Global Pos Z (North) (ログ比較用)

出力チャンネル設定:
- Composite Number 1-8: Calculated Distance (検出結果順, ニュートン法 v_los補正)
- Composite On/Off 8: KalmanFilter Init Request (パススルー)
- Composite On/Off 1: Sonar Active Mode Trigger (to Sonar Input Ch2)
--]]

-- 定数
local SOUND_SPEED = 1480
local TICKS_PER_SECOND = 60
local MAX_TARGETS = 8
local MAX_OUTPUT_CHANNELS = 8
local PI = math.pi
local PI2 = PI * 2
local NEWTON_ITERATIONS = 1

-- プロパティ読み込み
local PING_INTERVAL_TICKS = property.getNumber("PING_INTERVAL_TICKS") or 60
local SEND_LOGIC_DELAY = property.getNumber("SEND_LOGIC_DELAY") or 0
local RECEIVE_LOGIC_DELAY = property.getNumber("RECEIVE_LOGIC_DELAY") or 0
local MAX_FOV_AZI = property.getNumber("MAX_FOV_AZI") or 0.25
local MAX_FOV_ELE = property.getNumber("MAX_FOV_ELE") or 0.25

-- グローバル変数
local pingSentTick = 0
local isPinging = false
local currentTick = 0
local pingGlobalPosition = { x = 0, y = 0, z = 0 }

-- === ヘルパー関数 (ベクトル, クォータニオン, ニュートン法, 座標変換) ===
function vectorMagnitude(v)
    local x = v[1] or v.x or 0; local y = v[2] or v.y or 0; local z = v[3] or v.z or 0; return math.sqrt(x ^ 2 + y ^ 2 +
        z ^ 2)
end

function vectorSub(v1, v2)
    local x1 = v1[1] or v1.x or 0; local y1 = v1[2] or v1.y or 0; local z1 = v1[3] or v1.z or 0; local x2 = v2[1] or v2
        .x or 0; local y2 = v2[2] or v2.y or 0; local z2 = v2[3] or v2.z or 0; return { x1 - x2, y1 - y2, z1 - z2 }
end

-- ★ 内積関数を追加 ★
function vectorDot(v1, v2)
    local x1 = v1[1] or v1.x or 0; local y1 = v1[2] or v1.y or 0; local z1 = v1[3] or v1.z or 0; local x2 = v2[1] or v2
        .x or 0; local y2 = v2[2] or v2.y or 0; local z2 = v2[3] or v2.z or 0; return x1 * x2 + y1 * y2 + z1 * z2
end

function multiplyQuaternions(q_a, q_b)
    local w1 = q_a[1]; local x1 = q_a[2]; local y1 = q_a[3]; local z1 = q_a[4]; local w2 = q_b[1]; local x2 = q_b[2]; local y2 =
        q_b[3]; local z2 = q_b[4]; local w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2; local x = w1 * x2 + x1 * w2 + y1 * z2 -
        z1 * y2; local y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2; local z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; return {
        w,
        x, y, z }
end

function eulerZYX_to_quaternion(r, y, p)
    local hr = r * 0.5; local hy = y * 0.5; local hp = p * 0.5; local cr = math.cos(hr); local sr = math.sin(hr); local cy =
        math.cos(hy); local sy = math.sin(hy); local cp = math.cos(hp); local sp = math.sin(hp); local w = cr * cy * cp +
        sr * sy * sp; local x = cr * cy * sp - sr * sy * cp; local y = cr * sy * cp + sr * cy * sp; local z = sr * cy *
        cp -
        cr * sy * sp; return { w, x, y, z }
end

function rotateVectorByInverseQuaternion(v, q)
    local px = v[1] or v.x or 0; local py = v[2] or v.y or 0; local pz = v[3] or v.z or 0; local p_vec = { 0, px, py, pz }; local q_conj = {
        q[1], -q[2], -q[3], -q[4] }; local temp = multiplyQuaternions(q_conj, p_vec); local p_prime = multiplyQuaternions(
        temp, q); return { p_prime[2], p_prime[3], p_prime[4] }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local relVecGlob = vectorSub(globalTargetPos, ownGlobalPos); local locVec = rotateVectorByInverseQuaternion(
        relVecGlob, ownOrientationQuat); return { x = locVec[1], y = locVec[2], z = locVec[3] }
end

function f(d, E, A, dx, dy, dz, st, pt)
    local ce = math.cos(E); local se = math.sin(E); local ca = math.cos(A); local sa = math.sin(A); local t1 = (d * ce * sa + dx) ^
        2; local t2 = (d * se + dy) ^ 2; local t3 = (d * ce * ca + dz) ^ 2; return math.sqrt(t1 + t2 + t3) + d -
        (st * pt)
end

function f_prime(d, E, A, dx, dy, dz)
    local ce = math.cos(E); local se = math.sin(E); local ca = math.cos(A); local sa = math.sin(A); local rxh = ce * sa; local ryh =
        se; local rzh = ce * ca; local vx = d * rxh + dx; local vy = d * ryh + dy; local vz = d * rzh + dz; local vm =
        math
        .sqrt(vx ^ 2 + vy ^ 2 + vz ^ 2); if vm < 1e-6 then return 1 end; local dot = vx * rxh + vy * ryh + vz * rzh; return
        dot / vm + 1
end

function NewtonMethod(di, E, A, dx, dy, dz, st, pt)
    local d = di
    local fv = 0
    for i = 1, NEWTON_ITERATIONS do
        fv = f(d, E, A, dx, dy, dz, st, pt)
        local fpv = f_prime(d, E, A, dx, dy, dz)
        if math["math.abs"](fpv or 0) < 1e-6 then break end
        local dn =
            d - fv / fpv
        d = dn
    end
    if d < 0 then d = 0 end
    return d
end

function clamp(value, min, max) return math.max(min, math.min(value, max)) end

-- === メイン処理 ===
function onTick()
    local ownGlobalPos, pitch, yaw, roll, ownOrientation, localVelX, localVelY, localVelZ, localVel, dataLinkTarget

    currentTick = currentTick + 1

    -- --- Ping 開始/終了処理 ---
    if not isPinging and currentTick >= pingSentTick + PING_INTERVAL_TICKS then
        pingSentTick = currentTick + SEND_LOGIC_DELAY
        isPinging = true
        pingGlobalPosition.x = input.getNumber(17)
        pingGlobalPosition.y = input.getNumber(18)
        pingGlobalPosition.z = input.getNumber(19)
    elseif isPinging then
        if currentTick >= pingSentTick + PING_INTERVAL_TICKS then
            isPinging = false
        end
    end

    local detectionsThisTick = {}

    -- --- エコー受信 & 距離計算 & 角度比較処理 ---

    -- 自機状態読み取り
    ownGlobalPos = { x = input.getNumber(17), y = input.getNumber(18), z = input.getNumber(19) }
    pitch = input.getNumber(20)
    yaw = input.getNumber(21)
    roll = input.getNumber(22)
    ownOrientation = eulerZYX_to_quaternion(roll, yaw, pitch)

    -- ★ ローカル速度ベクトルを直接読み取り ★
    localVelX = input.getNumber(23)
    localVelY = input.getNumber(24)
    localVelZ = input.getNumber(25)
    localVel = { localVelX, localVelY, localVelZ }

    -- 目標の真のグローバル座標読み取り (ログ用)
    local trueTargetGlobalPos = { x = input.getNumber(26), y = input.getNumber(27), z = input.getNumber(28) }
    local isTrueTargetPosValid = not (trueTargetGlobalPos.x == 0 and trueTargetGlobalPos.y == 0 and trueTargetGlobalPos.z == 0)
    dataLinkTarget = trueTargetGlobalPos
    if isPinging then
        -- 各ソナー入力チャンネル(1-8)をチェック
        for i = 1, MAX_TARGETS do
            local echoDetected = input.getBool(i)

            if echoDetected then
                local echoReceivedTick_actual = currentTick - RECEIVE_LOGIC_DELAY
                local pingTimeTick = echoReceivedTick_actual - pingSentTick

                if pingTimeTick > 0 then
                    local measuredAziTurns = input.getNumber((i - 1) * 2 + 1)
                    local measuredEleTurns = input.getNumber((i - 1) * 2 + 2)

                    if math["math.abs"](measuredAziTurns) > MAX_FOV_AZI or math["math.abs"](measuredEleTurns) > MAX_FOV_ELE then
                        goto continue
                    end

                    -- 角度をラジアンに変換
                    local A = measuredAziTurns * PI2
                    local E = measuredEleTurns * PI2

                    -- ★ 視線(LOS)方向の速度成分 v_los を計算 ★
                    -- 1. 目標方向のローカル単位ベクトル L_hat
                    local cosE = math.cos(E); local sinE = math.sin(E)
                    local cosA = math.cos(A); local sinA = math.sin(A)
                    local losX = cosE * sinA
                    local losY = sinE
                    local losZ = cosE * cosA
                    local localLOS_hat = { losX, losY, losZ } -- 正規化されているはず
                    -- 2. ローカル速度ベクトルと目標方向単位ベクトルの内積
                    local v_los = vectorDot(localVel, localLOS_hat)

                    -- ★ 実効的な音速(Tickあたり)を v_los を使って計算 ★
                    local effectiveSoundSpeed = SOUND_SPEED - v_los
                    if effectiveSoundSpeed < 1 then effectiveSoundSpeed = 1 end
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
                    local calculated_distance = NewtonMethod(d_initial, E, A, dispLocalX, dispLocalY, dispLocalZ,
                        sSpeed_tick, pingTimeTick)
                    local localDist = 0
                    -- 真のローカル角度の計算 (ログ用)
                    local trueAziTurns = 0; local trueEleTurns = 0
                    if isTrueTargetPosValid then
                        local trueTargetLocalPos = globalToLocalCoords(trueTargetGlobalPos, ownGlobalPos, ownOrientation)
                        localDist = vectorMagnitude(trueTargetLocalPos)
                        if localDist > 1e-6 then
                            trueAziTurns = math.atan(trueTargetLocalPos.x, trueTargetLocalPos.z) / PI2
                            local asin_arg = clamp(trueTargetLocalPos.y / localDist, -1.0, 1.0)
                            trueEleTurns = math.asin(asin_arg) / PI2
                        end
                    end

                    -- ログ出力 (v_losも追加)
                    --[[ debug.log(string.format(
                        "Tick:%d Ch:%d Dist:%.2f(M)/%.2f(T) Azi:%.4f(M)/%.4f(T) Ele:%.4f(M)/%.4f(T) vLOS:%.2f sSpdTick:%.3f",
                        currentTick, i, calculated_distance, localDist,
                        measuredAziTurns, trueAziTurns,
                        measuredEleTurns, trueEleTurns,
                        v_los, sSpeed_tick)) ]]

                    table.insert(detectionsThisTick, { distance = calculated_distance, azimuth = A, elevation = E })

                    ::continue::
                end
            end
        end
    end

    -- 出力ノード初期化
    for i = 1, 32 do
        output.setBool(i, false)
        output.setNumber(i, 0)
    end
    -- --- 距離データの出力 ---
    for i = 1, #detectionsThisTick do
        if detectionsThisTick[i] ~= nil then
            local baseChannel = (i - 1) * 3
            output.setNumber(baseChannel + 1, detectionsThisTick[i].distance)
            output.setNumber(baseChannel + 2, detectionsThisTick[i].azimuth)
            output.setNumber(baseChannel + 3, detectionsThisTick[i].elevation)
            output.setBool(i, true)
        else
            output.setNumber(i, 0)
        end
    end
    output.setBool(8, input.getBool(10)) -- 初期化フラグパススルー
    output.setNumber(19, ownGlobalPos.x)
    output.setNumber(20, ownGlobalPos.y)
    output.setNumber(21, ownGlobalPos.z)
    output.setNumber(22, pitch)
    output.setNumber(23, yaw)
    output.setNumber(24, roll)
    output.setNumber(25, dataLinkTarget.x)
    output.setNumber(26, dataLinkTarget.y)
    output.setNumber(27, dataLinkTarget.z)
end

-- onDraw関数は省略
