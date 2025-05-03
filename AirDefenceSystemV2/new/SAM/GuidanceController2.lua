--[[
================================================================================
GuidanceController.lua (v0.2 - 初期誘導フェーズ追加)
================================================================================
機能:
- 発射後、一定時間は単純追尾で目標に機首を向ける。
- その後、比例航法に切り替えて終末誘導を行う。
- (他はv0.1と同様)

入力: (変更なし)
出力: (変更なし)
プロパティ:
- INITIAL_GUIDANCE_DURATION_SECONDS: 初期誘導を行う時間(秒) (例: 5.0)
- (他のプロパティはv0.1と同様)

- 近接速度はプラスが接近、マイナスが離脱
================================================================================
]]
local PI, PI2, DT, NAVIGATION_GAIN, FIN_GAIN, SIMPLE_TRACK_GAIN, PROXIMITY_DISTANCE, MAX_CONTROL, PN_GUIDANCE_EPSILON_THRESHOLD, RADAR_EFFECTIVE_RANGE
local INITIAL_GUIDANCE_TICKS, GUIDANCE_START_ALTITUDE, INITIAL_GUIDANCE_DURATION_SECONDS, FUSE_PROXIMITY_DURATION_TICKS, VT_FUSE_SAMPLING_TICKS
-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60

-- プロパティ読み込み
RADAR_EFFECTIVE_RANGE = property.getNumber("RadarEffectiveRange")
NAVIGATION_GAIN = property.getNumber("N")                                                   -- 航法定数
FIN_GAIN = property.getNumber("FinGain")                                                    -- 比例航法の動翼の強さ係数
SIMPLE_TRACK_GAIN = property.getNumber("SimpleTrackGain")                                   -- 単追尾の動翼の強さ係数
PROXIMITY_DISTANCE = property.getNumber("ProximityDistance")                                -- 信管動作距離
MAX_CONTROL = property.getNumber("MaxControl")                                              -- 動翼への出力の上限
PN_GUIDANCE_EPSILON_THRESHOLD = property.getNumber("PN_GUIDANCE_EPSILON_THRESHOLD")         -- 例: 閾値20
INITIAL_GUIDANCE_DURATION_SECONDS = property.getNumber("INITIAL_GUIDANCE_DURATION_SECONDS") -- 初期誘導時間(秒)
INITIAL_GUIDANCE_TICKS = INITIAL_GUIDANCE_DURATION_SECONDS * 60                             -- Tick数に変換
FUSE_PROXIMITY_DURATION_TICKS = property.getNumber("FUSE_PROM_DURATION_TICKS")              -- 必要に応じて調整
GUIDANCE_START_ALTITUDE = property.getNumber("GUIDANCE_START_ALTITUDE")                     -- 誘導開始高度
VT_FUSE_SAMPLING_TICKS = property.getNumber("VT_FUSE_SAMPLE_TICKS")                         -- VT信管近接速度計算で、直近何ティックのデータを平均化するか
-- グローバル変数
local launchTickCounter = 0                                                                 -- 発射後のTickカウンター
local oldDataLinkPosVec = { 0, 0, 0 }                                                       -- データリンク速度計算用
local VcAverage = 0
local proximityFuseTrigger = false
local targetPosX, targetPosY, targetPosZ, targetVelX, targetVelY, targetVelZ = 0, 0, 0, 0, 0, 0
local VcArry = {}
local guidanceStart = false
-- ============================================================================
-- ヘルパー関数群 (変更なし - vectorNormalize を含むこと)
-- ============================================================================
function vectorNormalize(v)
    local mag, x, y, z
    mag = vectorMagnitude(v)
    if mag < 1e-9 then
        return { 0, 0, 0 }
    else
        x = v[1] or v.x or 0
        y = v[2] or v.y or 0
        z = v[3] or v.z or 0
        return { x / mag, y / mag, z / mag }
    end
end

function vectorMagnitude(v)
    local x, y, z
    x = v[1] or v.x or 0
    y = v[2] or v.y or 0
    z = v[3] or v.z or 0
    return math.sqrt(x ^ 2 + y ^ 2 +
        z ^ 2)
end

function vectorSub(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1] or v1.x or 0
    y1 = v1[2] or v1.y or 0
    z1 = v1[3] or v1.z or 0
    x2 = v2[1] or v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return { x1 - x2, y1 - y2, z1 - z2 }
end

function vectorDot(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1] or v1.x or 0
    y1 = v1[2] or v1.y or 0
    z1 = v1[3] or v1.z or 0
    x2 = v2[1] or v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return x1 * x2 + y1 * y2 + z1 * z2
end

function vectorCross(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1] or v1.x or 0
    y1 = v1[2] or v1.y or 0
    z1 = v1[3] or v1.z or 0
    x2 = v2[1] or v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return { y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 *
    y2 -
    y1 * x2 }
end

function vectorScalarMul(s, v)
    local x, y, z
    x = v[1] or v.x or 0
    y = v[2] or v.y or 0
    z = v[3] or v.z or 0
    return { s * x, s * y, s * z }
end

function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2, w_result, x_result, y_result, z_result
    w1 = q_a[1]
    x1 = q_a[2]
    y1 = q_a[3]
    z1 = q_a[4]
    w2 = q_b[1]
    x2 = q_b[2]
    y2 = q_b[3]
    z2 = q_b[4]
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 +
        x1 * y2 -
        y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z
    half_roll = roll * 0.5
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    w = cr * cy * cp + sr * sy * sp
    x = cr * cy * sp - sr * sy * cp
    y = cr * sy * cp + sr * cy * sp
    z = sr * cy * cp - cr * sy * sp
    return { w, x, y, z }
end

function rotateVectorByQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp = multiplyQuaternions(q, p)
    p_prime = multiplyQuaternions(temp, q_conj)
    return { p_prime[2], p_prime[3], p_prime
        [4] }
end

function rotateVectorByInverseQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp = multiplyQuaternions(q_conj, p)
    p_prime = multiplyQuaternions(temp, q)
    return { p_prime[2], p_prime[3], p_prime
        [4] }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local gX, gY, gZ, oX, oY, oZ, relativeVectorGlobal, localVector
    gX = globalTargetPos.x or globalTargetPos[1] or 0
    gY = globalTargetPos.y or globalTargetPos[2] or 0
    gZ = globalTargetPos.z or globalTargetPos[3] or 0
    oX = ownGlobalPos.x or 0
    oY = ownGlobalPos.y or 0
    oZ = ownGlobalPos.z or 0
    relativeVectorGlobal = { gX - oX, gY - oY, gZ - oZ }
    localVector = rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientationQuat)
    return {
        x = localVector[1],
        y = localVector[2],
        z = localVector[3]
    }
end

function VcAverageCalculator(Vc)
    local VcAverage = 0
    if #VcArry > VT_FUSE_SAMPLING_TICKS then
        table.remove(VcArry, 1)
    end
    table.insert(VcArry, Vc)
    for i = 1, #VcArry do
        VcAverage = VcAverage + VcArry[i]
    end
    VcAverage = VcAverage / #VcArry
    return VcAverage
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    local isLaunch = input.getBool(2)
    if isLaunch then
        -- 関数冒頭でローカル変数を宣言
        local dataLinkX, dataLinkY, dataLinkZ, dataLinkTargetPosVec, targetEpsilon, isTracking, distance
        local ownPosX, ownPosY, ownPosZ, ownPitch, ownYaw, ownRoll
        local ownOrientation, ownPosVec, ownVelVec, targetPosVec, targetVelVec
        local pitchControl, yawControl, dataLinkVelVec
        local R_vec, V_vec, R_mag, Vc, LOS_Rate_vector, Accel_cmd_global, Accel_cmd_local
        local targetLocal, horizontalDistance, currentLocalAzimuth, currentLocalElevation, currentTargetPosVec, currentTargetVelVec
        local R_hat, Omega_cross_Rhat -- PN計算用変数

        -- 1. 入力読み取り
        isTracking = input.getBool(1) -- KalmanFilterからのフラグを使用

        if isTracking then
            targetPosX = input.getNumber(1)
            targetPosY = input.getNumber(2)
            targetPosZ = input.getNumber(3)
            targetVelX = input.getNumber(4)
            targetVelY = input.getNumber(5)
            targetVelZ = input.getNumber(6)
        else
            targetPosX = targetPosX + targetVelX
            targetPosY = targetPosY + targetVelY
            targetPosZ = targetPosZ + targetVelZ
            --debug.log("No Target Found")
        end
        --debug.log("isTracking: " .. tostring(isTracking))
        --debug.log("targetPosX: " .. targetPosX .. " targetPosY: " .. targetPosY .. " targetPosZ: " .. targetPosZ)
        --debug.log("targetVelX: " .. targetVelX .. " targetVelY: " .. targetVelY .. " targetVelZ: " .. targetVelZ)
        targetEpsilon = input.getNumber(32)

        dataLinkX = input.getNumber(7)
        dataLinkY = math.max(input.getNumber(8), 100) -- 中間誘導で海面に突っ込まないようにデータリンク目標高度の下限を100に設定
        dataLinkZ = input.getNumber(9)
        dataLinkTargetPosVec = { dataLinkX, dataLinkY, dataLinkZ }

        ownPosX = input.getNumber(10)
        ownPosY = input.getNumber(11)
        ownPosZ = input.getNumber(12)
        ownPitch = input.getNumber(13)
        ownYaw = input.getNumber(14)
        ownRoll = input.getNumber(15)

        if ownPosY > GUIDANCE_START_ALTITUDE then
            guidanceStart = true
        end
        if guidanceStart then
            -- Tickカウンター更新
            launchTickCounter = launchTickCounter + 1
            -- 2. 姿勢管理
            ownOrientation = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch) -- 自機姿勢(クォータニオン)
            if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end  -- エラー時は単位クォータニオン

            -- 3. 変数準備
            ownPosVec = { ownPosX, ownPosY, ownPosZ }
            targetPosVec = { targetPosX, targetPosY, targetPosZ }
            targetVelVec = { targetVelX, targetVelY, targetVelZ }

            dataLinkVelVec = { 60 * (dataLinkTargetPosVec[1] - oldDataLinkPosVec[1]),
                60 * (dataLinkTargetPosVec[2] - oldDataLinkPosVec[2]),
                60 * (dataLinkTargetPosVec[3] - oldDataLinkPosVec[3]) }
            oldDataLinkPosVec = dataLinkTargetPosVec

            -- 自機速度取得と変換
            local ownVelLocalX = input.getNumber(16)                             -- ローカル速度X読み取り
            local ownVelLocalY = input.getNumber(17)                             -- ローカル速度Y読み取り
            local ownVelLocalZ = input.getNumber(18)                             -- ローカル速度Z読み取り
            local ownVelLocalVec = { ownVelLocalX, ownVelLocalY, ownVelLocalZ }  -- ローカル速度ベクトル作成
            ownVelVec = rotateVectorByQuaternion(ownVelLocalVec, ownOrientation) -- グローバル速度に変換
            if ownVelVec == nil then ownVelVec = { 0, 0, 0 } end                 -- 変換失敗時はゼロベクトル

            -- 4. 誘導計算と近接信管
            pitchControl = 0 -- 初期化
            yawControl = 0   -- 初期化

            -- === 誘導モード決定 ===
            if launchTickCounter < INITIAL_GUIDANCE_TICKS then
                --if distance < RADAR_EFFECTIVE_RANGE - 100 then
                --debug.log("PPN")
                -- === 初期誘導フェーズ (単純追尾) ===
                if not (dataLinkX == 0 and dataLinkY == 0 and dataLinkZ == 0) then
                    targetPosVec = dataLinkTargetPosVec -- データリンク座標を目標とする
                    targetLocal = globalToLocalCoords(targetPosVec, { x = ownPosX, y = ownPosY, z = ownPosZ },
                        ownOrientation)
                    if targetLocal ~= nil then -- 座標変換成功確認
                        horizontalDistance = math.sqrt(targetLocal.x ^ 2 + targetLocal.z ^ 2)
                        if horizontalDistance > 1e-6 then
                            currentLocalAzimuth = math.atan(targetLocal.x, targetLocal.z)        -- atan(左右, 前後)
                            currentLocalElevation = math.atan(targetLocal.y, horizontalDistance) -- atan(上下, 水平距離)
                        else
                            -- 真上/真下などの場合
                            currentLocalAzimuth = 0
                            if targetLocal.y > 0 then
                                currentLocalElevation = PI / 2
                            elseif targetLocal.y < 0 then
                                currentLocalElevation = -PI / 2
                            else
                                currentLocalElevation = 0
                            end
                        end
                        pitchControl = currentLocalElevation * SIMPLE_TRACK_GAIN
                        yawControl = currentLocalAzimuth * SIMPLE_TRACK_GAIN
                    else
                        -- 座標変換失敗時の処理
                        pitchControl = 0
                        yawControl = 0
                    end
                else
                    -- データリンク無効時は直進(何もしない)
                    pitchControl = 0
                    yawControl = 0
                end
                proximityFuseTrigger = false -- 初期誘導では作動しない
            else
                -- === 通常誘導フェーズ (比例航法) ===
                -- Kalman Filterからの情報が有効かチェック
                local distanceSq = (targetPosVec[1] - ownPosVec[1]) ^ 2 + (targetPosVec[2] - ownPosVec[2]) ^ 2 +
                    (targetPosVec[3] - ownPosVec[3]) ^ 2
                local usePnWithFilterOutput = isTracking and targetEpsilon < PN_GUIDANCE_EPSILON_THRESHOLD and
                    targetEpsilon ~= 0

                --至近距離まで近接したら強制終末誘導
                if usePnWithFilterOutput or distanceSq < 500 ^ 2 then
                    --debug.log("ARH")
                    -- Kalman Filterの推定値を使う
                    currentTargetPosVec = targetPosVec
                    currentTargetVelVec = targetVelVec
                else
                    --debug.log("DL")
                    -- Kalman Filterが無効 or Epsilon大 -> データリンク情報を目標とする
                    currentTargetPosVec = dataLinkTargetPosVec
                    currentTargetVelVec = dataLinkVelVec
                end

                -- 目標位置があればPN計算実行
                if not (currentTargetPosVec[1] == 0 and currentTargetPosVec[2] == 0 and currentTargetPosVec[3] == 0) then
                    --debug.log("PN")
                    R_vec = vectorSub(currentTargetPosVec, ownPosVec)
                    -- ↓↓↓ 目標速度は場合分けした currentTargetVelVec を使う ↓↓↓
                    V_vec = vectorSub(currentTargetVelVec, ownVelVec)
                    R_mag = vectorMagnitude(R_vec)
                    --debug.log("ownV_mag: " .. vectorMagnitude(ownVelVec))
                    if R_mag > 1e-6 then -- ゼロ除算回避
                        R_hat = vectorNormalize(R_vec)
                        Vc = -vectorDot(R_vec, V_vec) / R_mag
                        LOS_Rate_vector = vectorScalarMul(1.0 / (R_mag ^ 2), vectorCross(R_vec, V_vec))
                        Omega_cross_Rhat = vectorCross(LOS_Rate_vector, R_hat)
                        --Accel_cmd_global = vectorScalarMul(NAVIGATION_GAIN, vectorCross(LOS_Rate_vector, ownVelVec)) -- 近接速度を使わない方が安定する可能性が高い
                        Accel_cmd_global = vectorScalarMul(NAVIGATION_GAIN * Vc, Omega_cross_Rhat)
                        Accel_cmd_local = rotateVectorByInverseQuaternion(Accel_cmd_global, ownOrientation)

                        -- nil チェックを省略しているので、計算失敗時は以降の行でエラーになる可能性あり
                        pitchControl = Accel_cmd_local[2] * FIN_GAIN
                        yawControl = Accel_cmd_local[1] * FIN_GAIN

                        VcAverage = VcAverageCalculator(Vc)

                        -- 近接速度がマイナスの場合は自爆
                        if VcAverage < 0 and #VcArry > 10 then
                            proximityFuseTrigger = true
                        end
                        --[[debug.log("VcAVG: " ..
                            VcAverage .. " dist: " .. R_mag - (VcAverage * DT * FUSE_PROXIMITY_DURATION_TICKS))]]
                        --debug.log("ownSpeed: " .. math.sqrt(ownVelVec[1] ^ 2 + ownVelVec[2] ^ 2 + ownVelVec[3] ^ 2))
                        if R_mag - (VcAverage * DT * FUSE_PROXIMITY_DURATION_TICKS) < PROXIMITY_DISTANCE then
                            proximityFuseTrigger = true
                        end
                    else
                        -- 距離ゼロの場合 (フェイルセーフ)
                        pitchControl = 0
                        yawControl = 0
                        if usePnWithFilterOutput then proximityFuseTrigger = true end
                    end
                else
                    -- 誘導目標なし
                    pitchControl = 0
                    yawControl = 0
                    proximityFuseTrigger = false
                end
            end -- 初期誘導/通常誘導の分岐終了

            -- 5. 制御量制限
            pitchControl = math.max(-MAX_CONTROL, math.min(MAX_CONTROL, pitchControl or 0))
            yawControl = math.max(-MAX_CONTROL, math.min(MAX_CONTROL, yawControl or 0))

            -- 6. 出力
            output.setBool(1, proximityFuseTrigger)
            output.setNumber(1, yawControl)   -- ch1: ヨー制御
            output.setNumber(2, pitchControl) -- ch2: ピッチ制御
        end
    end
end
