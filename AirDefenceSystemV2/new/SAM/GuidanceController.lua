--[[
================================================================================
GuidanceController.lua (v0.1 - 骨子)
================================================================================
機能:
- KalmanFilter_SAMからの目標情報と自機センサー情報に基づき誘導計算を行う。
- isTrackingフラグに応じて、比例航法(終末誘導)と単純追尾(中間/慣性)を切り替える。
- 比例航法は速度ベクトルベースで計算する。
- クォータニオンで自機姿勢を管理する。
- 計算されたヨー・ピッチ制御量と近接信管信号を出力する。

入力 (コンポジット信号 - 想定):
- 数値 1: 推定目標X (targetPosX)
- 数値 2: 推定目標Y (targetPosY)
- 数値 3: 推定目標Z (targetPosZ)
- 数値 4: 推定目標Vx (targetVelX)
- 数値 5: 推定目標Vy (targetVelY)
- 数値 6: 推定目標Vz (targetVelZ)
- 数値 7: データリンク目標X (dataLinkX)
- 数値 8: データリンク目標Y (dataLinkY)
- 数値 9: データリンク目標Z (dataLinkZ)
- 数値 10: 自機位置X (ownPosX)
- 数値 11: 自機位置Y (ownPosY)
- 数値 12: 自機位置Z (ownPosZ)
- 数値 13: 自機Pitch (ownPitch)
- 数値 14: 自機Yaw (ownYaw)
- 数値 15: 自機Roll (ownRoll)
- 数値 16: 自機速度Vx (ownVelX)
- 数値 17: 自機速度Vy (ownVelY)
- 数値 18: 自機速度Vz (ownVelZ)
- 数値 32: イプシロン (targetEpsilon)

- オンオフ 1: トラッキング成功フラグ (isTracking)

出力 (コンポジット信号):
- オンオフ 1: 近接信管起動 (proximityFuseTrigger)
- 数値 1: ヨー制御 (yawControl)
- 数値 2: ピッチ制御 (pitchControl)

前提:
- 必要なベクトル演算、クォータニオン演算、座標変換関数が定義されていること。
- 各種パラメータはプロパティから読み込む。
================================================================================
]]
local PI, PI2, DT, NAVIGATION_GAIN, FIN_GAIN, SIMPLE_TRACK_GAIN, PROXIMITY_DISTANCE, MAX_CONTROL, PN_GUIDANCE_EPSILON_THRESHOLD, LOGIC_DELAY
-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60 -- 必要に応じて使用

-- プロパティ読み込み
NAVIGATION_GAIN = property.getNumber("N") or 3.5
FIN_GAIN = property.getNumber("FinGain") or 1.0
SIMPLE_TRACK_GAIN = property.getNumber("SimpleTrackGain") or 1.0
PROXIMITY_DISTANCE = property.getNumber("ProximityDistance") or 5.0
MAX_CONTROL = property.getNumber("MaxControl") or 1.0
PN_GUIDANCE_EPSILON_THRESHOLD = property.getNumber("PN_GUIDANCE_EPSILON_THRESHOLD")
LOGIC_DELAY = property.getNumber("LOGIC_DELAY") + 1
-- グローバル変数 (状態保持用)
-- (このシンプルな例では不要かもしれないが、必要に応じて追加)


-- ヘルパー関数群
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
    x2 = v2[1] or
        v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return { x1 - x2, y1 - y2, z1 - z2 }
end

function vectorDot(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1] or v1.x or 0
    y1 = v1[2] or v1.y or 0
    z1 = v1[3] or v1.z or 0
    x2 = v2[1] or
        v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return x1 * x2 + y1 * y2 + z1 * z2
end

function vectorCross(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1] or v1.x or 0
    y1 = v1[2] or v1.y or 0
    z1 = v1[3] or v1.z or 0
    x2 = v2[1] or
        v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return { y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2 }
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
    z1 =
        q_a[4]
    w2 = q_b[1]
    x2 = q_b[2]
    y2 = q_b[3]
    z2 = q_b[4]
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result =
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 +
        x1 * y2 -
        y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z
    half_roll = roll * 0.5
    half_yaw = yaw *
        0.5
    half_pitch = pitch * 0.5
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cy = math.cos(half_yaw)
    sy =
        math
        .sin(half_yaw)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    w = cr * cy * cp + sr * sy * sp
    x = cr *
        cy *
        sp - sr * sy * cp
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
    return { p_prime[2], p_prime[3], p_prime[4] }
end

function rotateVectorByInverseQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz =
        vector[3] or vector.z or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp =
        multiplyQuaternions(q_conj, p)
    p_prime = multiplyQuaternions(temp, q)
    return { p_prime[2], p_prime[3], p_prime
        [4] }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local gX, gY, gZ, oX, oY, oZ, relativeVectorGlobal, localVector
    gX = globalTargetPos.x or globalTargetPos[1] or 0
    gY =
        globalTargetPos.y or globalTargetPos[2] or 0
    gZ = globalTargetPos.z or globalTargetPos[3] or 0
    oX =
        ownGlobalPos.x or
        0
    oY = ownGlobalPos.y or 0
    oZ = ownGlobalPos.z or 0
    relativeVectorGlobal = { gX - oX, gY - oY, gZ - oZ }
    localVector =
        rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientationQuat)
    return {
        x = localVector[1],
        y =
            localVector[2],
        z = localVector[3]
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    -- 関数冒頭でローカル変数を宣言
    local targetPosX, targetPosY, targetPosZ, targetVelX, targetVelY, dataLinkX, dataLinkY, dataLinkZ, dataLinkTargetPosVec, targetVelZ, targetEpsilon, isTracking
    local ownPosX, ownPosY, ownPosZ, ownVelX, ownVelY, ownVelZ, ownPitch, ownYaw, ownRoll
    local ownOrientation, ownPosVec, ownVelVec, targetPosVec, targetVelVec
    local pitchControl, yawControl, proximityFuseTrigger
    local R_vec, V_vec, R_mag, Vc, LOS_Rate_vector, Accel_cmd_global, Accel_cmd_local
    local targetLocal, horizontalDistance, currentLocalAzimuth, currentLocalElevation

    -- 1. 入力読み取り
    targetPosX = input.getNumber(1)
    targetPosY = input.getNumber(2)
    targetPosZ = input.getNumber(3)
    targetVelX = input.getNumber(4)
    targetVelY = input.getNumber(5)
    targetVelZ = input.getNumber(6)
    targetEpsilon = input.getNumber(32)
    isTracking = input.getBool(1)

    dataLinkX = input.getNumber(7)
    dataLinkY = input.getNumber(8)
    dataLinkZ = input.getNumber(9)
    dataLinkTargetPosVec = { dataLinkX, dataLinkY, dataLinkZ }

    ownPosX = input.getNumber(10)
    ownPosY = input.getNumber(11)
    ownPosZ = input.getNumber(12)
    ownPitch = input.getNumber(13)
    ownYaw = input.getNumber(14)
    ownRoll = input.getNumber(15)

    -- 2. 姿勢管理
    ownOrientation = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch) -- 自機姿勢(クォータニオン)
    if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end  -- エラー時は単位クォータニオン

    -- 3. 変数準備
    ownPosVec = { ownPosX, ownPosY, ownPosZ }
    targetPosVec = { targetPosX, targetPosY, targetPosZ }
    targetVelVec = { targetVelX, targetVelY, targetVelZ }

    -- ↓↓↓ 修正箇所 ↓↓↓
    local ownVelLocalX = input.getNumber(16)                            -- ローカル速度X読み取り
    local ownVelLocalY = input.getNumber(17)                            -- ローカル速度Y読み取り
    local ownVelLocalZ = input.getNumber(18)                            -- ローカル速度Z読み取り
    local ownVelLocalVec = { ownVelLocalX, ownVelLocalY, ownVelLocalZ } -- ローカル速度ベクトル作成

    -- ローカル速度をグローバル速度に変換
    local ownVelVec = rotateVectorByQuaternion(ownVelLocalVec, ownOrientation)
    if ownVelVec == nil then ownVelVec = { 0, 0, 0 } end -- 変換失敗時はゼロベクトル
    -- ↑↑↑ 修正箇所 ↑↑↑

    targetPosVec = { targetPosX, targetPosY, targetPosZ }
    targetVelVec = { targetVelX, targetVelY, targetVelZ }

    -- 4. 誘導計算と近接信管
    pitchControl = 0             -- 初期化
    yawControl = 0               -- 初期化
    proximityFuseTrigger = false -- 初期化


    if isTracking and targetEpsilon < PN_GUIDANCE_EPSILON_THRESHOLD then -- 閾値はプロパティから設定できるようにするべき
        -- === 終末誘導 (比例航法) ===
        R_vec = vectorSub(targetPosVec, ownPosVec)
        V_vec = vectorSub(targetVelVec, ownVelVec)
        R_mag = vectorMagnitude(R_vec)
        debug.log("R_vecX: " ..
            tostring(R_vec[1]) .. ", R_vecY: " .. tostring(R_vec[2]) .. ", R_vecZ: " .. tostring(R_vec[3]))
        debug.log("V_vecX: " ..
            tostring(V_vec[1]) .. ", V_vecY: " .. tostring(V_vec[2]) .. ", V_vecZ: " .. tostring(V_vec[3]))
        debug.log("R_mag: " .. tostring(R_mag))
        if R_mag > 1e-6 then -- ゼロ除算回避 (最小限のチェック)
            Vc = -vectorDot(R_vec, V_vec) / R_mag
            debug.log("Vc: " .. tostring(Vc))
            LOS_Rate_vector = vectorScalarMul(1.0 / (R_mag ^ 2), vectorCross(R_vec, V_vec))
            debug.log("LOS_Rate_vectorX: " ..
                tostring(LOS_Rate_vector[1]) ..
                ", LOS_Rate_vectorY: " ..
                tostring(LOS_Rate_vector[2]) .. ", LOS_Rate_vectorZ: " .. tostring(LOS_Rate_vector[3]))
            Accel_cmd_global = vectorScalarMul(NAVIGATION_GAIN, vectorCross(LOS_Rate_vector, ownVelVec))
            debug.log("Accel_cmd_globalX: " ..
                tostring(Accel_cmd_global[1]) ..
                ", Accel_cmd_globalY: " ..
                tostring(Accel_cmd_global[2]) .. ", Accel_cmd_globalZ: " .. tostring(Accel_cmd_global[3]))
            Accel_cmd_local = rotateVectorByInverseQuaternion(Accel_cmd_global, ownOrientation)
            debug.log("Accel_cmd_localX: " ..
                tostring(Accel_cmd_local[1]) ..
                ", Accel_cmd_localY: " ..
                tostring(Accel_cmd_local[2]) .. ", Accel_cmd_localZ: " .. tostring(Accel_cmd_local[3]))

            -- ローカル加速度からフィン制御量へ
            pitchControl = Accel_cmd_local[2] * FIN_GAIN -- Local Y -> Pitch
            yawControl = Accel_cmd_local[1] * FIN_GAIN   -- Local X -> Yaw

            -- 近接信管判定
            if R_mag - DT * Vc * LOGIC_DELAY < PROXIMITY_DISTANCE then
                proximityFuseTrigger = true
            end
        else
            -- 距離がほぼゼロの場合の処理（フェイルセーフ）
            pitchControl = 0
            yawControl = 0
            proximityFuseTrigger = true -- 衝突したとみなす
        end
    else
        -- === 中間/慣性誘導 (単純追尾) ===
        -- フィルターからの位置情報が有効か確認 (基本的なチェック)
        if not (dataLinkX == 0 and dataLinkY == 0 and dataLinkZ == 0) then
            targetPosVec = dataLinkTargetPosVec
            targetLocal = globalToLocalCoords(targetPosVec, { x = ownPosX, y = ownPosY, z = ownPosZ }, ownOrientation)
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
            -- フィルターからの位置情報が無効な場合
            pitchControl = 0
            yawControl = 0
        end
        proximityFuseTrigger = false -- 中間誘導では作動しない
    end

    -- 5. 制御量制限
    pitchControl = math.max(-MAX_CONTROL, math.min(MAX_CONTROL, pitchControl or 0)) -- nilガードを追加
    yawControl = math.max(-MAX_CONTROL, math.min(MAX_CONTROL, yawControl or 0))     -- nilガードを追加

    -- 6. 出力
    output.setBool(1, proximityFuseTrigger)
    output.setNumber(1, yawControl)
    output.setNumber(2, pitchControl)
end
