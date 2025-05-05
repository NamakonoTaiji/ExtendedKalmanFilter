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
local PI, SIMPLE_TRACK_GAIN, MAX_CONTROL
-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60

-- プロパティ読み込み
SIMPLE_TRACK_GAIN = property.getNumber("SimpleTrackGain") -- 単追尾の動翼の強さ係数
MAX_CONTROL = property.getNumber("MaxControl")            -- 動翼への出力の上限
-- グローバル変数
local launchTickCounter = 0                               -- 発射後のTickカウンター
local targetPosX, targetPosY, targetPosZ, targetVelX, targetVelY, targetVelZ = 0, 0, 0, 0, 0, 0
local deployParachute = false
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

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    local isLaunch = input.getBool(1)
    if isLaunch and not deployParachute then
        -- 関数冒頭でローカル変数を宣言
        local dataLinkX, dataLinkY, dataLinkZ, dataLinkTargetPosVec
        local ownPosX, ownPosY, ownPosZ, ownPitch, ownYaw, ownRoll
        local ownOrientation, targetPosVec, boosterStrength
        local pitchControl, yawControl, dataLinkDist, ownPosVec
        local targetLocal, horizontalDistance, currentLocalAzimuth, currentLocalElevation

        dataLinkX = input.getNumber(1)
        dataLinkY = input.getNumber(2)
        dataLinkZ = input.getNumber(3)
        dataLinkTargetPosVec = { dataLinkX, dataLinkY, dataLinkZ }
        ownPosX = input.getNumber(4)
        ownPosY = input.getNumber(5)
        ownPosZ = input.getNumber(6)
        ownPitch = input.getNumber(7)
        ownYaw = input.getNumber(8)
        ownRoll = input.getNumber(9)
        ownPosVec = { ownPosX, ownPosY, ownPosZ }
        dataLinkDist = vectorMagnitude(vectorSub(ownPosVec, dataLinkTargetPosVec))

        if dataLinkDist > 2000 then
            dataLinkTargetPosVec[2] = math.max(200, dataLinkY)
        end
        if dataLinkDist < 500 and dataLinkDist > 100 then
            boosterStrength = 0.6
        elseif dataLinkDist < 100 then
            deployParachute = true
            boosterStrength = 0
        else
            boosterStrength = 1
        end
        -- Tickカウンター更新
        launchTickCounter = launchTickCounter + 1
        -- 2. 姿勢管理
        ownOrientation = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch) -- 自機姿勢(クォータニオン)
        if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end  -- エラー時は単位クォータニオン

        -- 3. 変数準備
        targetPosVec = { targetPosX, targetPosY, targetPosZ }

        pitchControl = 0 -- 初期化
        yawControl = 0   -- 初期化

        -- === 初期誘導フェーズ (単純追尾) ===

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

        -- 5. 制御量制限
        pitchControl = math.max(-MAX_CONTROL, math.min(MAX_CONTROL, pitchControl or 0))
        yawControl = math.max(-MAX_CONTROL, math.min(MAX_CONTROL, yawControl or 0))
        -- 6. 出力
        output.setBool(1, deployParachute)
        output.setNumber(1, yawControl)      -- ch1: ヨー制御
        output.setNumber(2, pitchControl)    -- ch2: ピッチ制御
        output.setNumber(3, boosterStrength) -- ch3: ブースター燃焼速度
    end
end
