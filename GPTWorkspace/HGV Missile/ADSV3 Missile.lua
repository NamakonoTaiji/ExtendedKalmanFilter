--[[
ミサイルの誘導部分を担うスクリプト。アクティブレーダー圏内までは防空システムと通信して中間誘導/単追尾を行う。終末誘導時はアクティブ誘導/比例航法
至近距離ではミサイル出力を使用して近接信管で目標を破壊する。

-- 入力:
- bool 1: KFが有効な追跡目標を出力中 (現行誘導では未使用)
- bool 2: 対水上モードか否か
- bool 3: 発射済みか否か
- bool 32: ミサイル出力用レーダーが目標を検出中
- num 1: 推定目標座標 X
- num 2: 推定目標座標 Y
- num 3: 推定目標座標 Z
- num 4: 推定目標速度 Vx
- num 5: 推定目標速度 Vy
- num 6: 推定目標速度 Vz
- num 10: レーダー方位角マニュアル制御
- num 11: レーダー仰角マニュアル制御
- num 12: トラック中の目標ID
- num 13-15: 自機X,Y,Z座標パススルー
- num 16-19: 自機姿勢(クォータニオンw,x,y,z)
- num 32: 最新のイプシロンε

-- 出力:
- bool 1: ミサイル出力レーダーon/off
- bool 2: 対水上用衝撃信管on/off
- num 1: Yaw軸フィン
- num 2: Pitch軸フィン
- num 3: 近接速度

-- 追加プロパティ:
- bool GRAVITY_COMP_ENABLE: 重力補償の有効化
- num GRAVITY_COMP_GAIN: 重力補償のフィン加算ゲイン
]]


DT                             = 1 / 60
PI                             = math.pi
PI2                            = PI * 2

oldDistance                    = 0
missileRadarIO                 = false
isLaunched                     = false
lauchedCount                   = 0
initialGuidanceCounter         = 0
isGuidanceStart                = false
isPPN                          = true
isHeadCapture                  = false
mainRadarIO                    = false
targetCoords                   = { 0, 0, 0 }
targetVelocity                 = { 0, 0, 0 }
previousOwnCoords              = nil
previousPreviousOwnCoords      = nil
selfDetonateCount              = 0
fuseIO                         = false

--------------------------------------------------------------------------------
-- ビーム機動試験用デバッグ設定・集計状態
-- 誘導・終末レーダー有効化・最接近・自爆を記録する。FOV計測はMissileFOVDebug.lua側で行う。
--------------------------------------------------------------------------------
DEBUG_LOG_ENABLED              = true
debugTick                      = 0
debugLaunchTick                = nil
debugPNStarted                 = false
debugPreviousTerminalRadar     = false
debugEverRadarDetection        = false
debugCurrentLossTicks          = 0
debugMaximumLossTicks          = 0
debugReacquired                = false
debugMinimumDistance           = math.huge
debugWasClosing                = false
debugClosestPassLogged         = false
debugFinalSummaryLogged        = false

DIVE_START_TANJENT             = math.tan(math.rad(70))                               -- 巡航モードからダイブを開始させる角度
PN_FIN_STRENGTH                = property.getNumber("PN_FIN_STRENGTH")                -- 比例航法時のフィンにかける係数
PPN_FIN_STRENGTH               = property.getNumber("PPN_FIN_STRENGTH")               -- 単追尾時のフィンにかける係数
MISSILE_FIN_DISTANCE_THRESHOLD = property.getNumber("MISSILE_FIN_DISTANCE_THRESHOLD") -- 至近距離で誘導を行う広角レーダーの有効範囲
SKIMMING_ALT                   = property.getNumber("SKIMMING_ALT")                   -- 対水上モード巡航高度
GUIDANCE_START_ALTITUDE        = property.getNumber("GUIDANCE_START_ALTITUDE")        -- 誘導開始高度
LOGIC_DELAY                    = property.getNumber("LOGIC_DELAY")

-- 重力補償（高度保持は行わない）
GRAVITY_COMP_ENABLE            = property.getBool("GRAVITY_COMP_ENABLE")
GRAVITY_COMP_GAIN              = property.getNumber("GRAVITY_COMP_GAIN")

HEAD_CORRIDOR_RADIUS           = property.getNumber("HEAD_CORRIDOR_RADIUS")                           -- 目標の進行軸を中心とした、終末誘導へ切り替えてよい円筒状の回廊半径
HEAD_UP_OFFSET                 = math.min(property.getNumber("HEAD_UP_OFFSET"), HEAD_CORRIDOR_RADIUS) -- 待ち伏せ点を、目標の進行方向上の点からさらに垂直上方へ何mずらすか
HEAD_RELEASE_DISTANCE          = property.getNumber("HEAD_RELEASE_DISTANCE")                          -- 目標から見て、迎撃ミサイルが進行方向前方へ何m出たら待ち伏せ完了と判定するか
BALLISTIC_MIN_SPEED            = property.getNumber("BALLISTIC_MIN_SPEED")                            -- 高高度目標へ待ち伏せ誘導を適用するための、最低速度判定
HEAD_LEAD_MIN_SCALE            = property.getNumber("HEAD_LEAD_MIN_SCALE")
HEAD_LEAD_MAX_SCALE            = property.getNumber("HEAD_LEAD_MAX_SCALE")

-- グローバル座標系での前フレームの正規化されたLOSベクトルを保存
-- {x, y, z} 形式で保存
oldLOS_vec_global_normalized   = { x = 0, y = 0, z = 1 } -- 初期値 (例: 前方)

oldLOS                         = { azimuth = 0, elevation = 0 }
currentLOS                     = { azimuth = 0, elevation = 0 }
LOStable                       = { old = oldLOS, current = currentLOS }
isInit                         = true -- 近接速度計算初期化
--- 3Dベクトル a から b を引きます (a - b)
---@param a Vector3 {x: number, y: number, z: number} または {number, number, number}
---@param b Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return Vector3 結果のベクトル {x, y, z}
function subtract(a, b)
    local ax = a.x or a[1] or 0
    local ay = a.y or a[2] or 0
    local az = a.z or a[3] or 0
    local bx = b.x or b[1] or 0
    local by = b.y or b[2] or 0
    local bz = b.z or b[3] or 0
    return { x = ax - bx, y = ay - by, z = az - bz }
end

--- 3Dベクトル v の大きさを計算します
---@param v Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return number ベクトルの大きさ
function vector_magnitude(v)
    local x = v.x or v[1] or 0
    local y = v.y or v[2] or 0
    local z = v.z or v[3] or 0
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

--- 3Dベクトル v を正規化 (単位ベクトル化) します
---@param v Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return Vector3 正規化されたベクトル {x, y, z}
function normalize(v)
    local x = v.x or v[1] or 0
    local y = v.y or v[2] or 0
    local z = v.z or v[3] or 0
    local mag = math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
    if mag > 1e-9 then -- ゼロ除算を避ける
        return { x / mag, y / mag, z / mag }
    else
        return { 0, 0, 1 } -- ゼロベクトルの場合は前方 Z を返す (安全策)
    end
end

--- 2つの3Dベクトルの外積 (クロス積) を計算します (a x b)
---@param a Vector3 {x: number, y: number, z: number} または {number, number, number}
---@param b Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return Vector3 外積ベクトル {rx, ry, rz} (配列形式)
function cross_product(a, b)
    local ax = a.x or a[1] or 0
    local ay = a.y or a[2] or 0
    local az = a.z or a[3] or 0
    local bx = b.x or b[1] or 0
    local by = b.y or b[2] or 0
    local bz = b.z or b[3] or 0

    local rx = ay * bz - az * by
    local ry = az * bx - ax * bz
    local rz = ax * by - ay * bx

    return { rx, ry, rz }
end

function dot(a, b)
    local ax = a.x or a[1] or 0
    local ay = a.y or a[2] or 0
    local az = a.z or a[3] or 0

    local bx = b.x or b[1] or 0
    local by = b.y or b[2] or 0
    local bz = b.z or b[3] or 0

    return ax * bx + ay * by + az * bz
end

--------------------------------------------------------------------------------
-- クォータニオン演算関数
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2, w_result, x_result, y_result, z_result
    -- nilチェックは原則削除
    w1 = q_a[1]
    x1 = q_a[2]
    y1 = q_a[3]
    z1 = q_a[4]
    w2 = q_b[1]
    x2 = q_b[2]
    y2 = q_b[3]
    z2 = q_b[4]
    -- nilチェックは原則削除
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

-- 物理センサーのロールピッチヨーからクォータニオンへ変換
function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z
    -- nilチェックは原則削除
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

function rotateVectorByInverseQuaternion(vector, quaternion)
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
    temp = multiplyQuaternions(q_conj, p)
    -- nilチェックは原則削除
    p_prime = multiplyQuaternions(temp, q)
    -- nilチェックは原則削除
    return { p_prime[2], p_prime[3], p_prime[4] }
end

--------------------------------------------------------------------------------
-- 重力補償
--------------------------------------------------------------------------------

-- ワールド座標の重力方向を機体ローカル座標へ変換する。
function getLocalGravity(quaternion)
    local gravityLocal =
        rotateVectorByInverseQuaternion(
            { 0, -1, 0 },
            quaternion
        )

    return {
        x = gravityLocal[1],
        y = gravityLocal[2],
        z = gravityLocal[3]
    }
end

-- ローカル座標上の補正加速度方向をpitch/yawフィン補正へ変換する。
function vectorToFinCorrection(correctionVector, velocityDir)
    -- 速度方向成分は進行方向を変えないため除去する。
    local parallel =
        correctionVector.x * velocityDir.x +
        correctionVector.y * velocityDir.y +
        correctionVector.z * velocityDir.z

    local normal = {
        x = correctionVector.x - parallel * velocityDir.x,
        y = correctionVector.y - parallel * velocityDir.y,
        z = correctionVector.z - parallel * velocityDir.z
    }

    -- 魚雷側と同じ座標軸・フィン符号。
    local turnAxis = cross_product(normal, velocityDir)

    return turnAxis[1], -turnAxis[2]
end

-- 速度に直交する重力成分だけを打ち消す。
-- 高度目標や高度誤差は使用しない。
function getGravityCorrection(localVelocity, gravity)
    if not GRAVITY_COMP_ENABLE then
        return 0, 0
    end

    local speed =
        math.sqrt(
            localVelocity.x * localVelocity.x +
            localVelocity.y * localVelocity.y +
            localVelocity.z * localVelocity.z
        )

    if speed <= 0.1 then
        return 0, 0
    end

    local velocityDir = {
        x = localVelocity.x / speed,
        y = localVelocity.y / speed,
        z = localVelocity.z / speed
    }

    local gravityParallel =
        gravity.x * velocityDir.x +
        gravity.y * velocityDir.y +
        gravity.z * velocityDir.z

    local gravityNormal = {
        x = gravity.x - gravityParallel * velocityDir.x,
        y = gravity.y - gravityParallel * velocityDir.y,
        z = gravity.z - gravityParallel * velocityDir.z
    }

    local gravityCorrectionVector = {
        x = -gravityNormal.x * GRAVITY_COMP_GAIN,
        y = -gravityNormal.y * GRAVITY_COMP_GAIN,
        z = -gravityNormal.z * GRAVITY_COMP_GAIN
    }

    return vectorToFinCorrection(
        gravityCorrectionVector,
        velocityDir
    )
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

--- ワールド座標系のグローバル座標を、特定のオブジェクトを基準としたローカル座標に変換します。
---@description 基準オブジェクトからターゲットへの相対ベクトルを計算し、
--              オブジェクトの姿勢（逆クォータニオン）で回転させることでローカル座標を求めます。
---@param globalTargetPos Vector3 変換したいターゲットのグローバル座標。
---@param objectGlobalPos Vector3 基準となるオブジェクトのグローバル座標。
---@param objectOrientationQuat Quaternion 基準となるオブジェクトの姿勢を表すクォータニオン。
---@return Vector3 基準オブジェクトから見たターゲットのローカル座標。
function globalToLocal(globalTargetPos, objectGlobalPos, objectOrientationQuat)
    -- 1. 基準オブジェクトからターゲットへの相対ベクトルをグローバル座標系で計算
    local relativeVecGlobal = {
        x = (globalTargetPos.x or 0) - (objectGlobalPos.x or 0),
        y = (globalTargetPos.y or 0) - (objectGlobalPos.y or 0),
        z = (globalTargetPos.z or 0) - (objectGlobalPos.z or 0)
    }

    -- 2. 逆クォータニオンを使ってグローバルな相対ベクトルを回転させ、ローカル座標系でのベクトルに変換
    local localVec = rotateVectorByInverseQuaternion(
        { relativeVecGlobal.x, relativeVecGlobal.y, relativeVecGlobal.z },
        objectOrientationQuat
    )

    -- 3. ローカル座標ベクトルを {x, y, z} 形式のテーブルとして返す
    return { x = localVec[1], y = localVec[2], z = localVec[3] }
end

--- ローカル座標から方位角と仰角へ変換
---@param localPosVec Vector3 変換したいローカル座標(X右方向, Y上方向, Z前方向)
---@return table azimuthとelevationを返します(ラジアン)
function coordsToAngle(localPosVec)
    local horizontalDistance, currentLocalAzimuth, currentLocalElevation
    horizontalDistance = math.sqrt(localPosVec.x ^ 2 + localPosVec.z ^ 2)
    currentLocalAzimuth = math.atan(localPosVec.x, localPosVec.z)        -- atan(左右, 前後)
    currentLocalElevation = math.atan(localPosVec.y, horizontalDistance) -- atan(上下, 水平距離)
    return { azimuth = currentLocalAzimuth, elevation = currentLocalElevation }
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

--------------------------------------------------------------------------------
-- ビーム機動試験用デバッグログ
-- debug.logはこの関数へ集約する。距離はm、時間はtick。
--------------------------------------------------------------------------------
function updateGuidanceDebug(isLaunch, isPN, terminalRadarOn, isRadarDetecting,
                             distance, approachVelocity, selfDetonation)
    debugTick = debugTick + 1
    if not DEBUG_LOG_ENABLED then return end

    local function rounded(value, scale)
        if value >= 0 then
            return math.floor(value * scale + 0.5) / scale
        end
        return math.ceil(value * scale - 0.5) / scale
    end

    if isLaunch and debugLaunchTick == nil then
        debugLaunchTick = debugTick
        debug.log("[AA_DBG] LAUNCH tick=" .. debugTick .. " distance_m=" .. rounded(distance, 10))
    end

    if not isLaunch then
        debugPreviousTerminalRadar = terminalRadarOn
        return
    end

    local flightTick = debugTick - debugLaunchTick

    if distance > 0 and distance < debugMinimumDistance then
        debugMinimumDistance = distance
    end

    if isPN and not debugPNStarted then
        debugPNStarted = true
        debug.log("[AA_DBG] PN_START tick=" .. debugTick .. " flight_tick=" .. flightTick ..
            " distance_m=" .. rounded(distance, 10))
    end

    if terminalRadarOn and not debugPreviousTerminalRadar then
        debug.log("[AA_DBG] TERMINAL_RADAR_ON tick=" .. debugTick .. " flight_tick=" .. flightTick ..
            " distance_m=" .. rounded(distance, 10))
    end

    if terminalRadarOn then
        if isRadarDetecting then
            if debugCurrentLossTicks > 0 then
                debugReacquired = true
                debug.log("[AA_DBG] RADAR_REACQUIRED tick=" .. debugTick .. " flight_tick=" .. flightTick ..
                    " distance_m=" .. rounded(distance, 10) ..
                    " loss_ticks=" .. debugCurrentLossTicks)
            elseif not debugEverRadarDetection then
                debug.log("[AA_DBG] RADAR_ACQUIRED tick=" .. debugTick .. " flight_tick=" .. flightTick ..
                    " distance_m=" .. rounded(distance, 10))
            end
            debugEverRadarDetection = true
            debugCurrentLossTicks = 0
        elseif debugEverRadarDetection then
            debugCurrentLossTicks = debugCurrentLossTicks + 1
            if debugCurrentLossTicks > debugMaximumLossTicks then
                debugMaximumLossTicks = debugCurrentLossTicks
            end
            if debugCurrentLossTicks == 1 then
                debug.log("[AA_DBG] RADAR_LOST_START tick=" .. debugTick .. " flight_tick=" .. flightTick ..
                    " distance_m=" .. rounded(distance, 10))
            end
        end

        if flightTick % 60 == 0 then
            debug.log("[AA_DBG] STATUS tick=" .. debugTick .. " flight_tick=" .. flightTick ..
                " distance_m=" .. rounded(distance, 10) .. " pn=" .. tostring(isPN) ..
                " radar_detecting=" .. tostring(isRadarDetecting) ..
                " current_loss_ticks=" .. debugCurrentLossTicks ..
                " max_loss_ticks=" .. debugMaximumLossTicks ..
                " reacquired=" .. tostring(debugReacquired) ..
                " min_distance_m=" .. rounded(debugMinimumDistance, 10))
        end
    end

    if approachVelocity < 0 then
        debugWasClosing = true
    elseif approachVelocity > 0 and debugWasClosing and not debugClosestPassLogged then
        debugClosestPassLogged = true
        debug.log("[AA_DBG] CLOSEST_PASS tick=" .. debugTick .. " flight_tick=" .. flightTick ..
            " min_distance_m=" .. rounded(debugMinimumDistance, 10))
    end

    if selfDetonation and not debugFinalSummaryLogged then
        debugFinalSummaryLogged = true
        debug.log("[AA_DBG] FINAL reason=self_detonation tick=" .. debugTick .. " flight_tick=" .. flightTick ..
            " max_loss_ticks=" .. debugMaximumLossTicks ..
            " reacquired=" .. tostring(debugReacquired) ..
            " min_distance_m=" .. rounded(debugMinimumDistance, 10))
    end

    debugPreviousTerminalRadar = terminalRadarOn
end

function onTick()
    -- 1. 座標・オイラー角の取得
    local isMissileRadarDetecting, ownCoords, predictedOwnCoords, ownOrientation, worldVelocity, selfLocalVelocity
    local localVelocityArray, isAntiShipMode, isLaunch, distance, targetCoordsVec, ownCoordsVec, activeTargetCoordsVec, activeTargetVelocitysVec, LOS_vec_global
    isMissileRadarDetecting = input.getBool(32) -- 至近距離で使うミサイルレーダーが目標を検出中か

    targetCoords            = { input.getNumber(1), input.getNumber(2), input.getNumber(3) }

    targetVelocity          = { input.getNumber(4), input.getNumber(5), input.getNumber(6) }

    ownCoords               = { input.getNumber(13), input.getNumber(14), input.getNumber(15) }
    ownOrientation          = { input.getNumber(16), input.getNumber(17), input.getNumber(18), input.getNumber(19) }

    -- 自機速度は1階差分、加速度は速度の差分（位置の2階差分）から求める。
    -- 誘導LOSにはLOGIC_DELAY tick先の位置を使い、姿勢は現在値のままにする。
    worldVelocity           = { 0, 0, 0 }
    predictedOwnCoords      = {}
    local previousPosition = previousOwnCoords or ownCoords
    local delaySquaredHalf = LOGIC_DELAY ^ 2 * 0.5
    for axis = 1, 3 do
        local velocityPerTick = ownCoords[axis] - previousPosition[axis]
        local accelerationPerTick2 = 0
        worldVelocity[axis] = velocityPerTick / DT
        if previousPreviousOwnCoords then
            accelerationPerTick2 = velocityPerTick - previousPosition[axis] + previousPreviousOwnCoords[axis]
        end
        predictedOwnCoords[axis] = ownCoords[axis]
            + velocityPerTick * LOGIC_DELAY
            + accelerationPerTick2 * delaySquaredHalf
    end
    previousPreviousOwnCoords = previousOwnCoords
    previousOwnCoords = ownCoords

    localVelocityArray =
        rotateVectorByInverseQuaternion(
            worldVelocity,
            ownOrientation
        )
    selfLocalVelocity  = {
        x = localVelocityArray[1],
        y = localVelocityArray[2],
        z = localVelocityArray[3]
    }

    isAntiShipMode     = input.getBool(2)
    isLaunch           = input.getBool(3)
    distance           = vector_magnitude(subtract(targetCoords, predictedOwnCoords))

    if isLaunch and not isLaunched then
        lauchedCount = lauchedCount + 1
        if lauchedCount > 20 then
            isLaunched = true
        end
    end

    -- {x, y, z} 形式のベクトルテーブルに変換
    targetCoordsVec          = { x = targetCoords[1], y = targetCoords[2], z = targetCoords[3] }
    ownCoordsVec             = { x = predictedOwnCoords[1], y = predictedOwnCoords[2], z = predictedOwnCoords[3] }
    activeTargetCoordsVec    = targetCoordsVec
    activeTargetVelocitysVec = { vx = targetVelocity[1], vy = targetVelocity[2], vz = targetVelocity[3] }

    -- 指定された高度mまで垂直上昇
    if isLaunch then
        if ownCoordsVec.y < GUIDANCE_START_ALTITUDE and not isGuidanceStart then
            activeTargetCoordsVec = { x = ownCoordsVec.x, y = ownCoordsVec.y + 500, z = ownCoordsVec.z }
        else
            isGuidanceStart = true
        end
    end

    -- 指定された高度に到達したら一秒間単追尾で目標に指向
    if isGuidanceStart then
        mainRadarIO = true
        initialGuidanceCounter = initialGuidanceCounter + 1
        if initialGuidanceCounter > 60 then
            isPPN = false
        end
    end

    ----------------------------------------------------------------------------
    -- 対水上モード時の低空巡航 (シースキミング) 処理
    ----------------------------------------------------------------------------
    local yawAngle, pitchAngle
    if isGuidanceStart then
        local dx, dz, horizontalDist, diveStartDistance
        dx = targetCoordsVec.x - ownCoordsVec.x
        dz = targetCoordsVec.z - ownCoordsVec.z
        horizontalDist = math.sqrt(dx * dx + dz * dz)
        diveStartDistance = math.min(1500, math.max(SKIMMING_ALT / DIVE_START_TANJENT, 500))

        ---- 対水上モード
        if isAntiShipMode then
            targetCoordsVec.y = 0
            isHeadCapture = false
            -- 水平距離が急降下開始水平距離より離れている場合は自機から目標方向へ100m先を仮の目標にする
            if horizontalDist > diveStartDistance then
                isPPN = true
                local dirX = dx / horizontalDist
                local dirZ = dz / horizontalDist
                activeTargetCoordsVec = {
                    x = ownCoordsVec.x + dirX * 100,
                    y = math.max(ownCoordsVec.y / 1.5, SKIMMING_ALT),
                    z = ownCoordsVec.z + dirZ * 100
                }
            end
        elseif targetCoordsVec.y > 1000 and distance > 1500 then
            local vx, vy, vz, targetSpeed
            vx = activeTargetVelocitysVec.vx
            vy = activeTargetVelocitysVec.vy
            vz = activeTargetVelocitysVec.vz

            targetSpeed = math.sqrt(
                vx * vx + vy * vy + vz * vz
            )

            if targetSpeed > BALLISTIC_MIN_SPEED then
                local dirX, dirY, dirZ, relX, relY, relZ, relDistance, toMissileX, toMissileY, toMissileZ, facing, headOn, leadScale, leadDistance
                local alongTrack, crossX, crossY, crossZ, crossTrack, captureRequired, releaseAllowed
                dirX = vx / targetSpeed
                dirY = vy / targetSpeed
                dirZ = vz / targetSpeed

                -- 目標から迎撃ミサイルへの相対位置
                relX = ownCoordsVec.x - targetCoordsVec.x
                relY = ownCoordsVec.y - targetCoordsVec.y
                relZ = ownCoordsVec.z - targetCoordsVec.z

                relDistance = math.sqrt(
                    relX * relX
                    + relY * relY
                    + relZ * relZ
                )

                toMissileX = relX / math.max(relDistance, 1)
                toMissileY = relY / math.max(relDistance, 1)
                toMissileZ = relZ / math.max(relDistance, 1)

                -- 目標が迎撃ミサイルへどれだけ正対しているか
                facing =
                    dirX * toMissileX
                    + dirY * toMissileY
                    + dirZ * toMissileZ

                facing = math.max(-1, math.min(1, facing))

                -- 正対時のみリードを小さくする
                headOn = math.max(facing, 0)
                headOn = headOn * headOn

                leadScale =
                    HEAD_LEAD_MAX_SCALE
                    - (HEAD_LEAD_MAX_SCALE - HEAD_LEAD_MIN_SCALE)
                    * headOn

                leadDistance = distance * leadScale

                -- 目標進行方向に対する前後位置
                alongTrack =
                    relX * dirX
                    + relY * dirY
                    + relZ * dirZ

                -- 目標進行軸からの距離
                crossX = relX - dirX * alongTrack
                crossY = relY - dirY * alongTrack
                crossZ = relZ - dirZ * alongTrack

                crossTrack = math.sqrt(
                    crossX * crossX
                    + crossY * crossY
                    + crossZ * crossZ
                )

                captureRequired =
                    alongTrack < 0
                    or crossTrack > HEAD_CORRIDOR_RADIUS * 1.5

                releaseAllowed =
                    alongTrack > HEAD_RELEASE_DISTANCE
                    and crossTrack < HEAD_CORRIDOR_RADIUS

                if isHeadCapture then
                    if releaseAllowed then
                        isHeadCapture = false
                    end
                elseif captureRequired then
                    isHeadCapture = true
                end

                if isHeadCapture then
                    isPPN = true
                    activeTargetCoordsVec = {
                        x = targetCoordsVec.x
                            + dirX * leadDistance,

                        y = targetCoordsVec.y
                            + dirY * leadDistance
                            + HEAD_UP_OFFSET,

                        z = targetCoordsVec.z
                            + dirZ * leadDistance
                    }
                else
                    isPPN = false

                    activeTargetCoordsVec = {
                        x = targetCoordsVec.x,
                        y = targetCoordsVec.y,
                        z = targetCoordsVec.z
                    }
                end
            else
                -- 高高度だが低速なら通常の対空誘導へ戻す
                isHeadCapture = false
                isPPN = false
            end
        else -- 対空モード
            -- 海ポチャ対策として水平距離が500mより離れている場合は「自機から目標方向へ500m先、高度下限30m」を仮の目標にする
            isHeadCapture = false
            --[[             if horizontalDist > 1500 then
                local dirX = dx / horizontalDist
                local dirZ = dz / horizontalDist
                activeTargetCoordsVec = {
                    x = ownCoordsVec.x + dirX * 1000,
                    y = math.max(targetCoords[2], 30),
                    z = ownCoordsVec.z + dirZ * 1000
                }
            end ]]
        end
    end

    -- 3. グローバル座標系でのLOS (Line of Sight) ベクトルを計算
    LOS_vec_global = subtract(activeTargetCoordsVec, ownCoordsVec)
    if not isPPN then
        local currentLOS_vec_global_normalized, omega_LOS_global_vec, omega_LOS_global, omega_LOS_local, los_rate_pitch, los_rate_yaw
        -- 4. グローバルLOSベクトルを正規化
        currentLOS_vec_global_normalized = normalize(LOS_vec_global)

        -- 5. グローバル座標系でのLOS角速度ベクトル (omega) を計算
        --    omega_vec = (v_old x v_current)
        --    (v_old x v_current) の大きさは sin(theta)
        --    thetaが小さい場合, sin(theta) ~= theta (ラジアン)
        --    角速度 (rad/s) = theta / DT
        omega_LOS_global_vec             = cross_product(oldLOS_vec_global_normalized,
            currentLOS_vec_global_normalized)

        -- DTで割る (角速度ベクトルにする)
        omega_LOS_global                 = {
            omega_LOS_global_vec[1] / DT,
            omega_LOS_global_vec[2] / DT,
            omega_LOS_global_vec[3] / DT
        }

        -- 6. グローバルなLOS角速度ベクトルを、自機のローカル座標系に変換
        --    rotateVectorByInverseQuaternion は q_conj * p * q を行い、
        --    グローバルベクトル p をローカル座標系に変換します。
        --    p = omega_LOS_global (グローバルでのLOS回転)
        --    q = ownOrientation (自機の姿勢)
        --    結果 = omega_LOS_local (自機から見たLOS回転)
        omega_LOS_local                  = rotateVectorByInverseQuaternion(omega_LOS_global, ownOrientation)

        -- 7. ローカルなLOS角速度の各成分を取得
        --    自機の座標系 (X:右, Y:上, Z:前) と仮定
        --    coordsToAngle の定義 atan(x, z) [方位], atan(y, horiz) [仰角] より:
        --    - 方位角 (Yaw) の変化は、Y軸 (Up) 周りの回転
        --    - 仰角 (Pitch) の変化は、X軸 (Right) 周りの回転
        los_rate_pitch                   = omega_LOS_local[1] -- X軸 (Right) 周りの角速度 (rad/s)
        los_rate_yaw                     = omega_LOS_local[2] -- Y軸 (Up)    周りの角速度 (rad/s)
        -- local los_rate_roll  = omega_LOS_local[3] -- Z軸 (Fwd)   周りの角速度 (通常不要)

        -- 8. 状態を更新 (次のフレームのために)
        oldLOS_vec_global_normalized     = currentLOS_vec_global_normalized

        -- 9. 誘導指令として使う

        --    ご提示のコードの変数名に合わせる
        yawAngle                         = los_rate_yaw * PN_FIN_STRENGTH    -- (rad/tick)
        pitchAngle                       = -los_rate_pitch * PN_FIN_STRENGTH -- (rad/tick)
    else
        local targetLocalPosVec =
            globalToLocal(
                activeTargetCoordsVec,
                ownCoordsVec,
                ownOrientation
            )

        local targetAngle = coordsToAngle(targetLocalPosVec)

        yawAngle =
            targetAngle.azimuth
            * PPN_FIN_STRENGTH

        pitchAngle =
            targetAngle.elevation
            * PPN_FIN_STRENGTH

        oldLOS_vec_global_normalized =
            normalize(LOS_vec_global)
    end

    if isInit and isLaunched then
        oldDistance = distance
        isInit = false
    end
    local approach_Velocity
    if not isInit then
        approach_Velocity = distance - oldDistance
    else
        approach_Velocity = 0
    end
    oldDistance = distance

    -- 重力補償だけは常に効かせたいので重力補正の前で誘導を無効化
    if missileRadarIO and isMissileRadarDetecting then
        yawAngle = 0
        pitchAngle = 0
    end

    -- 誘導指令へ重力補償だけを加算する。
    -- 発射前は母機の移動を拾わないよう無効化する。
    if isLaunch then
        local gravity = getLocalGravity(ownOrientation)
        local gravityPitchCorrection,
        gravityYawCorrection =
            getGravityCorrection(
                selfLocalVelocity,
                gravity
            )

        pitchAngle = pitchAngle + gravityPitchCorrection
        yawAngle = yawAngle + gravityYawCorrection
    end
    -- ミサイルレーダーの有効圏内かつ対水上モードでない場合ミサイルレーダーを有効化・中間誘導用翼の出力をゼロに
    if distance + approach_Velocity * LOGIC_DELAY < MISSILE_FIN_DISTANCE_THRESHOLD and not isAntiShipMode and isLaunched then
        missileRadarIO = true
    end
    if distance < 500 and isLaunched then
        fuseIO = true
    end
    if approach_Velocity > 0 and isLaunched then
        selfDetonateCount = selfDetonateCount + 1
    else
        selfDetonateCount = 0
    end

    local selfDetonation = false
    if selfDetonateCount > 60 and not isAntiShipMode then
        selfDetonation = true
    end

--[[     updateGuidanceDebug(
        isLaunch,
        not isPPN,
        missileRadarIO,
        isMissileRadarDetecting,
        distance,
        approach_Velocity,
        selfDetonation
    ) ]]

    output.setBool(1, missileRadarIO)
    output.setBool(2, isAntiShipMode)
    output.setBool(3, mainRadarIO)
    output.setBool(4, selfDetonation)
    output.setBool(5, fuseIO)
    output.setNumber(1, yawAngle)
    output.setNumber(2, pitchAngle)
    output.setNumber(3, approach_Velocity)
end
