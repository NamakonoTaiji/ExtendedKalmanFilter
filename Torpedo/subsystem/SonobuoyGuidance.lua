--[[
ミサイルの誘導部分を担うスクリプト。アクティブレーダー圏内までは防空システムと通信して中間誘導/単追尾を行う。終末誘導時はアクティブ誘導/比例航法
至近距離ではミサイル出力を使用して近接信管で目標を破壊する。

-- 入力:
- bool 1: 目標を検出中
- bool 2: 対水上モードか否か
- bool 3: 発射済みか否か
- num 1: 推定目標座標 X
- num 2: 推定目標座標 Y
- num 3: 推定目標座標 Z
- num 4: 推定目標速度 Vx
- num 5: 推定目標速度 Vy
- num 6: 推定目標速度 Vz
- num 7: 推定目標加速度 Ax
- num 8: 推定目標加速度 Ay
- num 9: 推定目標加速度 Az
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
]]


DT                          = 1 / 60
PI                          = math.pi
PI2                         = PI * 2

oldDistance                 = 0
closingSpeedTable           = {}
launchedCount                = 0
currentTick                 = 0
isTargetFound               = false
chosenViewTargetID          = 0
targetInfos = {}

FIN_STRENGTH                = property.getNumber("FIN_STRENGTH")
SKIMMING_ALT                = property.getNumber("SKIMMING_ALT")
ORBIT_RADIUS                = property.getNumber("ORBIT_RADIUS")
ORBIT_LOOKAHEAD             = property.getNumber("ORBIT_LOOKAHEAD")
ORBIT_RADIAL_GAIN           = property.getNumber("ORBIT_RADIAL_GAIN")
ORBIT_DIRECTION             = property.getNumber("ORBIT_DIRECTION")
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")
INITIAL_PINGER_DISTANCE  = property.getNumber("CLOSEST_DISTANCE_THRESHOLD")
-- プロパティ未設定時の既定値
if ORBIT_RADIUS <= 0 then ORBIT_RADIUS = 500 end
if ORBIT_LOOKAHEAD <= 0 then ORBIT_LOOKAHEAD = 100 end
if ORBIT_RADIAL_GAIN <= 0 then ORBIT_RADIAL_GAIN = 1.5 end
if ORBIT_DIRECTION == 0 then ORBIT_DIRECTION = 1 end -- +1:時計回り、-1:反時計回り
ORBIT_DIRECTION = ORBIT_DIRECTION > 0 and 1 or -1
---@class Vector3
---@field x number
---@field y number
---@field z number

---@class Quaternion
---@field qw number
---@field qx number
---@field qy number
---@field qz number

oldLOS          = { azimuth = 0, elevation = 0 }
currentLOS      = { azimuth = 0, elevation = 0 }
LOStable        = { old = oldLOS, current = currentLOS }
targetCoords    = { 0, 0, 0 }
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

-- グローバル座標系での前フレームの正規化されたLOSベクトルを保存
-- {x, y, z} 形式で保存
oldLOS_vec_global_normalized = { x = 0, y = 0, z = 1 } -- 初期値 (例: 前方)

function onTick()
    currentTick = currentTick + 1
    -- 目標座標
    -- X=0を通過する目標も扱えるよう、3軸のどれかが入力されていれば更新する
    local tx = input.getNumber(24)
    local ty = input.getNumber(25)
    local tz = input.getNumber(26)
    local isDetected = input.getBool(1)
    local trackingID = input.getNumber(12)

    if tx ~= 0 or ty ~= 0 or tz ~= 0 then
        targetCoords = { tx, ty, tz }
        debug.log("Target coordinates initialized: " .. tx .. ", " .. ty .. ", " .. tz)
    end

    local ownCoords = {
        input.getNumber(27),
        input.getNumber(28),
        input.getNumber(29)
    }
    local ownOrientation =
        eulerZYX_to_quaternion(
            input.getNumber(32),
            input.getNumber(31),
            input.getNumber(30)
        )

    local targetCoordsVec = {
        x = targetCoords[1],
        y = targetCoords[2],
        z = targetCoords[3]
    }

    local ownCoordsVec = {
        x = ownCoords[1],
        y = ownCoords[2],
        z = ownCoords[3]
    }

    if isDetected then
        local receivedTarget = {
            x = input.getNumber(1),
            y = input.getNumber(2),
            z = input.getNumber(3),

            vX = input.getNumber(4),
            vY = input.getNumber(5),
            vZ = input.getNumber(6),

            epsilon = input.getNumber(8),
            lastSeenTick = input.getNumber(10),
            detectionTickLag = input.getNumber(11),
            id = trackingID
        }

        local found = false

        for i, target in ipairs(targetInfos) do
            if target.id == trackingID then
                targetInfos[i] = receivedTarget
                found = true
                break
            end
        end

        if not found then
            table.insert(targetInfos, receivedTarget)
        end
    end

    --------------------------------------------------------------------------
    -- 消失ターゲット削除
    --------------------------------------------------------------------------
    for i = #targetInfos, 1, -1 do
        local target = targetInfos[i]

        if currentTick - target.lastSeenTick > TARGET_LOST_THRESHOLD_TICKS then
            table.remove(targetInfos, i)
            debug.log("Target ID " .. target.id .. " removed due to loss of detection.")
        end
    end

    --------------------------------------------------------------------------
    -- 目標座標に最も近いターゲット選択
    --------------------------------------------------------------------------
    isTargetFound = false
    for i, target in ipairs(targetInfos) do
        if target.id == chosenViewTargetID then
            isTargetFound = true -- データリンクのIDが生きている場合はフラグを立ててループを抜ける
            targetCoords = {target.x, target.y, target.z}
            debug.log("Target found ID: " .. chosenViewTargetID.." Coordinates: "..target.x..", "..target.y..", "..target.z)
            break
        end
    end

    -- まだ補足している目標がいない場合、最も近いターゲットを選択する
    local closestDistanceSq = math.huge
    local closestDistanceThresholdSq = INITIAL_PINGER_DISTANCE ^ 2
    if not isTargetFound and chosenViewTargetID == 0 then
        for _, target in ipairs(targetInfos) do
            local distanceDiffSq = (target.x - targetCoordsVec.x) ^ 2 + (target.y - targetCoordsVec.y) ^ 2 +
            (target.z - targetCoordsVec.z) ^ 2
            if distanceDiffSq < closestDistanceSq then
                closestDistanceSq = distanceDiffSq
                if distanceDiffSq < closestDistanceThresholdSq and target.epsilon < 3 then
                    chosenViewTargetID = target.id
                    isTargetFound = true
                    debug.log("Target found, Distance = : " .. math.sqrt(closestDistanceSq) .. ", ID = " .. chosenViewTargetID.." Coordinates: "..target.x..", "..target.y..", "..target.z)
                end
            end
        end
    end

    ------------------------------------------------------------------------
    -- 目標座標を中心とする水平ベクトル場
    --
    -- 接線方向を基本とし、半径誤差に比例する半径方向成分を加える。
    -- 重要: この方向をLOS角速度へ変換せず、機体座標での方位誤差として
    --       直接フィンへ入力する。
    ------------------------------------------------------------------------
    local rx = ownCoordsVec.x - targetCoordsVec.x
    local rz = ownCoordsVec.z - targetCoordsVec.z
    local radiusNow = math.sqrt(rx * rx + rz * rz)

    if radiusNow < 1 then
        -- 中心にほぼ重なった場合だけ、現在の機首方向から半径方向を決める
        local forwardGlobal =
            rotateVectorByQuaternion({ 0, 0, 1 }, ownOrientation)
        rx = forwardGlobal[1]
        rz = forwardGlobal[3]
        radiusNow = math.sqrt(rx * rx + rz * rz)
        if radiusNow < 0.001 then
            rx, rz, radiusNow = 0, 1, 1
        end
    end

    local radialX = rx / radiusNow
    local radialZ = rz / radiusNow

    -- +1:時計回り、-1:反時計回り
    local tangentX = ORBIT_DIRECTION * radialZ
    local tangentZ = -ORBIT_DIRECTION * radialX

    -- 外側では内向き、内側では外向き。
    -- 誤差を半径で正規化するため、中心位置は常に入力目標座標になる。
    local radiusError = (radiusNow - ORBIT_RADIUS) / ORBIT_RADIUS
    local radialCorrection =
        math.max(-2, math.min(2, radiusError * ORBIT_RADIAL_GAIN))

    local desiredX = tangentX - radialX * radialCorrection
    local desiredZ = tangentZ - radialZ * radialCorrection
    local desiredLength = math.sqrt(desiredX * desiredX + desiredZ * desiredZ)

    if desiredLength < 0.001 then
        desiredX, desiredZ = tangentX, tangentZ
    else
        desiredX = desiredX / desiredLength
        desiredZ = desiredZ / desiredLength
    end

    ------------------------------------------------------------------------
    -- 水平誘導と深度誘導を分離
    --
    -- 水平: ベクトル場の進行方向へORBIT_LOOKAHEAD先
    -- 垂直: SKIMMING_ALTを絶対Y座標として追従
    --
    -- 深度誤差が大きくても水平半径計算には一切混ぜない。
    ------------------------------------------------------------------------
    local activeTargetCoordsVec = {
        x = ownCoordsVec.x + desiredX * ORBIT_LOOKAHEAD,
        y = SKIMMING_ALT,
        z = ownCoordsVec.z + desiredZ * ORBIT_LOOKAHEAD
    }

    local targetLocalPosVec =
        globalToLocal(activeTargetCoordsVec, ownCoordsVec, ownOrientation)
    local targetAngle = coordsToAngle(targetLocalPosVec)

    -- 方位・仰角の「角度誤差」を直接制御する。
    -- LOS角速度のみの比例航法は、周回軌道保持には適さない。
    local yawAngle = targetAngle.azimuth
    local pitchAngle = targetAngle.elevation

    output.setNumber(1, yawAngle * FIN_STRENGTH)
    output.setNumber(2, pitchAngle * FIN_STRENGTH)
    output.setNumber(3, targetCoords[1])
    output.setNumber(4, targetCoords[2])
    output.setNumber(5, targetCoords[3])
end
