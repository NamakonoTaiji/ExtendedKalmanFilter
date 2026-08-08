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
oldOwnY                     = nil
verticalSpeedFiltered       = 0

oldDistance                 = 0
closingSpeedTable           = {}
launchedCount                = 0
currentTick                 = 0
PN_FIN_STRENGTH             = property.getNumber("PN_FIN_STRENGTH")
PPN_FIN_STRENGTH            = property.getNumber("PPN_FIN_STRENGTH")
FUSE_LOGIC_DELAY            = property.getNumber("FUSE_LOGIC_DELAY")
SKIMMING_ALT                = property.getNumber("SKIMMING_ALT")
INITIAL_PINGER_DISTANCE  = property.getNumber("CLOSEST_DISTANCE_THRESHOLD")
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")
DISTANCE_LOOKAHEAD          = property.getNumber("DISTANCE_LOOKAHEAD")

TERMINAL_LEAD_TIME          = property.getNumber("TERMINAL_LEAD_TIME")    -- 旧PN版互換用（本誘導では未使用）
TERMINAL_HEADING_GAIN       = property.getNumber("TERMINAL_HEADING_GAIN") -- 旧PN版互換用（本誘導では未使用）
MAX_FIN_COMMAND             = property.getNumber("MAX_FIN_COMMAND")

-- 進路前方の仮想照準点へ単追尾するための設定
LEAD_MIN_SCALE              = property.getNumber("LEAD_MIN_SCALE")        -- 正対目標に対する最小先読み倍率
LEAD_MAX_SCALE              = property.getNumber("LEAD_MAX_SCALE")        -- 横切り・逃走目標に対する最大先読み倍率
LEAD_FADE_DISTANCE          = property.getNumber("LEAD_FADE_DISTANCE")    -- 至近距離で先読みを消し始める距離
LEAD_MIN_SPEED              = property.getNumber("LEAD_MIN_SPEED")        -- これ未満の目標速度を「ほぼ停止」とみなす閾値
ATTACK_START_DISTANCE       = property.getNumber("ATTACK_START_DISTANCE") -- 巡航深度から浮上を開始する距離
ATTACK_FULL_DISTANCE        = property.getNumber("ATTACK_FULL_DISTANCE")  -- 攻撃深度への移行が完了する距離
ATTACK_ALT                  = property.getNumber("ATTACK_ALT")            -- 最終的に狙う深度
DEPTH_LOOKAHEAD_TIME        = 2.0                                         -- 早めに浮上/潜航を止める。振動しにくくなる
DEPTH_GAIN                  = 0.05                                        -- 設定深度へ戻す力が強くなる
MAX_PITCH_COMMAND           = 0.2                                         --最大浮上/潜航能力が上がる

oldLOS                      = { azimuth = 0, elevation = 0 }
currentLOS                  = { azimuth = 0, elevation = 0 }
LOStable                    = { old = oldLOS, current = currentLOS }
targetCoords                = { 0, 0, 0 }
initialTargetCoords         = { 0, 0, 0 }
targetInfos                 = {}
chosenViewTargetID          = 0
velocityBuffer              = { x = 0, y = 0, z = 0 }
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
oldTerminalMode = false

function onTick()
    currentTick = currentTick + 1
    debug.log("-----Tick:------ " .. currentTick)
    local isLaunch = input.getNumber(23) == 1
    if isLaunch then
        launchedCount = launchedCount + 1
    end
    -- 目標座標
    -- X=0を通過する目標も扱えるよう、3軸のどれかが入力されていれば更新する
    local tx = input.getNumber(24)
    local ty = input.getNumber(25)
    local tz = input.getNumber(26)
    local isDetected = input.getBool(1)
    local trackingID = input.getNumber(12)
    if tx ~= 0 or ty ~= 0 or tz ~= 0 then
        initialTargetCoords = { tx, ty, tz }
        targetCoords = initialTargetCoords
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

    local rawVerticalSpeed = 0

    if oldOwnY then
        rawVerticalSpeed =
            (ownCoordsVec.y - oldOwnY) / DT
    end

    oldOwnY = ownCoordsVec.y

    -- 微分ノイズを少し落とす
    verticalSpeedFiltered =
        verticalSpeedFiltered * 0.8
        + rawVerticalSpeed * 0.2

    debug.log("SelfAlt: " .. ownCoordsVec.y)
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

        -- デバッグ
        local trueCoordsX = input.getNumber(13)
        local trueCoordsY = input.getNumber(14)
        local trueCoordsZ = input.getNumber(15)

        local totalDiff = math.sqrt((receivedTarget.x - trueCoordsX) ^ 2 +
            (receivedTarget.y - trueCoordsY) ^ 2 +
            (receivedTarget.z - trueCoordsZ) ^ 2)
        if totalDiff < 200 then
            --[[             debug.log("XDiff: " .. (receivedTarget.x - trueCoordsX) ..
                ", YDiff: " .. (receivedTarget.y - trueCoordsY) ..
                ", ZDiff: " .. (receivedTarget.z - trueCoordsZ) ..
                ", totalDiff: " .. totalDiff) ]]
        end

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
    else
        targetCoords = {
            targetCoords[1] + velocityBuffer.x * DT,
            targetCoords[2] + velocityBuffer.y * DT,
            targetCoords[3] + velocityBuffer.z * DT
        }
    end

    --------------------------------------------------------------------------
    -- 消失ターゲット削除
    --------------------------------------------------------------------------
    for i = #targetInfos, 1, -1 do
        local target = targetInfos[i]

        if currentTick - target.lastSeenTick > TARGET_LOST_THRESHOLD_TICKS then
            table.remove(targetInfos, i)
            -- debug.log("Target ID " .. target.id .. " removed due to loss of detection.")
            targetCoords = initialTargetCoords
        end
    end

    --------------------------------------------------------------------------
    -- 目標座標に最も近いターゲット選択
    --------------------------------------------------------------------------
    isTargetFound = false
    local selectedTargetVelocity = { x = 0, y = 0, z = 0 }
    for i, target in ipairs(targetInfos) do
        if target.id == chosenViewTargetID then
            isTargetFound = true -- データリンクのIDが生きている場合はフラグを立ててループを抜ける
            targetCoords = { target.x, target.y, target.z }
            selectedTargetVelocity = { x = target.vX, y = target.vY, z = target.vZ }
            velocityBuffer = {
                x = selectedTargetVelocity.x,
                y = selectedTargetVelocity.y,
                z = selectedTargetVelocity.z
            }
            debug.log("Target updated ID: " ..
                chosenViewTargetID .. " Coordinates: " .. target.x .. ", " .. target.y .. ", " .. target.z)
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
                if distanceDiffSq < closestDistanceThresholdSq then
                    chosenViewTargetID = target.id
                    isTargetFound = true
                    selectedTargetVelocity = { x = target.vX, y = target.vY, z = target.vZ }
                    targetCoords = { target.x, target.y, target.z }
                    velocityBuffer = selectedTargetVelocity
                    debug.log("Target found, Distance = : " ..
                        math.sqrt(closestDistanceSq) ..
                        ", ID = " .. chosenViewTargetID .. " Coordinates: " .. target.x .. ", " .. target.y .. ", " ..
                        target.z)
                end
            end
        end
    end

    -- {x, y, z} 形式のベクトルテーブルに変換
    local targetCoordsVec       = { x = targetCoords[1], y = targetCoords[2], z = targetCoords[3] }
    local ownCoordsVec          = { x = ownCoords[1], y = ownCoords[2], z = ownCoords[3] }

    --------------------------------------------------------------------------
    -- 進路前方オフセット単追尾
    --
    -- PN/LOS微分は使用しない。
    -- 水平(XZ)では目標速度方向の前方へ「現在距離×leadScale」の仮想点を置き、
    -- 魚雷はその点へ単追尾する。
    -- 正対してくる目標ほどleadScaleを小さく、横切り/逃走目標ほど大きくする。
    --
    -- 鉛直(Y)は目標速度から切り離し、通常はSKIMMING_ALTを維持。
    -- 至近距離だけATTACK_ALTへsmoothstepで滑らかに遷移する。
    --------------------------------------------------------------------------
    local dx                    = targetCoordsVec.x - ownCoordsVec.x
    local dz                    = targetCoordsVec.z - ownCoordsVec.z
    local horizontalDist        = math.sqrt(dx * dx + dz * dz)

    -- 現在観測が無いtickでは最後に保持した速度を使う
    local guideVX               = isTargetFound and selectedTargetVelocity.x or velocityBuffer.x
    local guideVZ               = isTargetFound and selectedTargetVelocity.z or velocityBuffer.z
    local targetHorizontalSpeed = math.sqrt(guideVX * guideVX + guideVZ * guideVZ)

    local aimX                  = targetCoordsVec.x
    local aimZ                  = targetCoordsVec.z
    local leadScale             = 0
    local leadDistance          = 0
    local facing                = 0

    if targetHorizontalSpeed > LEAD_MIN_SPEED then
        local dirX = guideVX / targetHorizontalSpeed
        local dirZ = guideVZ / targetHorizontalSpeed

        -- 目標から魚雷への水平単位ベクトル
        local relX = ownCoordsVec.x - targetCoordsVec.x
        local relZ = ownCoordsVec.z - targetCoordsVec.z
        local invHorizontalDist = 1 / math.max(horizontalDist, 1)
        local toTorpedoX = relX * invHorizontalDist
        local toTorpedoZ = relZ * invHorizontalDist

        -- +1: 目標が魚雷へ正対 / 0: 横切り / -1: 逃走
        facing = dirX * toTorpedoX + dirZ * toTorpedoZ
        facing = math.max(-1, math.min(1, facing))

        -- 正対時だけリード量を縮小する（ADSV3 Missileと同じ考え方）
        local headOn = math.max(facing, 0)
        headOn = headOn * headOn

        leadScale =
            LEAD_MAX_SCALE
            - (LEAD_MAX_SCALE - LEAD_MIN_SCALE) * headOn

        -- 至近距離ではオフセットを滑らかに0へ収束させる
        local leadFade = math.min(horizontalDist / LEAD_FADE_DISTANCE, 1)
        leadDistance = horizontalDist * leadScale * leadFade

        aimX = targetCoordsVec.x + dirX * leadDistance
        aimZ = targetCoordsVec.z + dirZ * leadDistance
    end

    -- 深度は水平リードから完全分離。
    -- ATTACK_START_DISTANCEからATTACK_FULL_DISTANCEにかけて
    -- SKIMMING_ALT -> ATTACK_ALTへ滑らかに遷移する。
    local depthBlend = 0
    local attackRange =
        math.max(ATTACK_START_DISTANCE - ATTACK_FULL_DISTANCE, 1)

    if horizontalDist < ATTACK_START_DISTANCE then
        depthBlend =
            (ATTACK_START_DISTANCE - horizontalDist) / attackRange
        depthBlend = math.max(0, math.min(1, depthBlend))
        depthBlend =
            depthBlend * depthBlend * (3 - 2 * depthBlend) -- smoothstep
    end

    local aimY =
        SKIMMING_ALT
        + (ATTACK_ALT - SKIMMING_ALT) * depthBlend

    local activeTargetCoordsVec = {
        x = aimX,
        y = aimY,
        z = aimZ
    }

    if isLaunch then
        -- 仮想照準点へ常時単追尾
        local targetLocalPosVec =
            globalToLocal(
                activeTargetCoordsVec,
                ownCoordsVec,
                ownOrientation
            )

        local targetAngle =
            coordsToAngle(targetLocalPosVec)

        -- 水平：今まで通り仮想リード点へ単追尾
        yawAngle =
            targetAngle.azimuth * PPN_FIN_STRENGTH

        -- 鉛直：仮想点の距離とは完全に分離
        local predictedY =
            ownCoordsVec.y
            + verticalSpeedFiltered * DEPTH_LOOKAHEAD_TIME

        local depthError =
            aimY - predictedY

        pitchAngle =
            depthError * DEPTH_GAIN

        pitchAngle =
            math.max(
                -MAX_PITCH_COMMAND,
                math.min(MAX_PITCH_COMMAND, pitchAngle)
            )

        -- 設定されている場合のみ合成フィン指令を制限
        if MAX_FIN_COMMAND > 0 then
            local commandMagnitude =
                math.sqrt(yawAngle * yawAngle + pitchAngle * pitchAngle)

            if commandMagnitude > MAX_FIN_COMMAND and commandMagnitude > 0 then
                local commandScale =
                    MAX_FIN_COMMAND / commandMagnitude

                yawAngle = yawAngle * commandScale
                pitchAngle = pitchAngle * commandScale
            end
        end

        debug.log(
            "Pursuit LeadScale: " .. leadScale
            .. ", LeadDistance: " .. leadDistance
            .. ", Facing: " .. facing
            .. ", TargetHSpeed: " .. targetHorizontalSpeed
        )
        debug.log(
            "AimXYZ: " .. activeTargetCoordsVec.x
            .. ", " .. activeTargetCoordsVec.y
            .. ", " .. activeTargetCoordsVec.z
            .. ", HRange: " .. horizontalDist
            .. ", DepthBlend: " .. depthBlend
        )
        debug.log(
            "yawAngle: " .. yawAngle
            .. ", pitchAngle: " .. pitchAngle
        )
    else
        debug.log("NotLaunched")
        yawAngle = 0
        pitchAngle = 0
    end

    output.setNumber(1, yawAngle)
    output.setNumber(2, pitchAngle)
    output.setNumber(3, targetCoords[1])
    output.setNumber(4, targetCoords[2])
    output.setNumber(5, targetCoords[3])
end
