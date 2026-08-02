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


DT                             = 1 / 60
PI                             = math.pi
PI2                            = PI * 2

VT_FUSE_SAMPLING_TICKS         = 10
oldDistance                    = 0
closingSpeedTable              = {}
missileRadarIO                 = false
isLaunched                     = false
lauchedCount                   = 0
initialGuidanceCounter         = 0
isGuidanceStart                = false
isPPN                          = true

DIVE_START_TANJENT             = math.tan(math.rad(70))
PN_FIN_STRENGTH                = property.getNumber("PN_FIN_STRENGTH")
PPN_FIN_STRENGTH               = property.getNumber("PPN_FIN_STRENGTH")
FUSE_LOGIC_DELAY               = property.getNumber("FUSE_LOGIC_DELAY")
MISSILE_FIN_DISTANCE_THRESHOLD = property.getNumber("MISSILE_FIN_DISTANCE_THRESHOLD")
SKIMMING_ALT                   = property.getNumber("SKIMMING_ALT")
GUIDANCE_START_ALTITUDE        = property.getNumber("GUIDANCE_START_ALTITUDE")
LEAD_ANGLE_TIME                = property.getNumber("LEAD_ANGLE_TIME")

INTERCEPT_SPEED                = property.getNumber("INTERCEPT_SPEED")         -- ミサイル平均速度 m/s
INTERCEPT_TIME_GAIN            = property.getNumber("INTERCEPT_TIME_GAIN")     -- 1.0～1.2
BALLISTIC_DESCENT_SPEED        = property.getNumber("BALLISTIC_DESCENT_SPEED") -- 例: 30 m/s
INTERCEPT_MIN_ALT              = property.getNumber("INTERCEPT_MIN_ALT")       -- 例: 100 m
BALLISTIC_GRAVITY              = 20
BALLISTIC_APEX_OFFSET          = 300
BALLISTIC_FORWARD_TIME         = 1.5
BALLISTIC_MAX_PREDICT_TIME     = 15
BALLISTIC_GUIDANCE_DISTANCE    = 2000
BALLISTIC_MIN_SPEED            = 100

---@class Vector3
---@field x number
---@field y number
---@field z number

---@class Quaternion
---@field qw number
---@field qx number
---@field qy number
---@field qz number


-- グローバル座標系での前フレームの正規化されたLOSベクトルを保存
-- {x, y, z} 形式で保存
oldLOS_vec_global_normalized = { x = 0, y = 0, z = 1 } -- 初期値 (例: 前方)

oldLOS                       = { azimuth = 0, elevation = 0 }
currentLOS                   = { azimuth = 0, elevation = 0 }
LOStable                     = { old = oldLOS, current = currentLOS }
isInit                       = true -- 近接速度計算初期化
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

-- |r + vt| = missileSpeed * t を解く
function solveInterceptTime(rx, ry, rz, vx, vy, vz, missileSpeed)
    local a = vx * vx + vy * vy + vz * vz - missileSpeed * missileSpeed
    local b = 2 * (rx * vx + ry * vy + rz * vz)
    local c = rx * rx + ry * ry + rz * rz

    if math.abs(a) < 1e-6 then
        if b < -1e-6 then
            return -c / b
        end
        return nil
    end

    local discriminant = b * b - 4 * a * c

    if discriminant < 0 then
        return nil
    end

    local root = math.sqrt(discriminant)
    local t1 = (-b - root) / (2 * a)
    local t2 = (-b + root) / (2 * a)
    local t

    if t1 > 0 then
        t = t1
    end

    if t2 > 0 and (not t or t2 < t) then
        t = t2
    end

    return t
end

function onTick()
    -- 1. 座標・オイラー角の取得
    local targetCoords   = { input.getNumber(1), input.getNumber(2), input.getNumber(3) }
    local targetVelocity = { input.getNumber(4), input.getNumber(5), input.getNumber(6) }
    local ownCoords      = { input.getNumber(13), input.getNumber(14), input.getNumber(15) }
    local ownOrientation = { input.getNumber(16), input.getNumber(17), input.getNumber(18), input.getNumber(19) }
    local isDetecting    = input.getBool(1)
    local isAntiShipMode = input.getBool(2)
    local isLaunch       = input.getBool(3)
    local distance       = vector_magnitude(subtract(targetCoords, ownCoords))

    if isLaunch and not isLaunched then
        lauchedCount = lauchedCount + 1
        if lauchedCount > 20 then
            isLaunched = true
        end
    end

    -- {x, y, z} 形式のベクトルテーブルに変換
    local targetCoordsVec          = { x = targetCoords[1], y = targetCoords[2], z = targetCoords[3] }
    local ownCoordsVec             = { x = ownCoords[1], y = ownCoords[2], z = ownCoords[3] }
    local activeTargetCoordsVec    = targetCoordsVec
    local activeTargetVelocitysVec = { vx = targetVelocity[1], vy = targetVelocity[2], vz = targetVelocity[3] }
    local LOS_vec_global

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
        initialGuidanceCounter = initialGuidanceCounter + 1
        if initialGuidanceCounter > 60 then
            isPPN = false
        end
    end

    ----------------------------------------------------------------------------
    -- 対水上モード時の低空巡航 (シースキミング) 処理
    ----------------------------------------------------------------------------

    if isAntiShipMode then
        targetCoords[2] = 0 -- 水上目標は高度0mで固定
    end

    if isGuidanceStart then
        local dx = targetCoordsVec.x - ownCoordsVec.x
        local dz = targetCoordsVec.z - ownCoordsVec.z
        local horizontalDist = math.sqrt(dx * dx + dz * dz)
        local diveStartDistance = math.min(1500, math.max(SKIMMING_ALT / DIVE_START_TANJENT, 500))

        ---- 対水上モード
        if isAntiShipMode then
            -- 水平距離が急降下開始水平距離より離れている場合は自機から目標方向へ100m先を仮の目標にする
            if horizontalDist > diveStartDistance or (horizontalDist > 500 and ownCoordsVec.y < 40) then
                isPPN = true
                local dirX = dx / horizontalDist
                local dirZ = dz / horizontalDist
                activeTargetCoordsVec = {
                    x = ownCoordsVec.x + dirX * 100,
                    y = math.max(ownCoords[2] / 1.5, SKIMMING_ALT),
                    z = ownCoordsVec.z + dirZ * 100
                }
            end
        elseif targetCoordsVec.y > 2000 and distance > 2000 then -- 対空/高高度からくる弾道弾に対して頭を押さえる
            -- 1. 目標の未来位置を計算 (等速直線運動を仮定)
            local leadX = targetCoordsVec.x + activeTargetVelocitysVec.vx * LEAD_ANGLE_TIME
            local leadY = targetCoordsVec.y + activeTargetVelocitysVec.vy * LEAD_ANGLE_TIME
            local leadZ = targetCoordsVec.z + activeTargetVelocitysVec.vz * LEAD_ANGLE_TIME

            -- 2. 自機から目標までの水平距離を計算
            local dx = targetCoordsVec.x - ownCoordsVec.x
            local dz = targetCoordsVec.z - ownCoordsVec.z
            local horizontalDist = math.sqrt(dx * dx + dz * dz)

            -- 3. 水平距離に応じた高度オフセット（上乗せ分）を計算
            -- 遠いほど高く上がり、近づくにつれて本来の高度（0）に収束する
            -- ※ 0.5 という係数はミサイルの機動性や好みの軌道に合わせて調整してください。
            -- 大きくするほど高く上がり、鋭角に降下します。
            local overheadOffset = horizontalDist * 0.5

            activeTargetCoordsVec = {
                x = leadX,
                y = leadY + overheadOffset, -- 未来位置のY座標にオフセットを加算
                z = leadZ
            }
        else -- 対空モード
            -- 海ポチャ対策として水平距離が500mより離れている場合は「自機から目標方向へ100m先、高度下限200m」を仮の目標にする
            if horizontalDist > 1000 then
                local dirX = dx / horizontalDist
                local dirZ = dz / horizontalDist
                activeTargetCoordsVec = {
                    x = ownCoordsVec.x + dirX * 100,
                    y = math.max(targetCoordsVec.y, 40),
                    z = ownCoordsVec.z + dirZ * 100
                }
            end
        end
    end

    -- 3. グローバル座標系でのLOS (Line of Sight) ベクトルを計算
    LOS_vec_global = subtract(activeTargetCoordsVec, ownCoordsVec)
    if not isPPN then
        debug.log("PN")
        -- 4. グローバルLOSベクトルを正規化
        local currentLOS_vec_global_normalized = normalize(LOS_vec_global)

        -- 5. グローバル座標系でのLOS角速度ベクトル (omega) を計算
        --    omega_vec = (v_old x v_current)
        --    (v_old x v_current) の大きさは sin(theta)
        --    thetaが小さい場合, sin(theta) ~= theta (ラジアン)
        --    角速度 (rad/s) = theta / DT
        local omega_LOS_global_vec             = cross_product(oldLOS_vec_global_normalized,
            currentLOS_vec_global_normalized)

        -- DTで割る (角速度ベクトルにする)
        local omega_LOS_global                 = {
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
        local omega_LOS_local                  = rotateVectorByInverseQuaternion(omega_LOS_global, ownOrientation)

        -- 7. ローカルなLOS角速度の各成分を取得
        --    自機の座標系 (X:右, Y:上, Z:前) と仮定
        --    coordsToAngle の定義 atan(x, z) [方位], atan(y, horiz) [仰角] より:
        --    - 方位角 (Yaw) の変化は、Y軸 (Up) 周りの回転
        --    - 仰角 (Pitch) の変化は、X軸 (Right) 周りの回転
        local los_rate_pitch                   = omega_LOS_local[1] -- X軸 (Right) 周りの角速度 (rad/s)
        local los_rate_yaw                     = omega_LOS_local[2] -- Y軸 (Up)    周りの角速度 (rad/s)
        -- local los_rate_roll  = omega_LOS_local[3] -- Z軸 (Fwd)   周りの角速度 (通常不要)

        -- 8. 状態を更新 (次のフレームのために)
        oldLOS_vec_global_normalized           = currentLOS_vec_global_normalized

        -- 9. 誘導指令として使う

        --    ご提示のコードの変数名に合わせる
        yawAngle                               = los_rate_yaw * DT * PN_FIN_STRENGTH    -- (rad/tick)
        pitchAngle                             = -los_rate_pitch * DT * PN_FIN_STRENGTH -- (rad/tick)
    else
        debug.log("PPN")

        local targetLocalPosVec =
            globalToLocal(activeTargetCoordsVec, ownCoordsVec, ownOrientation)

        local targetAngle = coordsToAngle(targetLocalPosVec)

        yawAngle = targetAngle.azimuth * PPN_FIN_STRENGTH
        pitchAngle = targetAngle.elevation * PPN_FIN_STRENGTH

        -- PN切替時の瞬間的なLOS差を防止
        oldLOS_vec_global_normalized = normalize(LOS_vec_global)
    end

    if isInit and isLaunched then
        oldDistance = distance
        isInit = false
    end

    if not isInit then
        approach_Velocity = distance - oldDistance
    else
        approach_Velocity = 0
    end
    oldDistance = distance
    -- ミサイルレーダーの有効圏内かつ対水上モードでない場合ミサイルレーダーを有効化・中間誘導用翼の出力をゼロに
    if distance + approach_Velocity * FUSE_LOGIC_DELAY < MISSILE_FIN_DISTANCE_THRESHOLD and not isAntiShipMode and isLaunched then
        missileRadarIO = true
    end
    if missileRadarIO then
        yawAngle = 0
        pitchAngle = 0
    end

    --[[     if distance + closingSpeedAverage * FUSE_LOGIC_DELAY < 0 and distance ~= 0 and isLaunched then
        detonate = true
    else
        detonate = false
    end ]]

    output.setBool(1, missileRadarIO)
    output.setBool(2, isAntiShipMode)
    output.setNumber(1, yawAngle)
    output.setNumber(2, pitchAngle)
    output.setNumber(3, approach_Velocity)
end
