---KalmanFilterから時分割で受け取った目標情報から脅威を選別しVLSの発射、データリンクを行うスクリプト
---detectionTickLagが1の時かつhitsが一定値以上超えている場合は撃墜するべき脅威として判定、VLSに目標情報、ID、発射信号が送信される。
---ミサイルはこの情報を受取ったのちはカルマンフィルターと直接交信を行い目標IDの座標へ中間誘導する。
--[[
入力:
- bool 1: 防空システムが目標を検出中か
- num 1-3: 全周防空システムからの目標情報 (X座標E-W,Y座標Altitude,Z座標N-S)
- num 4-6: 全周防空システムからの目標情報(X軸速度,Y軸速度,Z軸速度,m/s)
- num 7: 全周防空システムから目標に割り当てられたID
- num 8: 目標情報の推定精度(イプシロン)
- num 10: 目標が最後にレーダーで観測された時点のゲーム内tick
- num 11: 目標が最後にレーダーで観測されてから経過したTick数
- num 12: 目標を何回連続で同定できたか
- num 27: 自機X座標
- num 28: 自機Y座標
- num 29: 自機Z座標
- num 30: 自機オイラー角 Pitch (ラジアン)
- num 31: 自機オイラー角 Yaw (ラジアン)
- num 32: 自機オイラー角 Roll (ラジアン)
出力
- num 1-4: 迎撃目標1 X,Y,Z,目標ID
- num 8-9: 迎撃目標2
- num  10-12: 迎撃目標3
]]
launchChannel = 0

PI = math.pi
PI2 = math.pi * 2

INTERCEPT_THREAT_SPEED = property.getNumber("INTERCEPT_THREAT_SPEED") or 150 -- 迎撃する目標の近接速度
-- 「こちらを向いている」と判定する角度の閾値 (コサイン値)
-- 0.0 = 90度以内, 0.5 = 60度以内, 0.866 = 30度以内
THREAT_FACING_THRESHOLD = property.getNumber("THREAT_FACING_THRESHOLD") or 0.5
THREAT_HITS_THRESHOLD = property.getNumber("THREAT_HITS_THRESHOLD") or 5 -- 迎撃に必要な最低ヒット数 (Hits)
TARGET_LOST_TICK = property.getNumber("TARGET_LOST_TICK")
BASE_CHANNEL = 4
-- ミサイル管理用データ構造
currentTick = 0
threatTargets = {}
---@class Vector3
---@field x number
---@field y number
---@field z number

---@class Quaternion
---@field qw number
---@field qx number
---@field qy number
---@field qz number

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
function localCoordsToLocalAngle(localPosVec)
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

-- 角度差を計算 (-PI から PI の範囲)
function calculateAngleDifference(angle1, angle2)
    local diff = angle2 - angle1
    while diff <= -PI do diff = diff + PI2 end
    while diff > PI do diff = diff - PI2 end
    return diff
end

function onTick()
    currentTick = currentTick + 1
    -- -----------------------------------------------------------------
    -- 対空モードの処理
    -- -----------------------------------------------------------------

    local ownWorldCoords = { x = input.getNumber(27), y = input.getNumber(28), z = input.getNumber(29) }          -- 自機座標 {x -> 東, y -> 高度, z -> 北}
    local ownEulerAngles = { pitch = input.getNumber(30), yaw = input.getNumber(31), roll = input.getNumber(32) } -- 自機姿勢

    local isDetected = input.getNumber(9) == 1
    local trackingID = input.getNumber(7)

    for i = 1, 12 do
        output.setNumber(i, 0)
    end

    local ownOrientationQuat = eulerZYX_to_quaternion(ownEulerAngles.roll, ownEulerAngles.yaw, ownEulerAngles.pitch)
    
        detectedTarget                  = {
            x  = input.getNumber(1),
            y  = input.getNumber(2),
            z  = input.getNumber(3),
            vX = input.getNumber(4),
            vY = input.getNumber(5),
            vZ = input.getNumber(6),
        }

        local targetLocalCoords         = globalToLocal(
            { x = detectedTarget.x, y = detectedTarget.y, z = detectedTarget.z },
            ownWorldCoords, ownOrientationQuat)
        targetLocalCoords               = {
            x = targetLocalCoords.x,
            y = targetLocalCoords.y,
            z = targetLocalCoords.z,
        }
        local distance                  = math.sqrt(targetLocalCoords.x ^ 2 + targetLocalCoords.y ^ 2 +
            targetLocalCoords.z ^ 2)

        detectedTarget.distance         = distance
        detectedTarget.epsilon          = input.getNumber(8)
        detectedTarget.lastSeenTick     = input.getNumber(10)
        detectedTarget.detectionTickLag = input.getNumber(11)
        detectedTarget.hits             = input.getNumber(12)
        detectedTarget.id               = trackingID


        -- 1. 目標自身の絶対速度を計算 (m/s)
        local targetSpeed    = math.sqrt(detectedTarget.vX ^ 2 + detectedTarget.vY ^ 2 + detectedTarget.vZ ^ 2)
        detectedTarget.speed = targetSpeed -- (任意) 情報をテーブルに保存

        -- 2. 「こちらを向いているか」を判定
        local isFacingUs     = false
        if targetSpeed > 0.01 then -- 速度がゼロに近い場合は計算不可
            -- 目標から自機へ向かうベクトル (レーダーオフセット前の自機座標を使用)
            local vecToOwn = {
                x = ownWorldCoords.x - detectedTarget.x,
                y = ownWorldCoords.y - detectedTarget.y,
                z = ownWorldCoords.z - detectedTarget.z
            }
            -- 目標から自機までの距離
            local distToOwn = math.sqrt(vecToOwn.x ^ 2 + vecToOwn.y ^ 2 + vecToOwn.z ^ 2)

            if distToOwn > 0.1 then
                -- 目標の速度ベクトル (V_target) と 目標から自機へのベクトル (V_to_own) の内積を計算
                local dotProduct = detectedTarget.vX * vecToOwn.x +
                    detectedTarget.vY * vecToOwn.y +
                    detectedTarget.vZ * vecToOwn.z

                -- 正規化された内積 (コサイン類似度) を計算
                -- この値が 1 に近いほど、目標は正確に自機を向いている
                local normalizedDot = dotProduct / (targetSpeed * distToOwn)

                -- コサイン値が指定した閾値 (例: 0.5 = 60度以内) より大きければ「こちらを向いている」と判定
                if normalizedDot > THREAT_FACING_THRESHOLD then
                    isFacingUs = true
                end
            end
        end
        -- 3. 最終的な脅威判定
        local isThreat = isFacingUs and (targetSpeed > INTERCEPT_THREAT_SPEED)
        detectedTarget.isThreat = isThreat -- (任意) 情報をテーブルに保存
        local isDuplicate = false
        -- ipairsではなく、#threatTargets から 1 まで -1 ずつ減らす逆順ループを使う
        for i = #threatTargets, 1, -1 do
            local target = threatTargets[i]

            if target.id == detectedTarget.id then
                isDuplicate = true                              -- 被りを発見！

                if detectedTarget.isThreat then -- 目標がまだ脅威である場合は更新
                    threatTargets[i] = detectedTarget
                end
                -- ★重要: ローカル変数 target の中身も新しいアドレスに持ち替えておく
                -- （これを行わないと、下の判定で「古いターゲットのlastSeenTick」を参照してしまうため）
                target = detectedTarget
            else
                target.detectionTickLag = target.detectionTickLag + 1
            end

            -- TARGET_LOST_TICK を超えていたら、そのインデックスを安全に削除
            if target.detectionTickLag > TARGET_LOST_TICK then
                table.remove(threatTargets, i)
            end
        end

        -- 被りがなかった場合のみ追加
        if not isDuplicate and isThreat then
            table.insert(threatTargets, detectedTarget)
        end
        -- 距離順ソート
        table.sort(threatTargets, function(a, b)
            return a.distance < b.distance
        end)
        
        -- 迎撃目標出力
        for i = 1, math.min(#threatTargets, 3) do
            if threatTargets[i].detectionTickLag == 1 and threatTargets[i].hits > THREAT_HITS_THRESHOLD then
                output.setNumber(i * BASE_CHANNEL - 3, threatTargets[i].x)
                output.setNumber(i * BASE_CHANNEL - 2, threatTargets[i].y)
                output.setNumber(i * BASE_CHANNEL - 1, threatTargets[i].z)
                output.setNumber(i * BASE_CHANNEL, threatTargets[i].id)
            end
        end
    end

