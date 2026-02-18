PI = math.pi
PI2 = math.pi * 2

HMD_FOV_HEIGHT = 58
MAX_TARGETS = 5
-- 座席の頭部位置から見てレーダーがどれだけ離れているか入力(offsetX右方向, offsetY上方向, offsetZ前方向)
offsetX, offsetY, offsetZ = property.getNumber("offset1X"), property.getNumber("offset1Y"),
    property.getNumber("offset1Z")

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
    local fovLevel
    local ownWorldCoords = { x = input.getNumber(12), y = input.getNumber(16), z = input.getNumber(20) }          -- 自機座標 {x -> 東, y -> 高度, z -> 北}
    local ownEulerAngles = { pitch = input.getNumber(24), yaw = input.getNumber(28), roll = input.getNumber(32) } -- 自機姿勢
    fuel = input.getNumber(23)                                                                                    -- 残燃料
    speed = input.getNumber(25)                                                                                   -- 速度
    engineTemperature = input.getNumber(26)                                                                       -- エンジン温度

    -- ウェイポイント座標
    waypoint_1_WorldXZ = { x = input.getNumber(27), z = input.getNumber(29) }
    waypoint_2_WorldXZ = { x = input.getNumber(30), z = input.getNumber(31) }

    -- 座席視点
    seatViewX = input.getNumber(4)
    seatViewY = input.getNumber(8)

    targetsLocalDistAngle = {}
    for i = 1, MAX_TARGETS do
        local isDetected = input.getBool(i)
        if isDetected then
            local ch_base = (i - 1) * 4
            local distance = input.getNumber(ch_base + 1)
            local localAzimuthRad = input.getNumber(ch_base + 2) * PI2
            local localElevationRad = input.getNumber(ch_base + 3) * PI2

            local targetLocalVec = localAngleDistToLocalCoords(distance, localAzimuthRad, localElevationRad) -- ローカル座標に変換
            targetLocalVec = {
                x = targetLocalVec.x + offsetX,
                y = targetLocalVec.y + offsetY,
                z = targetLocalVec.z + offsetZ
            }

            local targetLocalAngle = localCoordsToLocalAngle(targetLocalVec) -- ローカル極座標に戻す
            if distance > 0 then
                table.insert(targetsLocalDistAngle, {
                    dist = distance,
                    azimuth = targetLocalAngle.azimuth,
                    elevation = targetLocalAngle.elevation,
                    isWaypoint = false,
                    drawTextString = ""
                })
            end

            output.setNumber(ch_base + 1, distance)
            output.setNumber(ch_base + 2, targetLocalAngle.azimuth / PI2)
            output.setNumber(ch_base + 3, targetLocalAngle.elevation / PI2)
        end
    end

    ------ ウェイポイントをローカル角度に変換
    if waypoint_1_WorldXZ.x ~= 0 or waypoint_1_WorldXZ.z ~= 0 or waypoint_2_WorldXZ.x ~= 0 or waypoint_2_WorldXZ.z ~= 0 then
        local ownOrientationQuat = eulerZYX_to_quaternion(ownEulerAngles.roll, ownEulerAngles.yaw, ownEulerAngles.pitch)

        if waypoint_1_WorldXZ.x ~= 0 or waypoint_1_WorldXZ.z ~= 0 then
            local waypoint_1_LocalCoords = globalToLocal({ x = waypoint_1_WorldXZ.x, y = 0, z = waypoint_1_WorldXZ.z },
                ownWorldCoords, ownOrientationQuat)
            local distance = math.sqrt(waypoint_1_LocalCoords.x ^ 2 + waypoint_1_LocalCoords.y ^ 2 +
                waypoint_1_LocalCoords.z ^ 2)
            local waypoint_1_LocalAngle = localCoordsToLocalAngle(waypoint_1_LocalCoords)

            table.insert(targetsLocalDistAngle, {
                dist = distance,
                azimuth = waypoint_1_LocalAngle.azimuth,
                elevation = waypoint_1_LocalAngle.elevation,
                isWaypoint = true,
                drawTextString = "POINT1"
            })
        end

        if waypoint_2_WorldXZ.x ~= 0 or waypoint_2_WorldXZ.z ~= 0 then
            local waypoint_2_LocalCoords = globalToLocal({ x = waypoint_2_WorldXZ.x, y = 0, z = waypoint_2_WorldXZ.z },
                ownWorldCoords, ownOrientationQuat)
            local distance = math.sqrt(waypoint_2_LocalCoords.x ^ 2 + waypoint_2_LocalCoords.y ^ 2 +
                waypoint_2_LocalCoords.z ^ 2)
            local waypoint_2_LocalAngle = localCoordsToLocalAngle(waypoint_2_LocalCoords)

            table.insert(targetsLocalDistAngle, {
                dist = distance,
                azimuth = waypoint_2_LocalAngle.azimuth,
                elevation = waypoint_2_LocalAngle.elevation,
                isWaypoint = true,
                drawTextString = "POINT2"
            })
        end
    end
end

function onDraw()
    screen.setColor(0, 255, 0)
    local w = screen.getWidth()
    local h = screen.getHeight()
    local centerX = w / 2
    local centerY = h / 2

    -- 1. HMDの垂直FOV (ラジアン)
    local fov_rad_vertical = HMD_FOV_HEIGHT * (PI / 180) -- 58度をラジアンに

    -- 2. 垂直方向のスケール (tan(FOV/2) の逆数)
    -- (rh = 画面高さの半分 / tan(垂直FOVの半分) )
    local rh = centerY / math.tan(fov_rad_vertical / 2)

    -- 3. 水平FOV (ラジアン) をアスペクト比から計算
    local fov_rad_horizontal = 2 * math.atan(math.tan(fov_rad_vertical / 2) * (w / h))
    -- 4. 水平方向のスケール
    -- (rw = 画面幅の半分 / tan(水平FOVの半分) )
    local rw = centerX / math.tan(fov_rad_horizontal / 2)

    -- 5. 現在のHMD視点角度 (ラジアン)
    local viewAzimuthRad = seatViewX * PI2
    local viewElevationRad = seatViewY * PI2

    -- 自機情報インターフェースの描画
    screen.drawText(6, 6, string.format("%-10s %7.1f kt", "SPEED", speed * 1.943844))
    screen.drawText(6, 12, string.format("%-10s %7.1f L", "FUEL", fuel))
    screen.drawText(6, 18, string.format("%-10s %7.1f C", "ENG TEMP", engineTemperature))

    --screen.drawText(5, 5, HMD_FOV)

    -- 画面全体の描画スケールを計算します。
    -- HMDの視野角(度)を回転単位に変換し、それで画面幅を割ることで、
    -- 「1回転単位あたり何ピクセルか」を求めます。
    local hmd_fov_rot = HMD_FOV_HEIGHT / 360
    local scale = w / hmd_fov_rot

    for i, target in ipairs(targetsLocalDistAngle) do
        -- 6. ターゲットの角度 (ラジアン)
        local targetAzimuthRad = target.azimuth
        local targetElevationRad = target.elevation

        -- 7. 視点中心からの相対角度 (ラジアン) を計算
        -- (例: 視点が右(0.1), ターゲットが右(0.15) -> 相対角度は 0.05)
        local relative_azimuth_rad = calculateAngleDifference(viewAzimuthRad, targetAzimuthRad)
        local relative_elevation_rad = calculateAngleDifference(viewElevationRad, targetElevationRad)
        if math.abs(relative_azimuth_rad) < (PI / 2) and math.abs(relative_elevation_rad) < (PI / 2) then
            -- 8. 透視投影 (タンジェントを使用)
            local tan_azim = math.tan(relative_azimuth_rad)
            local tan_elev = math.tan(relative_elevation_rad)

            -- 9. 画面座標に変換
            local screenX = centerX + rw * tan_azim
            local screenY = centerY - rh * tan_elev -- Y軸反転 (画面上はYが下向きのため)

            -- 目標マーカーを描画
            if target.isWaypoint == false then
                screen.setColor(255, 0, 0)
            elseif target.isWaypoint == true then
                if target.drawTextString == "POINT1" then
                    screen.setColor(230, 0, 0)
                elseif target.drawTextString == "POINT2" then
                    screen.setColor(0, 0, 230)
                end
                screen.drawText(screenX, screenY + 5, target.drawTextString)
            end
            screen.drawCircle(screenX, screenY, 3)

            -- 距離情報を描画
            local dist_text = math.floor(target.dist) .. "m"
            screen.drawText(screenX + 8, screenY - 3, dist_text)
        end
    end
end
