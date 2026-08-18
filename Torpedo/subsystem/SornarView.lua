PI = math.pi
PI2 = PI * 2

offsetX = property.getNumber("offsetX")
offsetY = property.getNumber("offsetY")
offsetZ = property.getNumber("offsetZ")

HMD_FOV_HEIGHT = 58
RGB_ALPHA = property.getNumber("RGBAlpha")
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("TARGET_LOST_THRESHOLD_TICKS")

-- 角度差の二乗と比較するので、閾値側も二乗する
VIEW_SELECT_ANGLE = math.rad(30)
VIEW_SELECT_DISTANCE_SQ = VIEW_SELECT_ANGLE * VIEW_SELECT_ANGLE

-- Stormworks標準フォント
FONT_WIDTH = 4
FONT_HEIGHT = 5

currentTick = 0
targetInfos = {}
chosenViewTargetID = 0
seatViewX = 0
seatViewY = 0

chosedIdBuffer = 0
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

function clamp(value, minValue, maxValue)
    if value < minValue then return minValue end
    if value > maxValue then return maxValue end
    return value
end

function rectanglesOverlap(a, b)
    return
        a.x < b.x + b.w and
        a.x + a.w > b.x and
        a.y < b.y + b.h and
        a.y + a.h > b.y
end

function rectangleOverlapArea(a, b)
    local left = math.max(a.x, b.x)
    local right = math.min(a.x + a.w, b.x + b.w)
    local top = math.max(a.y, b.y)
    local bottom = math.min(a.y + a.h, b.y + b.h)

    if right <= left or bottom <= top then
        return 0
    end

    return (right - left) * (bottom - top)
end

-- ターゲット付近にラベルを配置する
-- 空いている候補を優先し、全候補が重なる場合は重なり面積が最小の位置を選ぶ
function findLabelPosition(markerX, markerY, labelWidth, labelHeight,
                           screenWidth, screenHeight, occupiedRects)
    local candidates = {
        { x = markerX + 8,              y = markerY - labelHeight - 4 },
        { x = markerX + 8,              y = markerY + 4 },
        { x = markerX - labelWidth - 8, y = markerY - labelHeight - 4 },
        { x = markerX - labelWidth - 8, y = markerY + 4 },
        { x = markerX - labelWidth / 2, y = markerY - labelHeight - 10 },
        { x = markerX - labelWidth / 2, y = markerY + 10 }
    }

    local bestRect
    local bestScore = math.huge

    for _, candidate in ipairs(candidates) do
        local rect = {
            x = clamp(candidate.x, 0, math.max(0, screenWidth - labelWidth)),
            y = clamp(candidate.y, 0, math.max(0, screenHeight - labelHeight)),
            w = labelWidth,
            h = labelHeight
        }

        local overlapScore = 0

        for _, occupied in ipairs(occupiedRects) do
            overlapScore = overlapScore + rectangleOverlapArea(rect, occupied)
        end

        -- 同じ重なり量なら、マーカーから近い候補を優先
        local centerX = rect.x + rect.w / 2
        local centerY = rect.y + rect.h / 2
        local dx = centerX - markerX
        local dy = centerY - markerY
        local distanceScore = (dx * dx + dy * dy) * 0.001

        local score = overlapScore * 1000 + distanceScore

        if score < bestScore then
            bestScore = score
            bestRect = rect

            -- 完全に重ならない候補が見つかった
            if overlapScore == 0 then
                break
            end
        end
    end

    table.insert(occupiedRects, bestRect)
    return bestRect
end

function onTick()
    isLockOn = input.getBool(1)
    currentTick = currentTick + 1

    local ownWorldCoords = {
        x = input.getNumber(27),
        y = input.getNumber(28),
        z = input.getNumber(29)
    }

    local ownEulerAngles = {
        pitch = input.getNumber(30),
        yaw = input.getNumber(31),
        roll = input.getNumber(32)
    }

    seatViewX = input.getNumber(25)
    seatViewY = input.getNumber(26)

    local seatViewAzimuth = seatViewX * PI2
    local seatViewElevation = -seatViewY * PI2

    local ownOrientationQuat = eulerZYX_to_quaternion(
        ownEulerAngles.roll,
        ownEulerAngles.yaw,
        ownEulerAngles.pitch
    )

    local isDetected = input.getNumber(9) == 1
    local trackingID = input.getNumber(7)

    --------------------------------------------------------------------------
    -- 時分割で受信したターゲットの登録・更新
    --------------------------------------------------------------------------
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

        if target.lastSeenTick < currentTick - TARGET_LOST_THRESHOLD_TICKS then
            table.remove(targetInfos, i)
        end
    end

    --------------------------------------------------------------------------
    -- 全ターゲットを現在の自機座標基準へ変換
    --
    -- ターゲット情報を受信していないtickでも更新するため、
    -- 自機が移動・回転した場合に表示が追従する
    --------------------------------------------------------------------------
    for _, target in ipairs(targetInfos) do
        local localCoords = globalToLocal(
            {
                x = target.x,
                y = target.y,
                z = target.z
            },
            ownWorldCoords,
            ownOrientationQuat
        )

        localCoords.x = localCoords.x + offsetX
        localCoords.y = localCoords.y + offsetY
        localCoords.z = localCoords.z + offsetZ

        local angle = localCoordsToLocalAngle(localCoords)

        target.localCoords = localCoords
        target.azimuth = angle.azimuth
        target.elevation = angle.elevation
        target.distance = math.sqrt(
            localCoords.x * localCoords.x +
            localCoords.y * localCoords.y +
            localCoords.z * localCoords.z
        )
    end

    --------------------------------------------------------------------------
    -- 視点に最も近いターゲットを毎tick選択
    --------------------------------------------------------------------------
    local closestDistanceSq = VIEW_SELECT_DISTANCE_SQ
    if isLockOn and chosenViewTargetID == 0 then
        for _, target in ipairs(targetInfos) do
            -- 方位角は±πの境界を考慮
            local azimuthDifference = calculateAngleDifference(
                seatViewAzimuth,
                target.azimuth
            )

            local elevationDifference =
                target.elevation - seatViewElevation

            local angularDistanceSq =
                azimuthDifference * azimuthDifference +
                elevationDifference * elevationDifference

            if angularDistanceSq < closestDistanceSq then
                closestDistanceSq = angularDistanceSq
                chosenViewTargetID = target.id
            end
        end
    end
    if not isLockOn then
        chosenViewTargetID = 0
    end
    output.setNumber(1, chosenViewTargetID)
end

function onDraw()
    local w = screen.getWidth()
    local h = screen.getHeight()
    local centerX = w / 2
    local centerY = h / 2

    local verticalFov = math.rad(HMD_FOV_HEIGHT)
    local verticalScale = centerY / math.tan(verticalFov / 2)

    local horizontalFov = 2 * math.atan(
        math.tan(verticalFov / 2) * w / h
    )
    local horizontalScale =
        centerX / math.tan(horizontalFov / 2)

    local viewAzimuth = seatViewX * PI2
    local viewElevation = -seatViewY * PI2

    local viewQuat = eulerZYX_to_quaternion(
        0,
        viewAzimuth,
        viewElevation
    )

    --------------------------------------------------------------------------
    -- 描画可能なターゲットをスクリーン座標へ投影
    --------------------------------------------------------------------------
    local projectedTargets = {}

    for _, target in ipairs(targetInfos) do
        if target.localCoords then
            local relativeCoords = rotateVectorByInverseQuaternion(
                {
                    target.localCoords.x,
                    target.localCoords.y,
                    target.localCoords.z
                },
                viewQuat
            )

            local relativeX = relativeCoords[1]
            local relativeY = relativeCoords[2]
            local relativeZ = relativeCoords[3]

            if relativeZ > 0.1 then
                local screenX =
                    centerX + horizontalScale * relativeX / relativeZ

                local screenY =
                    centerY - verticalScale * relativeY / relativeZ

                -- 完全に画面外のターゲットはラベル配置しない
                if screenX >= 0 and screenX < w and
                    screenY >= 0 and screenY < h then
                    table.insert(projectedTargets, {
                        target = target,
                        x = screenX,
                        y = screenY,
                        selected = target.id == chosenViewTargetID
                    })
                end
            end
        end
    end

    --------------------------------------------------------------------------
    -- 選択ターゲットを最優先、その後は近距離順
    --
    -- 優先度が高いラベルから空き場所を確保する
    --------------------------------------------------------------------------
    table.sort(projectedTargets, function(a, b)
        if a.selected ~= b.selected then
            return a.selected
        end

        return a.target.distance < b.target.distance
    end)

    --------------------------------------------------------------------------
    -- 先にマーカーを描画
    --------------------------------------------------------------------------
    for _, item in ipairs(projectedTargets) do
        local target = item.target
        local x = item.x
        local y = item.y

        screen.setColor(255, 0, 0, RGB_ALPHA)
        screen.drawLine(x - 1, y, x + 1, y)
        screen.drawLine(x, y - 1, x, y + 1)

        screen.setColor(255, 255, 0, RGB_ALPHA / 2)
        screen.drawCircle(x, y, target.epsilon * 10)

        if item.selected then
            screen.setColor(255, 0, 0, RGB_ALPHA)
            screen.drawCircle(x, y, 10)
        end
    end

    --------------------------------------------------------------------------
    -- 衝突を避けながらラベルを配置
    --------------------------------------------------------------------------
    local occupiedRects = {}

    for _, item in ipairs(projectedTargets) do
        local target = item.target

        local idText = "ID " .. math.floor(target.id)
        local distanceText = math.floor(target.distance) .. "m"

        local longestLength = math.max(
            #idText,
            #distanceText
        )

        -- 左右2px、上下2pxの背景余白
        local labelWidth = longestLength * FONT_WIDTH + 4
        local labelHeight = FONT_HEIGHT * 2 + 5

        local labelRect = findLabelPosition(
            item.x,
            item.y,
            labelWidth,
            labelHeight,
            w,
            h,
            occupiedRects
        )

        ----------------------------------------------------------------------
        -- マーカーとラベルを結ぶ線
        ----------------------------------------------------------------------
        local lineEndX

        if labelRect.x > item.x then
            lineEndX = labelRect.x
        else
            lineEndX = labelRect.x + labelRect.w
        end

        local lineEndY = clamp(
            item.y,
            labelRect.y,
            labelRect.y + labelRect.h
        )

        screen.setColor(0, 180, 100, RGB_ALPHA / 2)
        screen.drawLine(
            item.x,
            item.y,
            lineEndX,
            lineEndY
        )

        ----------------------------------------------------------------------
        -- 半透明背景
        ----------------------------------------------------------------------
        screen.setColor(0, 0, 0, math.min(RGB_ALPHA, 180))
        screen.drawRectF(
            labelRect.x,
            labelRect.y,
            labelRect.w,
            labelRect.h
        )

        if item.selected then
            screen.setColor(255, 255, 0, RGB_ALPHA)
            screen.drawRect(
                labelRect.x,
                labelRect.y,
                labelRect.w,
                labelRect.h
            )
        end

        ----------------------------------------------------------------------
        -- 文字
        ----------------------------------------------------------------------
        screen.setColor(0, 120, 255, RGB_ALPHA)
        screen.drawText(
            labelRect.x + 2,
            labelRect.y + 2,
            idText
        )

        screen.setColor(0, 255, 0, RGB_ALPHA)
        screen.drawText(
            labelRect.x + 2,
            labelRect.y + FONT_HEIGHT + 3,
            distanceText
        )
    end
end
