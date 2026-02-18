DT = 1 / 60
PI = math.pi
PI2 = PI * 2

PITCH_SENSI = property.getNumber("pitchSensitivity")
YAW_SENSI = property.getNumber("yawSensitivity")
ZOOM_SENSI = property.getNumber("zoomSensitivity")
YAW_CONTROL_P = property.getNumber("YawControlP")
YAW_CONTROL_I = property.getNumber("YawControlI")
YAW_CONTROL_D = property.getNumber("YawControlD")
YAW_PIVOT_MAX_SPEED = property.getNumber("YawPivotMaxSpeed")
STAB_LINE_UI_SIZE = 15
SIGHT_LINE_UI_SIZE = 25
YAW_PIVOT_ANGLE_LIMIT = 0.45
PITCH_ELEVATION_MAX = 0.05
PITCH_ELEVATION_MIN = -1
laserRangeFinder = 4000
isTargetLock = false
isStabilizer = false
stabTargetGlobalCoords = {}
zoomInput = 0
local cameraPitchAngle_Turn = 0
local cameraYawAngle_Turn = 0
function clamp(value, min, max)
    return math.max(min, math.min(value, max))
end

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

PID = {}

function PID.new(Kp, Ki, Kd)
    return {
        Kp = Kp,
        Ki = Ki,
        Kd = Kd,
        prev_error = 0,
        integral = 0
    }
end

--- PIDの内部状態をリセットします
function PID.reset(self)
    self.prev_error = 0
    self.integral = 0
end

--- PIDの計算を更新し、操作量を出力します
function PID.update(self, setpoint, measurement, dt, outputLimit)
    if outputLimit == nil then
        outputLimit = math.huge
    end

    local error = setpoint - measurement

    self.integral = self.integral + error * dt

    local derivative = (error - self.prev_error) / dt
    local output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    if output > outputLimit then
        self.integral = outputLimit - (self.Kp * error + self.Kd * derivative)
        output = outputLimit
    elseif output < -outputLimit then
        self.integral = -outputLimit - (self.Kp * error + self.Kd * derivative)
        output = -outputLimit
    end
    self.prev_error = error
    return output
end

yawControlPID = {
    pid = PID.new(YAW_CONTROL_P, YAW_CONTROL_I, YAW_CONTROL_D),
}

function onTick()
    local targetPosVec      = { x = input.getNumber(1), y = input.getNumber(2), z = input.getNumber(3) }
    local targetVelocityVec = { x = input.getNumber(4), y = input.getNumber(5), z = input.getNumber(6) }
    local ownPosVec         = { x = input.getNumber(13), y = input.getNumber(14), z = input.getNumber(15) }
    local cameraEulerAngles = { pitch = input.getNumber(16), yaw = input.getNumber(17), roll = input.getNumber(18) }
    local cameraOrientation = eulerZYX_to_quaternion(cameraEulerAngles.roll, cameraEulerAngles.yaw,
        cameraEulerAngles.pitch)
    local bodyEulerAngles   = { pitch = input.getNumber(22), yaw = input.getNumber(23), roll = input.getNumber(24) }
    local bodyOrientation   = eulerZYX_to_quaternion(bodyEulerAngles.roll, bodyEulerAngles.yaw,
        bodyEulerAngles.pitch)
    cameraAzimuth           = -input.getNumber(26) * PI2
    isTargetLock            = input.getNumber(19) == 1
    isStabilizer            = input.getNumber(20) == 1
    isIRNV                  = input.getNumber(25) == 1

    pilotSeatViewX          = input.getNumber(27)
    pilotSeatViewY          = input.getNumber(28)
    isPilotSeatViewEnable   = pilotSeatViewX ~= 0 or pilotSeatViewY ~= 0

    if input.getNumber(21) < 4000 then
        laserRangeFinder = clamp(input.getNumber(21), 50, 4000)
    end
    local cameraPivotYawRotation = input.getNumber(12)

    local adInput, wsInput
    if isPilotSeatViewEnable then
        adInput = 0
        wsInput = 0
        zoomInput = 0
    else
        adInput   = input.getNumber(9) * 0.5
        wsInput   = input.getNumber(10)
        zoomInput = clamp(zoomInput + input.getNumber(11) * ZOOM_SENSI, 0, 1)
    end

    fovRadian          = -2.175 * zoomInput + 2.2

    targetInfoFromCam  = {}
    targetInfoFromBody = {}
    if targetPosVec.x ~= 0 or targetPosVec.z ~= 0 then
        local targetLocalCoords = globalToLocal(targetPosVec, ownPosVec, cameraOrientation)
        local targetLocalAngle = localCoordsToLocalAngle(targetLocalCoords)
        local distance = math.sqrt(targetLocalCoords.x ^ 2 + targetLocalCoords.y ^ 2 + targetLocalCoords.z ^ 2)

        local projected_x_over_z = 0
        local projected_y_over_z = 0
        local is_forward = targetLocalCoords.z > 0.1 -- ゼロ除算とカメラ後方の目標を回避

        if is_forward then
            projected_x_over_z = targetLocalCoords.x / targetLocalCoords.z
            projected_y_over_z = targetLocalCoords.y / targetLocalCoords.z
        end

        targetInfoFromCam = {
            azimuth = targetLocalAngle.azimuth,
            elevation = targetLocalAngle.elevation,
            dist = distance,
            drawTextString = "",
            proj_x = projected_x_over_z, -- X/Z を onDraw に渡す
            proj_y = projected_y_over_z, -- Y/Z を onDraw に渡す
            isForward = is_forward       -- 前方フラグを onDraw に渡す
        }
        targetLocalCoords = globalToLocal(targetPosVec, ownPosVec, bodyOrientation)
        targetLocalAngle = localCoordsToLocalAngle(targetLocalCoords)
        targetInfoFromBody = {
            azimuth = targetLocalAngle.azimuth,
            elevation = targetLocalAngle.elevation,
            dist = distance,
            drawTextString = ""
        }
    end

    if isTargetLock and targetInfoFromBody.dist ~= nil and not isStabilizer then
        cameraYawAngle_Turn = clamp(targetInfoFromBody.azimuth / PI2, -YAW_PIVOT_ANGLE_LIMIT, YAW_PIVOT_ANGLE_LIMIT)
        cameraPitchAngle_Turn = clamp(targetInfoFromBody.elevation / PI2, PITCH_ELEVATION_MIN, PITCH_ELEVATION_MAX)
    elseif isStabilizer then
        if stabTargetGlobalCoords == nil or adInput ~= 0 or wsInput ~= 0 then
            local zoomSensitivityFactor = 0.5 * (1.0 - 0.9 * zoomInput)
            cameraYawAngle_Turn = clamp(cameraYawAngle_Turn + adInput * YAW_SENSI * zoomSensitivityFactor,
                -YAW_PIVOT_ANGLE_LIMIT,
                YAW_PIVOT_ANGLE_LIMIT)
            cameraPitchAngle_Turn = clamp(cameraPitchAngle_Turn + wsInput * PITCH_SENSI * zoomSensitivityFactor,
                PITCH_ELEVATION_MIN, PITCH_ELEVATION_MAX)
            stabTargetLocalCoords = localAngleDistToLocalCoords(laserRangeFinder, cameraYawAngle_Turn * PI2,
                cameraPitchAngle_Turn * PI2)
            stabTargetGlobalCoords = localToGlobal(stabTargetLocalCoords, ownPosVec, bodyOrientation)
        else
            local stabTargetLocalCoords = globalToLocal(stabTargetGlobalCoords, ownPosVec, bodyOrientation)
            stabTargetLocalAngle = localCoordsToLocalAngle(stabTargetLocalCoords)
            cameraYawAngle_Turn = clamp(stabTargetLocalAngle.azimuth / PI2, -YAW_PIVOT_ANGLE_LIMIT, YAW_PIVOT_ANGLE_LIMIT)
            cameraPitchAngle_Turn = clamp(stabTargetLocalAngle.elevation / PI2, PITCH_ELEVATION_MIN, PITCH_ELEVATION_MAX)
        end
    else
        stabTargetGlobalCoords = nil
        cameraYawAngle_Turn = clamp(cameraYawAngle_Turn + adInput * YAW_SENSI, -YAW_PIVOT_ANGLE_LIMIT,
            YAW_PIVOT_ANGLE_LIMIT)
        cameraPitchAngle_Turn = clamp(cameraPitchAngle_Turn + wsInput * PITCH_SENSI, PITCH_ELEVATION_MIN,
            PITCH_ELEVATION_MAX)
    end
    cameraYawOutput = PID.update(yawControlPID.pid, cameraYawAngle_Turn, cameraPivotYawRotation, DT,
        YAW_PIVOT_MAX_SPEED)

    output.setNumber(1, cameraYawOutput)
    output.setNumber(2, cameraPitchAngle_Turn * 4)
    output.setNumber(3, zoomInput)
    if stabTargetGlobalCoords ~= nil and isStabilizer then
        output.setBool(1, true)
        output.setNumber(4, stabTargetGlobalCoords.x)
        output.setNumber(5, stabTargetGlobalCoords.y)
        output.setNumber(6, stabTargetGlobalCoords.z)
    else
        output.setBool(1, false)
    end
    local cameraLocalAngleTurn = { azimuth = cameraPivotYawRotation, elevation = cameraPitchAngle_Turn }
    output.setNumber(9, cameraLocalAngleTurn.azimuth)
    output.setNumber(10, cameraLocalAngleTurn.elevation)
end

function onDraw()
    local w, h, centerX, centerY
    w = screen.getWidth()  -- 288
    h = screen.getHeight() -- 160
    centerX = w / 2
    centerY = h / 2

    -- 1. カメラのFOV (ラジアン)

    -- 描画色をHUDに合わせて設定 (画像参照)
    screen.setColor(255, 180, 0)

    local compassY = 20 -- 方位角表示テープのY座標 (画面上部からの位置)
    local currentAngleRad = cameraAzimuth
    local currentAngleDeg = currentAngleRad * 180 / PI

    -- 1. 中央のマーカー (現在向いている方向を示す三角)
    screen.drawTriangleF(centerX, compassY + 5, centerX - 3, compassY, centerX + 3, compassY)

    -- 2. 現在の方位角 (度数) を中央マーカーの上に表示 (000°～359°の範囲)
    --    (currentAngleDeg + 360) % 360 で角度を 0-359.9... の範囲に正規化
    local degText = string.format("%03.0f", (currentAngleDeg + 360) % 360)
    --    文字幅 (4px * 3文字 = 12px) を考慮して中央揃え (centerX - 12/2)
    screen.drawText(centerX - 6, compassY - 8, degText)

    -- 3. 目盛りを描画
    local pixelsPerDegree = 3                      -- 1度あたりのピクセル幅 (数値を大きくすると間隔が広がる)
    local tickStartY = compassY + 7                -- 目盛りが始まるY座標
    local majorTickHeight = 5                      -- 主要な目盛り (N, E, S, Wなど) の高さ
    local minorTickHeight = 3                      -- 副次的な目盛り (10度ごと) の高さ
    local textY = tickStartY + majorTickHeight + 2 -- 方位文字 (N, Eなど) のY座標

    -- 画面幅全体をカバーするのに十分な角度範囲を計算
    -- (centerXから片側 w/2 ピクセル分)
    local angleRangeDeg = (w / 2) / pixelsPerDegree
    local startAngleDeg = math.floor(currentAngleDeg - angleRangeDeg)
    local endAngleDeg = math.ceil(currentAngleDeg + angleRangeDeg)

    -- 描画開始角度を5の倍数に丸める (処理負荷軽減のため)
    startAngleDeg = math.floor(startAngleDeg / 5) * 5

    -- 5度ずつループして目盛りを描画
    for angle = startAngleDeg, endAngleDeg, 5 do
        -- 現在の中央角度 (currentAngleDeg) からの差分を計算
        local angleDiffDeg = angle - currentAngleDeg
        -- 画面上でのX座標を計算
        local screenX = centerX + angleDiffDeg * pixelsPerDegree

        -- 目盛りが画面内に収まっているかチェック (左右に少しマージン)
        if screenX >= 5 and screenX <= (w - 5) then
            -- 角度を 0-359.9... の範囲に正規化
            local normalizedAngle = (angle + 360) % 360

            -- 浮動小数点数の比較のため、非常に近い値 (例: 89.999... と 90) を同一視する
            local function isAngleNear(a, b)
                return math.abs(a - b) < 0.1
            end

            if isAngleNear(normalizedAngle % 90, 0) then -- N(0), E(90), S(180), W(270)
                screen.drawLine(screenX, tickStartY, screenX, tickStartY + majorTickHeight)
                local compassChar = ""
                if isAngleNear(normalizedAngle, 0) then
                    compassChar = "N"
                elseif isAngleNear(normalizedAngle, 90) then
                    compassChar = "E"
                elseif isAngleNear(normalizedAngle, 180) then
                    compassChar = "S"
                elseif isAngleNear(normalizedAngle, 270) then
                    compassChar = "W"
                end
                screen.drawText(screenX - 2, textY, compassChar) -- 1文字 (幅4px) を中央揃え (screenX - 4/2) [cite: 133, 134]
            elseif isAngleNear(normalizedAngle % 45, 0) then     -- NE(45), SE(135), SW(225), NW(315)
                screen.drawLine(screenX, tickStartY, screenX, tickStartY + majorTickHeight)
                local compassChar = ""
                if isAngleNear(normalizedAngle, 45) then
                    compassChar = "NE"
                elseif isAngleNear(normalizedAngle, 135) then
                    compassChar = "SE"
                elseif isAngleNear(normalizedAngle, 225) then
                    compassChar = "SW"
                elseif isAngleNear(normalizedAngle, 315) then
                    compassChar = "NW"
                end
                screen.drawText(screenX - 4, textY, compassChar) -- 2文字 (幅4*2=8px) を中央揃え (screenX - 8/2)
            elseif isAngleNear(normalizedAngle % 10, 0) then     -- 10度ごとの目盛り (45度ごとを除く)
                screen.drawLine(screenX, tickStartY, screenX, tickStartY + minorTickHeight)
            else                                                 -- 5度ごとの短い目盛り (10度ごとを除く)
                screen.drawLine(screenX, tickStartY, screenX, tickStartY + minorTickHeight / 2)
            end
        end
    end

    -- 4. 水平線 (テープの基準線)
    screen.drawLine(0, tickStartY, w, tickStartY)

    -- 2. 垂直方向のスケール (tan(FOV/2) の逆数)
    -- (rh = 画面高さの半分 / tan(垂直FOVの半分) )
    local rh = centerY / math.tan(fovRadian / 2)

    -- 3. 水平FOV (ラジアン) をアスペクト比から計算
    local fov_rad_horizontal = 2 * math.atan(math.tan(fovRadian / 2) * (w / h))
    -- 4. 水平方向のスケール
    -- (rw = 画面幅の半分 / tan(水平FOVの半分) )
    local rw = centerX / math.tan(fov_rad_horizontal / 2)

    sightLineUI_Start = math.floor(SIGHT_LINE_UI_SIZE / 7)
    screen.drawLine(centerX - sightLineUI_Start, centerY, centerX - SIGHT_LINE_UI_SIZE, centerY)
    screen.drawLine(centerX + sightLineUI_Start, centerY, centerX + SIGHT_LINE_UI_SIZE, centerY)
    screen.drawLine(centerX, centerY - sightLineUI_Start, centerX, centerY - SIGHT_LINE_UI_SIZE * 0.7)
    screen.drawLine(centerX, centerY + sightLineUI_Start, centerX, centerY + SIGHT_LINE_UI_SIZE * 0.7)

    if isStabilizer then
        screen.setColor(255, 180, 0)
        screen.drawText(5, 5, "MANUAL AIM")
        screen.drawText(5, 11, "STAB")
        stabLineUI_Start = math.floor(STAB_LINE_UI_SIZE / 3)

        if isTargetLock then
            screen.drawText(centerX - 9, centerY - 23, "FIRE")
        end
        screen.drawLine(centerX - stabLineUI_Start, centerY - stabLineUI_Start, centerX - STAB_LINE_UI_SIZE,
            centerY - STAB_LINE_UI_SIZE)
        screen.drawLine(centerX + stabLineUI_Start, centerY - stabLineUI_Start, centerX + STAB_LINE_UI_SIZE,
            centerY - STAB_LINE_UI_SIZE)
        screen.drawLine(centerX - stabLineUI_Start, centerY + stabLineUI_Start, centerX - STAB_LINE_UI_SIZE,
            centerY + STAB_LINE_UI_SIZE)
        screen.drawLine(centerX + stabLineUI_Start, centerY + stabLineUI_Start, centerX + STAB_LINE_UI_SIZE,
            centerY + STAB_LINE_UI_SIZE)
    elseif isTargetLock then
        if targetInfoFromBody.dist ~= nil then
            screen.setColor(255, 180, 0)
            screen.drawText(5, 11, "LOCK")

            screen.drawText(centerX - 9, centerY - 100, "FIRE")
        else
            screen.setColor(255, 213, 0, 50)
            screen.drawText(5, 11, "SELECT TARGET")
        end
        screen.drawText(5, 5, "AUTO AIM")
    else
        screen.setColor(0, 255, 0)
    end

    if isIRNV then
        screen.drawText(5, 17, "IRNV")
    end


    if targetInfoFromCam.dist ~= nil then
        -- 6. ターゲットがカメラの前方にある場合のみ描画
        if targetInfoFromCam.isForward then
            -- 8. onTickで計算済みの投影値 (X/Z と Y/Z) を取得
            local projected_x_over_z = targetInfoFromCam.proj_x
            local projected_y_over_z = targetInfoFromCam.proj_y

            -- 9. 画面座標に変換 (正しい透視投影)
            --    screenX は X/Z に比例
            --    screenY は Y/Z に比例
            local screenX = centerX + rw * projected_x_over_z
            local screenY = centerY - rh * projected_y_over_z -- Y軸反転 [cite: 160]
            -- 10. 描画
            screen.drawRect(screenX - 3, screenY - 3, 6, 6)
            screen.drawText(screenX, screenY + 5, targetInfoFromCam.drawTextString)

            local dist_text = math.floor(targetInfoFromCam.dist) .. "m"
            screen.drawText(screenX + 8, screenY - 3, dist_text)
        end
    end
end
