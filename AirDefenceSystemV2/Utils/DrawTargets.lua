-- 定数
local PI              = math.pi
local PI2             = PI * 2
local DT              = 1 / 60 -- EKF更新の時間ステップ (秒)

-- グローバル変数
local trackedTarget   = nil   -- { X, P, epsilon, lastTick, lastSeenTick } 追跡中の単一目標の状態
local currentTick     = 0
local isTracking      = false -- 現在有効な追跡を行っているか
local targetPositions = {}
local ownOrientation  = {}
local ownPos          = {}
local ownQuaternion   = {}
local cameraZoomLevel = 0
--------------------------------------------------------------------------------
-- ベクトル演算ヘルパー関数
--------------------------------------------------------------------------------
function vectorMagnitude(v)
    local x, y, z
    x = v[1] or v.x or 0
    y = v[2] or v.y or 0
    z = v[3] or v.z or 0
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

function vectorNormalize(v)
    local mag, x, y, z = vectorMagnitude(v)
    if mag < 1e-9 then
        return { 0, 0, 0 }
    else
        x = v[1] or v.x or 0
        y = v[2] or v.y or 0
        z = v[3] or v.z or 0
        return { x / mag, y / mag, z / mag }
    end
end

function vectorAdd(v1, v2)
    local x1, x2, y1, y2, z1, z2

    x1 = v1[1] or v1.x or 0
    y1 = v1[2] or v1.y or 0
    z1 = v1[3] or v1.z or 0
    x2 = v2[1] or v2.x or 0
    y2 = v2[2] or v2.y or 0
    z2 = v2[3] or v2.z or 0
    return { x1 + x2, y1 + y2, z1 + z2 }
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

function vectorScalarMul(s, v)
    local x, y, z
    x = v[1] or v.x or 0
    y = v[2] or v.y or 0
    z = v[3] or v.z or 0
    return { s * x, s * y, s * z }
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
    return { y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2 }
end

--------------------------------------------------------------------------------
-- 行列演算ヘルパー関数
--------------------------------------------------------------------------------
function zeros(rows, cols)
    local m = {}
    for r = 1, rows do
        m[r] = {}
        for c = 1, cols do m[r][c] = 0 end
    end
    return m
end

function MatrixCopy(M)
    local N = {}
    for r, row in ipairs(M) do
        N[r] = { table.unpack(row) }
    end
    return N
end

function scalar(s, M)
    local R = zeros(#M, #M[1])
    for r = 1, #M do
        for c = 1, #M[1] do
            R[r][c] = M[r][c] * s
        end
    end
    return R
end

function sum(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do
            R[r][c] = A[r][c] + B[r][c]
        end
    end
    return R
end

function sub(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do
            R[r][c] = A[r][c] - B[r][c]
        end
    end
    return R
end

function mul(...)
    local mats, A, R, B, sVal = { ... }
    A = mats[1]
    for i = 2, #mats do
        B = mats[i]
        -- nilチェックは原則削除
        R = zeros(#A, #B[1])
        for r = 1, #A do
            for c = 1, #B[1] do
                sVal = 0
                for k = 1, #B do
                    -- nilチェックは原則削除
                    sVal = sVal + A[r][k] * B[k][c]
                end
                R[r][c] = sVal
            end
        end
        A = R -- 次の乗算のために結果をAに代入
    end
    return A  -- 最終結果
end

function T(M)
    local rows, cols, R
    -- nilチェックは原則削除
    rows = #M
    cols = #M[1]
    R = zeros(cols, rows)
    for r = 1, rows do
        -- nilチェックは原則削除
        for c = 1, cols do
            -- nilチェックは原則削除
            R[c][r] = M[r][c]
        end
    end
    return R
end

function inv(M)
    if M == nil or #M == 0 or #M[1] == 0 then return nil end
    local n = #M
    if n ~= #M[1] then return nil end -- 基本チェック
    local aug = {}
    for r = 1, n do
        aug[r] = {}
        if M[r] == nil then return nil end
        for c = 1, n do
            local v = M[r][c]
            if v == nil or v ~= v or v == math.huge or v == -math.huge then return nil end
            aug[r][c] = v
        end
        for c = 1, n do aug[r][n + c] = (r == c) and 1 or 0 end
    end -- 入力チェック
    for r = 1, n do
        local piv = aug[r][r]
        if piv == nil or math.abs(piv) < 1e-12 then return nil end -- ピボットチェック
        for c = r, 2 * n do
            if aug[r][c] == nil then return nil end
            aug[r][c] = aug[r][c] / piv
        end -- 除算前 nil チェック
        for i = 1, n do
            if i ~= r then
                local f = aug[i][r]
                if f == nil then return nil end
                for c = r, 2 * n do
                    if aug[i][c] == nil or aug[r][c] == nil then return nil end
                    aug[i][c] = aug[i][c] - f * aug[r][c]
                end
            end
        end
    end
    local invM = zeros(n, n)
    for r = 1, n do
        for c = 1, n do
            local v = aug[r][n + c]
            if v == nil or v ~= v or v == math.huge or v == -math.huge then
                invM[r][c] = 0
            else
                invM[r][c] = v
            end
        end
    end -- 結果チェック
    return invM
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
-- 座標変換関数
--------------------------------------------------------------------------------
-- ローカル直交座標からグローバル直交座標へ
function localToGlobalCoords(localPosVec, ownGlobalPos, ownOrientationQuat)
    local localX, localY, localZ, globalRelativeVector, globalX, globalY, globalZ
    -- vector は {x, y, z} または {<index 1>, <index 2>, <index 3>} 形式のテーブルを想定
    -- nilチェックは原則削除
    localX = localPosVec[1] or localPosVec.x or 0
    localY = localPosVec[2] or localPosVec.y or 0
    localZ = localPosVec[3] or localPosVec.z or 0
    -- nilチェックは原則削除
    globalRelativeVector = rotateVectorByQuaternion({ localX, localY, localZ }, ownOrientationQuat)
    -- nilチェックは原則削除
    globalX = globalRelativeVector[1] + ownGlobalPos.x
    globalY = globalRelativeVector[2] + ownGlobalPos.y
    globalZ = globalRelativeVector[3] + ownGlobalPos.z
    return { x = globalX, y = globalY, z = globalZ }
end

-- グローバル直交座標からローカル直交座標へ
function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local gX, gY, gZ, oX, oY, oZ, relativeVectorGlobal, localVector
    -- 各入力が {x, y, z} 形式のテーブルを想定
    -- nilチェックは原則削除
    gX = globalTargetPos.x or 0
    gY = globalTargetPos.y or 0
    gZ = globalTargetPos.z or 0
    oX = ownGlobalPos.x or 0
    oY = ownGlobalPos.y or 0
    oZ = ownGlobalPos.z or 0
    -- nilチェックは原則削除
    relativeVectorGlobal = { gX - oX, gY - oY, gZ - oZ }
    localVector = rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientationQuat)
    -- nilチェックは原則削除
    return { x = localVector[1], y = localVector[2], z = localVector[3] }
end

-- 角度差を計算 (-PI から PI の範囲)
function calculateAngleDifference(angle1, angle2)
    -- nilチェックは原則削除
    local diff = angle2 - angle1
    while diff <= -PI do diff = diff + PI2 end
    while diff > PI do diff = diff - PI2 end
    return
        diff
end

function onTick()
    local isDataLinkInput = input.getBool(32)
    local targetLocalCoords, dist, local_elev_rad, local_azim_rad, GV_relative_vec

    if isDataLinkInput then
        targetPositions = {}
        for i = 1, 10 do
            local baseChannel = (i - 1) * 3
            if input.getNumber(baseChannel + 1) ~= 0 then
                local targetPos = {
                    x = input.getNumber(baseChannel + 1),
                    y = input.getNumber(baseChannel + 2),
                    z = input.getNumber(baseChannel + 3),
                    id = i,
                    dist = 0,
                    azim = 0,
                    elev = 0,
                    relX = 0,
                    relY = 0,
                    relZ = 0,
                }
                table.insert(targetPositions, targetPos)
            end
        end
    else
        local roll, pitch, yaw
        cameraZoomLevel = input.getNumber(20)
        pitch = input.getNumber(4)
        yaw = input.getNumber(5)
        roll = input.getNumber(6)
        ownQuaternion = eulerZYX_to_quaternion(roll, yaw, pitch)
        ownPos = {
            x = input.getNumber(1),
            y = input.getNumber(2),
            z = input.getNumber(3)
        }
    end

    for i = 1, #targetPositions do
        targetLocalCoords = globalToLocalCoords(targetPositions[i], ownPos, ownQuaternion)
        GV_relative_vec = vectorSub(targetPositions[i], ownPos)
        dist = math.sqrt(targetLocalCoords.x ^ 2 + targetLocalCoords.y ^ 2 + targetLocalCoords.z ^ 2)
        local_elev_rad = math.asin(targetLocalCoords.y / dist)
        local_azim_rad = math.atan(targetLocalCoords.x, targetLocalCoords.z)
        targetPositions[i].dist = dist
        targetPositions[i].azim = local_azim_rad
        targetPositions[i].elev = local_elev_rad
        targetPositions[i].relX = GV_relative_vec[1]
        targetPositions[i].relY = GV_relative_vec[2]
        targetPositions[i].relZ = GV_relative_vec[3]
    end
end

function onDraw()
    local w = screen.getWidth()
    local h = screen.getHeight()
    local cw = w / 2
    local ch = h / 2

    local fov = (cameraZoomLevel + 1) / 2
    local fovRad = -2.175 * fov + 2.2
    local rw = (cw) / math.tan(fovRad / 2)
    local rh = (ch) / math.tan(fovRad / 2)
    if fovRad <= 0 then fovRad = PI / 2 end
    screen.setColor(0, 255, 0)
    for i = 1, #targetPositions do
        local tan_azim = math.tan(targetPositions[i].azim)
        local tan_elev = math.tan(targetPositions[i].elev)
        local echoX = cw + rw * tan_azim * (3 / 5)
        local echoY = ch - rh * tan_elev
        screen.setColor(0, targetPositions[i].id * 25, 255 - targetPositions[i].id * 25) -- idによって表示名の色を替えて視覚的にわかりやすく
        screen.drawText(math.floor(echoX + 0.5), math.floor(echoY - 8 + 0.5), targetPositions[i].id)
    end
end
