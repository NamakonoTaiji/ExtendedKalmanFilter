--[[
================================================================================
ADSV3 KalmanFilter_SingleTarget.lua (RCS最小目標専用版)
================================================================================
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60           -- EKF更新の時間ステップ (秒)
MAX_RADAR_TARGETS = 6 -- 処理するレーダー目標の最大数
NUM_STATES = 9        -- EKF状態数 (x, vx, ax, y, vy, ay, z, vz, az)

-- EKF パラメータ
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")                       -- 目標ロスト判定のTick数
PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") -- 予測信頼度低下係数
INITIAL_ACCELERATION_VARIANCE = 1e+6
INITIAL_VELOCITY_VARIANCE = 1e+6
DATALINK_GATE_RADIUS = property.getNumber("DL_GATE") -- データリンク許容離隔距離

-- PI制御パラメータ
NOISE_TARGET_EPSILON = property.getNumber("NOISE_TARGET_EPS")
NOISE_INTEGRAL_GAIN = property.getNumber("NOISE_I_GAIN")
NOISE_OUTPUT_MIN = -8
NOISE_OUTPUT_MAX = 1

LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
R0_DIST_VAR_FACTOR = 6.67e-4
R0_ANGLE_VAR = 2.63e-4
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }

-- グローバル変数
track = nil -- 単一ターゲット情報
currentTick = 0
radarManualSweepX = 0
radarManualSweepY = 0
dataLinkGlobalPos = { 0, 0, 0 }
dataLinkTargetVelocity = { 0, 0, 0 }

--------------------------------------------------------------------------------
-- ベクトル・行列演算ヘルパー関数
--------------------------------------------------------------------------------
function vectorMagnitude(v)
    return math.sqrt(v[1] ^ 2 + v[2] ^ 2 + v[3] ^ 2)
end

function vectorSub(v1, v2)
    return { v1[1] - v2[1], v1[2] - v2[2], v1[3] - v2[3] }
end

function zeros(rows, cols)
    local m = {}
    for r = 1, rows do
        m[r] = {}
        for c = 1, cols do m[r][c] = 0 end
    end
    return m
end

identityMatrix9x9 = zeros(9, 9)
for i = 1, 9 do identityMatrix9x9[i][i] = 1 end

function MatrixCopy(M)
    local N = {}
    for r, row in ipairs(M) do N[r] = { table.unpack(row) } end
    return N
end

function scalar(s, M)
    local R = zeros(#M, #M[1])
    for r = 1, #M do
        for c = 1, #M[1] do R[r][c] = M[r][c] * s end
    end
    return R
end

function sum(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do R[r][c] = A[r][c] + B[r][c] end
    end
    return R
end

function sub(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do R[r][c] = A[r][c] - B[r][c] end
    end
    return R
end

function mul(...)
    local mats = { ... }
    local A = mats[1]
    for i = 2, #mats do
        local B = mats[i]
        local R = zeros(#A, #B[1])
        for r = 1, #A do
            for c = 1, #B[1] do
                local sVal = 0
                for k = 1, #B do sVal = sVal + A[r][k] * B[k][c] end
                R[r][c] = sVal
            end
        end
        A = R
    end
    return A
end

function T(M)
    local rows, cols = #M, #M[1]
    local R = zeros(cols, rows)
    for r = 1, rows do
        for c = 1, cols do R[c][r] = M[r][c] end
    end
    return R
end

function inv3(m)
    local a, b, c = m[1][1], m[1][2], m[1][3]
    local d, e, f = m[2][1], m[2][2], m[2][3]
    local g, h, i = m[3][1], m[3][2], m[3][3]
    local det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if math.abs(det) < 1e-15 then return nil end
    local invDet = 1 / det
    return {
        { (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet },
        { (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet },
        { (d * h - e * g) * invDet, (g * b - a * h) * invDet, (a * e - b * d) * invDet }
    }
end

--------------------------------------------------------------------------------
-- 姿勢・座標変換関数
--------------------------------------------------------------------------------
function eulerZYX_to_quaternion(roll, yaw, pitch)
    local hr, hy, hp = roll * 0.5, yaw * 0.5, pitch * 0.5
    local cr, sr = math.cos(hr), math.sin(hr)
    local cy, sy = math.cos(hy), math.sin(hy)
    local cp, sp = math.cos(hp), math.sin(hp)
    return {
        cr * cy * cp + sr * sy * sp,
        cr * cy * sp - sr * sy * cp,
        cr * sy * cp + sr * cy * sp,
        sr * cy * cp - cr * sy * sp
    }
end

function rotateVectorByQuaternion(vector, quaternion, isInverse)
    local w, x, y, z = quaternion[1], quaternion[2], quaternion[3], quaternion[4]
    local vx, vy, vz = vector[1], vector[2], vector[3]
    local tx = 2 * (y * vz - z * vy)
    local ty = 2 * (z * vx - x * vz)
    local tz = 2 * (x * vy - y * vx)
    if isInverse then w = -w end
    return {
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx)
    }
end

function localToGlobalCoords(localPosVec, ownGlobalPos, ownOrientationQuat)
    local gRel = rotateVectorByQuaternion(localPosVec, ownOrientationQuat)
    return { gRel[1] + ownGlobalPos[1], gRel[2] + ownGlobalPos[2], gRel[3] + ownGlobalPos[3] }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local relG = { globalTargetPos[1] - ownGlobalPos[1], globalTargetPos[2] - ownGlobalPos[2], globalTargetPos[3] -
    ownGlobalPos[3] }
    return rotateVectorByQuaternion(relG, ownOrientationQuat, true)
end

function localCoordsToLocalAngle(localPosVec)
    local horizDist = math.sqrt(localPosVec[1] ^ 2 + localPosVec[3] ^ 2)
    return {
        azimuth = math.atan(localPosVec[1], localPosVec[3]),
        elevation = math.atan(localPosVec[2], horizDist)
    }
end

function polarCoordsToLocalCoords(dist, localEleRad, localAziRad)
    return {
        dist * math.cos(localEleRad) * math.sin(localAziRad),
        dist * math.sin(localEleRad),
        dist * math.cos(localEleRad) * math.cos(localAziRad)
    }
end

--------------------------------------------------------------------------------
-- EKF コア関数
--------------------------------------------------------------------------------
function getObservationJacobianAndPrediction(stateVector, ownPosition)
    local targetX, targetY, targetZ = stateVector[1][1], stateVector[4][1], stateVector[7][1]
    local relX, relY, relZ = targetX - ownPosition[1], targetY - ownPosition[2], targetZ - ownPosition[3]
    local r_sq = relX ^ 2 + relY ^ 2 + relZ ^ 2
    local rh_sq = relX ^ 2 + relZ ^ 2
    if r_sq < 1e-9 then r_sq = 1e-9 end
    if rh_sq < 1e-9 then rh_sq = 1e-9 end
    local r, rh = math.sqrt(r_sq), math.sqrt(rh_sq)

    local h = { { r }, { math.asin(math.max(-1.0, math.min(1.0, relY / r))) }, { math.atan(relX, relZ) } }
    local H = zeros(3, NUM_STATES)
    H[1][1], H[2][1], H[3][1] = relX / r, (relX * -relY) / (r_sq * rh), relZ / rh_sq
    H[1][4], H[2][4], H[3][4] = relY / r, rh / r_sq, 0
    H[1][7], H[2][7], H[3][7] = relZ / r, (relZ * -relY) / (r_sq * rh), -relX / rh_sq

    return H, h
end

function calculateAngleDifference(angle1, angle2)
    local diff = angle2 - angle1
    while diff <= -PI do diff = diff + PI2 end
    while diff > PI do diff = diff - PI2 end
    return diff
end

function predictStep(currentTarget, dt_sec)
    local lastEps = currentTarget.epsilon or NOISE_TARGET_EPSILON
    local lastErr = currentTarget.integralError or 0

    local err = NOISE_TARGET_EPSILON - lastEps
    local newErr = lastErr + err * NOISE_INTEGRAL_GAIN
    local cU = math.max(NOISE_OUTPUT_MIN, math.min(NOISE_OUTPUT_MAX, newErr))
    if cU == NOISE_OUTPUT_MIN or cU == NOISE_OUTPUT_MAX then newErr = lastErr end

    local noiseScale_bH = 10 ^ -(3 + cU)

    local dt2_half = dt_sec * dt_sec * 0.5
    local F = MatrixCopy(identityMatrix9x9)
    F[1][2], F[1][3], F[2][3] = dt_sec, dt2_half, dt_sec
    F[4][5], F[4][6], F[5][6] = dt_sec, dt2_half, dt_sec
    F[7][8], F[7][9], F[8][9] = dt_sec, dt2_half, dt_sec

    local X_pred = mul(F, currentTarget.X)
    X_pred[3][1] = X_pred[3][1] * 0.9
    X_pred[6][1] = X_pred[6][1] * 0.9
    X_pred[9][1] = X_pred[9][1] * 0.9

    local dt2 = dt_sec * dt_sec
    local dt3 = dt2 * dt_sec
    local q_block = {
        { (dt3 * dt3) / 36, (dt2 * dt3) / 12, (dt2 * dt2) / 6 },
        { (dt2 * dt3) / 12, (dt2 * dt2) / 4,  dt3 / 2 },
        { (dt2 * dt2) / 6,  dt3 / 2,          dt2 }
    }
    local Q_base = zeros(NUM_STATES, NUM_STATES)
    for i = 0, 2 do
        for r = 1, 3 do
            for c = 1, 3 do Q_base[i * 3 + r][i * 3 + c] = q_block[r][c] end
        end
    end

    local Q_adapted = scalar(noiseScale_bH, Q_base)
    local uncFactor = 1.0 + (PREDICTION_UNCERTAINTY_FACTOR_BASE * dt_sec)
    local P_pred = sum(scalar(uncFactor, mul(F, currentTarget.P, T(F))), Q_adapted)

    return X_pred, P_pred, newErr
end

function calculateInnovation(X_pred, P_pred, obs, ownPos)
    local Z = { { obs.distance }, { obs.elevation }, { obs.azimuth } }
    local H, h = getObservationJacobianAndPrediction(X_pred, ownPos)

    local R_mat = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R_mat[1][1] = R_mat[1][1] * (obs.distance ^ 2)

    local Y = zeros(3, 1)
    Y[1][1] = Z[1][1] - h[1][1]
    Y[2][1] = calculateAngleDifference(h[2][1], Z[2][1])
    Y[3][1] = calculateAngleDifference(h[3][1], Z[3][1])

    local S = sum(mul(H, P_pred, T(H)), R_mat)
    local S_inv = inv3(S)
    if S_inv == nil then return math.huge, nil, nil, nil, nil end

    local eps = 1.0
    local epsM = mul(T(Y), S_inv, Y)
    if epsM and epsM[1] and epsM[1][1] then eps = epsM[1][1] end

    local vx, vy, vz = X_pred[2][1], X_pred[5][1], X_pred[8][1]
    local vSpeed = math.sqrt(vx ^ 2 + vy ^ 2 + vz ^ 2)
    if vSpeed > 50 then
        local dx, dy, dz = obs.globalX - X_pred[1][1], obs.globalY - X_pred[4][1], obs.globalZ - X_pred[7][1]
        local dDist = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
        if dDist > 1e-3 then
            local cosTheta = (vx * dx + vy * dy + vz * dz) / (vSpeed * dDist)
            if cosTheta < 0.8 then eps = eps * (2.0 - cosTheta) end
        end
    end

    return eps, Y, S_inv, H, R_mat
end

function updateStep(X_pred, P_pred, Y, S_inv, H, R_mat)
    local K = mul(P_pred, T(H), S_inv)
    local X_up = sum(X_pred, mul(K, Y))

    local I_m_KH = sub(identityMatrix9x9, mul(K, H))
    local P_up = sum(mul(I_m_KH, P_pred, T(I_m_KH)), mul(K, R_mat, T(K)))

    local eps = 1.0
    local epsM = mul(T(Y), S_inv, Y)
    if epsM and epsM[1] and epsM[1][1] then eps = epsM[1][1] end

    return X_up, P_up, eps
end

function initializeFilterState(obs, tick)
    local X_init = {
        { obs.globalX }, { 0 }, { 0 },
        { obs.globalY }, { 0 }, { 0 },
        { obs.globalZ }, { 0 }, { 0 }
    }
    local P_init = zeros(NUM_STATES, NUM_STATES)
    local R_init = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R_init[1][1] = R_init[1][1] * (obs.distance ^ 2)
    local scale = 10

    P_init[1][1] = (R_init[3][3] + R_init[1][1]) * scale
    P_init[4][4] = (R_init[2][2] + R_init[1][1]) * scale
    P_init[7][7] = (R_init[3][3] + R_init[1][1]) * scale
    P_init[2][2], P_init[5][5], P_init[8][8] = INITIAL_VELOCITY_VARIANCE, INITIAL_VELOCITY_VARIANCE,
        INITIAL_VELOCITY_VARIANCE
    P_init[3][3], P_init[6][6], P_init[9][9] = INITIAL_ACCELERATION_VARIANCE, INITIAL_ACCELERATION_VARIANCE,
        INITIAL_ACCELERATION_VARIANCE

    return {
        X = X_init,
        P = P_init,
        epsilon = 1.0,
        lastTick = tick,
        misses = 0,
        integralError = 0
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    currentTick = currentTick + 1
    local isLaunch = input.getNumber(31) == 1

    -- センサー・データリンクの取得
    ownGlobalPos = { input.getNumber(25), input.getNumber(26), input.getNumber(27) }
    ownEuler = { Pitch = input.getNumber(28), Yaw = input.getNumber(29), Roll = input.getNumber(30) }
    ownOrientation = eulerZYX_to_quaternion(ownEuler.Roll, ownEuler.Yaw, ownEuler.Pitch) or { 1, 0, 0, 0 }

    if input.getNumber(22) ~= 0 then
        dataLinkGlobalPos = { input.getNumber(22), input.getNumber(23), input.getNumber(24) }
        dataLinkTargetVelocity = { input.getNumber(19), input.getNumber(20), input.getNumber(21) }
    end

    -- RCS最小目標（一番後ろの有効チャンネル）の抽出
    local obs = nil
    if isLaunch then
        for i = MAX_RADAR_TARGETS, 1, -1 do
            local base = (i - 1) * 3
            local dist = input.getNumber(base + 1)
            if dist > 0 then
                local localAziRad = input.getNumber(base + 2) * PI2
                local localEleRad = input.getNumber(base + 3) * PI2
                local targetLocalVec = polarCoordsToLocalCoords(dist, localEleRad, localAziRad)
                local targetGlobal = localToGlobalCoords(targetLocalVec, ownGlobalPos, ownOrientation)

                if targetGlobal then
                    local relG = vectorSub(targetGlobal, ownGlobalPos)
                    local distCheck = vectorMagnitude(relG)
                    local gEle = math.asin(math.max(-1.0, math.min(1.0, relG[2] / distCheck)))
                    local gAzi = math.atan(relG[1], relG[3])

                    obs = {
                        distance = dist,
                        azimuth = gAzi,
                        elevation = gEle,
                        globalX = targetGlobal[1],
                        globalY = targetGlobal[2],
                        globalZ = targetGlobal[3]
                    }
                    break -- 最低RCS目標を1点取得したら終了
                end
            end
        end
    end

    -- EKF 更新処理
    if track ~= nil then
        local dt_pred_sec = math.max(0, (currentTick - track.lastTick) * DT)
        local X_pred, P_pred, newIntErr = predictStep(track, dt_pred_sec)

        if X_pred == nil or P_pred == nil then
            track = nil
        else
            track.integralError = newIntErr

            if obs ~= nil then
                -- 観測値が得られた場合はフィルターを更新
                local eps, Y, S_inv, H, R_mat = calculateInnovation(X_pred, P_pred, obs, ownGlobalPos)
                if S_inv ~= nil then
                    local X_up, P_up, eps_up = updateStep(X_pred, P_pred, Y, S_inv, H, R_mat)
                    track.X = X_up
                    track.P = P_up
                    track.epsilon = eps_up
                    track.lastTick = currentTick
                    track.misses = 0
                else
                    track.misses = track.misses + 1
                end
            else
                -- 観測値が無い場合は予測値を維持
                track.X = X_pred
                track.P = P_pred
                track.misses = track.misses + 1
            end

            -- ロスト判定
            if track.misses > TARGET_LOST_THRESHOLD_TICKS then
                track = nil
            end
        end
    end

    -- トラック未保持状態で新しい観測を得た場合は初期化
    if track == nil and obs ~= nil then
        track = initializeFilterState(obs, currentTick)
    end

    -- 出力処理
    output.setBool(3, isLaunch)

    if track ~= nil then
        local trackX = track.X
        local dt_delay = DT * LOGIC_DELAY
        local dt_delay2_half = dt_delay ^ 2 * 0.5

        local outputX = trackX[1][1] + trackX[2][1] * dt_delay + trackX[3][1] * dt_delay2_half
        local outputY = trackX[4][1] + trackX[5][1] * dt_delay + trackX[6][1] * dt_delay2_half
        local outputZ = trackX[7][1] + trackX[8][1] * dt_delay + trackX[9][1] * dt_delay2_half

        output.setNumber(1, outputX)
        output.setNumber(2, outputY)
        output.setNumber(3, outputZ)
        output.setNumber(4, trackX[2][1])
        output.setNumber(5, trackX[5][1])
        output.setNumber(6, trackX[8][1])
        output.setNumber(7, trackX[3][1])
        output.setNumber(8, trackX[6][1])
        output.setNumber(9, trackX[9][1])
        output.setNumber(12, 1) -- 単一トラックID
        output.setNumber(32, track.epsilon)

        local trkLocal = globalToLocalCoords({ outputX, outputY, outputZ }, ownGlobalPos, ownOrientation)
        local trkAngle = localCoordsToLocalAngle(trkLocal)
        radarManualSweepX = trkAngle.azimuth / PI2
        radarManualSweepY = trkAngle.elevation / PI2

        output.setBool(1, true)
        output.setNumber(10, radarManualSweepX)
        output.setNumber(11, radarManualSweepY)
    else
        -- トラック不在時：データリンク位置を出力
        local dlLocalPos = globalToLocalCoords(dataLinkGlobalPos, ownGlobalPos, ownOrientation)
        local dlLocalAngle = localCoordsToLocalAngle(dlLocalPos)
        radarManualSweepX = dlLocalAngle.azimuth / PI2
        radarManualSweepY = dlLocalAngle.elevation / PI2
        output.setNumber(1, dataLinkGlobalPos[1])
        output.setNumber(2, dataLinkGlobalPos[2])
        output.setNumber(3, dataLinkGlobalPos[3])
        output.setNumber(4, dataLinkTargetVelocity[1])
        output.setNumber(5, dataLinkTargetVelocity[2])
        output.setNumber(6, dataLinkTargetVelocity[3])
        for i = 7, 9 do output.setNumber(i, 0) end
        output.setBool(1, false)
        output.setNumber(10, radarManualSweepX)
        output.setNumber(11, radarManualSweepY)
    end

    -- 自機姿勢・位置パススルー
    output.setNumber(13, ownGlobalPos[1])
    output.setNumber(14, ownGlobalPos[2])
    output.setNumber(15, ownGlobalPos[3])
    output.setNumber(16, ownOrientation[1])
    output.setNumber(17, ownOrientation[2])
    output.setNumber(18, ownOrientation[3])
    output.setNumber(19, ownOrientation[4])
end
