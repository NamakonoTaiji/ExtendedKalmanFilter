--[[
================================================================================
SAM Guidance System with KF and Velocity-Vector PN Guidance (v0.8)
================================================================================
機能:
- 中間誘導: データリンク位置とその差分から推定した速度で状態を保持。目標へ単純追尾。
- 終末誘導(ARH): ARH切替時に状態引き継ぎ。レーダー観測値でEKF更新。
                 フィルター出力(位置/速度)と自機速度からLOSレートベクトルを計算しPN誘導。
- 慣性誘導: 最後のEKF状態で等速直線運動予測し単純追尾。
- EKF更新dtは固定(DT=1/60)。慣性誘導予測は経過時間考慮。

変更点 (v0.6 -> v0.8):
- ARHモードのPN誘導を速度ベクトルベースに変更 (v0.2相当のロジック)。
  - 相対位置ベクトルと相対速度ベクトルからLOSレートベクトルを計算。
  - 接近速度とLOSレートベクトルからグローバル加速度指令を計算。
  - グローバル加速度指令をローカル座標系に変換してフィン制御量を決定。
- 自機速度の入力(ch 7,8,9)を再度使用。
- ヘルパー関数 vectorDot, vectorCross を復活。
- 角度ベースPN用の previousLocalAzimuth/Elevation は不要になったため削除。

入力/出力/プロパティ/前提: (v0.6と同様だが、自機速度入力(ch 7,8,9)が必須)
================================================================================
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60 -- EKF更新の時間ステップ (秒)
MAX_RADAR_TARGETS = 6
NUM_STATES = 6
GUIDANCE_SWITCH_DISTANCE = property.getNumber("GuidanceSwitchDistance") or 1800

-- EKF パラメータ
DATA_ASSOCIATION_MAX_DISTANCE = property.getNumber("D_ASSOC_DIST") or 200
PROCESS_NOISE_BASE = property.getNumber("P_BASE") or 1e-3
PROCESS_NOISE_ADAPTIVE_SCALE = property.getNumber("P_ADPT") or 1e+4
PROCESS_NOISE_EPSILON_THRESHOLD = property.getNumber("P_NOISE_EPS_THRS") or 70
PROCESS_NOISE_EPSILON_SLOPE = property.getNumber("P_NOISE_EPS_SLOPE") or 100
PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") or 1.01
R0_DIST_VAR_FACTOR = (0.02 ^ 2) / 12
R0_ANGLE_VAR = ((2e-3 * PI2) ^ 2) / 12
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }
INITIAL_VELOCITY_VARIANCE = (100 ^ 2)

-- PN Guidance パラメータ
NAVIGATION_GAIN = property.getNumber("N") or 3.5 -- 加速度ベースPN用
FIN_GAIN = property.getNumber("FinGain") or 1.0  -- 加速度指令 -> フィン指令値変換ゲイン (要調整)
SIMPLE_TRACK_GAIN = property.getNumber("SimpleTrackGain") or 1.0

-- グローバル変数
trackedTarget = nil
currentTick = 0
isTracking = false
guidanceMode = "Idle"
-- previousLocalAzimuth/Elevation は不要
previousDatalinkTargetGlobal = nil

-- 単位行列
identityMatrix6x6 = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }

--------------------------------------------------------------------------------
-- ベクトル演算ヘルパー関数 (nil チェック削除)
--------------------------------------------------------------------------------
function vectorMagnitude(v)
    x = v[1] or v.x or 0; y = v[2] or v.y or 0; z = v[3] or v.z or 0; return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

function vectorNormalize(v)
    mag = vectorMagnitude(v); if mag < 1e-9 then
        return { 0, 0, 0 }
    else
        x = v[1] or v.x or 0; y = v[2] or v.y or 0; z = v[3] or v.z or 0; return { x / mag, y / mag, z / mag }
    end
end

function vectorAdd(v1, v2)
    x1 = v1[1] or v1.x or 0; y1 = v1[2] or v1.y or 0; z1 = v1[3] or v1.z or 0; x2 = v2[1] or v2.x or 0; y2 = v2[2] or
        v2.y or 0; z2 = v2[3] or v2.z or 0; return { x1 + x2, y1 + y2, z1 + z2 }
end

function vectorSub(v1, v2)
    x1 = v1[1] or v1.x or 0; y1 = v1[2] or v1.y or 0; z1 = v1[3] or v1.z or 0; x2 = v2[1] or v2.x or 0; y2 = v2[2] or
        v2.y or 0; z2 = v2[3] or v2.z or 0; return { x1 - x2, y1 - y2, z1 - z2 }
end

function vectorScalarMul(s, v)
    x = v[1] or v.x or 0; y = v[2] or v.y or 0; z = v[3] or v.z or 0; return { s * x, s * y, s * z }
end

function vectorDot(v1, v2)
    x1 = v1[1] or v1.x or 0; y1 = v1[2] or v1.y or 0; z1 = v1[3] or v1.z or 0; x2 = v2[1] or v2.x or 0; y2 = v2[2] or
        v2.y or 0; z2 = v2[3] or v2.z or 0; return x1 * x2 + y1 * y2 + z1 * z2
end

function vectorCross(v1, v2)
    x1 = v1[1] or v1.x or 0; y1 = v1[2] or v1.y or 0; z1 = v1[3] or v1.z or 0; x2 = v2[1] or v2.x or 0; y2 = v2[2] or
        v2.y or 0; z2 = v2[3] or v2.z or 0; return { y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2 }
end

--------------------------------------------------------------------------------
-- 行列演算ヘルパー関数 (nil チェック削除)
--------------------------------------------------------------------------------
function zeros(rows, cols)
    m = {}; for r = 1, rows do
        m[r] = {}; for c = 1, cols do m[r][c] = 0 end
    end; return m
end

function MatrixCopy(M)
    N = {}; for r, row in ipairs(M) do N[r] = { table.unpack(row) } end; return N
end

function scalar(s, M)
    R = zeros(#M, #M[1]); for r = 1, #M do for c = 1, #M[1] do R[r][c] = M[r][c] * s end end; return R
end

function sum(A, B)
    R = zeros(#A, #A[1]); for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] + B[r][c] end end; return R
end

function sub(A, B)
    R = zeros(#A, #A[1]); for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] - B[r][c] end end; return R
end

function mul(...)
    mats = { ... }; A = mats[1]; R = nil; for i = 2, #mats do
        B = mats[i]; if #A[1] ~= #B then return nil end; R = zeros(#A, #B[1]); for r = 1, #A do
            for c = 1, #B[1] do
                sVal = 0; for k = 1, #B do sVal = sVal + A[r][k] * B[k][c] end; R[r][c] = sVal
            end
        end; A = R
    end; return A
end

function T(M)
    rows = #M; cols = #M[1]; R = zeros(cols, rows); for r = 1, rows do for c = 1, cols do R[c][r] = M[r][c] end end; return
        R
end

function inv(M)
    n = #M; aug = {}; for r = 1, n do
        aug[r] = {}; for c = 1, n do aug[r][c] = M[r][c] end; for c = 1, n do aug[r][n + c] = (r == c) and 1 or 0 end
    end; for r = 1, n do
        pivot = aug[r][r]; if math.abs(pivot) < 1e-12 then return nil end; for c = r, 2 * n do
            aug[r][c] = aug[r][c] /
                pivot
        end; for i = 1, n do
            if i ~= r then
                factor = aug[i][r]; for c = r, 2 * n do aug[i][c] = aug[i][c] - factor * aug[r][c] end
            end
        end
    end; invM = zeros(n, n); for r = 1, n do for c = 1, n do invM[r][c] = aug[r][n + c] end end; return invM
end -- 簡略化のため一部チェック削除

--------------------------------------------------------------------------------
-- クォータニオン演算関数 (nil チェック削除)
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    w1 = q_a[1]; x1 = q_a[2]; y1 = q_a[3]; z1 = q_a[4]; w2 = q_b[1]; x2 = q_b[2]; y2 = q_b[3]; z2 = q_b[4]; w_result = w1 *
        w2 - x1 * x2 - y1 * y2 - z1 * z2; x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2; y_result = w1 * y2 - x1 * z2 +
        y1 * w2 + z1 * x2; z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; return { w_result, x_result, y_result,
        z_result }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    half_roll = roll * 0.5; half_yaw = yaw * 0.5; half_pitch = pitch * 0.5; cr = math.cos(half_roll); sr = math.sin(
        half_roll); cy = math.cos(half_yaw); sy = math.sin(half_yaw); cp = math.cos(half_pitch); sp = math.sin(
        half_pitch); w =
        cr * cy * cp + sr * sy * sp; x = cr * cy * sp - sr * sy * cp; y = cr * sy * cp + sr * cy * sp; z = sr * cy * cp -
        cr * sy * sp; return { w, x, y, z }
end

function rotateVectorByQuaternion(vector, quaternion)
    px = vector[1] or vector.x or 0; py = vector[2] or vector.y or 0; pz = vector[3] or vector.z or 0; p = { 0, px, py,
        pz }; q = quaternion; q_conj = { q[1], -q[2], -q[3], -q[4] }; temp = multiplyQuaternions(q, p); p_prime =
        multiplyQuaternions(temp, q_conj); return { p_prime[2], p_prime[3], p_prime[4] }
end

function rotateVectorByInverseQuaternion(vector, quaternion)
    px = vector[1] or vector.x or 0; py = vector[2] or vector.y or 0; pz = vector[3] or vector.z or 0; p = { 0, px, py,
        pz }; q = quaternion; q_conj = { q[1], -q[2], -q[3], -q[4] }; temp = multiplyQuaternions(q_conj, p); p_prime =
        multiplyQuaternions(temp, q); return { p_prime[2], p_prime[3], p_prime[4] }
end

--------------------------------------------------------------------------------
-- 座標変換関数 (nil チェック削除)
--------------------------------------------------------------------------------
function localToGlobalCoords(localPos, ownGlobalPos, ownOrientation)
    globalRelativeVector = rotateVectorByQuaternion(localPos, ownOrientation); globalX = globalRelativeVector[1] +
        ownGlobalPos.x; globalY = globalRelativeVector[2] + ownGlobalPos.y; globalZ = globalRelativeVector[3] +
        ownGlobalPos.z; return { x = globalX, y = globalY, z = globalZ }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientation)
    gX = globalTargetPos[1] or globalTargetPos.x or 0; gY = globalTargetPos[2] or globalTargetPos.y or 0; gZ =
        globalTargetPos[3] or globalTargetPos.z or 0; oX = ownGlobalPos.x; oY = ownGlobalPos.y; oZ = ownGlobalPos.z; relativeVectorGlobal = {
        gX - oX, gY - oY, gZ - oZ }; localVector = rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientation); return {
        localVector[1], localVector[2], localVector[3] }
end

--------------------------------------------------------------------------------
-- EKF 関連関数 (nil チェック削除)
--------------------------------------------------------------------------------
function getObservationJacobianAndPrediction(stateVector, ownPosition)
    targetX = stateVector[1][1]
    debug.log("targetX: ", tostring(targetX))
    targetY = stateVector[3][1]; targetZ = stateVector[5][1]; relativeX = targetX -
        ownPosition.x; relativeY = targetY - ownPosition.y; relativeZ = targetZ - ownPosition.z; r_sq = relativeX ^ 2 +
        relativeY ^ 2 + relativeZ ^ 2; rh_sq = relativeX ^ 2 + relativeZ ^ 2; if r_sq < 1e-9 then r_sq = 1e-9 end; if rh_sq < 1e-9 then rh_sq = 1e-9 end; r =
        math.sqrt(r_sq); rh = math.sqrt(rh_sq); predictedDistance = r; asin_arg = math.max(-1.0,
        math.min(1.0, relativeY / r)); predictedElevation =
        math.asin(asin_arg); predictedAzimuth = math.atan(relativeX, relativeZ); h = { { predictedDistance }, { predictedElevation }, { predictedAzimuth } }; H =
        zeros(3, NUM_STATES); H[1][1] = relativeX / r; H[1][3] = relativeY / r; H[1][5] = relativeZ / r; H[2][1] = (relativeX * -relativeY) /
        (r_sq * rh); H[2][3] = rh / r_sq; H[2][5] = (relativeZ * -relativeY) / (r_sq * rh); H[3][1] = relativeZ / rh_sq; H[3][3] = 0; H[3][5] = -
        relativeX / rh_sq; return H, h
end

function calculateAngleDifference(angle1, angle2)
    diff = angle2 - angle1; while diff <= -PI do diff = diff + PI2 end; while diff > PI do diff = diff - PI2 end; return
        diff
end

function extendedKalmanFilterUpdate(currentTarget, observation, ownPosition, R_matrix)
    debug.log("currentTargetX: " .. tostring(currentTarget))
    stateVector = currentTarget.X; covariance = currentTarget.P; lastEpsilon = currentTarget.epsilon; dt_sec = DT; F =
        MatrixCopy(identityMatrix6x6); F[1][2] = dt_sec; F[3][4] = dt_sec; F[5][6] = dt_sec; X_predicted = mul(F,
        stateVector); dt2 =
        dt_sec * dt_sec; dt3 = dt2 * dt_sec / 2; dt4 = dt3 * dt_sec / 2; Q_base = { { dt4, dt3, 0, 0, 0, 0 }, { dt3, dt2, 0, 0, 0, 0 }, { 0, 0, dt4, dt3, 0, 0 }, { 0, 0, dt3, dt2, 0, 0 }, { 0, 0, 0, 0, dt4, dt3 }, { 0, 0, 0, 0, dt3, dt2 } }; adaptiveFactor =
        PROCESS_NOISE_BASE +
        PROCESS_NOISE_ADAPTIVE_SCALE /
        (1 + math.exp(-(lastEpsilon - PROCESS_NOISE_EPSILON_THRESHOLD) * PROCESS_NOISE_EPSILON_SLOPE)); Q_adapted =
        scalar(
            adaptiveFactor, Q_base); uncertaintyIncreaseFactor = PREDICTION_UNCERTAINTY_FACTOR_BASE ^ 2; P_pred_term1 =
        mul(
            F,
            covariance, T(F)); P_predicted = sum(scalar(uncertaintyIncreaseFactor, P_pred_term1), Q_adapted); Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }; H, h =
        getObservationJacobianAndPrediction(X_predicted, ownPosition); R = R_matrix; Y = zeros(3, 1); Y[1][1] = Z[1][1] -
        h[1][1]; Y[2][1] = calculateAngleDifference(h[2][1], Z[2][1]); Y[3][1] = calculateAngleDifference(h[3][1],
        Z[3][1]); S_term1 =
        mul(H, P_predicted, T(H)); S = sum(S_term1, R); S_inv = inv(S); if S_inv == nil then
        return stateVector, covariance,
            lastEpsilon, false
    end; K_term1 = mul(P_predicted, T(H)); K = mul(K_term1, S_inv); KY_term = mul(K, Y); X_updated =
        sum(X_predicted, KY_term); KH_term = mul(K, H); I_minus_KH = sub(identityMatrix6x6, KH_term); P_up_term1 = mul(
        I_minus_KH, P_predicted, T(I_minus_KH)); P_up_term2 = mul(K, R, T(K)); P_updated = sum(P_up_term1, P_up_term2); epsilon_matrix =
        mul(T(Y), S_inv, Y); epsilon = 1; if epsilon_matrix and epsilon_matrix[1] and epsilon_matrix[1][1] then
        epsilon =
            epsilon_matrix[1][1]
    end; return X_updated, P_updated, epsilon, true
end

function initializeOrUpdateFilterState(position, estimatedVelocity, tick)
    vel = estimatedVelocity or { 0, 0, 0 }; X_state = { { position.x }, { vel[1] }, { position.y }, { vel[2] }, { position.z }, { vel[3] } }; P_state =
        zeros(NUM_STATES, NUM_STATES); temp_dist = 1000; R_init = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE); R_init[1][1] =
        R_init[1][1] * (temp_dist ^ 2); pos_variance_scale = 10; P_state[1][1] = (R_init[3][3] + R_init[1][1]) *
        pos_variance_scale; P_state[3][3] = (R_init[2][2] + R_init[1][1]) * pos_variance_scale; P_state[5][5] = (R_init[3][3] + R_init[1][1]) *
        pos_variance_scale; vel_variance = estimatedVelocity and (50 ^ 2) or INITIAL_VELOCITY_VARIANCE; P_state[2][2] =
        vel_variance; P_state[4][4] = vel_variance; P_state[6][6] = vel_variance; return {
        X = X_state,
        P = P_state,
        epsilon = 1.0,
        lastTick =
            tick
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick) (nil チェック削除)
--------------------------------------------------------------------------------
function onTick()
    currentTick = currentTick + 1

    -- 1. 入力読み込み
    ownGlobalPos = { x = input.getNumber(4), y = input.getNumber(8), z = input.getNumber(12) }
    ownGlobalVel = { input.getNumber(7), input.getNumber(8), input.getNumber(9) }
    ownPitch = input.getNumber(16); ownYaw = input.getNumber(20); ownRoll = input.getNumber(24);
    ownOrientation = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)
    datalinkTargetGlobal = { x = input.getNumber(29), y = input.getNumber(30), z = input.getNumber(31) }
    if datalinkTargetGlobal.x == 0 and datalinkTargetGlobal.y == 0 and datalinkTargetGlobal.z == 0 then isDatalinkValid = false else isDatalinkValid = true end -- 0,0,0 判定に変更

    -- レーダー観測値の処理
    currentObservations = {}
    isRadarDetecting = false
    for i = 1, MAX_RADAR_TARGETS do
        baseChannel = (i - 1) * 4 + 1
        dist = input.getNumber(baseChannel)
        aziRot = input.getNumber(baseChannel + 1)
        eleRot = input.getNumber(baseChannel + 2)
        if dist > 0 then -- nil チェック削除 (入力は 0 になる想定)
            isRadarDetecting = true
            aziRad = aziRot * PI2; eleRad = eleRot * PI2
            localX = dist * math.cos(eleRad) * math.sin(aziRad); localY = dist * math.sin(eleRad); localZ = dist *
                math.cos(eleRad) * math.cos(aziRad)
            targetLocalPos = { localX, localY, localZ }
            targetGlobal = localToGlobalCoords(targetLocalPos, ownGlobalPos, ownOrientation)
            -- localToGlobalCoords が nil を返す可能性は低いが、そのまま進む
            distCheck = vectorMagnitude(vectorSub({ targetGlobal.x, targetGlobal.y, targetGlobal.z },
                { ownGlobalPos.x, ownGlobalPos.y, ownGlobalPos.z }))
            if distCheck > 1e-6 then
                globalElevation = math.asin(math.max(-1.0, math.min(1.0, (targetGlobal.y - ownGlobalPos.y) / distCheck)))
                globalAzimuth = math.atan(targetGlobal.x - ownGlobalPos.x, targetGlobal.z - ownGlobalPos.z)
                table.insert(currentObservations,
                    {
                        distance = dist,
                        azimuth = globalAzimuth,
                        elevation = globalElevation,
                        globalX = targetGlobal.x,
                        globalY =
                            targetGlobal.y,
                        globalZ = targetGlobal.z
                    })
            end
        end
    end

    -- 2. 誘導モード決定
    targetDistance = 0
    currentDatalinkVector = { datalinkTargetGlobal.x, datalinkTargetGlobal.y, datalinkTargetGlobal.z }
    ownPosVector = { ownGlobalPos.x, ownGlobalPos.y, ownGlobalPos.z }
    if isDatalinkValid then
        targetDistance = vectorMagnitude(vectorSub(currentDatalinkVector, ownPosVector))
    elseif trackedTarget ~= nil then
        targetPosVec = { trackedTarget.X[1][1], trackedTarget.X[3][1], trackedTarget.X[5][1] }; targetDistance =
            vectorMagnitude(vectorSub(targetPosVec, ownPosVector))
    end
    previousGuidanceMode = guidanceMode
    if not isDatalinkValid then
        if trackedTarget ~= nil then guidanceMode = "Inertial" else guidanceMode = "Idle" end
    elseif targetDistance >= GUIDANCE_SWITCH_DISTANCE then
        guidanceMode = "MidCourse"
    else
        guidanceMode = "ARH"
    end

    -- 3. 状態更新 / フィルター起動
    isTracking = false
    if guidanceMode == "MidCourse" then
        estimatedVel = nil
        if previousDatalinkTargetGlobal ~= nil and currentTick > 1 then -- isDatalinkValid は上でチェック済み
            prevDatalinkVector = { previousDatalinkTargetGlobal.x, previousDatalinkTargetGlobal.y,
                previousDatalinkTargetGlobal.z }
            estimatedVel = vectorScalarMul(1.0 / DT, vectorSub(currentDatalinkVector, prevDatalinkVector))
        end
        trackedTarget = initializeOrUpdateFilterState(datalinkTargetGlobal, estimatedVel, currentTick)
        if trackedTarget == nil or trackedTarget.X == nil or trackedTarget.X[1] == nil then
            debug.log("initFilterState FAILED")
        end
        isTracking = false
        if trackedTarget == nil then guidanceMode = "Idle" end
    elseif guidanceMode == "ARH" then
        associatedObservation = nil
        if #currentObservations > 0 and isDatalinkValid then
            minDistanceSq = DATA_ASSOCIATION_MAX_DISTANCE ^ 2; bestMatchIndex = -1
            for i, obs in ipairs(currentObservations) do
                dx = obs.globalX - datalinkTargetGlobal.x; dy = obs.globalY - datalinkTargetGlobal.y; dz = obs.globalZ -
                    datalinkTargetGlobal.z
                distanceSq = dx ^ 2 + dy ^ 2 + dz ^ 2
                if distanceSq < minDistanceSq then
                    minDistanceSq = distanceSq; bestMatchIndex = i
                end
            end
            if bestMatchIndex ~= -1 then associatedObservation = currentObservations[bestMatchIndex] end
        end
        if associatedObservation then
            if (previousGuidanceMode == "MidCourse" or previousGuidanceMode == "Inertial") and trackedTarget ~= nil then
                currentVel = { trackedTarget.X[2][1], trackedTarget.X[4][1], trackedTarget.X[6][1] }
                trackedTarget = initializeOrUpdateFilterState(associatedObservation, currentVel, currentTick)
                if trackedTarget == nil or trackedTarget.X == nil or trackedTarget.X[1] == nil then
                    debug.log("initFilterState FAILED")
                end
            elseif trackedTarget == nil then
                trackedTarget = initializeOrUpdateFilterState(associatedObservation, nil, currentTick)
                if trackedTarget == nil or trackedTarget.X == nil or trackedTarget.X[1] == nil then
                    debug.log("initFilterState FAILED")
                end
            end
            if trackedTarget ~= nil then
                R_matrix_arh = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
                R_matrix_arh[1][1] = R_matrix_arh[1][1] * (associatedObservation.distance ^ 2)
                X_up, P_up, eps_up, success = extendedKalmanFilterUpdate(trackedTarget, associatedObservation,
                    ownGlobalPos, R_matrix_arh)
                if success then
                    trackedTarget.X = X_up; trackedTarget.P = P_up; trackedTarget.epsilon = eps_up; trackedTarget.lastTick =
                        currentTick; isTracking = true
                else
                    isTracking = false; trackedTarget = nil
                end
            else
                isTracking = false; trackedTarget = nil
            end
        else
            isTracking = false; trackedTarget = nil
        end
    elseif guidanceMode == "Inertial" then
        if trackedTarget ~= nil then
            dt_pred_ticks = currentTick - trackedTarget.lastTick
            if dt_pred_ticks < 0 then dt_pred_ticks = 0 end
            dt_pred_sec = dt_pred_ticks * DT
            F_pred = MatrixCopy(identityMatrix6x6)
            F_pred[1][2] = dt_pred_sec; F_pred[3][4] = dt_pred_sec; F_pred[5][6] = dt_pred_sec
            predictedX = mul(F_pred, trackedTarget.X)
            if predictedX then trackedTarget.X = predictedX else trackedTarget = nil end
            isTracking = false
        else
            trackedTarget = nil
        end
    else
        trackedTarget = nil; isTracking = false
    end

    -- 4. 誘導計算とフィン制御出力
    yawControl = 0; pitchControl = 0; targetPos = nil
    if trackedTarget ~= nil then
        targetPos = { trackedTarget.X[1][1], trackedTarget.X[3][1], trackedTarget.X[5][1] }
        targetPosVecForLocal = { targetPos[1], targetPos[2], targetPos[3] }
        targetLocal = globalToLocalCoords(targetPosVecForLocal, ownGlobalPos, ownOrientation)
        -- targetLocal が nil でないか確認 (globalToLocalCoords が nil を返す可能性)
        if targetLocal ~= nil then
            currentLocalAzimuth = math.atan(targetLocal[1], targetLocal[3])
            horizontalDistance = math.sqrt(targetLocal[1] ^ 2 + targetLocal[3] ^ 2)
            if horizontalDistance > 1e-6 then
                currentLocalElevation = math.atan(targetLocal[2], horizontalDistance)
            else
                if targetLocal[2] > 0 then
                    currentLocalElevation = PI / 2
                elseif targetLocal[2] < 0 then
                    currentLocalElevation = -
                        PI / 2
                else
                    currentLocalElevation = 0
                end
            end

            if guidanceMode == "MidCourse" or guidanceMode == "Inertial" then
                yawControl = currentLocalAzimuth * SIMPLE_TRACK_GAIN
                pitchControl = currentLocalElevation * SIMPLE_TRACK_GAIN
            elseif guidanceMode == "ARH" then
                if isTracking then -- isTracking が true (EKF更新成功) の場合のみ
                    targetVel = { trackedTarget.X[2][1], trackedTarget.X[4][1], trackedTarget.X[6][1] }
                    ownVelVector = { ownGlobalVel[1], ownGlobalVel[2], ownGlobalVel[3] }
                    R_vec = vectorSub(targetPos, ownPosVector)
                    V_vec = vectorSub(targetVel, ownVelVector)
                    R_mag = vectorMagnitude(R_vec)
                    if R_mag > 1e-6 then
                        Vc = -vectorDot(R_vec, V_vec) / R_mag
                        R_cross_V = vectorCross(R_vec, V_vec)
                        LOS_Rate_vector = vectorScalarMul(1.0 / (R_mag ^ 2), R_cross_V)
                        Accel_cmd_global = vectorScalarMul(NAVIGATION_GAIN * Vc, LOS_Rate_vector)
                        Accel_cmd_local = rotateVectorByInverseQuaternion(Accel_cmd_global, ownOrientation)
                        -- nil チェック削除（パフォーマンス優先、エラー時はLuaエラーになる）
                        yawControl = Accel_cmd_local[1] * FIN_GAIN
                        pitchControl = Accel_cmd_local[2] * FIN_GAIN
                    end
                end
            end
            maxControl = 1.0
            yawControl = math.max(-maxControl, math.min(maxControl, yawControl))
            pitchControl = math.max(-maxControl, math.min(maxControl, pitchControl))
        end
    end

    -- 5. 出力処理
    output.setBool(1, isTracking)
    output.setNumber(1, yawControl or 0)
    output.setNumber(2, pitchControl or 0)
    debug.log("GuidanceMode:" .. guidanceMode)
    -- 6. 今回の状態を次回のために保存
    if isDatalinkValid then previousDatalinkTargetGlobal = datalinkTargetGlobal else previousDatalinkTargetGlobal = nil end
end
