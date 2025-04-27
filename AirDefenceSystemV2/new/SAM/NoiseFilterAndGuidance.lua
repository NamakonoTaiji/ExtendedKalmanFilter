--[[
================================================================================
SAM Guidance System with KF, PN, MidCourse, ARH, Inertial Modes (v0.6)
================================================================================
機能:
- 中間誘導: データリンク位置とその差分から推定した速度で状態を保持。目標へ単純追尾。
- 終末誘導(ARH): ARH切替時に中間誘導の状態(位置と推定速度)を引き継ぎEKF起動。
                 レーダー観測値でEKF更新し角度ベースPN誘導。
- 慣性誘導: データリンクロスト時、最後のフィルター状態で等速直線運動予測し単純追尾。
- EKF更新時のdtは固定(DT=1/60)。慣性誘導では経過時間を考慮。
- フォーマットを修正し、可読性を向上。

変更点 (v0.5 -> v0.6):
- 中間誘導中に位置デルタから速度を推定し、trackedTarget に保持するロジックを追加。
- ARH切り替え時に、中間誘導で保持していた trackedTarget を初期状態としてEKFを開始するよう修正。
- EKF更新関数 extendedKalmanFilterUpdate から時間差(dt_ticks, dt_sec)の計算を削除し、固定値 DT を使用。
- コード全体のフォーマットを修正。

入力/出力/プロパティ/前提: (v0.4/v0.5 と同様)
================================================================================
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60 -- EKF更新の時間ステップ (秒)
MAX_RADAR_TARGETS = 6
NUM_STATES = 6
GUIDANCE_SWITCH_DISTANCE = property.getNumber("GuidanceSwitchDistance") or 1800 -- 中間誘導からARH切替の距離 (m)

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
-- DATALINK_OBSERVATION_NOISE は MidCourseでEKFを使わないため不要に
INITIAL_VELOCITY_VARIANCE = (100 ^ 2) -- EKF初期化時の速度分散

-- PN Guidance パラメータ
NAVIGATION_GAIN = property.getNumber("N") or 3.5
FIN_GAIN = property.getNumber("FinGain") or 1
SIMPLE_TRACK_GAIN = property.getNumber("SimpleTrackGain") or 1.0

-- グローバル変数
trackedTarget = nil        -- { X, P, epsilon, lastTick } EKFの状態 / MidCourse時の状態
currentTick = 0
isTracking = false         -- EKFがレーダー観測で有効に更新されているか (ARH中のみtrueになりうる)
guidanceMode = "MidCourse" -- "ARH", "MidCourse", "Inertial"
previousLocalAzimuth = nil
previousLocalElevation = nil
previousDatalinkTargetGlobal = { 0, 0, 0 }    -- 中間誘導での速度推定用
isInitPreviousDatalinkTargetGlobal = true     -- データリンク座標初期化フラグ
datalinkTargetSpeed = { x = 0, y = 0, z = 0 } -- データリンク座標のデルタから計算した目標の速度
targetPosition = { 0, 0, 0 }
prevTargetGlobalAngle = { azimuth = 0, elevation = 0 }
yawControl, pitchControl = 0, 0
-- 単位行列
identityMatrix6x6 = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }

--------------------------------------------------------------------------------
-- ベクトル演算ヘルパー関数 (フォーマット修正)
--------------------------------------------------------------------------------
-- ベクトルの長さを計算します。
function vectorMagnitude(v)
    x = v[1] or v.x
    y = v[2] or v.y
    z = v[3] or v.z
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

-- ベクトルを正規化します。
function vectorNormalize(v)
    mag = vectorMagnitude(v)
    if mag < 1e-9 then
        return { 0, 0, 0 }
    else
        x = v[1] or v.x
        y = v[2] or v.y
        z = v[3] or v.z
        return { x / mag, y / mag, z / mag }
    end
end

-- ベクトルの足し算
function vectorAdd(v1, v2)
    x1 = v1[1] or v1.x; y1 = v1[2] or v1.y; z1 = v1[3] or v1.z
    x2 = v2[1] or v2.x; y2 = v2[2] or v2.y; z2 = v2[3] or v2.z
    return { x1 + x2, y1 + y2, z1 + z2 }
end

-- ベクトルの引き算
function vectorSub(v1, v2)
    x1 = v1[1] or v1.x; y1 = v1[2] or v1.y; z1 = v1[3] or v1.z
    x2 = v2[1] or v2.x; y2 = v2[2] or v2.y; z2 = v2[3] or v2.z
    return { x1 - x2, y1 - y2, z1 - z2 }
end

-- ベクトルをスカラー倍
function vectorScalarMul(s, v)
    x = v[1] or v.x
    y = v[2] or v.y
    z = v[3] or v.z
    return { s * x, s * y, s * z }
end

--------------------------------------------------------------------------------
-- 行列演算ヘルパー関数 (フォーマット修正)
--------------------------------------------------------------------------------
function zeros(rows, cols)
    m = {}
    for r = 1, rows do
        m[r] = {}
        for c = 1, cols do
            m[r][c] = 0
        end
    end
    return m
end

function MatrixCopy(M)
    N = {}
    for r, row in ipairs(M) do
        N[r] = { table.unpack(row) }
    end
    return N
end

function scalar(s, M)
    R = zeros(#M, #M[1])
    for r = 1, #M do
        for c = 1, #M[1] do
            R[r][c] = M[r][c] * s
        end
    end
    return R
end

function sum(A, B)
    R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do
            R[r][c] = A[r][c] + B[r][c]
        end
    end
    return R
end

function sub(A, B)
    R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do
            R[r][c] = A[r][c] - B[r][c]
        end
    end
    return R
end

function mul(...)
    mats = { ... }
    A = mats[1]
    R = nil
    for i = 2, #mats do
        B = mats[i]
        -- 基本的なnilチェックを追加
        if A == nil or B == nil or #A < 1 or #B < 1 or #A[1] == 0 or #B[1] == 0 or #A[1] ~= #B then
            return nil
        end
        R = zeros(#A, #B[1])
        for r = 1, #A do
            for c = 1, #B[1] do
                sVal = 0
                for k = 1, #B do
                    -- 要素がnilでないかチェック (より安全に)
                    if A[r] == nil or B[k] == nil or A[r][k] == nil or B[k][c] == nil then return nil end
                    sVal = sVal + A[r][k] * B[k][c]
                end
                R[r][c] = sVal
            end
        end
        A = R
    end
    return A
end

function T(M)
    if M == nil or #M < 1 or M[1] == nil then return nil end -- チェック追加
    rows = #M
    cols = #M[1]
    R = zeros(cols, rows)
    for r = 1, rows do
        if M[r] == nil then return nil end -- チェック追加
        for c = 1, cols do
            R[c][r] = M[r][c]
        end
    end
    return R
end

function inv(M)
    if M == nil or #M == 0 or type(M[1]) ~= "table" or #M[1] == 0 then return nil end
    n = #M
    if n ~= #M[1] then return nil end
    aug = {}
    for r = 1, n do
        aug[r] = {}
        if M[r] == nil then return nil end
        for c = 1, n do
            v = M[r][c]
            if v == nil or v ~= v or v == math.huge or v == -math.huge then return nil end
            aug[r][c] = v
        end
        for c = 1, n do
            aug[r][n + c] = (r == c) and 1 or 0
        end
    end
    for r = 1, n do
        pivot = aug[r][r]
        if pivot == nil or math.abs(pivot) < 1e-12 then return nil end
        for c = r, 2 * n do
            if aug[r][c] == nil then return nil end
            aug[r][c] = aug[r][c] / pivot
        end
        for i = 1, n do
            if i ~= r then
                factor = aug[i][r]
                if factor == nil then return nil end
                for c = r, 2 * n do
                    if aug[i][c] == nil or aug[r][c] == nil then return nil end
                    aug[i][c] = aug[i][c] - factor * aug[r][c]
                    if aug[i][c] ~= aug[i][c] or aug[i][c] == math.huge or aug[i][c] == -math.huge then return nil end
                end
            end
        end
    end
    invM = zeros(n, n)
    for r = 1, n do
        for c = 1, n do
            value = aug[r][n + c]
            if value == nil or value ~= value or value == math.huge or value == -math.huge then
                return nil
            else
                invM[r][c] =
                    value
            end
        end
    end
    return invM
end

--------------------------------------------------------------------------------
-- クォータニオン演算関数 (フォーマット修正)
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    w1 = q_a[1]; x1 = q_a[2]; y1 = q_a[3]; z1 = q_a[4]
    w2 = q_b[1]; x2 = q_b[2]; y2 = q_b[3]; z2 = q_b[4]
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    half_roll = roll * 0.5; half_yaw = yaw * 0.5; half_pitch = pitch * 0.5
    cr = math.cos(half_roll); sr = math.sin(half_roll)
    cy = math.cos(half_yaw); sy = math.sin(half_yaw)
    cp = math.cos(half_pitch); sp = math.sin(half_pitch)
    w = cr * cy * cp + sr * sy * sp
    x = cr * cy * sp - sr * sy * cp
    y = cr * sy * cp + sr * cy * sp
    z = sr * cy * cp - cr * sy * sp
    return { w, x, y, z }
end

function rotateVectorByQuaternion(vector, quaternion)
    px = vector[1] or vector.x or 0; py = vector[2] or vector.y or 0; pz = vector[3] or vector.z or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp = multiplyQuaternions(q, p)
    p_prime = multiplyQuaternions(temp, q_conj)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

function rotateVectorByInverseQuaternion(vector, quaternion)
    px = vector[1] or vector.x or 0; py = vector[2] or vector.y or 0; pz = vector[3] or vector.z or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp = multiplyQuaternions(q_conj, p)
    p_prime = multiplyQuaternions(temp, q)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

--------------------------------------------------------------------------------
-- 座標変換関数 (フォーマット修正)
--------------------------------------------------------------------------------
function localToGlobalCoords(localPos, ownGlobalPos, ownOrientation)
    globalRelativeVector = rotateVectorByQuaternion(localPos, ownOrientation)
    globalX = globalRelativeVector[1] + ownGlobalPos.x
    globalY = globalRelativeVector[2] + ownGlobalPos.y
    globalZ = globalRelativeVector[3] + ownGlobalPos.z
    return { globalX, globalY, globalZ }
end

function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientation)
    relativeVectorGlobal = { globalTargetPos[1] - ownGlobalPos.x, globalTargetPos[2] - ownGlobalPos.y, globalTargetPos
    [3] -
    ownGlobalPos.z }
    localVector = rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientation)
    return { localVector[1], localVector[2], localVector[3] }
end

--------------------------------------------------------------------------------
-- EKF 関連関数 (dt=DT固定, フォーマット修正)
--------------------------------------------------------------------------------
function getObservationJacobianAndPrediction(stateVector, ownPosition)
    targetX = stateVector[1][1]; targetY = stateVector[3][1]; targetZ = stateVector[5][1]
    relativeX = targetX - ownPosition.x; relativeY = targetY - ownPosition.y; relativeZ = targetZ - ownPosition.z
    r_sq = relativeX ^ 2 + relativeY ^ 2 + relativeZ ^ 2; rh_sq = relativeX ^ 2 + relativeZ ^ 2
    if r_sq < 1e-9 then r_sq = 1e-9 end; if rh_sq < 1e-9 then rh_sq = 1e-9 end
    r = math.sqrt(r_sq); rh = math.sqrt(rh_sq)
    predictedDistance = r
    asin_arg = math.max(-1.0, math.min(1.0, relativeY / r)); predictedElevation = math.asin(asin_arg)
    predictedAzimuth = math.atan(relativeX, relativeZ)
    h = { { predictedDistance }, { predictedElevation }, { predictedAzimuth } }
    H = zeros(3, NUM_STATES)
    H[1][1] = relativeX / r; H[1][3] = relativeY / r; H[1][5] = relativeZ / r
    H[2][1] = (relativeX * -relativeY) / (r_sq * rh); H[2][3] = rh / r_sq; H[2][5] = (relativeZ * -relativeY) /
        (r_sq * rh)
    H[3][1] = relativeZ / rh_sq; H[3][3] = 0; H[3][5] = -relativeX / rh_sq
    return H, h
end

function calculateAngleDifference(angle1, angle2)
    diff = angle2 - angle1
    while diff <= -PI do diff = diff + PI2 end
    while diff > PI do diff = diff - PI2 end
    return diff
end

--- EKFの予測・更新ステップを実行します。(dtは固定DT=1/60を使用)
function extendedKalmanFilterUpdate(currentTarget, observation, ownPosition, R_matrix)
    stateVector = currentTarget.X; covariance = currentTarget.P; lastEpsilon = currentTarget.epsilon
    -- dt_ticks, lastTick は不要
    dt_sec = DT -- 固定値を使用
    F = MatrixCopy(identityMatrix6x6); F[1][2] = dt_sec; F[3][4] = dt_sec; F[5][6] = dt_sec;
    X_predicted = mul(F, stateVector)
    if X_predicted == nil then return stateVector, covariance, lastEpsilon, false end

    dt2 = dt_sec * dt_sec; dt3 = dt2 * dt_sec / 2; dt4 = dt3 * dt_sec / 2; Q_base = { { dt4, dt3, 0, 0, 0, 0 }, { dt3, dt2, 0, 0, 0, 0 }, { 0, 0, dt4, dt3, 0, 0 }, { 0, 0, dt3, dt2, 0, 0 }, { 0, 0, 0, 0, dt4, dt3 }, { 0, 0, 0, 0, dt3, dt2 } }
    adaptiveFactor = PROCESS_NOISE_BASE +
        PROCESS_NOISE_ADAPTIVE_SCALE /
        (1 + math.exp(-(lastEpsilon - PROCESS_NOISE_EPSILON_THRESHOLD) * PROCESS_NOISE_EPSILON_SLOPE))
    Q_adapted = scalar(adaptiveFactor, Q_base)
    -- dt_ticks = 1 固定なので常に PRED_UNCERTAINTY_FACTOR_BASE^2
    uncertaintyIncreaseFactor = PREDICTION_UNCERTAINTY_FACTOR_BASE ^ 2
    P_pred_term1 = mul(F, covariance, T(F))
    if P_pred_term1 == nil then return stateVector, covariance, lastEpsilon, false end
    P_predicted = sum(scalar(uncertaintyIncreaseFactor, P_pred_term1), Q_adapted)

    Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }
    H, h = getObservationJacobianAndPrediction(X_predicted, ownPosition); if H == nil or h == nil then
        return stateVector,
            covariance, lastEpsilon, false
    end
    R = R_matrix
    Y = zeros(3, 1); Y[1][1] = Z[1][1] - h[1][1]; Y[2][1] = calculateAngleDifference(h[2][1], Z[2][1]); Y[3][1] =
        calculateAngleDifference(h[3][1], Z[3][1])
    S_term1 = mul(H, P_predicted, T(H))
    if S_term1 == nil then return stateVector, covariance, lastEpsilon, false end
    S = sum(S_term1, R); S_inv = inv(S); if S_inv == nil then return stateVector, covariance, lastEpsilon, false end
    K_term1 = mul(P_predicted, T(H))
    if K_term1 == nil then return stateVector, covariance, lastEpsilon, false end
    K = mul(K_term1, S_inv)
    if K == nil then return stateVector, covariance, lastEpsilon, false end
    X_updated = sum(X_predicted, mul(K, Y)); if X_updated == nil then return stateVector, covariance, lastEpsilon, false end

    KH_term = mul(K, H)
    if KH_term == nil then return stateVector, covariance, lastEpsilon, false end
    I_minus_KH = sub(identityMatrix6x6, KH_term)
    P_up_term1 = mul(I_minus_KH, P_predicted, T(I_minus_KH))
    P_up_term2 = mul(K, R, T(K))
    if P_up_term1 == nil or P_up_term2 == nil then return stateVector, covariance, lastEpsilon, false end
    P_updated = sum(P_up_term1, P_up_term2)

    epsilon_matrix = mul(T(Y), S_inv, Y); epsilon = 1; if epsilon_matrix and epsilon_matrix[1] and epsilon_matrix[1][1] then
        epsilon =
            epsilon_matrix[1][1]
    end
    return X_updated, P_updated, epsilon, true -- lastTickは返さない, successフラグを返す
end

--- フィルターを初期化または速度推定値で更新します。
function initializeOrUpdateFilterState(position, estimatedVelocity, tick)
    vel = estimatedVelocity or { 0, 0, 0 }
    X_state = { { position.x }, { vel[1] }, { position.y }, { vel[2] }, { position.z }, { vel[3] } }

    P_state = zeros(NUM_STATES, NUM_STATES)
    temp_dist = 1000
    R_init = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R_init[1][1] = R_init[1][1] * (temp_dist ^ 2)
    pos_variance_scale = 10
    P_state[1][1] = (R_init[3][3] + R_init[1][1]) * pos_variance_scale
    P_state[3][3] = (R_init[2][2] + R_init[1][1]) * pos_variance_scale
    P_state[5][5] = (R_init[3][3] + R_init[1][1]) * pos_variance_scale
    vel_variance = estimatedVelocity and (50 ^ 2) or INITIAL_VELOCITY_VARIANCE -- 推定速度があれば分散を少し小さく
    P_state[2][2] = vel_variance
    P_state[4][4] = vel_variance
    P_state[6][6] = vel_variance

    return { X = X_state, P = P_state, epsilon = 1.0, lastTick = tick }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    debug.log("------------------onTick()-------------------")
    currentTick = currentTick + 1
    isLaunched = input.getNumber(25) ~= 0 -- 25番が1なら発射状態
    -- 発射状態の確認
    if not isLaunched then
        debug.log("SAM is not launched")
        return
    end
    isRadarDetected = input.getBool(1)
    -- 1. 入力読み込み
    ownGlobalPos = { x = input.getNumber(4), y = input.getNumber(8), z = input.getNumber(12) }
    ownPitch = input.getNumber(16); ownYaw = input.getNumber(20); ownRoll = input.getNumber(24);
    ownOrientation = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)
    datalinkTargetGlobal = { x = input.getNumber(29), y = input.getNumber(30), z = input.getNumber(31) }
    isDatalinkValid = (datalinkTargetGlobal.x ~= 0 or datalinkTargetGlobal.y ~= 0 or datalinkTargetGlobal.z ~= 0)
    if isDatalinkValid then
        targetPosition = { datalinkTargetGlobal.x, datalinkTargetGlobal.y, datalinkTargetGlobal.z }
    end

    -- 初回データリンク時は前のデータリンク座標を今のデータリンク座標で初期化
    if isInitPreviousDatalinkTargetGlobal then
        if isDatalinkValid then
            previousDatalinkTargetGlobal = {
                x = datalinkTargetGlobal.x,
                y = datalinkTargetGlobal.y,
                z = datalinkTargetGlobal.z
            }
            isInitPreviousDatalinkTargetGlobal = false
        end
    end

    -- データリンク座標のデルタから慣性誘導用の速度を常に計算
    if isDatalinkValid then
        datalinkTargetSpeed = vectorSub(datalinkTargetGlobal, previousDatalinkTargetGlobal)
        previousDatalinkTargetGlobal = datalinkTargetGlobal
    end

    -- データリンク座標が無効の場合は速度を利用して座標を更新する
    if not isDatalinkValid and isInitPreviousDatalinkTargetGlobal == false then
        targetPosition = vectorAdd(targetPosition, datalinkTargetSpeed)
    end

    -- レーダー観測値の処理
    currentObservations = {}
    for i = 1, MAX_RADAR_TARGETS do
        local baseChannel = (i - 1) * 4 + 1
        local dist = input.getNumber(baseChannel)
        if dist > 0 then
            local aziRad = input.getNumber(baseChannel + 1) * PI2
            local eleRad = input.getNumber(baseChannel + 2) * PI2
            local localX = dist * math.cos(eleRad) * math.sin(aziRad)
            local localY = dist * math.sin(eleRad)
            local localZ = dist * math.cos(eleRad) * math.cos(aziRad)
            local targetLocalPos = { localX, localY, localZ }
            local targetGlobal = localToGlobalCoords(targetLocalPos, ownGlobalPos, ownOrientation)
            local globalElevation = math.asin(math.max(-1.0, math.min(1.0, (targetGlobal[2] - ownGlobalPos.y) / dist)))
            local globalAzimuth = math.atan(targetGlobal[1] - ownGlobalPos.x, targetGlobal[3] - ownGlobalPos.z)
            -- obsTick は EKF 更新関数内で使われなくなったので不要
            table.insert(currentObservations,
                {
                    distance = dist,
                    azimuth = globalAzimuth,
                    elevation = globalElevation,
                    globalX = targetGlobal[1],
                    globalY = targetGlobal[2],
                    globalZ = targetGlobal[3]
                })
        end
    end

    -- 誘導モード選択のために目標との距離計算
    targetDistance = vectorMagnitude(vectorSub(targetPosition, ownGlobalPos))

    -- 目標がレーダー圏内ならARH、それ以外はデータリンク座標があれば中間誘導、なければ慣性誘導
    if targetDistance < GUIDANCE_SWITCH_DISTANCE and isRadarDetected then
        guidanceMode = "ARH"
    elseif isDatalinkValid then
        guidanceMode = "MidCourse"
    else
        guidanceMode = "Inertial"
    end

    debug.log("guidanceMode: " .. guidanceMode)
    currentTargetAngle = {
        azimuth = math.atan(targetPosition[1] - ownGlobalPos.x, targetPosition[3] - ownGlobalPos
            .z),
        elevation = math.asin(math.max(-1.0,
            math.min(1.0, (targetPosition[2] - ownGlobalPos.y) / targetDistance)))
    }

    -- ミッドコース誘導の処理
    if guidanceMode == "MidCourse" then
        -- データリンク座標があれば、EKFを更新する。
        if isDatalinkValid then
            debug.log("previousLocalAngle: " .. prevTargetGlobalAngle.azimuth .. ", " .. prevTargetGlobalAngle.elevation)
            yawControl = calculateAngleDifference(prevTargetGlobalAngle.azimuth, currentTargetAngle.azimuth)
            pitchControl = calculateAngleDifference(prevTargetGlobalAngle.elevation, currentTargetAngle.elevation)
            prevTargetGlobalAngle = currentTargetAngle
            debug.log("currentAngle: " .. currentTargetAngle.azimuth .. ", " .. currentTargetAngle.elevation)
            debug.log("yawControl: " .. yawControl .. ", pitchControl: " .. pitchControl)
        end
    elseif guidanceMode == "ARH" then
        -- ARH誘導の処理
        if isRadarDetected then
            -- レーダー観測値をEKFの観測値としてEKFを更新する。

            local losRateAzimuth = calculateAngleDifference(prevTargetGlobalAngle.azimuth, currentTargetAngle.azimuth)
            local losRateGlobal = calculateAngleDifference(prevTargetGlobalAngle.elevation, currentTargetAngle.elevation)
            prevTargetGlobalAngle = currentTargetAngle
        end
    elseif guidanceMode == "Inertial" then
        -- 慣性誘導の処理
        if isRadarDetected then
            yawControl = calculateAngleDifference(prevTargetGlobalAngle.azimuth, currentTargetAngle.azimuth)
            pitchControl = calculateAngleDifference(prevTargetGlobalAngle.elevation, currentTargetAngle.elevation)
            prevTargetGlobalAngle = currentTargetAngle
        end
    end
    --[[
    -- ARH誘導の処理
    -- データリンク座標をキューとしてレーダー観測値から最も近いものを探す。
    -- 適切なレーダー観測値が見つかった場合EKFを回す

    -- 誘導アルゴリズム
    -- ここでフィンに出力するパラメータを計算する。
    --常に比例航法で行う。
    -- 慣性誘導の場合は、保存しておいた速度情報、位置情報を利用して比例航法を行う。

    -- フィン出力
    -- 2. 誘導モード決定
    targetDistance = 0
    if isDatalinkValid then
        targetDistance = vectorMagnitude(vectorSub(datalinkTargetGlobal, ownGlobalPos))
    elseif trackedTarget ~= nil and trackedTarget.X ~= nil then
        targetPosVec = { trackedTarget.X[1][1], trackedTarget.X[3][1], trackedTarget.X[5][1] }
        targetDistance = vectorMagnitude(vectorSub(targetPosVec, ownGlobalPos))
    end
    previousGuidanceMode = guidanceMode
    if not isDatalinkValid then
        if trackedTarget ~= nil then guidanceMode = "Inertial" else guidanceMode = "Idle" end
    elseif targetDistance >= GUIDANCE_SWITCH_DISTANCE then
        guidanceMode = "MidCourse"
    else
        guidanceMode = "ARH"
    end


    -- 3. 状態更新 / フィルター起動 (モード別処理)
    isTracking = false -- リセット

    if guidanceMode == "MidCourse" then
        estimatedVel = nil
        -- 前回のデータリンク座標があれば速度推定
        if previousDatalinkTargetGlobal ~= nil and isDatalinkValid and currentTick > 1 then
            estimatedVel = vectorScalarMul(1.0 / DT, vectorSub(datalinkTargetGlobal, previousDatalinkTargetGlobal))
        end
        -- フィルター状態をデータリンク位置と推定速度で更新/初期化
        -- ★ trackedTarget が nil でなくても、中間誘導中は常にこれで状態を上書きする
        trackedTarget = initializeOrUpdateFilterState(datalinkTargetGlobal, estimatedVel, currentTick)
        isTracking = false -- EKF更新ではない
    elseif guidanceMode == "ARH" then
        -- データアソシエーション
        associatedObservation = nil
        if #currentObservations > 0 then
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
            -- ★ ARHモード開始時の処理: 中間誘導からの状態引き継ぎ
            if previousGuidanceMode == "MidCourse" and trackedTarget ~= nil then
                -- trackedTarget は中間誘導の最後の状態 (位置と推定速度)
                -- P行列は initializeOrUpdateFilterState で再設定されている
                -- この状態で EKF 更新をかける
                R_matrix_arh = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
                R_matrix_arh[1][1] = R_matrix_arh[1][1] * (associatedObservation.distance ^ 2)
                X_up, P_up, eps_up, success = extendedKalmanFilterUpdate(trackedTarget, associatedObservation,
                    ownGlobalPos, R_matrix_arh)
                if success then
                    trackedTarget.X = X_up; trackedTarget.P = P_up; trackedTarget.epsilon = eps_up; trackedTarget.lastTick =
                        currentTick
                    isTracking = true
                end
            elseif trackedTarget == nil then
                -- 直接ARH開始 or 中間誘導状態なし: レーダー観測で初期化してから更新
                trackedTarget = initializeOrUpdateFilterState(associatedObservation, nil, currentTick)
                if trackedTarget ~= nil then
                    R_matrix_arh = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
                    R_matrix_arh[1][1] = R_matrix_arh[1][1] * (associatedObservation.distance ^ 2)
                    X_up, P_up, eps_up, success = extendedKalmanFilterUpdate(trackedTarget, associatedObservation,
                        ownGlobalPos, R_matrix_arh)
                    if success then
                        trackedTarget.X = X_up; trackedTarget.P = P_up; trackedTarget.epsilon = eps_up; trackedTarget.lastTick =
                            currentTick
                        isTracking = true
                    end
                end
            else
                -- 中間誘導以外から遷移し、かつ trackedTarget が存在する場合 (Inertialから復帰など)
                -- そのまま EKF 更新
                R_matrix_arh = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
                R_matrix_arh[1][1] = R_matrix_arh[1][1] * (associatedObservation.distance ^ 2)
                X_up, P_up, eps_up, success = extendedKalmanFilterUpdate(trackedTarget, associatedObservation,
                    ownGlobalPos, R_matrix_arh)
                if success then
                    trackedTarget.X = X_up; trackedTarget.P = P_up; trackedTarget.epsilon = eps_up; trackedTarget.lastTick =
                        currentTick
                    isTracking = true
                end
            end
        end
        -- isTracking フラグが false のままなら、追跡失敗として trackedTarget をリセット
        if not isTracking then trackedTarget = nil end
    elseif guidanceMode == "Inertial" then
        if trackedTarget ~= nil and trackedTarget.X ~= nil then
            -- 経過時間に基づいて予測ステップを実行
            dt_pred_ticks = currentTick - trackedTarget.lastTick -- 最後にEKF更新したTickからの経過
            if dt_pred_ticks < 0 then dt_pred_ticks = 0 end
            dt_pred_sec = dt_pred_ticks * DT
            F_pred = MatrixCopy(identityMatrix6x6)
            F_pred[1][2] = dt_pred_sec; F_pred[3][4] = dt_pred_sec; F_pred[5][6] = dt_pred_sec
            predictedX = mul(F_pred, trackedTarget.X)
            if predictedX then
                trackedTarget.X = predictedX
            else
                trackedTarget = nil
            end
            -- lastTick は更新しない
            isTracking = false
        else
            trackedTarget = nil
        end
    else -- Idle
        trackedTarget = nil; isTracking = false
    end

    if guidanceMode ~= previousGuidanceMode then
        previousLocalAzimuth = nil; previousLocalElevation = nil
    end

    -- 4. 誘導計算とフィン制御出力
    yawControl = 0; pitchControl = 0; currentLocalAzimuth = nil; currentLocalElevation = nil; targetPos = nil
    if trackedTarget ~= nil and trackedTarget.X ~= nil then
        targetPos = { trackedTarget.X[1][1], trackedTarget.X[3][1], trackedTarget.X[5][1] }
        targetLocal = globalToLocalCoords(targetPos, ownGlobalPos, ownOrientation)
        currentLocalAzimuth = math.atan(targetLocal[1], targetLocal[2])
        horizontalDistance = math.sqrt(targetLocal[1] ^ 2 + targetLocal[2] ^ 2)
        if horizontalDistance > 1e-6 then
            currentLocalElevation = math.atan(targetLocal[3], horizontalDistance)
        else
            if targetLocal[3] > 0 then
                currentLocalElevation = PI / 2
            elseif targetLocal[3] < 0 then
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
            -- EKF更新成功時のみ角度ベースPN誘導
            if isTracking and previousLocalAzimuth ~= nil and previousLocalElevation ~= nil then
                localLosRateYaw = calculateAngleDifference(previousLocalAzimuth, currentLocalAzimuth) / DT
                localLosRatePitch = calculateAngleDifference(previousLocalElevation, currentLocalElevation) / DT
                yawControl = -localLosRateYaw * NAVIGATION_GAIN * FIN_GAIN
                pitchControl = -localLosRatePitch * NAVIGATION_GAIN * FIN_GAIN
            end
        end
        maxControl = 1.0
        yawControl = math.max(-maxControl, math.min(maxControl, yawControl))
        pitchControl = math.max(-maxControl, math.min(maxControl, pitchControl))
    end



    -- 6. 今回の状態を次回のために保存
    --    角度ベースPN誘導に必要な角度情報を保存
    if (guidanceMode == "ARH" and isTracking) and currentLocalAzimuth ~= nil and currentLocalElevation ~= nil then
        previousLocalAzimuth = currentLocalAzimuth
        previousLocalElevation = currentLocalElevation
    else
        previousLocalAzimuth = nil; previousLocalElevation = nil
    endIndex
    -- データリンク座標も保存 (中間誘導での速度推定用)
    if isDatalinkValid then previousDatalinkTargetGlobal = datalinkTargetGlobal else previousDatalinkTargetGlobal = nil end
    -- 5. 出力処理]]
    output.setBool(1, isTracking) -- EKF更新成功フラグ (ARH中のみTrueになりうる)
    output.setNumber(1, yawControl * FIN_GAIN)
    output.setNumber(2, pitchControl * FIN_GAIN)
end
