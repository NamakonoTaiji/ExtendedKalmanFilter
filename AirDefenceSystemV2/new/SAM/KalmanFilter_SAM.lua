--[[
================================================================================
KalmanFilter_SAM.lua (v0.3 - 新コーディングルール適用: nilチェック削除、local宣言整理)
================================================================================
機能:
- データリンク座標 (X, Y) を入力として受け取る (Zは0扱い)。
- ミサイル搭載レーダーからの観測値 (最大6目標 x 4ch/目標) を入力として受け取る。
- データリンク座標 または 追跡中の目標の予測位置 を基準として、
  レーダー観測値とのデータアソシエーション (EKF試算によるε最小化) を行う。
- アソシエーションに成功した観測値を用いてEKFを実行し、目標状態を更新する。
- 試験段階として、自機位置・姿勢はセンサー入力とする。
- フィルター結果の出力はデバッグ用に行う。

入力 (コンポジット信号):
- オンオフ1: データリンク要求フラグ(追跡中の目標と有効なデータリンクがかけ離れている場合に受信する)
- オンオフ2: レーダー圏内フラグ
- オンオフ 3: 発射フラグ
- 数値 1-4: レーダー目標1 (距離, 方位角(回転単位), 仰角(回転単位), 経過時間)
- 数値 5-8: レーダー目標2 ...
- ...
- 数値 21-24: レーダー目標6
- 数値 8: 自機グローバル位置 X
- 数値 12: 自機グローバル位置 Y
- 数値 16: 自機グローバル位置 Z
- 数値 20: 自機オイラー角 Pitch (ラジアン)
- 数値 24: 自機オイラー角 Yaw (ラジアン)
- 数値 25: 自機オイラー角 Roll (ラジアン)
- 数値 26: データリンク目標座標 X
- 数値 27: データリンク目標座標 Y
- 数値 28: データリンク目標座標 Z
- 数値 29: 別のLuaブロックで計算したデータリンク目標との距離 ^ 2
- 数値 30: 自機X軸速度(パススルー)
- 数値 31: 自機Y軸速度(パススルー)
- 数値 32: 自機Z軸速度(パススルー)

出力 (コンポジット信号 - デバッグ用):
- オンオフ 1: トラッキング成功フラグ (isTracking)
- オンオフ 2: 発射フラグ
- 数値 1: 推定目標座標 X
- 数値 2: 推定目標座標 Y
- 数値 3: 推定目標座標 Z
- 数値 4: 推定目標速度 Vx
- 数値 5: 推定目標速度 Vy
- 数値 6: 推定目標速度 Vz
- 数値 7: データリンク座標 X
- 数値 8: データリンク座標 Y
- 数値 9: データリンク座標 Z
- 数値 10: 自機グローバル位置 X
- 数値 11: 自機グローバル位置 Y
- 数値 12: 自機グローバル位置 Z
- 数値 13: 自機オイラー角 Pitch (ラジアン)
- 数値 14: 自機オイラー角 Yaw (ラジアン)
- 数値 15: 自機オイラー角 Roll (ラジアン)
- 数値 16: 自機X軸速度(パススルー)
- 数値 17: 自機Y軸速度(パススルー)
- 数値 18: 自機Z軸速度(パススルー)
- 数値 32: 最新のイプシロンε

前提:
- 座標系は Physics Sensor 座標系 (X:東, Y:上, Z:北, 左手系) を基準とする。
- EKFの各種パラメータはプロパティから読み込む想定。
- inv 関数は別途追記が必要。
================================================================================
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60           -- EKF更新の時間ステップ (秒)
MAX_RADAR_TARGETS = 6 -- 処理するレーダー目標の最大数
NUM_STATES = 6        -- EKF状態数 (x, vx, y, vy, z, vz)

-- EKF パラメータ (プロパティから読み込む想定)
DATA_ASSOCIATION_EPSILON_THRESHOLD = property.getNumber("D_ASOC_EPS")            -- データアソシエーションのε閾値
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")                       -- 目標ロスト判定のTick数 (約2秒)
INIT_MAX_DISTANCE = property.getNumber("INIT_MAX_DIST")                          -- 初期化時のデータリンク座標との最大許容距離(m)
PROCESS_NOISE_BASE = property.getNumber("P_BASE")                                -- プロセスノイズの大きさを調整
PROCESS_NOISE_ADAPTIVE_SCALE = property.getNumber("P_ADPT")                      -- プロセスノイズの適応的調整。観測と予測の差が大きいほどプロセスノイズが増える。
PROCESS_NOISE_EPSILON_THRESHOLD = property.getNumber("P_NOISE_EPS_THRS")         -- P_ADPTによるスケーリングを開始するεの閾値。εがこれを超えると適応的調整が入り始める。
PROCESS_NOISE_EPSILON_SLOPE = property.getNumber("P_NOISE_EPS_SLOPE")            -- プロセスノイズ適応調整のε傾き。これが大きいほどプロセスノイズの増加が急になる。
PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") -- 観測が無い間に予測の信頼を下げる係数。値が大きいほど観測がない間に予測を信頼しなくなる。
LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
R0_DIST_VAR_FACTOR = 3.3333                                                      --(0.02 ^ 2) / 12(文字数対策のため直接計算)
R0_ANGLE_VAR = 1.3159                                                            --((2e-3 * PI2) ^ 2) / 12(文字数対策のため直接計算)
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }
INITIAL_VELOCITY_VARIANCE = (1000 ^ 2)

-- グローバル変数
trackedTarget = nil -- { X, P, epsilon, lastTick, lastSeenTick } 追跡中の単一目標の状態
currentTick = 0
isTracking = false  -- 現在有効な追跡を行っているか

-- 単位行列
identityMatrix6x6 = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }

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

--------------------------------------------------------------------------------
-- EKF 関連関数
--------------------------------------------------------------------------------
-- 観測ヤコビアン H と 観測予測値 h を計算
function getObservationJacobianAndPrediction(stateVector, ownPosition)
    -- 関数冒頭でローカル変数を宣言
    local targetX, targetY, targetZ, relativeX, relativeY, relativeZ, r_sq, rh_sq, r, rh, predictedDistance, asin_arg, predictedElevation, predictedAzimuth, h, H

    -- nilチェックは原則削除
    targetX = stateVector[1][1]
    targetY = stateVector[3][1]
    targetZ = stateVector[5][1]
    relativeX = targetX - ownPosition.x
    relativeY = targetY - ownPosition.y
    relativeZ = targetZ - ownPosition.z

    r_sq = relativeX ^ 2 + relativeY ^ 2 + relativeZ ^ 2
    rh_sq = relativeX ^ 2 + relativeZ ^ 2
    -- ゼロ除算防止
    if r_sq < 1e-9 then r_sq = 1e-9 end
    if rh_sq < 1e-9 then rh_sq = 1e-9 end
    r = math.sqrt(r_sq)
    rh = math.sqrt(rh_sq)

    -- 観測予測値 h = [距離, グローバル仰角, グローバル方位角]^T
    predictedDistance = r
    asin_arg = math.max(-1.0, math.min(1.0, relativeY / r))
    predictedElevation = math.asin(asin_arg)           -- YがUp
    predictedAzimuth = math.atan(relativeX, relativeZ) -- atan(East, North) -> Z軸(北)基準の方位角
    h = { { predictedDistance }, { predictedElevation }, { predictedAzimuth } }

    -- 観測ヤコビ行列 H = dh/dX (3x6)
    H = zeros(3, NUM_STATES)
    H[1][1] = relativeX / r
    H[1][3] = relativeY / r
    H[1][5] = relativeZ / r -- d(dist)/d(x,y,z)
    -- d(elevation)/d(x,y,z) - YがUp, ZがNorth
    H[2][1] = (relativeX * -relativeY) / (r_sq * rh)
    H[2][3] = rh / r_sq
    H[2][5] = (relativeZ * -relativeY) / (r_sq * rh)
    -- d(azimuth)/d(x,y,z) - atan(X,Z)
    H[3][1] = relativeZ / rh_sq
    H[3][3] = 0
    H[3][5] = -relativeX / rh_sq
    -- 速度項の偏微分はゼロ

    return H, h
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

-- EKF 更新ステップ (dtは固定DTを使用)
function extendedKalmanFilterUpdate(currentTarget, observation, ownPosition)
    -- 関数冒頭でローカル変数を宣言
    local covariance, lastEpsilon, dt_sec, F, X_predicted, dt2, dt3, dt4, Q_base, adaptiveFactor, Q_adapted
    local uncertaintyIncreaseFactor, P_pred_term1, P_predicted, Z, H, h, R_matrix, Y, S_term1, S, S_inv, K_term1, K, KY_term, X_updated
    local KH_term, I_minus_KH, P_up_term1, P_up_term2, P_updated, epsilon, epsilon_matrix, stateVector
    -- nilチェックは原則削除
    stateVector = currentTarget.X
    covariance = currentTarget.P
    lastEpsilon = currentTarget.epsilon or 1.0

    -- === 1. 予測ステップ ===
    dt_sec = DT -- 固定時間ステップ
    F = MatrixCopy(identityMatrix6x6);
    -- nilチェックは原則削除
    F[1][2] = dt_sec
    F[3][4] = dt_sec
    F[5][6] = dt_sec
    X_predicted = mul(F, stateVector)
    -- nilチェックは原則削除

    -- プロセスノイズ Q の計算 (適応的)
    dt2 = dt_sec * dt_sec
    dt3 = dt2 * dt_sec / 2
    dt4 = dt3 * dt_sec / 2
    Q_base = { { dt4, dt3, 0, 0, 0, 0 }, { dt3, dt2, 0, 0, 0, 0 }, { 0, 0, dt4, dt3, 0, 0 }, { 0, 0, dt3, dt2, 0, 0 }, { 0, 0, 0, 0, dt4, dt3 }, { 0, 0, 0, 0, dt3, dt2 } }
    adaptiveFactor = PROCESS_NOISE_BASE +
        PROCESS_NOISE_ADAPTIVE_SCALE /
        (1 + math.exp(-(lastEpsilon - PROCESS_NOISE_EPSILON_THRESHOLD) * PROCESS_NOISE_EPSILON_SLOPE))
    Q_adapted = scalar(adaptiveFactor, Q_base)
    -- nilチェックは原則削除

    -- 共分散 P の予測
    uncertaintyIncreaseFactor = PREDICTION_UNCERTAINTY_FACTOR_BASE ^ 2 -- dt_ticks=1固定
    P_pred_term1 = mul(F, covariance, T(F))
    -- nilチェックは原則削除
    P_predicted = sum(scalar(uncertaintyIncreaseFactor, P_pred_term1), Q_adapted)
    -- nilチェックは原則削除

    -- === 2. 更新ステップ ===
    -- 観測ベクトル Z = [距離, グローバル仰角, グローバル方位角]^T
    Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }

    -- ヤコビアン H と 観測予測 h を計算
    H, h = getObservationJacobianAndPrediction(X_predicted, ownPosition)
    -- nilチェックは原則削除

    -- 観測ノイズ R の計算 (距離に応じて変動)
    R_matrix = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    -- nilチェックは原則削除
    R_matrix[1][1] = R_matrix[1][1] * (observation.distance ^ 2) -- 距離^2 に比例

    -- イノベーション(観測残差) Y = Z - h (角度は正規化)
    Y = zeros(3, 1)
    Y[1][1] = Z[1][1] - h[1][1]                          -- 距離
    Y[2][1] = calculateAngleDifference(h[2][1], Z[2][1]) -- 仰角
    Y[3][1] = calculateAngleDifference(h[3][1], Z[3][1]) -- 方位角

    -- イノベーション共分散 S = H * P_pred * H^T + R
    S_term1 = mul(H, P_predicted, T(H))
    -- nilチェックは原則削除
    S = sum(S_term1, R_matrix)
    -- nilチェックは原則削除
    S_inv = inv(S)
    -- nilチェックは原則削除 (invがnilを返す可能性は残る)
    if S_inv == nil then
        -- 逆行列計算失敗時は更新せず終了
        return stateVector, covariance, lastEpsilon, false
    end


    -- カルマンゲイン K = P_pred * H^T * S^-1
    K_term1 = mul(P_predicted, T(H))
    -- nilチェックは原則削除
    K = mul(K_term1, S_inv)
    -- nilチェックは原則削除

    -- 状態 X の更新: X_up = X_pred + K * Y
    KY_term = mul(K, Y)
    -- nilチェックは原則削除
    X_updated = sum(X_predicted, KY_term)
    -- nilチェックは原則削除

    -- 共分散 P の更新 (Joseph form): P_up = (I - K*H)*P_pred*(I - K*H)^T + K*R*K^T
    KH_term = mul(K, H)
    -- nilチェックは原則削除
    I_minus_KH = sub(identityMatrix6x6, KH_term)
    -- nilチェックは原則削除
    P_up_term1 = mul(I_minus_KH, P_predicted, T(I_minus_KH))
    P_up_term2 = mul(K, R_matrix, T(K))
    -- nilチェックは原則削除
    P_updated = sum(P_up_term1, P_up_term2)
    -- nilチェックは原則削除

    -- 誤差指標 epsilon の計算: epsilon = Y^T * S^-1 * Y
    epsilon = 1 -- デフォルト値
    epsilon_matrix = mul(T(Y), S_inv, Y)
    if epsilon_matrix and epsilon_matrix[1] and epsilon_matrix[1][1] then
        epsilon = epsilon_matrix[1][1]
    end

    return X_updated, P_updated, epsilon, true -- 更新成功
end

-- フィルター状態を初期化
function initializeFilterState(initialObservation, tick)
    -- 関数冒頭でローカル変数を宣言
    local X_init, P_init, R_init, pos_variance_scale

    -- nilチェックは原則削除
    -- 初期速度はゼロと仮定
    X_init = { { initialObservation.globalX }, { 0 }, { initialObservation.globalY }, { 0 }, { initialObservation.globalZ }, { 0 } }

    -- 初期共分散行列 P の設定
    P_init = zeros(NUM_STATES, NUM_STATES)
    -- 観測ノイズから初期位置の分散を設定 (KalmanFilterRefactor.lua 参照)
    R_init = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    -- nilチェックは原則削除
    R_init[1][1] = R_init[1][1] * (initialObservation.distance ^ 2)
    pos_variance_scale = 10                                           -- 初期位置の不確かさを観測ノイズの10倍程度に設定
    -- 座標軸と観測値(距離,仰角,方位角)の分散の関係から簡易的に設定
    P_init[1][1] = (R_init[3][3] + R_init[1][1]) * pos_variance_scale -- X(East) は Azimuth と Distance の影響
    P_init[3][3] = (R_init[2][2] + R_init[1][1]) * pos_variance_scale -- Y(Up) は Elevation と Distance の影響
    P_init[5][5] = (R_init[3][3] + R_init[1][1]) * pos_variance_scale -- Z(North) は Azimuth と Distance の影響
    -- 初期速度の分散を大きく設定
    P_init[2][2] = INITIAL_VELOCITY_VARIANCE
    P_init[4][4] = INITIAL_VELOCITY_VARIANCE
    P_init[6][6] = INITIAL_VELOCITY_VARIANCE

    return {
        X = X_init,
        P = P_init,
        epsilon = 1.0,      -- 初期イプシロン
        lastTick = tick,    -- 最後にEKF更新/初期化されたTick
        lastSeenTick = tick -- 最後に観測が紐付けられたTick
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    -- 関数冒頭でローカル変数を宣言

    currentTick = currentTick + 1
    isTracking = false -- Tick開始時にリセット

    -- 1. 入力読み込み
    isDataLinkInitRequired = input.getBool(1)
    isRadarEffectiveRange = input.getBool(2)
    isDataLinkUpdateStopped = input.getBool(4)
    -- 自機情報 (試験用)
    ownGlobalPos = { x = input.getNumber(8), y = input.getNumber(12), z = input.getNumber(16) }
    ownEuler = { Pitch = input.getNumber(20), Yaw = input.getNumber(24), Roll = input.getNumber(25) }
    ownVector = { x = input.getNumber(30), y = input.getNumber(31), z = input.getNumber(32) }
    -- 自機姿勢をクォータニオンに変換
    ownOrientation = eulerZYX_to_quaternion(ownEuler.Roll, ownEuler.Yaw, ownEuler.Pitch)
    -- nilチェックは原則削除 (エラー発生時は単位クォータニオンで代替)
    if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end

    -- データリンク目標座標 (VLSFCSから)
    dataLinkTargetGlobal = { x = input.getNumber(26), y = input.getNumber(27), z = input.getNumber(28) }
    -- データリンク座標が (0,0,0) でないことを確認して有効性を判断
    isDataLinkValid = not (dataLinkTargetGlobal.x == 0 or dataLinkTargetGlobal.y == 0 or dataLinkTargetGlobal.z == 0)

    -- レーダー観測値の処理
    currentObservations = {} -- このTickで有効なレーダー観測リスト
    isRadarDetecting = false
    -- レーダー有効範囲の時だけ観測を行う
    if isRadarEffectiveRange then
        for i = 1, MAX_RADAR_TARGETS do
            baseChannel = (i - 1) * 4 + 1       -- 1, 5, 9, 13, 17, 21
            dist = input.getNumber(baseChannel) -- 距離
            if dist > 0 then
                isRadarDetecting = true
                localAziRad = input.getNumber(baseChannel + 1) * PI2 -- 方位角(回転単位)
                localEleRad = input.getNumber(baseChannel + 2) * PI2 -- 仰角(回転単位)
                -- 経過時間は読み飛ばす (baseChannel + 3)

                -- ローカル極座標からローカル直交座標へ
                localX = dist * math.cos(localEleRad) * math.sin(localAziRad)
                localY = dist * math.sin(localEleRad)
                localZ = dist * math.cos(localEleRad) * math.cos(localAziRad)
                targetLocalPosVec = { localX, localY, localZ }

                -- ローカル直交座標からグローバル直交座標へ
                targetGlobal = localToGlobalCoords(targetLocalPosVec, ownGlobalPos, ownOrientation)

                -- nilチェックは原則削除
                if targetGlobal ~= nil then
                    -- グローバル座標からグローバルな仰角・方位角を計算 (EKF用)
                    relativeGlobalVec = vectorSub(targetGlobal, ownGlobalPos)
                    -- nilチェックは原則削除
                    if relativeGlobalVec ~= nil then
                        distCheck = vectorMagnitude(relativeGlobalVec)
                        if distCheck > 1e-6 then
                            globalElevation = math.asin(math.max(-1.0, math.min(1.0, relativeGlobalVec[2] / distCheck)))
                            globalAzimuth = math.atan(relativeGlobalVec[1], relativeGlobalVec[3]) -- atan(East, North)

                            table.insert(currentObservations, {
                                distance = dist,             -- 元の観測距離
                                azimuth = globalAzimuth,     -- グローバル方位角 (Z軸基準)
                                elevation = globalElevation, -- グローバル仰角 (Y軸基準)
                                globalX = targetGlobal.x,
                                globalY = targetGlobal.y,
                                globalZ = targetGlobal.z
                            })
                        end
                    end -- relativeGlobalVec nil check end
                end     -- targetGlobal nil check end
            end         -- dist > 0 end
            -- ::continue:: は不要になった
        end             -- radar loop end
    end
    -- 2. データアソシエーションとEKF更新
    associatedObservation = nil -- このTickで trackedTarget に紐付けられた観測

    if isRadarDetecting and #currentObservations > 0 then
        if trackedTarget == nil or isDataLinkInitRequired then
            -- === トラックが存在しない場合、もしくはデータリンクを要求された時: データリンク座標に最も近い観測で初期化 ===
            if isDataLinkValid then
                if isDataLinkUpdateStopped then
                    minInitDistSq = math.huge()
                else
                    minInitDistSq = INIT_MAX_DISTANCE ^ 2 -- 初期化時の最大許容距離(m)^2
                end

                bestInitObs = nil
                for _, obs in ipairs(currentObservations) do
                    dx = obs.globalX - dataLinkTargetGlobal.x
                    dy = obs.globalY - dataLinkTargetGlobal.y
                    dz = obs.globalZ - dataLinkTargetGlobal.z
                    distSq = dx ^ 2 + dy ^ 2 + dz ^ 2
                    if distSq < minInitDistSq then
                        minInitDistSq = distSq
                        bestInitObs = obs
                    end
                end

                if bestInitObs ~= nil then
                    trackedTarget = initializeFilterState(bestInitObs, currentTick)
                    -- nilチェックは原則削除
                    if trackedTarget ~= nil then
                        isTracking = true                   -- 初期化成功もトラッキングとみなす
                        associatedObservation = bestInitObs -- 記録しておく
                    end
                end
            end
        else
            -- === トラックが存在する場合: 予測位置とのε最小でアソシエーション ===
            minEpsilon = DATA_ASSOCIATION_EPSILON_THRESHOLD + 1 -- 閾値より大きい値で初期化
            bestMatchObsIndex = -1
            tempState = MatrixCopy(trackedTarget.X)             -- 予測計算用にコピー
            tempCovar = MatrixCopy(trackedTarget.P)

            -- nilチェックは原則削除

            -- 予測ステップ (アソシエーション試算用、状態は変更しない)
            dt_pred_sec = (currentTick - trackedTarget.lastTick) * DT
            if dt_pred_sec < 0 then dt_pred_sec = 0 end
            F_pred = MatrixCopy(identityMatrix6x6);
            -- nilチェックは原則削除
            F_pred[1][2] = dt_pred_sec
            F_pred[3][4] = dt_pred_sec
            F_pred[5][6] = dt_pred_sec
            X_assoc_pred = mul(F_pred, tempState) -- 予測位置を計算

            -- nilチェックは原則削除
            if X_assoc_pred ~= nil then
                for i, obs in ipairs(currentObservations) do
                    -- EKF更新を試算してεを計算 (実際の状態は更新しない)
                    _, _, epsilon_try, success_try = extendedKalmanFilterUpdate(
                        { X = X_assoc_pred, P = tempCovar, epsilon = trackedTarget.epsilon }, obs, ownGlobalPos)

                    if success_try and epsilon_try < minEpsilon then
                        minEpsilon = epsilon_try
                        bestMatchObsIndex = i
                    end
                end

                if bestMatchObsIndex ~= -1 and minEpsilon <= DATA_ASSOCIATION_EPSILON_THRESHOLD then
                    -- === 最良マッチが見つかった: EKF更新を実行 ===
                    associatedObservation = currentObservations[bestMatchObsIndex]
                    X_up, P_up, eps_up, success_update = extendedKalmanFilterUpdate(trackedTarget, associatedObservation,
                        ownGlobalPos)

                    if success_update then
                        trackedTarget.X = X_up
                        trackedTarget.P = P_up
                        trackedTarget.epsilon = eps_up
                        trackedTarget.lastTick = currentTick     -- EKFが更新されたTick
                        trackedTarget.lastSeenTick = currentTick -- 観測が見られたTick
                        isTracking = true
                    else
                        isTracking = false
                        trackedTarget = nil -- 更新失敗したらトラックを破棄
                    end
                else
                    isTracking = false
                end
            end -- X_assoc_pred nil check end
        end     -- trackedTarget nil check end
    else
        isTracking = false
    end -- radar detection check end

    -- 3. 目標ロスト判定
    if trackedTarget ~= nil and not isTracking then
        -- lastSeenTick nil チェック
        if trackedTarget.lastSeenTick == nil then
            trackedTarget.lastSeenTick = trackedTarget.lastTick or
                (currentTick - TARGET_LOST_THRESHOLD_TICKS)
        end

        ticksSinceLastSeen = currentTick - trackedTarget.lastSeenTick
        if ticksSinceLastSeen >= TARGET_LOST_THRESHOLD_TICKS then
            trackedTarget = nil
        else
            -- 予測ステップのみ実行して状態を維持 (Inertial相当)
            -- lastTick nil チェック
            if trackedTarget.lastTick == nil then trackedTarget.lastTick = currentTick end
            dt_pred_sec = (currentTick - trackedTarget.lastTick) * DT
            if dt_pred_sec > 0 then
                F_pred = MatrixCopy(identityMatrix6x6)
                -- nilチェックは原則削除
                if F_pred ~= nil then
                    F_pred[1][2] = dt_pred_sec
                    F_pred[3][4] = dt_pred_sec
                    F_pred[5][6] = dt_pred_sec
                    predictedX = mul(F_pred, trackedTarget.X)
                    if predictedX then
                        trackedTarget.X = predictedX
                        -- Pも更新するべきだが、簡略化のため省略
                        trackedTarget.lastTick = currentTick -- 予測ステップを実行したTickに更新
                    else
                        trackedTarget = nil
                    end
                else
                    trackedTarget = nil -- 安全のため破棄
                end
            end
        end
    end

    -- 4. 出力 (デバッグ用)
    output.setBool(1, isTracking)                                                             -- トラッキング成功フラグ
    output.setBool(2, input.getBool(3))                                                       -- 発射したか否か
    if trackedTarget ~= nil and trackedTarget.X ~= nil then                                   -- Xのnilチェックは残す
        -- Xの各要素が存在するかはチェックしない (nilチェック原則禁止のため)
        output.setNumber(1, trackedTarget.X[1][1] + trackedTarget.X[2][1] * DT * LOGIC_DELAY) -- 推定 X
        output.setNumber(2, trackedTarget.X[3][1] + trackedTarget.X[4][1] * DT * LOGIC_DELAY) -- 推定 Y
        output.setNumber(3, trackedTarget.X[5][1] + trackedTarget.X[6][1] * DT * LOGIC_DELAY) -- 推定 Z
        output.setNumber(4, trackedTarget.X[2][1])                                            -- 推定 Vx
        output.setNumber(5, trackedTarget.X[4][1])                                            -- 推定 Vy
        output.setNumber(6, trackedTarget.X[6][1])                                            -- 推定 Vz
        output.setNumber(32, trackedTarget.epsilon or 0)                                      -- 最新のε (nilなら0)
    else
        -- トラックがない場合は0を出力
        for i = 1, 6 do
            output.setNumber(i, 0)
        end
    end
    -- ミサイル制御マイコンへのパススルー
    output.setNumber(7, dataLinkTargetGlobal.x)
    output.setNumber(8, dataLinkTargetGlobal.y)
    output.setNumber(9, dataLinkTargetGlobal.z)
    output.setNumber(10, ownGlobalPos.x)
    output.setNumber(11, ownGlobalPos.y)
    output.setNumber(12, ownGlobalPos.z)
    output.setNumber(13, ownEuler.Pitch)
    output.setNumber(14, ownEuler.Yaw)
    output.setNumber(15, ownEuler.Roll)
    output.setNumber(16, ownVector.x)
    output.setNumber(17, ownVector.y)
    output.setNumber(18, ownVector.z)
end
