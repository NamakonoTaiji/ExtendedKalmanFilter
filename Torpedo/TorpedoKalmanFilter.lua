--[[
機能:一次フィルターからの入力を元にカルマンフィルターによるノイズ抑制と目標の同定を行う。

入力 (コンポジット信号):
- num 1-3: レーダー目標1 (距離, 方位角(ラジアン), 仰角(ラジアン))
- num 4-6: レーダー目標2 ...
- ...
- num 19-21: レーダー目標7
- num 25: 自機グローバル位置 X
- num 26: 自機グローバル位置 Y
- num 27: 自機グローバル位置 Z
- num 28: 自機オイラー角 Pitch (ラジアン)
- num 29: 自機オイラー角 Yaw (ラジアン)
- num 30: 自機オイラー角 Roll (ラジアン)
- num 31: パイロットシート視線方位角(回転単位)
- num 32: パイロットシート視線仰角(回転単位)

出力 (コンポジット信号 - デバッグ用):
- bool 1: 目標を検出中
- num 1: 推定目標座標 X
- num 2: 推定目標座標 Y
- num 3: 推定目標座標 Z
- num 4: 推定目標速度 Vx
- num 5: 推定目標速度 Vy
- num 6: 推定目標速度 Vz
- num 7-9: 0 (6状態化後の互換用予約出力)
- num 10: Last Seen Tick (最終観測Tick)
- num 11: Detection Tick Lag (観測遅延Tick数)
- num 12: Tracked Target ID (トラック中の目標ID)
- num 13: Target Hit Count (目標の連続ヒット数)
- num 32: 最新のイプシロンε

前提:
- 座標系は Physics Sensor 座標系 (X:東, Y:上, Z:北, 左手系) を基準とする。
- EKFの各種パラメータはプロパティから読み込む想定
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60           -- EKF更新の時間ステップ (秒)
MAX_RADAR_TARGETS = 6 -- 処理するレーダー目標の最大数
NUM_STATES = 6        -- EKF状態数 (x, vx, y, vy, z, vz)
BASE_CHANNEL = 4

-- EKF パラメータ (プロパティから読み込む想定)
DATA_ASSOCIATION_EPSILON_THRESHOLD = property.getNumber("D_ASOC_EPS")            -- データアソシエーションのε閾値
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")                       -- 目標ロスト判定のTick数

PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") -- 観測が無い間に予測の信頼を下げる係数。値が大きいほど観測がない間に予測を信頼しなくなる。
INITIAL_VELOCITY_VARIANCE = 10000

-- ★ PI制御パラメータ (新規追加)
NOISE_TARGET_EPSILON = property.getNumber("NOISE_TARGET_EPS") -- PI制御の目標epsilon値
NOISE_INTEGRAL_GAIN = property.getNumber("NOISE_I_GAIN")      -- PI制御の積分ゲイン
NOISE_OUTPUT_MIN = -8                                         -- PI制御出力の下限
NOISE_OUTPUT_MAX = 5                                          -- PI制御出力の上限

LOGIC_DELAY = property.getNumber("LOGIC_DELAY")

R0_DIST_VAR_FACTOR = 200 --(0.02 ^ 2) / 12(文字数対策のため直接計算)
R0_ANGLE_VAR = 2.63e-3   --((2e-3 * PI2) ^ 2) / 12(文字数対策のため直接計算)
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }


-- グローバル変数
trackedTargets = {} -- 複数のトラックを保持するテーブル
nextTrackID = 1     -- 新規トラックに割り当てるID
currentTick = 0
trackingID = nil
trackedTargetsIndex = 0

tentativeClusters = {}
nextClusterID = 1

-- ソナー向けクラスタリング設定
-- 必要に応じてプロパティ化してください。
CLUSTER_CONFIRM_HITS = 3                     -- 正式トラック化に必要な独立観測数
CLUSTER_CONFIRM_WINDOW = 180                 -- 候補を維持する最大観測時刻差[tick]
CLUSTER_MIN_HIT_GAP = 2                      -- 同時刻付近の重複を別ヒットとして数えない
CLUSTER_BASE_GATE = 40                       -- 候補クラスタの基礎ゲート[m]
CLUSTER_RANGE_FACTOR = 0.07                  -- 距離に比例して広げるゲート
OLD_ECHO_MAX_AGE = 240                       -- 古いエコーとして吸収する最大時刻差[tick]
OLD_ECHO_BASE_GATE = CLUSTER_BASE_GATE * 0.8 -- 既存トラック近傍判定の基礎ゲート[m]
OLD_ECHO_RANGE_FACTOR = 0.025                -- 距離に比例して広げる近傍ゲート
MAX_INITIAL_SPEED = 250                      -- クラスタ速度の上限
--------------------------------------------------------------------------------
-- ベクトル演算ヘルパー関数
--------------------------------------------------------------------------------
function vectorMagnitude(v)
    local x, y, z
    x = v[1]
    y = v[2]
    z = v[3]
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

function vectorSub(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1]
    y1 = v1[2]
    z1 = v1[3]
    x2 = v2[1]
    y2 = v2[2]
    z2 = v2[3]
    return { x1 - x2, y1 - y2, z1 - z2 }
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

-- 単位行列
identityMatrix6x6 = zeros(6, 6)
for i = 1, 6 do identityMatrix6x6[i][i] = 1 end

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

        R = zeros(#A, #B[1])
        for r = 1, #A do
            for c = 1, #B[1] do
                sVal = 0
                for k = 1, #B do
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
    --
    rows = #M
    cols = #M[1]
    R = zeros(cols, rows)
    for r = 1, rows do
        --
        for c = 1, cols do
            --
            R[c][r] = M[r][c]
        end
    end
    return R
end

-- 3x3行列専用の逆行列関数 (汎用invの代替)
function inv3(m)
    local a, b, c = m[1][1], m[1][2], m[1][3]
    local d, e, f = m[2][1], m[2][2], m[2][3]
    local g, h, i = m[3][1], m[3][2], m[3][3]

    -- 行列式 (Sarrus)
    local det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    -- 特異行列判定
    if math.abs(det) < 1e-15 then return nil end

    local invDet = 1 / det

    -- クラメルの公式に基づく余因子行列の転置 * (1/det)
    return {
        { (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet },
        { (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet },
        { (d * h - e * g) * invDet, (g * b - a * h) * invDet, (a * e - b * d) * invDet }
    }
end

--------------------------------------------------------------------------------
-- クォータニオン演算関数
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2, w_result, x_result, y_result, z_result
    --
    w1 = q_a[1]
    x1 = q_a[2]
    y1 = q_a[3]
    z1 = q_a[4]
    w2 = q_b[1]
    x2 = q_b[2]
    y2 = q_b[3]
    z2 = q_b[4]

    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z

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

    px = vector[1]
    py = vector[2]
    pz = vector[3]

    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }

    temp = multiplyQuaternions(q, p)

    p_prime = multiplyQuaternions(temp, q_conj)

    return { p_prime[2], p_prime[3], p_prime[4] }
end

function localToGlobalCoords(localPosVec, ownGlobalPos, ownOrientationQuat)
    local localX, localY, localZ, globalRelativeVector, globalX, globalY, globalZ
    -- vector は {x, y, z} または {<index 1>, <index 2>, <index 3>} 形式のテーブルを想定

    localX = localPosVec[1]
    localY = localPosVec[2]
    localZ = localPosVec[3]

    globalRelativeVector = rotateVectorByQuaternion({ localX, localY, localZ }, ownOrientationQuat)

    globalX = globalRelativeVector[1] + ownGlobalPos[1]
    globalY = globalRelativeVector[2] + ownGlobalPos[2]
    globalZ = globalRelativeVector[3] + ownGlobalPos[3]
    return { globalX, globalY, globalZ }
end

-- グローバル直交座標からローカル直交座標へ

--- ローカル座標から方位角と仰角へ変換
---@param localPosVec Vector3 変換したいローカル座標(X右方向, Y上方向, Z前方向)
---@return table azimuthとelevationを返します(ラジアン)

function polarCoordsToLocalCoords(dist, localEleRad, localAziRad)
    local localX, localY, localZ
    localX = dist * math.cos(localEleRad) * math.sin(localAziRad)
    localY = dist * math.sin(localEleRad)
    localZ = dist * math.cos(localEleRad) * math.cos(localAziRad)
    return { localX, localY, localZ }
end

--------------------------------------------------------------------------------
-- EKF 関連関数
--------------------------------------------------------------------------------
-- 観測ヤコビアン H と 観測予測値 h を計算
function getObservationJacobianAndPrediction(stateVector, ownPosition)
    -- 関数冒頭でローカル変数を宣言
    local targetX, targetY, targetZ, relativeX, relativeY, relativeZ, r_sq, rh_sq, r, rh, predictedDistance, asin_arg, predictedElevation, predictedAzimuth, h, H


    targetX = stateVector[1][1]
    targetY = stateVector[3][1]
    targetZ = stateVector[5][1]
    relativeX = targetX - ownPosition[1]
    relativeY = targetY - ownPosition[2]
    relativeZ = targetZ - ownPosition[3]

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
    -- d(h)/d(x) (1列目)
    H[1][1] = relativeX / r
    H[2][1] = (relativeX * -relativeY) / (r_sq * rh)
    H[3][1] = relativeZ / rh_sq
    -- d(h)/d(y) (3列目)
    H[1][3] = relativeY / r
    H[2][3] = rh / r_sq
    H[3][3] = 0
    -- d(h)/d(z) (5列目)
    H[1][5] = relativeZ / r
    H[2][5] = (relativeZ * -relativeY) / (r_sq * rh)
    H[3][5] = -relativeX / rh_sq
    -- 速度項の偏微分はゼロ

    return H, h
end

-- 角度差を計算 (-PI から PI の範囲)
function calculateAngleDifference(angle1, angle2)
    local diff = angle2 - angle1
    while diff <= -PI do diff = diff + PI2 end
    while diff > PI do diff = diff - PI2 end
    return
        diff
end

-- EKF 更新ステップ (dtは固定DTを使用)
-- ★【新関数 1】EKF 予測ステップ
---@param currentTarget table 追跡中の目標オブジェクト
---@param dt_sec number 前回の更新からの経過秒数
---@return table X_predicted, table P_predicted, number newIntegralError (予測状態、予測共分散、新しい積分誤差)
function predictStep(currentTarget, dt_sec)
    local stateVector, covariance, lastEpsilon, integral_state
    local error, new_integral_state, cU, noiseScale_bH
    local F, Q_base, Q_adapted, P_pred_term1, uncertaintyIncreaseFactor
    local dt2, dt3, q_block, X_predicted, P_predicted

    stateVector = currentTarget.X
    covariance = currentTarget.P
    lastEpsilon = currentTarget.epsilon or NOISE_TARGET_EPSILON
    integral_state = currentTarget.integralError or 0

    error = NOISE_TARGET_EPSILON - lastEpsilon
    new_integral_state = integral_state + error * NOISE_INTEGRAL_GAIN
    cU = math.max(NOISE_OUTPUT_MIN, math.min(NOISE_OUTPUT_MAX, new_integral_state))
    if cU == NOISE_OUTPUT_MIN or cU == NOISE_OUTPUT_MAX then
        new_integral_state = integral_state
    end
    noiseScale_bH = 0.1

    -- 等速(CV)モデル
    F = MatrixCopy(identityMatrix6x6)
    F[1][2] = dt_sec
    F[3][4] = dt_sec
    F[5][6] = dt_sec
    X_predicted = mul(F, stateVector)

    -- 白色加速度を仮定したCVモデルのプロセスノイズ
    dt2 = dt_sec * dt_sec
    dt3 = dt2 * dt_sec
    Q_base = zeros(NUM_STATES, NUM_STATES)
    q_block = {
        { dt3 / 3, dt2 / 2 },
        { dt2 / 2, dt_sec }
    }
    for i = 0, 2 do
        for r = 1, 2 do
            for c = 1, 2 do
                Q_base[i * 2 + r][i * 2 + c] = q_block[r][c]
            end
        end
    end
    Q_adapted = scalar(noiseScale_bH, Q_base)

    uncertaintyIncreaseFactor =
        1.0 + PREDICTION_UNCERTAINTY_FACTOR_BASE * dt_sec
    P_pred_term1 = mul(F, covariance, T(F))
    P_predicted = sum(Q_adapted, P_pred_term1)


    return X_predicted, P_predicted, new_integral_state
end

-- ★【新関数 2】マハラノビス距離 (epsilon) の計算
---@param X_predicted table 予測状態
---@param P_predicted table 予測共分散
---@param observation table 観測値
---@param ownPosition table 自機位置
---@return number epsilon, table Y, table S_inv, table H , table R_matrix (マハラノビス距離), table Y (イノベーション), table S_inv (イノベーション共分散の逆), table H, table R_matrix (更新ステップ用)
function calculateInnovation(X_predicted, P_predicted, observation, ownPosition)
    local Z, epsilon, Y, S_inv, H, R_matrix, h, S, epsilon_matrix
    -- 観測ベクトル Z = [距離, グローバル仰角, グローバル方位角]^T
    Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }

    -- ヤコビアン H と 観測予測 h を計算
    H, h = getObservationJacobianAndPrediction(X_predicted, ownPosition)
    -- if H == nil then return math.huge, nil, nil, nil, nil end

    -- 観測ノイズ R
    R_matrix = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R_matrix[1][1] = R_matrix[1][1] * (observation.distance ^ 2)

    -- イノベーション(観測残差) Y
    Y = zeros(3, 1)
    Y[1][1] = Z[1][1] - h[1][1]                          -- 距離
    Y[2][1] = calculateAngleDifference(h[2][1], Z[2][1]) -- 仰角
    Y[3][1] = calculateAngleDifference(h[3][1], Z[3][1]) -- 方位角

    -- イノベーション共分散 S = H * P_pred * H^T + R
    S = sum(mul(H, P_predicted, T(H)), R_matrix)
    S_inv = inv3(S)

    -- if S_inv == nil then
    --     -- 逆行列計算失敗
    --     return math.huge, nil, nil, nil, nil
    -- end

    -- 誤差指標 epsilon (マハラノビス距離) の計算: epsilon = Y^T * S^-1 * Y
    epsilon = 1.0
    epsilon_matrix = mul(T(Y), S_inv, Y)
    if epsilon_matrix and epsilon_matrix[1] and epsilon_matrix[1][1] then
        epsilon = epsilon_matrix[1][1]
    end
    -- (既存の epsilon 計算の直後に追加)

    -- 1. トラックの現在推測速度を取得 (Vx, Vy, Vz)
    local vx, vy, vz = X_predicted[2][1], X_predicted[4][1], X_predicted[6][1]
    local vSpeed = math.sqrt(vx ^ 2 + vy ^ 2 + vz ^ 2)

    -- 2. ミサイル等、一定以上の速度（例: 50m/s以上）で移動している目標の場合のみ適用
    if vSpeed > 50 then
        -- 予測位置から今回の観測位置への方向ベクトル
        local dx = observation.globalX - X_predicted[1][1]
        local dy = observation.globalY - X_predicted[3][1]
        local dz = observation.globalZ - X_predicted[5][1]
        local dDist = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)

        if dDist > 1e-3 then
            -- 速度ベクトルと移動ベクトルのコサイン類似度 (1.0 = 進行方向と完全に一致)
            local cosTheta = (vx * dx + vy * dy + vz * dz) / (vSpeed * dDist)

            -- 進行方向から外れているほど epsilon を激しく倍増させるペナルティ
            if cosTheta < 0.6 then
                epsilon = epsilon * (2.0 - cosTheta)
            end
        end
    end

    return epsilon, Y, S_inv, H
end

-- ★【新関数 3】EKF 更新ステップ
---@return table X_updated, table P_updated, number epsilon, boolean success
function updateStep(X_predicted, P_predicted, Y, S_inv, H)
    local K = mul(P_predicted, T(H), S_inv)
    local X_updated = sum(X_predicted, mul(K, Y))
    local P_updated = mul(sub(identityMatrix6x6, mul(K, H)), P_predicted)
    return X_updated, P_updated
end

-- フィルター状態を初期化
function initializeFilterState(c, id)
    local o = c.lastObservation

    local X = { { o.globalX }, { 0 }, { o.globalY }, { 0 }, { o.globalZ }, { 0 } }

    local P = zeros(6, 6)
    local d = o.distance
    local v = d * d * 2.325e-4

    P[1][1] = v
    P[3][3] = v
    P[5][5] = v
    P[2][2] = INITIAL_VELOCITY_VARIANCE
    P[4][4] = INITIAL_VELOCITY_VARIANCE
    P[6][6] = INITIAL_VELOCITY_VARIANCE

    return {
        id = id,
        X = X,
        P = P,
        epsilon = 1,
        lastTick = o.targetReachedTick,
        lastSeenTick = o.targetReachedTick,
        hits = c.hits,
        misses = 0,
        integralError = 0
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
-- 観測時刻におけるトラックの簡易位置を返す。
-- 未来方向はCVモデル、過去方向は現在状態からの逆算を使用する。
function getTrackPositionAtTick(track, observationTick)
    local dt = (observationTick - track.lastTick) * DT
    return {
        track.X[1][1] + track.X[2][1] * dt,
        track.X[3][1] + track.X[4][1] * dt,
        track.X[5][1] + track.X[6][1] * dt
    }
end

function positionErrorToObservation(track, obs)
    local p = getTrackPositionAtTick(track, obs.targetReachedTick)
    local dx = obs.globalX - p[1]
    local dy = obs.globalY - p[2]
    local dz = obs.globalZ - p[3]
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

-- EKFで時系列更新できない古い観測、または既存トラック近傍の余剰観測を
-- 「別目標」ではなく重複エコーとして吸収する。
function isEchoOfExistingTrack(obs)
    for _, track in pairs(trackedTargets) do
        local age = track.lastTick - obs.targetReachedTick
        if age >= 0 and age <= OLD_ECHO_MAX_AGE then
            local gate = OLD_ECHO_BASE_GATE
                + obs.distance * OLD_ECHO_RANGE_FACTOR
            if positionErrorToObservation(track, obs) < gate then
                return true
            end
        end
    end
    return false
end

function removeExpiredTentativeClusters()
    for id, cluster in pairs(tentativeClusters) do
        if currentTick - cluster.lastReceiveTick > CLUSTER_CONFIRM_WINDOW then
            tentativeClusters[id] = nil
        end
    end
end

-- 未関連付け観測を候補クラスタへ追加する。
-- 1観測だけでは正式トラックを作らず、時間的に独立した複数観測で確認する。
function addTentativeObservation(obs)
    local bestID = nil
    local bestError = math.huge

    for id, cluster in pairs(tentativeClusters) do
        local dtTicks = obs.targetReachedTick - cluster.lastTick
        if dtTicks >= 0 and dtTicks <= CLUSTER_CONFIRM_WINDOW then
            local dt = dtTicks * DT
            local px = cluster.x + cluster.vx * dt
            local py = cluster.y + cluster.vy * dt
            local pz = cluster.z + cluster.vz * dt
            local dx = obs.globalX - px
            local dy = obs.globalY - py
            local dz = obs.globalZ - pz
            local err = math.sqrt(dx * dx + dy * dy + dz * dz)
            local gate = CLUSTER_BASE_GATE
                + obs.distance * CLUSTER_RANGE_FACTOR

            if err < gate and err < bestError then
                bestError = err
                bestID = id
            end
        end
    end

    if bestID == nil then
        local id = nextClusterID
        nextClusterID = nextClusterID + 1
        tentativeClusters[id] = {
            id = id,
            x = obs.globalX,
            y = obs.globalY,
            z = obs.globalZ,
            vx = 0,
            vy = 0,
            vz = 0,
            lastTick = obs.targetReachedTick,
            lastReceiveTick = currentTick,
            hits = 1,
            lastObservation = obs
        }
        return nil
    end

    local c = tentativeClusters[bestID]
    local tickGap = obs.targetReachedTick - c.lastTick

    -- 同一時刻付近の重複出力はヒット数に加算しない。
    if tickGap < CLUSTER_MIN_HIT_GAP then
        return nil
    end

    local dt = tickGap * DT
    if dt > 0 then
        local measuredVx = (obs.globalX - c.x) / dt
        local measuredVy = (obs.globalY - c.y) / dt
        local measuredVz = (obs.globalZ - c.z) / dt
        local a = 1 / math.min(c.hits + 1, 4)
        c.vx = c.vx + (measuredVx - c.vx) * a
        c.vy = c.vy + (measuredVy - c.vy) * a
        c.vz = c.vz + (measuredVz - c.vz) * a
    end

    c.x = obs.globalX
    c.y = obs.globalY
    c.z = obs.globalZ
    c.lastTick = obs.targetReachedTick
    c.lastReceiveTick = currentTick
    c.lastObservation = obs
    c.hits = c.hits + 1

    if c.hits >= CLUSTER_CONFIRM_HITS then
        tentativeClusters[bestID] = nil
        return c
    end
    return nil
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    currentTick = currentTick + 1

    ownGlobalPos = {
        input.getNumber(25),
        input.getNumber(26),
        input.getNumber(27)
    }
    ownEuler = {
        Pitch = input.getNumber(28),
        Yaw = input.getNumber(29),
        Roll = input.getNumber(30)
    }
    ownOrientation = eulerZYX_to_quaternion(
        ownEuler.Roll,
        ownEuler.Yaw,
        ownEuler.Pitch
    )
    if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end

    local currentObservations = {}

    for i = 1, MAX_RADAR_TARGETS do
        local dist = input.getNumber(BASE_CHANNEL * i - 3)
        if dist > 0 then
            local localAziRad = input.getNumber(BASE_CHANNEL * i - 2)
            local localEleRad = input.getNumber(BASE_CHANNEL * i - 1)
            local targetReachedTick = input.getNumber(BASE_CHANNEL * i)
            local localPos = polarCoordsToLocalCoords(
                dist,
                localEleRad,
                localAziRad
            )
            local globalPos = localToGlobalCoords(
                localPos,
                ownGlobalPos,
                ownOrientation
            )

            if globalPos ~= nil then
                local relative = vectorSub(globalPos, ownGlobalPos)
                local distCheck = vectorMagnitude(relative)
                if distCheck > 1e-6 then
                    table.insert(currentObservations, {
                        distance = dist,
                        elevation = math.asin(math.max(
                            -1,
                            math.min(1, relative[2] / distCheck)
                        )),
                        azimuth = math.atan(relative[1], relative[3]),
                        localAzimuthRad = localAziRad,
                        localElevationRad = localEleRad,
                        globalX = globalPos[1],
                        globalY = globalPos[2],
                        globalZ = globalPos[3],
                        targetReachedTick = targetReachedTick
                    })
                end
            end
        end
    end

    -- 波面の到達順と同じく、古い観測時刻から逐次処理する。
    table.sort(currentObservations, function(a, b)
        return a.targetReachedTick < b.targetReachedTick
    end)

    local updatedTracks = {}

    for _, obs in ipairs(currentObservations) do
        local bestTrackID = nil
        local bestEpsilon = math.huge
        local bestCache = nil

        -- 1観測ごとに最適な既存トラックを探す。
        -- assignedTrackIDsは使わないため、同じTick中でも時系列が進んでいれば
        -- 同一トラックを複数回更新できる。
        for trackID, track in pairs(trackedTargets) do
            local dtTicks = obs.targetReachedTick - track.lastTick

            -- 同時刻・古い観測は再更新せず、後段の重複エコー判定へ送る。
            if dtTicks > 0 then
                local XPred, PPred, integralError =
                    predictStep(track, dtTicks * DT)

                if XPred ~= nil and PPred ~= nil then
                    local epsilon, Y, SInv, H =
                        calculateInnovation(
                            XPred,
                            PPred,
                            obs,
                            ownGlobalPos
                        )

                    if epsilon < DATA_ASSOCIATION_EPSILON_THRESHOLD
                        and epsilon < bestEpsilon then
                        bestEpsilon = epsilon
                        bestTrackID = trackID
                        bestCache = {
                            XPred = XPred,
                            PPred = PPred,
                            Y = Y,
                            SInv = SInv,
                            H = H,
                            integralError = integralError
                        }
                    end
                end
            end
        end

        if bestTrackID ~= nil then
            local track = trackedTargets[bestTrackID]
            local XUp, PUp = updateStep(
                bestCache.XPred,
                bestCache.PPred,
                bestCache.Y,
                bestCache.SInv,
                bestCache.H
            )

            if XUp then
                local vx, vy, vz = XUp[2][1], XUp[4][1], XUp[6][1]
                local speed = math.sqrt(vx * vx + vy * vy + vz * vz)

                if speed > 250 then
                    local k = 250 / speed
                    XUp[2][1] = vx * k
                    XUp[4][1] = vy * k
                    XUp[6][1] = vz * k
                end

                track.X = XUp
                track.P = PUp
                track.epsilon = bestEpsilon
                track.integralError = bestCache.integralError
                track.lastTick = obs.targetReachedTick
                track.lastReceiveTick = currentTick
                track.lastSeenTick = obs.targetReachedTick
                track.hits = track.hits + 1
                track.misses = 0
                updatedTracks[bestTrackID] = true
            end
        elseif not isEchoOfExistingTrack(obs) then
            local c = addTentativeObservation(obs)
            if c then
                local id = nextTrackID
                nextTrackID = id + 1
                trackedTargets[id] = initializeFilterState(c, id)
                updatedTracks[id] = true
            end
        end
    end

    -- 観測を受けなかったトラックだけ、1ゲームTickにつき1回ミス加算する。
    local deleteIDs = {}
    for id, track in pairs(trackedTargets) do
        if not updatedTracks[id] then
            track.misses = track.misses + 1
        end
        if track.misses > TARGET_LOST_THRESHOLD_TICKS then
            table.insert(deleteIDs, id)
        end
    end
    for _, id in ipairs(deleteIDs) do
        debug.log("d-ID")
        trackedTargets[id] = nil
    end

    removeExpiredTentativeClusters()

    -- ID順に時分割出力する。
    local sortedTracks = {}
    for _, track in pairs(trackedTargets) do
        table.insert(sortedTracks, track)
    end

    local targetToOutput = nil
    if #sortedTracks > 0 then
        table.sort(sortedTracks, function(a, b) return a.id < b.id end)
        trackedTargetsIndex = trackedTargetsIndex + 1
        if trackedTargetsIndex > #sortedTracks then
            trackedTargetsIndex = 1
        end
        targetToOutput = sortedTracks[trackedTargetsIndex]
    else
        trackedTargetsIndex = 0
    end

    if targetToOutput ~= nil then
        local X = targetToOutput.X
        local predictionTicks =
            currentTick - targetToOutput.lastTick + LOGIC_DELAY
        if targetToOutput.hits < 6 then
            predictionTicks = math.min(predictionTicks, 60)
        end
        if predictionTicks < 0 then predictionTicks = 0 end

        local dt = predictionTicks * DT

        output.setNumber(1, X[1][1] + X[2][1] * dt)
        output.setNumber(2, X[3][1] + X[4][1] * dt)
        output.setNumber(3, X[5][1] + X[6][1] * dt)
        output.setNumber(4, X[2][1])
        output.setNumber(5, X[4][1])
        output.setNumber(6, X[6][1])
        output.setNumber(10, targetToOutput.lastSeenTick)
        output.setNumber(
            11,
            currentTick - targetToOutput.lastSeenTick
        )
        output.setNumber(12, targetToOutput.id)
        output.setNumber(13, targetToOutput.hits)
        output.setNumber(32, targetToOutput.epsilon)
        output.setBool(1, true)
    else
        for i = 1, 13 do output.setNumber(i, 0) end
        output.setNumber(32, 0)
        output.setBool(1, false)
    end
end
