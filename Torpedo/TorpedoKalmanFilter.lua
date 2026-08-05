--[[
機能:
一次フィルター（Torpedo NewtonDistanceFinder）で同一Ping内の重複反射を
1観測へ統合したデータを受け取り、6状態EKFで時系列追跡する。

処理:
1. 距離・方位角・仰角をグローバル座標へ変換
2. 各観測をマハラノビス距離が最小の既存トラックへ関連付け
3. 関連付けできない観測を仮クラスタへ追加
4. 仮クラスタが複数回確認されたら正式トラック化
5. 観測がない間は等速モデルで予測し、距離依存時間後に削除

一次フィルター側で空間的な重複反射を統合済みのため、
このコードでは正式トラック同士の近接統合や重複エコー吸収を行わない。

入力（Composite Number）:
- ch 1～24: 最大6目標、1目標につき4ch
  - 4n-3: 距離 [m]
  - 4n-2: ローカル方位角 [rad]
  - 4n-1: ローカル仰角 [rad]
  - 4n  : 音波が目標へ到達した絶対Tick
- ch 25: 自機グローバル位置 X（東）
- ch 26: 自機グローバル位置 Y（上）
- ch 27: 自機グローバル位置 Z（北）
- ch 28: 自機Pitch [rad]
- ch 29: 自機Yaw [rad]
- ch 30: 自機Roll [rad]

出力:
- bool 1: 正式トラックあり
- num 1～3: 現在時刻へ外挿した推定位置 X,Y,Z
- num 4～6: 推定速度 Vx,Vy,Vz
- num 7～9: 予約（0）
- num 10: 最終観測の目標到達Tick
- num 11: 現在Tickから最終観測Tickまでの遅延
- num 12: トラックID
- num 13: 累積ヒット数
- num 32: 最新のマハラノビス距離 epsilon

座標系:
Physics Sensor座標系（X:東、Y:上、Z:北、左手系）。
状態ベクトルは [x,vx,y,vy,z,vz]。
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60           -- EKF更新の時間ステップ (秒)
MAX_RADAR_TARGETS = 6 -- 一次フィルターから受け取る最大目標数
NUM_STATES = 6        -- EKF状態数 (x, vx, y, vy, z, vz)
BASE_CHANNEL = 4

-- EKF パラメータ (プロパティから読み込む想定)
DATA_ASSOCIATION_EPSILON_THRESHOLD = property.getNumber("D_ASOC_EPS")            -- データアソシエーションのε閾値
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")                       -- 目標ロスト判定のTick数

PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") -- 観測が無い間に予測の信頼を下げる係数。値が大きいほど観測がない間に予測を信頼しなくなる。
INITIAL_VELOCITY_VARIANCE = 10000
PROCESS_NOISE = property.getNumber("PROCESS_NOISE")
-- ★ PI制御パラメータ (新規追加)
NOISE_TARGET_EPSILON = property.getNumber("NOISE_TARGET_EPS") -- PI制御の目標epsilon値
NOISE_INTEGRAL_GAIN = property.getNumber("NOISE_I_GAIN")      -- PI制御の積分ゲイン

LOGIC_DELAY = property.getNumber("LOGIC_DELAY")

R0_DIST_VAR_FACTOR = 0.005 --(0.02 ^ 2) / 12(文字数対策のため直接計算)
R0_ANGLE_VAR = 2.63e-4     --((2e-3 * PI2) ^ 2) / 12(文字数対策のため直接計算)
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }


-- グローバル変数
trackedTargets = {} -- 複数のトラックを保持するテーブル
nextTrackID = 1     -- 新規トラックに割り当てるID
currentTick = 0
trackedTargetsIndex = 0

tentativeClusters = {}
nextClusterID = 1

-- ソナー向けクラスタリング設定
-- 必要に応じてプロパティ化してください。
CLUSTER_CONFIRM_HITS = 3     -- 正式トラック化に必要な独立観測数
CLUSTER_CONFIRM_WINDOW = 360 -- 候補を維持する最大観測時刻差[tick]
CLUSTER_MIN_HIT_GAP = 2      -- 同時刻付近の重複を別ヒットとして数えない
CLUSTER_BASE_GATE = 40       -- 候補クラスタの基礎ゲート[m]
CLUSTER_RANGE_FACTOR = 0.01  -- 距離に比例して広げるゲート
MAX_INITIAL_SPEED = 250      -- クラスタ速度の上限
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
    local a, b, c, d, e, f, g, h, i, det, invDet
    a, b, c = m[1][1], m[1][2], m[1][3]
    d, e, f = m[2][1], m[2][2], m[2][3]
    g, h, i = m[3][1], m[3][2], m[3][3]

    -- 行列式 (Sarrus)
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    -- 特異行列判定
    if math.abs(det) < 1e-15 then return nil end

    invDet = 1 / det

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
---@return table X_predicted, table P_predicted(予測状態、予測共分散、新しい積分誤差)

function predictStep(currentTarget, dt)
    local X = currentTarget.X
    local P = currentTarget.P
    local F = MatrixCopy(identityMatrix6x6)

    F[1][2] = dt
    F[3][4] = dt
    F[5][6] = dt

    local XP = mul(F, X)
    local d2 = dt * dt
    local d3 = d2 * dt
    local Q = zeros(6, 6)

    for i = 0, 2 do
        local n = i * 2
        Q[n + 1][n + 1] = d3 / 3 * PROCESS_NOISE
        Q[n + 1][n + 2] = d2 / 2 * PROCESS_NOISE
        Q[n + 2][n + 1] = d2 / 2 * PROCESS_NOISE
        Q[n + 2][n + 2] = dt * PROCESS_NOISE
    end

    return XP, sum(mul(F, P, T(F)), Q)
end

-- ★【新関数 2】マハラノビス距離 (epsilon) の計算
---@param X_predicted table 予測状態
---@param P_predicted table 予測共分散
---@param observation table 観測値
---@param ownPosition table 自機位置
---@return number epsilon, table Y, table|nil S_inv, table H (マハラノビス距離), table Y (イノベーション), table S_inv (イノベーション共分散の逆), table H, table R_matrix (更新ステップ用)
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

    local vx, vy, vz, vSpeed, dx, dy, dz, dDist, cosTheta
    -- 1. トラックの現在推測速度を取得 (Vx, Vy, Vz)
    vx, vy, vz = X_predicted[2][1], X_predicted[4][1], X_predicted[6][1]
    vSpeed = math.sqrt(vx ^ 2 + vy ^ 2 + vz ^ 2)

    -- 2. ミサイル等、一定以上の速度（例: 50m/s以上）で移動している目標の場合のみ適用
    if vSpeed > 50 then
        -- 予測位置から今回の観測位置への方向ベクトル
        dx = observation.globalX - X_predicted[1][1]
        dy = observation.globalY - X_predicted[3][1]
        dz = observation.globalZ - X_predicted[5][1]
        dDist = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)

        if dDist > 1e-3 then
            -- 速度ベクトルと移動ベクトルのコサイン類似度 (1.0 = 進行方向と完全に一致)
            cosTheta = (vx * dx + vy * dy + vz * dz) / (vSpeed * dDist)

            -- 進行方向から外れているほど epsilon を激しく倍増させるペナルティ
            if cosTheta < 0.3 then
                epsilon = epsilon * (1.5 - cosTheta*0.5)
            end
        end
    end

    return epsilon, Y, S_inv, H
end

-- ★【新関数 3】EKF 更新ステップ
---@return table X_updated, table P_updated
function updateStep(X_predicted, P_predicted, Y, S_inv, H)
    local K, X_updated, P_updated
    K = mul(P_predicted, T(H), S_inv)
    X_updated = sum(X_predicted, mul(K, Y))
    P_updated = mul(sub(identityMatrix6x6, mul(K, H)), P_predicted)
    return X_updated, P_updated
end

-- フィルター状態を初期化
function initializeFilterState(c, id)
    local o, X, P, d, v
    o = c.lastObservation

    X = {
        { o.globalX }, { c.vx },
        { o.globalY }, { c.vy },
        { o.globalZ }, { c.vz }
    }

    P = zeros(6, 6)
    d = o.distance
    v = d * d * 2.325e-4

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
        lastReceiveTick = currentTick,
        hits = c.hits,
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function removeExpiredTentativeClusters()
    for id, c in pairs(tentativeClusters) do
        local timeout = math.max(
            CLUSTER_CONFIRM_WINDOW,
            c.distance * 120 / 1480 + 120
        )
        if currentTick - c.lastReceiveTick > timeout then
            tentativeClusters[id] = nil
        end
    end
end

-- 未関連付け観測を候補クラスタへ追加する。
-- 1観測だけでは正式トラックを作らず、時間的に独立した複数観測で確認する。
function addTentativeObservation(obs)
    local bestID, bestError
    bestID, bestError = nil, math.huge

    for id, c in pairs(tentativeClusters) do
        local dtTicks = obs.targetReachedTick - c.lastTick
        if dtTicks >= 0 then
            local dt = dtTicks * DT
            local px, py, pz = c.x, c.y, c.z
            local gate = CLUSTER_BASE_GATE +
                obs.distance * CLUSTER_RANGE_FACTOR

            if c.hits < 2 then
                gate = gate + MAX_INITIAL_SPEED * dt
            else
                px = px + c.vx * dt
                py = py + c.vy * dt
                pz = pz + c.vz * dt
                gate = gate + 20 * dt
            end

            local dx = obs.globalX - px
            local dy = obs.globalY - py
            local dz = obs.globalZ - pz
            local err = math.sqrt(dx * dx + dy * dy + dz * dz)

            if err < gate and err < bestError then
                bestError, bestID = err, id
            end
        end
    end

    if bestID == nil then
        local id = nextClusterID
        nextClusterID = id + 1
        tentativeClusters[id] = {
            id = id,
            x = obs.globalX,
            y = obs.globalY,
            z = obs.globalZ,

            firstX = obs.globalX,
            firstY = obs.globalY,
            firstZ = obs.globalZ,
            firstTick = obs.targetReachedTick,

            vx = 0,
            vy = 0,
            vz = 0,
            lastTick = obs.targetReachedTick,
            lastReceiveTick = currentTick,
            distance = obs.distance,
            hits = 1,
            lastObservation = obs
        }
        return nil
    end

    local c = tentativeClusters[bestID]
    local tickGap = obs.targetReachedTick - c.lastTick
    if tickGap < CLUSTER_MIN_HIT_GAP then return nil end

    local dt = tickGap * DT
    local dt = (obs.targetReachedTick - c.firstTick) * DT

    if dt > 0 then
        local vx = (obs.globalX - c.firstX) / dt
        local vy = (obs.globalY - c.firstY) / dt
        local vz = (obs.globalZ - c.firstZ) / dt
        local s = math.sqrt(vx * vx + vy * vy + vz * vz)

        if s > MAX_INITIAL_SPEED then
            local k = MAX_INITIAL_SPEED / s
            vx, vy, vz = vx * k, vy * k, vz * k
        end

        c.vx, c.vy, c.vz = vx, vy, vz
    end

    c.x, c.y, c.z = obs.globalX, obs.globalY, obs.globalZ
    c.lastTick = obs.targetReachedTick
    c.lastReceiveTick = currentTick
    c.lastObservation = obs
    c.distance = obs.distance
    c.hits = c.hits + 1

    if c.hits >= CLUSTER_CONFIRM_HITS then
        tentativeClusters[bestID] = nil
        return c
    end
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
        local dist, localAziRad, localEleRad, targetReachedTick, localPos, globalPos, relative, distCheck
        dist = input.getNumber(BASE_CHANNEL * i - 3)
        if dist > 0 then
            localAziRad = input.getNumber(BASE_CHANNEL * i - 2)
            localEleRad = input.getNumber(BASE_CHANNEL * i - 1)
            targetReachedTick = input.getNumber(BASE_CHANNEL * i)
            localPos = polarCoordsToLocalCoords(
                dist,
                localEleRad,
                localAziRad
            )
            globalPos = localToGlobalCoords(
                localPos,
                ownGlobalPos,
                ownOrientation
            )

            if globalPos ~= nil then
                relative = vectorSub(globalPos, ownGlobalPos)
                distCheck = vectorMagnitude(relative)
                if distCheck > 1e-6 then
                    currentObservations[#currentObservations + 1] = {
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
                    }
                end
            end
        end
    end

    -- 波面の到達順と同じく、古い観測時刻から逐次処理する。
    table.sort(currentObservations, function(a, b)
        return a.targetReachedTick < b.targetReachedTick
    end)


    for _, obs in ipairs(currentObservations) do
        local bestTrackID, bestEpsilon, bestCache, dtTicks, XPred, PPred
        bestTrackID = nil
        bestEpsilon = math.huge
        bestCache = nil

        -- 各観測を、予測と最も整合する1本の既存トラックへ関連付けする。
        -- 一次側で同一Ping内の重複反射は統合済み。
        for trackID, track in pairs(trackedTargets) do
            dtTicks = obs.targetReachedTick - track.lastTick

            -- 同時刻または古い観測ではトラックを巻き戻して更新しない。
            if dtTicks > 0 then
                XPred, PPred =
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
                        }
                    end
                end
            end
        end

        if bestTrackID ~= nil then
            local track, oldX, oldTick, XUp, PUp
            track = trackedTargets[bestTrackID]
            oldX = track.X
            oldTick = track.lastTick

            XUp, PUp = updateStep(
                bestCache.XPred,
                bestCache.PPred,
                bestCache.Y,
                bestCache.SInv,
                bestCache.H
            )

            if XUp then
                local dt, mvx, mvy, mvz, a, vx, vy, vz, speed, k
                dt = (obs.targetReachedTick - oldTick) * DT

                -- 観測位置差分から求めた速度を少し混ぜる
                if dt > 0 then
                    mvx = (obs.globalX - oldX[1][1]) / dt
                    mvy = (obs.globalY - oldX[3][1]) / dt
                    mvz = (obs.globalZ - oldX[5][1]) / dt
                    a = math.min(0.3, dt * 0.1)

                    XUp[2][1] = XUp[2][1] + (mvx - XUp[2][1]) * a
                    XUp[4][1] = XUp[4][1] + (mvy - XUp[4][1]) * a
                    XUp[6][1] = XUp[6][1] + (mvz - XUp[6][1]) * a
                end

                -- 補正後の速度に上限をかける
                vx = XUp[2][1]
                vy = XUp[4][1]
                vz = XUp[6][1]
                speed = math.sqrt(vx * vx + vy * vy + vz * vz)

                if speed > 250 then
                    k = 250 / speed
                    XUp[2][1] = vx * k
                    XUp[4][1] = vy * k
                    XUp[6][1] = vz * k
                end

                debug.log(
                    "dt:" .. dt ..
                    " mv:" .. math.sqrt(mvx * mvx + mvy * mvy + mvz * mvz) ..
                    " kfv:" .. math.sqrt(
                        XUp[2][1] ^ 2 +
                        XUp[4][1] ^ 2 +
                        XUp[6][1] ^ 2
                    ) ..
                    " a:" .. a
                )
                track.X = XUp
                track.P = PUp
                track.epsilon = bestEpsilon
                track.lastTick = obs.targetReachedTick
                track.lastReceiveTick = currentTick
                track.lastSeenTick = obs.targetReachedTick
                track.hits = track.hits + 1
            end
        else
            local c, id
            c = addTentativeObservation(obs)
            if c then
                id = nextTrackID
                nextTrackID = id + 1
                trackedTargets[id] = initializeFilterState(c, id)
            end
        end
    end

    -- 最終受信から距離依存の待機時間を超えたトラックを削除する。
    local deleteIDs = {}

    for id, track in pairs(trackedTargets) do
        local dx, dy, dz, distance, timeout
        dx = track.X[1][1] - ownGlobalPos[1]
        dy = track.X[3][1] - ownGlobalPos[2]
        dz = track.X[5][1] - ownGlobalPos[3]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        timeout = math.max(
            TARGET_LOST_THRESHOLD_TICKS,
            distance * 120 / 1480 + 180
        )

        if currentTick - track.lastReceiveTick > timeout then
            deleteIDs[#deleteIDs + 1] = id
        end
    end
    for _, id in ipairs(deleteIDs) do
        trackedTargets[id] = nil
    end

    removeExpiredTentativeClusters()

    -- ID順に時分割出力する。
    local sortedTracks = {}
    for _, track in pairs(trackedTargets) do
        sortedTracks[#sortedTracks + 1] = track
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
        local X, predictionTicks, dt
        X = targetToOutput.X
        predictionTicks =
            currentTick - targetToOutput.lastTick + LOGIC_DELAY
        if predictionTicks < 0 then predictionTicks = 0 end

        dt = predictionTicks * DT

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
        debug.log("spd: " .. math.sqrt(X[2][1] ^ 2 + X[4][1] ^ 2 + X[6][1] ^ 2))
    else
        for i = 1, 13 do output.setNumber(i, 0) end
        output.setNumber(32, 0)
        output.setBool(1, false)
    end
end
