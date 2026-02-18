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
- num 7: 推定目標加速度 Ax
- num 8: 推定目標加速度 Ay
- num 9: 推定目標加速度 Az
- num 10: レーダー方位角マニュアル制御
- num 11: レーダー仰角マニュアル制御
- num 12: トラック中の目標ID
- num 32: 最新のイプシロンε

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
MAX_RADAR_TARGETS = 8 -- 処理するレーダー目標の最大数
NUM_STATES = 9        -- EKF状態数 (x, vx, ax, y, vy, ay, z, vz, az)
BASE_CHANNEL = 3

-- EKF パラメータ (プロパティから読み込む想定)
DATA_ASSOCIATION_EPSILON_THRESHOLD = property.getNumber("D_ASOC_EPS") -- データアソシエーションのε閾値
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")            -- 目標ロスト判定のTick数
--PROCESS_NOISE_BASE = property.getNumber("P_BASE")                     -- プロセスノイズの大きさを調整
--PROCESS_NOISE_ADAPTIVE_SCALE = property.getNumber("P_ADPT")                      -- epsilon が非常に大きい（機動時）に、P_BASE に追加されるノイズの最大量
--PROCESS_NOISE_EPSILON_THRESHOLD = property.getNumber("P_NOISE_EPS_THRS")         -- P_ADPTによるスケーリングを開始するεの閾値。εがこれを超えると適応的調整が入り始める。
--PROCESS_NOISE_EPSILON_SLOPE = property.getNumber("P_NOISE_EPS_SLOPE")            -- プロセスノイズ適応調整のε傾き。これが大きいほどプロセスノイズの増加が急になる。
PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") -- 観測が無い間に予測の信頼を下げる係数。値が大きいほど観測がない間に予測を信頼しなくなる。
INITIAL_ACCELERATION_VARIANCE = 1e+3
INITIAL_VELOCITY_VARIANCE = 1e+3

-- ★ PI制御パラメータ (新規追加)
NOISE_TARGET_EPSILON = property.getNumber("NOISE_TARGET_EPS") -- PI制御の目標epsilon値
NOISE_INTEGRAL_GAIN = property.getNumber("NOISE_I_GAIN")      -- PI制御の積分ゲイン
NOISE_OUTPUT_MIN = -8                                         -- PI制御出力の下限
NOISE_OUTPUT_MAX = 5                                          -- PI制御出力の上限

LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
R0_DIST_VAR_FACTOR = 6.67e-3 --(0.02 ^ 2) / 12(文字数対策のため直接計算)
R0_ANGLE_VAR = 2.63e-3       --((2e-3 * PI2) ^ 2) / 12(文字数対策のため直接計算)
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }


-- グローバル変数
trackedTargets = {} -- 複数のトラックを保持するテーブル
nextTrackID = 1     -- 新規トラックに割り当てるID
currentTick = 0
trackingID = nil
trackedTargetsIndex = 0
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

-- function vectorNormalize(v)
--     local mag, x, y, z = vectorMagnitude(v)
--     if mag < 1e-9 then
--         return { 0, 0, 0 }
--     else
--         x = v[1] or v.x
--         y = v[2] or v.y
--         z = v[3] or v.z
--         return { x / mag, y / mag, z / mag }
--     end
-- end

-- function vectorAdd(v1, v2)
--     local x1, x2, y1, y2, z1, z2

--     x1 = v1[1] or v1.x
--     y1 = v1[2] or v1.y
--     z1 = v1[3] or v1.z
--     x2 = v2[1] or v2.x
--     y2 = v2[2] or v2.y
--     z2 = v2[3] or v2.z
--     return { x1 + x2, y1 + y2, z1 + z2 }
-- end

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

-- function vectorScalarMul(s, v)
--     local x, y, z
--     x = v[1] or v.x
--     y = v[2] or v.y
--     z = v[3] or v.z
--     return { s * x, s * y, s * z }
-- end

-- function vectorDot(v1, v2)
--     local x1, x2, y1, y2, z1, z2
--     x1 = v1[1] or v1.x
--     y1 = v1[2] or v1.y
--     z1 = v1[3] or v1.z
--     x2 = v2[1] or v2.x
--     y2 = v2[2] or v2.y
--     z2 = v2[3] or v2.z
--     return x1 * x2 + y1 * y2 + z1 * z2
-- end

-- function vectorCross(v1, v2)
--     local x1, x2, y1, y2, z1, z2
--     x1 = v1[1] or v1.x
--     y1 = v1[2] or v1.y
--     z1 = v1[3] or v1.z
--     x2 = v2[1] or v2.x
--     y2 = v2[2] or v2.y
--     z2 = v2[3] or v2.z
--     return { y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2 }
-- end

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
identityMatrix9x9 = zeros(9, 9)
for i = 1, 9 do identityMatrix9x9[i][i] = 1 end

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

-- function inv(M)
--     if M == nil or #M == 0 or #M[1] == 0 then return nil end
--     local n = #M
--     if n ~= #M[1] then return nil end -- 基本チェック
--     local aug = {}
--     for r = 1, n do
--         aug[r] = {}
--         if M[r] == nil then return nil end
--         for c = 1, n do
--             local v = M[r][c]
--             if v == nil or v ~= v or v == math.huge or v == -math.huge then return nil end
--             aug[r][c] = v
--         end
--         for c = 1, n do
--             if r == c then
--                 aug[r][n + c] = 1
--             else
--                 aug[r][n + c] = 0
--             end
--         end
--     end -- 入力チェック
--     for r = 1, n do
--         local piv = aug[r][r]
--         if piv == nil or math.abs(piv) < 1e-12 then return nil end -- ピボットチェック
--         for c = r, 2 * n do
--             if aug[r][c] == nil then return nil end
--             aug[r][c] = aug[r][c] / piv
--         end -- 除算前 nil チェック
--         for i = 1, n do
--             if i ~= r then
--                 local f = aug[i][r]
--                 if f == nil then return nil end
--                 for c = r, 2 * n do
--                     if aug[i][c] == nil or aug[r][c] == nil then return nil end
--                     aug[i][c] = aug[i][c] - f * aug[r][c]
--                 end
--             end
--         end
--     end
--     local invM = zeros(n, n)
--     for r = 1, n do
--         for c = 1, n do
--             local v = aug[r][n + c]
--             if v == nil or v ~= v or v == math.huge or v == -math.huge then
--                 invM[r][c] = 0
--             else
--                 invM[r][c] = v
--             end
--         end
--     end -- 結果チェック
--     return invM
-- end
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

function rotateVectorByInverseQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime

    px = vector[1]
    py = vector[2]
    pz = vector[3]

    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }

    temp = multiplyQuaternions(q_conj, p)

    p_prime = multiplyQuaternions(temp, q)

    return { p_prime[2], p_prime[3], p_prime[4] }
end

--------------------------------------------------------------------------------
-- 座標変換関数
--------------------------------------------------------------------------------
-- ローカル直交座標からグローバル直交座標へ
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
function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local gX, gY, gZ, oX, oY, oZ, relativeVectorGlobal, localVector
    -- 各入力が {x, y, z} 形式のテーブルを想定

    gX = globalTargetPos[1]
    gY = globalTargetPos[2]
    gZ = globalTargetPos[3]
    oX = ownGlobalPos[1]
    oY = ownGlobalPos[2]
    oZ = ownGlobalPos[3]

    relativeVectorGlobal = { gX - oX, gY - oY, gZ - oZ }
    localVector = rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientationQuat)

    return { localVector[1], localVector[2], localVector[3] }
end

--- ローカル座標から方位角と仰角へ変換
---@param localPosVec Vector3 変換したいローカル座標(X右方向, Y上方向, Z前方向)
---@return table azimuthとelevationを返します(ラジアン)
function localCoordsToLocalAngle(localPosVec)
    local horizontalDistance, currentLocalAzimuth, currentLocalElevation
    horizontalDistance = math.sqrt(localPosVec[1] ^ 2 + localPosVec[3] ^ 2)
    currentLocalAzimuth = math.atan(localPosVec[1], localPosVec[3])       -- atan(左右, 前後)
    currentLocalElevation = math.atan(localPosVec[2], horizontalDistance) -- atan(上下, 水平距離)
    return { azimuth = currentLocalAzimuth, elevation = currentLocalElevation }
end

-- 極座標からローカル座標へ
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
    targetY = stateVector[4][1]
    targetZ = stateVector[7][1]
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
    -- d(h)/d(y) (4列目)
    H[1][4] = relativeY / r
    H[2][4] = rh / r_sq
    H[3][4] = 0
    -- d(h)/d(z) (7列目)
    H[1][7] = relativeZ / r
    H[2][7] = (relativeZ * -relativeY) / (r_sq * rh)
    H[3][7] = -relativeX / rh_sq
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
    local stateVector, covariance, lastEpsilon, lastIntegralError, setpoint, process_variable, integral_state, I_gain, error, new_integral_state, cU, noiseScale_bH, X_predicted, P_predicted
    local dt_sec2_half, F, Q_base, Q_adapted, P_pred_term1, uncertaintyIncreaseFactor, dt2, dt3, dt4, dt5, dt6, q_block

    stateVector = currentTarget.X
    covariance = currentTarget.P
    -- 前回のepsilonが無ければ目標値を使う (初期値 or ロストからの復帰時)
    lastEpsilon = currentTarget.epsilon or NOISE_TARGET_EPSILON
    -- 前回の積分誤差を取得 (なければ0)
    lastIntegralError = currentTarget.integralError or 0

    -- === PI Controller for Noise Scale ===
    -- (ワークショップ版KFのcF関数のロジックを再現)
    setpoint = NOISE_TARGET_EPSILON
    process_variable = lastEpsilon
    integral_state = lastIntegralError
    I_gain = NOISE_INTEGRAL_GAIN

    -- 誤差 (目標値 - 現在値)
    error = setpoint - process_variable

    -- 積分項の更新 (dt_sec を掛けて時間積分)
    -- 注意: Workshop版は dt を使っていないように見えたため、
    -- もし挙動がおかしければ dt_sec を掛けない方が良いかもしれない
    new_integral_state = integral_state + error * I_gain -- ★ dt_sec を使用

    -- PI制御出力 (P=0, D=0 なので積分項のみ) - ゲインは積分時に考慮済み
    cU = new_integral_state -- ★ 出力は積分状態そのものと仮定 (ゲインは更新時に適用)

    -- 出力制限 (リミッター)
    cU = math.max(NOISE_OUTPUT_MIN, math.min(NOISE_OUTPUT_MAX, cU))

    -- アンチワインドアップ: 出力が制限にかかった場合、積分が進みすぎないように戻す
    if cU == NOISE_OUTPUT_MIN or cU == NOISE_OUTPUT_MAX then
        new_integral_state = integral_state -- 積分状態を更新しない
    end

    -- 最終的なノイズスケール bH
    noiseScale_bH = 10 ^ -(3 + cU)
    -- === 1. 予測ステップ ===

    -- 状態遷移行列 F (CAモデル)
    dt_sec2_half = dt_sec * dt_sec * 0.5
    F = MatrixCopy(identityMatrix9x9)
    -- X軸 (1, 2, 3)
    F[1][2] = dt_sec
    F[1][3] = dt_sec2_half
    F[2][3] = dt_sec
    -- Y軸 (4, 5, 6)
    F[4][5] = dt_sec
    F[4][6] = dt_sec2_half
    F[5][6] = dt_sec
    -- Z軸 (7, 8, 9)
    F[7][8] = dt_sec
    F[7][9] = dt_sec2_half
    F[8][9] = dt_sec

    X_predicted = mul(F, stateVector)

    -- プロセスノイズ Q の計算 (適応的 CAモデル)
    dt2 = dt_sec * dt_sec
    dt3 = dt2 * dt_sec
    dt4 = dt3 * dt_sec
    dt5 = dt4 * dt_sec
    dt6 = dt5 * dt_sec
    Q_base = zeros(NUM_STATES, NUM_STATES)
    q_block = {
        { dt6 / 36, dt5 / 12, dt4 / 6 },
        { dt5 / 12, dt4 / 4,  dt3 / 2 },
        { dt4 / 6,  dt3 / 2,  dt2 }
    }
    for i = 0, 2 do -- X, Y, Z軸
        for r = 1, 3 do
            for c = 1, 3 do
                Q_base[i * 3 + r][i * 3 + c] = q_block[r][c]
            end
        end
    end

    Q_adapted = scalar(noiseScale_bH, Q_base)

    -- 共分散 P の予測
    -- (dt_ticks=1固定と仮定し、dt_sec の大きさに応じて不確かさを増やす)
    uncertaintyIncreaseFactor = 1.0 + (PREDICTION_UNCERTAINTY_FACTOR_BASE * dt_sec)

    P_pred_term1 = mul(F, covariance, T(F))
    P_predicted = sum(scalar(uncertaintyIncreaseFactor, P_pred_term1), Q_adapted)

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

    -- updateStep で再利用するため、計算結果を返す
    return epsilon, Y, S_inv, H, R_matrix
end

-- ★【新関数 3】EKF 更新ステップ
---@return table X_updated, table P_updated, number epsilon, boolean success
function updateStep(X_predicted, P_predicted, observation, ownPosition, Y, S_inv, H, R_matrix)
    local K, X_updated, I_minus_KH, P_up_term1, P_up_term2, P_updated, epsilon, epsilon_matrix
    -- カルマンゲイン K = P_pred * H^T * S^-1
    K = mul(P_predicted, T(H), S_inv)

    -- 状態 X の更新: X_up = X_pred + K * Y
    X_updated = sum(X_predicted, mul(K, Y))

    -- 共分散 P の更新 (Joseph form): P_up = (I - K*H)*P_pred*(I - K*H)^T + K*R*K^T
    I_minus_KH = sub(identityMatrix9x9, mul(K, H))
    P_up_term1 = mul(I_minus_KH, P_predicted, T(I_minus_KH))
    P_up_term2 = mul(K, R_matrix, T(K))
    P_updated = sum(P_up_term1, P_up_term2)

    -- 誤差指標 epsilon の計算 (calculateInnovation から再計算)
    epsilon = 1.0
    epsilon_matrix = mul(T(Y), S_inv, Y)
    if epsilon_matrix and epsilon_matrix[1] and epsilon_matrix[1][1] then
        epsilon = epsilon_matrix[1][1]
    end

    return X_updated, P_updated, epsilon, true -- 更新成功
end

-- フィルター状態を初期化
function initializeFilterState(initialObservation, tick, trackID)
    -- 関数冒頭でローカル変数を宣言
    local X_init, P_init, R_init, pos_variance_scale

    --
    -- 初期速度はゼロと仮定
    X_init = {
        { initialObservation.globalX }, { 0 }, { 0 }, -- x, vx=0, ax=0
        { initialObservation.globalY }, { 0 }, { 0 }, -- y, vy=0, ay=0
        { initialObservation.globalZ }, { 0 }, { 0 }  -- z, vz=0, az=0
    }
    -- 初期共分散行列 P の設定
    P_init = zeros(NUM_STATES, NUM_STATES)
    -- 観測ノイズから初期位置の分散を設定 (KalmanFilterRefactor.lua 参照)
    R_init = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    --
    R_init[1][1] = R_init[1][1] * (initialObservation.distance ^ 2)
    pos_variance_scale = 10                                           -- 初期位置の不確かさを観測ノイズの10倍程度に設定
    -- 位置の分散 (1,1), (4,4), (7,7)
    P_init[1][1] = (R_init[3][3] + R_init[1][1]) * pos_variance_scale -- X(East)
    P_init[4][4] = (R_init[2][2] + R_init[1][1]) * pos_variance_scale -- Y(Up)
    P_init[7][7] = (R_init[3][3] + R_init[1][1]) * pos_variance_scale -- Z(North)
    -- 速度の分散 (2,2), (5,5), (8,8)
    P_init[2][2] = INITIAL_VELOCITY_VARIANCE
    P_init[5][5] = INITIAL_VELOCITY_VARIANCE
    P_init[8][8] = INITIAL_VELOCITY_VARIANCE
    -- 加速度の分散 (3,3), (6,6), (9,9) - 新たに定義が必要

    P_init[3][3] = INITIAL_ACCELERATION_VARIANCE
    P_init[6][6] = INITIAL_ACCELERATION_VARIANCE
    P_init[9][9] = INITIAL_ACCELERATION_VARIANCE
    return {
        id = trackID, -- ★ トラックID
        X = X_init,
        P = P_init,
        epsilon = 1.0,
        lastTick = tick,     -- 最後に更新/初期化されたTick
        lastSeenTick = tick, -- 最後に観測が紐付けられたTick
        hits = 1,            -- ★ 連続ヒット数 (信頼性評価用)
        misses = 0,          -- ★ 連続ミス数 (削除判定用)
        integralError = 0    -- ★ PIコントローラーの積分状態を初期化
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    -- 関数冒頭でローカル変数を宣言

    currentTick = currentTick + 1

    -- 1. 入力読み込み

    -- 自機情報
    ownGlobalPos = { input.getNumber(25), input.getNumber(26), input.getNumber(27) }
    ownEuler = { Pitch = input.getNumber(28), Yaw = input.getNumber(29), Roll = input.getNumber(30) }

    -- 自機姿勢をクォータニオンに変換
    ownOrientation = eulerZYX_to_quaternion(ownEuler.Roll, ownEuler.Yaw, ownEuler.Pitch)
    --  (エラー発生時は単位クォータニオンで代替)
    if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end

    -- レーダー観測値の処理
    -- レーダー観測値の処理
    currentObservations = {} -- このTickで有効なレーダー観測リスト
    isRadarDetecting = false

    for i = 1, MAX_RADAR_TARGETS do
        dist = input.getNumber(BASE_CHANNEL * i - 2) -- 距離
        if dist > 0 then
            isRadarDetecting = true
            localAziRad = input.getNumber(BASE_CHANNEL * i - 1)
            localEleRad = input.getNumber(BASE_CHANNEL * i)

            -- ローカル極座標からローカル直交座標へ
            targetLocalPosVec = polarCoordsToLocalCoords(dist, localEleRad, localAziRad)

            -- ローカル直交座標からグローバル直交座標へ
            targetGlobal = localToGlobalCoords(targetLocalPosVec, ownGlobalPos, ownOrientation)

            -- nilチェックは原則削除
            if targetGlobal ~= nil then
                -- グローバル座標からグローバルな仰角・方位角を計算 (EKF用)
                relativeGlobalVec = vectorSub(targetGlobal, ownGlobalPos)
                -- nilチェックは原則削除
                if relativeGlobalVec ~= nil then
                    distCheck = vectorMagnitude(relativeGlobalVec)

                    globalElevation = math.asin(math.max(-1.0, math.min(1.0, relativeGlobalVec[2] / distCheck)))
                    globalAzimuth = math.atan(relativeGlobalVec[1], relativeGlobalVec[3]) -- atan(East, North)

                    table.insert(currentObservations, {
                        distance = dist,             -- 元の観測距離
                        azimuth = globalAzimuth,     -- グローバル方位角 (Z軸基準)
                        elevation = globalElevation, -- グローバル仰角 (Y軸基準)
                        localAzimuthRad = localAziRad,
                        localElevationRad = localEleRad,
                        globalX = targetGlobal[1],
                        globalY = targetGlobal[2],
                        globalZ = targetGlobal[3]
                    })
                end -- relativeGlobalVec nil check end
            end     -- targetGlobal nil check end
        end
    end             -- radar loop end
    -- -----------------------------------------------------------------
    -- 2. マルチターゲット処理
    -- -----------------------------------------------------------------

    -- (A) 全トラックの予測ステップ
    predictedTracks = {}  -- 予測後のトラックリスト (計算用)
    trackIDsToDelete = {} -- 削除対象トラックIDリスト

    for trackID, track in pairs(trackedTargets) do
        local dt_pred_sec = (currentTick - track.lastTick) * DT
        if dt_pred_sec < 0 then dt_pred_sec = 0 end

        local X_pred, P_pred, newIntegralError = predictStep(track, dt_pred_sec)

        if X_pred == nil or P_pred == nil then
            -- 予測失敗 -> 削除候補
            table.insert(trackIDsToDelete, trackID)
        else
            -- ★ 新しい積分誤差をトラックオブジェクトに保存
            track.integralError = newIntegralError
            -- 予測結果を一時保存
            predictedTracks[trackID] = {
                id = trackID,
                X_pred = X_pred,
                P_pred = P_pred,
                originalTrack = track -- 元のトラック情報への参照
            }
        end
    end
    -- 予測失敗トラックを削除
    for _, id in ipairs(trackIDsToDelete) do trackedTargets[id] = nil end
    trackIDsToDelete = {} -- リセット

    -- (B) データアソシエーション (Global Nearest Neighbor - GNN)
    associations = {}       -- {epsilon, trackID, obsIndex, cache={Y, S_inv, H, R}}
    assignedObsIndices = {} -- 割り当て済みの観測値インデックス
    assignedTrackIDs = {}   -- 割り当て済みのトラックID

    -- (B-1) 全ての組み合わせのマハラノビス距離を計算 (Gating含む)
    for trackID, predTrack in pairs(predictedTracks) do
        for obsIndex, obs in ipairs(currentObservations) do
            local epsilon, Y, S_inv, H, R_matrix = calculateInnovation(
                predTrack.X_pred, predTrack.P_pred, obs, ownGlobalPos
            )

            if epsilon < DATA_ASSOCIATION_EPSILON_THRESHOLD then
                -- 閾値以下のペアを候補に追加
                table.insert(associations, {
                    epsilon = epsilon,
                    trackID = trackID,
                    obsIndex = obsIndex,
                    cache = { Y = Y, S_inv = S_inv, H = H, R_matrix = R_matrix }
                })
            end
        end
    end

    -- (B-2) epsilon が小さい順にソート
    table.sort(associations, function(a, b) return a.epsilon < b.epsilon end)

    -- (B-3) GNNによる割り当て (最小epsilonから順に確定)
    for _, assoc in ipairs(associations) do
        local trackID = assoc.trackID
        local obsIndex = assoc.obsIndex

        -- まだ割り当てられていないペアなら確定
        if not assignedTrackIDs[trackID] and not assignedObsIndices[obsIndex] then
            -- トラックに割り当て情報を保存
            predictedTracks[trackID].assignedObsIndex = obsIndex
            predictedTracks[trackID].cache = assoc.cache

            -- 割り当て済みフラグを立てる
            assignedTrackIDs[trackID] = true
            assignedObsIndices[obsIndex] = true
        end
    end

    -- (C) トラックの更新 と ミス処理
    for trackID, predTrack in pairs(predictedTracks) do
        local track = predTrack.originalTrack -- 元のトラックオブジェクト

        if predTrack.assignedObsIndex then
            -- === マッチした: 更新ステップを実行 ===
            local obsIndex = predTrack.assignedObsIndex
            local obs = currentObservations[obsIndex]
            local cache = predTrack.cache

            local X_up, P_up, eps_up, success_update = updateStep(
                predTrack.X_pred, predTrack.P_pred, obs, ownGlobalPos,
                cache.Y, cache.S_inv, cache.H, cache.R_matrix
            )

            if success_update then
                track.X = X_up
                track.P = P_up
                track.epsilon = eps_up
                track.lastTick = currentTick
                track.lastSeenTick = currentTick
                track.hits = track.hits + 1
                track.misses = 0 -- ミスカウントリセット
            else
                -- 更新失敗 -> ミス扱い
                track.misses = track.misses + 1
            end
        else
            -- === マッチしなかった: ミス処理 ===
            -- 状態は予測値を保持 (predictStep で P は増加済み)
            -- track.X = predTrack.X_pred -- lastTick を更新しないので予測状態は保持しない方が良い
            -- track.P = predTrack.P_pred
            track.misses = track.misses + 1
        end

        -- ロスト判定 (連続ミス回数)
        if track.misses > TARGET_LOST_THRESHOLD_TICKS then
            table.insert(trackIDsToDelete, trackID)
        end
    end
    -- ロストトラックを削除
    for _, id in ipairs(trackIDsToDelete) do trackedTargets[id] = nil end

    -- (D) 新規トラックの初期化
    -- (どの既存トラックにも割り当てられなかった観測値を使う)

    for obsIndex, obs in ipairs(currentObservations) do
        if not assignedObsIndices[obsIndex] then
            -- ★ 新規トラックを作成
            local newTrackID = nextTrackID
            nextTrackID = nextTrackID + 1

            -- initializeFilterState で初期化
            local newTrack = initializeFilterState(obs, currentTick, newTrackID)

            -- trackedTargets テーブルに追加
            trackedTargets[newTrackID] = newTrack
        end
    end

    local sortedTracks = {}

    for trackID, track in pairs(trackedTargets) do
        table.insert(sortedTracks, track)
    end

    local targetToOutput = nil
    local primaryTrackID = nil

    if #sortedTracks > 0 then
        -- ★ ソートを実行
        table.sort(sortedTracks, function(a, b) return a.id < b.id end)

        -- ★ インデックスのインクリメントとラップアラウンド
        trackedTargetsIndex = trackedTargetsIndex + 1
        if trackedTargetsIndex > #sortedTracks then
            trackedTargetsIndex = 1 -- 最後の次は 1 に戻る
        end

        -- ★ 時分割でターゲットを選択
        targetToOutput = sortedTracks[trackedTargetsIndex]
        primaryTrackID = targetToOutput.id
    else
        -- トラックが一つもない場合
        trackedTargetsIndex = 0 -- インデックスをリセット
    end

    if targetToOutput ~= nil then
        -- ★ プライマリターゲットの情報を出力
        local trackX = targetToOutput.X
        local detectionTickLag = currentTick - targetToOutput.lastSeenTick
        local dt_delay = DT * (LOGIC_DELAY + detectionTickLag)
        local dt_delay2_half = dt_delay * dt_delay * 0.5
        local outputX, outputY, outputZ

        outputX = trackX[1][1] + trackX[2][1] * dt_delay + trackX[3][1] * dt_delay2_half
        outputY = trackX[4][1] + trackX[5][1] * dt_delay + trackX[6][1] * dt_delay2_half
        outputZ = trackX[7][1] + trackX[8][1] * dt_delay + trackX[9][1] * dt_delay2_half

        output.setNumber(1, outputX)                                -- X
        output.setNumber(2, outputY)                                -- Y
        output.setNumber(3, outputZ)                                -- Z
        output.setNumber(4, trackX[2][1] + trackX[3][1] * dt_delay) -- Vx
        output.setNumber(5, trackX[5][1] + trackX[6][1] * dt_delay) -- Vy
        output.setNumber(6, trackX[8][1] + trackX[9][1] * dt_delay) -- Vz
        output.setNumber(7, trackX[3][1])                           -- Ax
        output.setNumber(8, trackX[6][1])                           -- Ay
        output.setNumber(9, trackX[9][1])                           -- Az
        output.setNumber(10, targetToOutput.lastSeenTick)
        output.setNumber(11, detectionTickLag)
        output.setNumber(12, primaryTrackID or 0)
        output.setNumber(13, targetToOutput.hits)
        output.setNumber(32, targetToOutput.epsilon) -- Epsilon
        output.setBool(1, true)
    else
        -- トラックがない場合
        for i = 1, 9 do
            output.setNumber(i, 0)
        end
        output.setBool(1, false)
        trackedTargetsIndex = 0
    end
end
