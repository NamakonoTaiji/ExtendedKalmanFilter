--[[
機能:一次フィルターからの入力を元にカルマンフィルターによるノイズ抑制と目標の同定を行う。

入力 (コンポジット信号):
- num 1-3: レーダー目標1 (距離, 方位角(ラジアン), 仰角(ラジアン))
- num 4-6: レーダー目標2 ...
- ...
- num 22-24: レーダー目標8
- num 25: 自機グローバル位置 X
- num 26: 自機グローバル位置 Y
- num 27: 自機グローバル位置 Z
- num 28: 自機オイラー角 Pitch (ラジアン)
- num 29: 自機オイラー角 Yaw (ラジアン)
- num 30: 自機オイラー角 Roll (ラジアン)
- num 31: レーダーID（診断用）
- num 32: 一次フィルターが記録した観測tick
- bool 31: 一次フィルターの有効フレーム

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
  - num 10: 状態が対応する最終観測tick
  - num 11: 最終観測から出力時点までの経過tick
  - num 12: 追跡目標ID
  - num 13: 観測成功回数
- num 14: 観測失敗回数
- num 15: セクターID
  - num 16: 最後に更新したレーダーID
  - num 17: 予約（0）
  - num 18: 出力位置・速度が対応する基準tick
  - num 32: 最新のイプシロンε

前提:
- 座標系は Physics Sensor 座標系 (X:東, Y:上, Z:北, 左手系) を基準とする。
- EKFの各種パラメータはプロパティから読み込む想定
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
TARGET_LOST_THRESHOLD_TICKS = math.max(property.getNumber("T_LOST"), 30) -- 13tick走査で1回欠測しても保持
--PROCESS_NOISE_BASE = property.getNumber("P_BASE")                     -- プロセスノイズの大きさを調整
--PROCESS_NOISE_ADAPTIVE_SCALE = property.getNumber("P_ADPT")                      -- epsilon が非常に大きい（機動時）に、P_BASE に追加されるノイズの最大量
--PROCESS_NOISE_EPSILON_THRESHOLD = property.getNumber("P_NOISE_EPS_THRS")         -- P_ADPTによるスケーリングを開始するεの閾値。εがこれを超えると適応的調整が入り始める。
--PROCESS_NOISE_EPSILON_SLOPE = property.getNumber("P_NOISE_EPS_SLOPE")            -- プロセスノイズ適応調整のε傾き。これが大きいほどプロセスノイズの増加が急になる。
PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") -- 観測が無い間に予測の信頼を下げる係数。値が大きいほど観測がない間に予測を信頼しなくなる。
INITIAL_ACCELERATION_VARIANCE = 1e4
INITIAL_VELOCITY_VARIANCE = 3.6e5

-- ★ PI制御パラメータ (新規追加)
NOISE_TARGET_EPSILON = property.getNumber("NOISE_TARGET_EPS") -- PI制御の目標epsilon値
NOISE_INTEGRAL_GAIN = property.getNumber("NOISE_I_GAIN")      -- PI制御の積分ゲイン
NOISE_OUTPUT_MIN = -3                                         -- PI制御出力の下限
NOISE_OUTPUT_MAX = 3                                          -- PI制御出力の上限

LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
OBSERVATION_TICK_OFFSET = property.getNumber("OBS_TICK_OFFSET")
SECTOR_ID = property.getNumber("SECTOR_ID")
local outputHoldProperty = property.getNumber("OUTPUT_HOLD_TICKS")
OUTPUT_HOLD_TICKS = outputHoldProperty > 0 and math.max(1, math.floor(outputHoldProperty)) or 4
-- レーダ単体の理論値ではなく、4台間の取付・姿勢・時刻差を含めた実効共分散
R0_DIST_VAR_FACTOR = 6.67e-3
R0_ANGLE_VAR = 2.63e-3
OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_FACTOR, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }


-- グローバル変数
trackedTargets = {} -- 複数のトラックを保持するテーブル
nextTrackID = 1     -- 新規トラックに割り当てるID
currentTick = 0
lastFrameTick = 0
trackingID = nil
trackedTargetsIndex = 0
lastOutputSlot = -1
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

    -- === 案A: 加速度の動的減衰 (Distance-based Damping) ===
    -- 予測位置と自機位置から目標までの距離を算出
    local targetDist = 0
    if ownGlobalPos then
        local dx = X_predicted[1][1] - ownGlobalPos[1]
        local dy = X_predicted[4][1] - ownGlobalPos[2]
        local dz = X_predicted[7][1] - ownGlobalPos[3]
        targetDist = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
    end

    -- 距離に基づく減衰係数の計算 (線形補間)
    local dampingFactor = 0.96
    local maxDamp = 0.96 -- 3000m以下の減衰係数（急機動に追従）
    local minDamp = 0.10 -- 7000m以上の減衰係数（ほぼ等速直線運動とみなす）
    local minDist = 3000
    local maxDist = 7000

    if targetDist > maxDist then
        dampingFactor = minDamp
    elseif targetDist > minDist then
        local t = (targetDist - minDist) / (maxDist - minDist)
        dampingFactor = maxDamp * (1 - t) + minDamp * t
    end

    X_predicted[3][1] = X_predicted[3][1] * dampingFactor -- Ax
    X_predicted[6][1] = X_predicted[6][1] * dampingFactor -- Ay
    X_predicted[9][1] = X_predicted[9][1] * dampingFactor -- Az

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
    local Z, epsilon, Y, S_inv, H, R_matrix, h, S, epsilon_matrix, azimuthWeight
    -- 観測ベクトル Z = [距離, グローバル仰角, グローバル方位角]^T
    Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }

    -- ヤコビアン H と 観測予測 h を計算
    H, h = getObservationJacobianAndPrediction(X_predicted, ownPosition)
    -- if H == nil then return math.huge, nil, nil, nil, nil end

    -- 天頂・天底付近では方位角が定義できないため、仰角75.5度付近から
    -- 方位角の寄与を連続的に弱める（cos(elevation) = 水平距離 / 距離）
    azimuthWeight = math.min(1, math.cos(h[2][1]) * 4, math.cos(Z[2][1]) * 4) ^ 2
    H[3][1] = H[3][1] * azimuthWeight
    H[3][7] = H[3][7] * azimuthWeight

    -- 観測ノイズ R
    R_matrix = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R_matrix[1][1] = R_matrix[1][1] * (observation.distance ^ 2)

    -- イノベーション(観測残差) Y
    Y = zeros(3, 1)
    Y[1][1] = Z[1][1] - h[1][1]                          -- 距離
    Y[2][1] = calculateAngleDifference(h[2][1], Z[2][1]) -- 仰角
    Y[3][1] = calculateAngleDifference(h[3][1], Z[3][1]) * azimuthWeight -- 方位角

    -- イノベーション共分散 S = H * P_pred * H^T + R
    S = sum(mul(H, P_predicted, T(H)), R_matrix)
    S_inv = inv3(S)
    if S_inv == nil then return math.huge end

    -- 誤差指標 epsilon (マハラノビス距離) の計算: epsilon = Y^T * S^-1 * Y
    epsilon = 1.0
    epsilon_matrix = mul(T(Y), S_inv, Y)
    if epsilon_matrix and epsilon_matrix[1] and epsilon_matrix[1][1] then
        epsilon = epsilon_matrix[1][1]
    end
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
function initializeFilterState(initialObservation, tick, trackID, radarID, beamAzimuth)
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
        stateTick = tick,    -- 状態X/Pが対応している観測tick
        lastSeenTick = tick, -- 最後に観測が紐付けられたTick
        lastRadarID = radarID or 0,
        lastBeamAzimuth = beamAzimuth or 0,
        hits = 1,            -- ★ 連続ヒット数 (信頼性評価用)
        misses = 0,          -- ★ 連続ミス数 (削除判定用)
        integralError = 0    -- ★ PIコントローラーの積分状態を初期化
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    currentTick = currentTick + 1

    -- 出力は毎tick初期化し、選択した1トラックだけを書き込む
    for channel = 1, 32 do output.setNumber(channel, 0) end
    for channel = 1, 32 do output.setBool(channel, false) end

    local frameValid = input.getBool(31)
    local radarID = input.getNumber(31)
    local beamAzimuth = 0
    local observationTick = math.floor(input.getNumber(32) + OBSERVATION_TICK_OFFSET + 0.5)
    local currentObservations = {}

    -- 各一次フィルターと本Luaは同時に起動する前提
    -- 固定差がある場合はOBS_TICK_OFFSETで補正する
    if observationTick > currentTick then observationTick = currentTick end
    if observationTick <= 0 or observationTick <= lastFrameTick then frameValid = false end

    if frameValid then
        lastFrameTick = observationTick
        ownGlobalPos = { input.getNumber(25), input.getNumber(26), input.getNumber(27) }
        ownEuler = {
            Pitch = input.getNumber(28),
            Yaw = input.getNumber(29),
            Roll = input.getNumber(30)
        }
        ownOrientation = eulerZYX_to_quaternion(ownEuler.Roll, ownEuler.Yaw, ownEuler.Pitch)
        if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end

        -- 全観測を探知更新時の自機位置姿勢でグローバル座標へ変換する
        for target = 1, MAX_RADAR_TARGETS do
            local base = BASE_CHANNEL * target
            local distance = input.getNumber(base - 2)
            if distance > 0 then
                local localAzimuth = input.getNumber(base - 1)
                local localElevation = input.getNumber(base)
                local localPosition = polarCoordsToLocalCoords(
                    distance, localElevation, localAzimuth
                )
                local globalPosition = localToGlobalCoords(
                    localPosition, ownGlobalPos, ownOrientation
                )
                local relative = vectorSub(globalPosition, ownGlobalPos)
                local checkedDistance = vectorMagnitude(relative)
                if checkedDistance > 1e-6 then
                    currentObservations[#currentObservations + 1] = {
                        distance = distance,
                        azimuth = math.atan(relative[1], relative[3]),
                        elevation = math.asin(
                            math.max(-1, math.min(1, relative[2] / checkedDistance))
                        ),
                        globalX = globalPosition[1],
                        globalY = globalPosition[2],
                        globalZ = globalPosition[3]
                    }
                end
            end
        end
    end

    local predictedTracks = {}
    local deleteIDs = {}
    local associations = {}
    local assignedObservations = {}
    local assignedTracks = {}

    if frameValid and #currentObservations > 0 then
        -- 保存状態を到着tickではなく観測tickまで予測する
        for trackID, track in pairs(trackedTargets) do
            if observationTick >= track.stateTick then
                local dt = (observationTick - track.stateTick) * DT
                local XPredicted, PPredicted, newIntegralError = predictStep(track, dt)
                if XPredicted and PPredicted then
                    predictedTracks[trackID] = {
                        id = trackID,
                        X = XPredicted,
                        P = PPredicted,
                        track = track,
                        integralError = newIntegralError
                    }
                else
                    deleteIDs[#deleteIDs + 1] = trackID
                end
            end
        end

        for _, trackID in ipairs(deleteIDs) do trackedTargets[trackID] = nil end
        deleteIDs = {}

        -- マハラノビス距離が小さい組み合わせから一対一に割り当てる
        for trackID, predicted in pairs(predictedTracks) do
            for observationIndex, observation in ipairs(currentObservations) do
                local epsilon, Y, SInverse, H, R = calculateInnovation(
                    predicted.X, predicted.P, observation, ownGlobalPos
                )
                if epsilon < DATA_ASSOCIATION_EPSILON_THRESHOLD then
                    associations[#associations + 1] = {
                        epsilon = epsilon,
                        trackID = trackID,
                        observationIndex = observationIndex,
                        Y = Y,
                        SInverse = SInverse,
                        H = H,
                        R = R
                    }
                end
            end
        end

        table.sort(associations, function(a, b) return a.epsilon < b.epsilon end)

        for _, association in ipairs(associations) do
            local trackID = association.trackID
            local observationIndex = association.observationIndex
            if not assignedTracks[trackID] and not assignedObservations[observationIndex] then
                local predicted = predictedTracks[trackID]
                predicted.association = association
                assignedTracks[trackID] = true
                assignedObservations[observationIndex] = true
            end
        end

        -- 割り当てられたトラックだけを観測tickで更新する
        for trackID, predicted in pairs(predictedTracks) do
            local track = predicted.track
            local association = predicted.association
            if association then
                local observation = currentObservations[association.observationIndex]
                local XUpdated, PUpdated, epsilon, success = updateStep(
                    predicted.X,
                    predicted.P,
                    observation,
                    ownGlobalPos,
                    association.Y,
                    association.SInverse,
                    association.H,
                    association.R
                )
                if success then
                    track.X = XUpdated
                    track.P = PUpdated
                    track.epsilon = epsilon
                    track.integralError = predicted.integralError
                    track.stateTick = observationTick
                    track.lastSeenTick = observationTick
                    track.lastRadarID = radarID
                    track.lastBeamAzimuth = beamAzimuth
                    track.hits = track.hits + 1
                    track.misses = 0
                else
                    track.misses = track.misses + 1
                end
            end
        end

        -- 未割当観測から新規トラックを作る
        for observationIndex, observation in ipairs(currentObservations) do
            if not assignedObservations[observationIndex] then
                local trackID = nextTrackID
                nextTrackID = nextTrackID + 1
                trackedTargets[trackID] = initializeFilterState(
                    observation, observationTick, trackID, radarID, beamAzimuth
                )
            end
        end
    end

    -- 観測時刻からの経過が閾値を超えたトラックを削除する
    for trackID, track in pairs(trackedTargets) do
        if currentTick - track.lastSeenTick > TARGET_LOST_THRESHOLD_TICKS then
            deleteIDs[#deleteIDs + 1] = trackID
        end
    end
    for _, trackID in ipairs(deleteIDs) do trackedTargets[trackID] = nil end

    -- 確定トラックをラウンドロビンで1件出力する
    -- 4セクターを時分割取得しても目標を飛ばさないよう、既定では同じ目標を4tick保持する
    local confirmedTracks = {}
    for _, track in pairs(trackedTargets) do
        if track.hits > 1 then confirmedTracks[#confirmedTracks + 1] = track end
    end
    table.sort(confirmedTracks, function(a, b) return a.id < b.id end)

    local outputTrack
    if #confirmedTracks > 0 then
        local outputSlot = math.floor((currentTick - 1) / OUTPUT_HOLD_TICKS)
        if trackedTargetsIndex == 0 then
            trackedTargetsIndex = 1
        elseif outputSlot ~= lastOutputSlot then
            trackedTargetsIndex = trackedTargetsIndex + 1
        end
        if trackedTargetsIndex > #confirmedTracks then trackedTargetsIndex = 1 end
        lastOutputSlot = outputSlot
        outputTrack = confirmedTracks[trackedTargetsIndex]
    else
        trackedTargetsIndex = 0
        lastOutputSlot = math.floor((currentTick - 1) / OUTPUT_HOLD_TICKS)
    end

    if outputTrack then
        local state = outputTrack.X
        local age = math.max(0, currentTick - outputTrack.stateTick)
        local dt = (age + LOGIC_DELAY) * DT
        local halfDt2 = dt * dt * 0.5

        output.setNumber(1, state[1][1] + state[2][1] * dt + state[3][1] * halfDt2)
        output.setNumber(2, state[4][1] + state[5][1] * dt + state[6][1] * halfDt2)
        output.setNumber(3, state[7][1] + state[8][1] * dt + state[9][1] * halfDt2)
        output.setNumber(4, state[2][1] + state[3][1] * dt)
        output.setNumber(5, state[5][1] + state[6][1] * dt)
        output.setNumber(6, state[8][1] + state[9][1] * dt)
        output.setNumber(7, state[3][1])
        output.setNumber(8, state[6][1])
        output.setNumber(9, state[9][1])
        output.setNumber(10, outputTrack.stateTick)
        output.setNumber(11, age)
        output.setNumber(12, outputTrack.id)
        output.setNumber(13, outputTrack.hits)
        output.setNumber(14, outputTrack.misses)
        output.setNumber(15, SECTOR_ID)
        output.setNumber(16, outputTrack.lastRadarID)
        output.setNumber(17, outputTrack.lastBeamAzimuth)
        output.setNumber(18, currentTick + LOGIC_DELAY)
        output.setNumber(32, outputTrack.epsilon)
        output.setBool(1, true)
    end
end
