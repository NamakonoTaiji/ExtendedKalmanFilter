--[[
================================================================================
 Sonar_EKF_Tracker_Core.lua (v1.1 - 分割版, Max 6 Targets)
================================================================================
 機能:
 - 別の「距離計算ブロック」から送られてくる最大6目標分のソナー観測値
   （距離、ローカル方位角、ローカル仰俯角）と自機センサー情報を用いて、
   単一目標の位置と速度を拡張カルマンフィルタ(EKF)で推定する。
 - 外部から指定された初期目標位置に最も近い観測でEKFを初期化する。
 - データアソシエーション（予測位置とのε最小化）を行う。
 - 目標ロスト判定と予測ステップを実行する。
 - 推定された目標の位置・速度、トラッキング状態などをコンポジット信号で出力する。

 座標系:
 - グローバル座標: Physics Sensor 基準 (X:東, Y:上, Z:北, 左手系)
 - ローカル座標: センサー基準 (Z:前方, X:右方, Y:上方, 左手系)
 - 角度単位: ラジアン

 入力 (コンポジット信号):
 - オンオフ 1-6:   Echo Detected Flag Ch 1-6 (距離計算ブロックから) <- EKF更新トリガー (6ch)
 - オンオフ 7:   初期化要求フラグ (外部から)
 - 数値 1:      Ch1 Distance (メートル) (距離計算ブロックから)
 - 数値 2:      Ch1 Local Azimuth (Radian) (距離計算ブロックから)
 - 数値 3:      Ch1 Local Elevation (Radian) (距離計算ブロックから)
 - 数値 4:      Ch2 Distance (メートル)
 - 数値 5:      Ch2 Local Azimuth (Radian)
 - 数値 6:      Ch2 Local Elevation (Radian)
 - ... (Ch6 まで続く) ...
 - 数値 16:     Ch6 Distance (メートル)
 - 数値 17:     Ch6 Local Azimuth (Radian)
 - 数値 18:     Ch6 Local Elevation (Radian) (目標データ計 18ch)
 - 数値 19:     Physics Sensor Global Pos X
 - 数値 20:     Physics Sensor Global Pos Y
 - 数値 21:     Physics Sensor Global Pos Z (自機位置計 3ch)
 - 数値 22:     Physics Sensor Euler Pitch (Radian)
 - 数値 23:     Physics Sensor Euler Yaw (Radian)
 - 数値 24:     Physics Sensor Euler Roll (Radian) (自機姿勢計 3ch)
 - 数値 25:     初期目標グローバル位置 X (外部から)
 - 数値 26:     初期目標グローバル位置 Y (外部から)
 - 数値 27:     初期目標グローバル位置 Z (外部から) (初期目標計 3ch)
 -- 合計: オンオフ 7ch, 数値 27ch (32ch以内に収まる)

 出力 (コンポジット信号):
 - オンオフ 1:   (未使用)
 - オンオフ 2:   トラッキング成功フラグ (isTracking)
 - 数値 1:      推定目標グローバル位置 X
 - 数値 2:      推定目標グローバル位置 Y
 - 数値 3:      推定目標グローバル位置 Z
 - 数値 4:      推定目標グローバル速度 Vx
 - 数値 5:      推定目標グローバル速度 Vy
 - 数値 6:      推定目標グローバル速度 Vz
 - 数値 7:      初期目標グローバル位置 X (パススルー)
 - 数値 8:      初期目標グローバル位置 Y (パススルー)
 - 数値 9:      初期目標グローバル位置 Z (パススルー)
 - 数値 10:     自機グローバル位置 X (パススルー)
 - 数値 11:     自機グローバル位置 Y (パススルー)
 - 数値 12:     自機グローバル位置 Z (パススルー)
 - 数値 13:     自機オイラー角 Pitch (パススルー)
 - 数値 14:     自機オイラー角 Yaw (パススルー)
 - 数値 15:     自機オイラー角 Roll (パススルー)
 - 数値 32:     最新のイプシロン ε (EKF信頼度)
================================================================================
]]

-- 定数
local PI = math.pi
local PI2 = PI * 2
local TICKS_PER_SECOND = 60
local DT = 1 / TICKS_PER_SECOND
local MAX_TARGETS = 6 -- 最大目標数を6に変更
local NUM_STATES = 6

-- プロパティ読み込み (EKFパラメータ - 変更なし)
local DATA_ASSOCIATION_EPSILON_THRESHOLD = property.getNumber("D_ASOC_EPS") or 100.0
local TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST") or 120
local PROCESS_NOISE_BASE = property.getNumber("P_BASE") or 1.0
local PROCESS_NOISE_ADAPTIVE_SCALE = property.getNumber("P_ADPT") or 5.0
local PROCESS_NOISE_EPSILON_THRESHOLD = property.getNumber("P_NOISE_EPS_THRS") or 50.0
local PROCESS_NOISE_EPSILON_SLOPE = property.getNumber("P_NOISE_EPS_SLOPE") or 0.1
local PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT") or 1.01
local R0_DIST_VAR_COEFF = (0.02 ^ 2 / 12)
local R0_ANGLE_VAR = ((2e-3 * PI2) ^ 2 / 12)
local INITIAL_VELOCITY_VARIANCE = (100 ^ 2)
local LOGIC_DELAY_PREDICTION = property.getNumber("LOGIC_DELAY") or 0

-- グローバル変数 (変更なし)
local trackedTarget = nil
local currentTick = 0
local isTracking = false

-- 単位行列 & 観測ノイズテンプレート (変更なし)
local identityMatrix6x6 = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }
local OBSERVATION_NOISE_MATRIX_TEMPLATE = { { R0_DIST_VAR_COEFF, 0, 0 }, { 0, R0_ANGLE_VAR, 0 }, { 0, 0, R0_ANGLE_VAR } }

--------------------------------------------------------------------------------
-- ヘルパー関数 (ここから - 変更なし)
--------------------------------------------------------------------------------
-- === ベクトル演算 ===
function vectorMagnitude(v)
    local x, y, z = v[1] or v.x or 0, v[2] or v.y or 0, v[3] or v.z or 0; return math.sqrt(x * x + y * y + z * z)
end

function vectorNormalize(v)
    local mag = vectorMagnitude(v); if mag < 1e-9 then
        return { 0, 0, 0 }
    else
        local x, y, z = v[1] or v.x or 0, v[2] or v.y or 0, v[3] or v.z or 0; return { x / mag, y / mag, z / mag }
    end
end

function vectorAdd(v1, v2)
    local x1, y1, z1, x2, y2, z2 = v1[1] or v1.x or 0, v1[2] or v1.y or 0, v1[3] or v1.z or 0, v2[1] or v2.x or 0,
        v2[2] or v2.y or 0, v2[3] or v2.z or 0; return { x1 + x2, y1 + y2, z1 + z2 }
end

function vectorSub(v1, v2)
    local x1, y1, z1, x2, y2, z2 = v1[1] or v1.x or 0, v1[2] or v1.y or 0, v1[3] or v1.z or 0, v2[1] or v2.x or 0,
        v2[2] or v2.y or 0, v2[3] or v2.z or 0; return { x1 - x2, y1 - y2, z1 - z2 }
end

function vectorScalarMul(s, v)
    local x, y, z = v[1] or v.x or 0, v[2] or v.y or 0, v[3] or v.z or 0; return { s * x, s * y, s * z }
end

function vectorDot(v1, v2)
    local x1, y1, z1, x2, y2, z2 = v1[1] or v1.x or 0, v1[2] or v1.y or 0, v1[3] or v1.z or 0, v2[1] or v2.x or 0,
        v2[2] or v2.y or 0, v2[3] or v2.z or 0; return x1 * x2 + y1 * y2 + z1 * z2
end

function vectorCross(v1, v2)
    local x1, y1, z1, x2, y2, z2 = v1[1] or v1.x or 0, v1[2] or v1.y or 0, v1[3] or v1.z or 0, v2[1] or v2.x or 0,
        v2[2] or v2.y or 0, v2[3] or v2.z or 0; return { y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2 }
end

-- === 行列演算 ===
function zeros(r, c)
    local m = {}; for i = 1, r do
        m[i] = {}; for j = 1, c do m[i][j] = 0 end
    end; return m
end

function MatrixCopy(M)
    local N = {}; for r, row in ipairs(M) do N[r] = { table.unpack(row) } end; return N
end

function scalar(s, M)
    local R = zeros(#M, #M[1]); for r = 1, #M do for c = 1, #M[1] do R[r][c] = M[r][c] * s end end; return R
end

function sum(A, B)
    local R = zeros(#A, #A[1]); for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] + B[r][c] end end; return R
end

function sub(A, B)
    local R = zeros(#A, #A[1]); for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] - B[r][c] end end; return R
end

function mul(...)
    local m, A, R, B, s = { ... }, m[1]; for i = 2, #m do
        B = m[i]; R = zeros(#A, #B[1]); for r = 1, #A do
            for c = 1, #B[1] do
                s = 0; for k = 1, #B do s = s + A[r][k] * B[k][c] end
                R[r][c] = s
            end
        end
        A = R
    end
    return A
end

function T(M)
    local r, c, R = #M, #M[1], zeros(c, r); for i = 1, r do for j = 1, c do R[j][i] = M[i][j] end end
    return R
end

function inv(M)
    local n = #M; local A = MatrixCopy(M); local I = zeros(n, n); for i = 1, n do I[i][i] = 1 end; for i = 1, n do
        local p = i; for j = i + 1, n do if math.abs(A[j][i]) > math.abs(A[p][i]) then p = j end end
        A[i], A[p] = A[p], A[i]; I[i], I[p] = I[p], I[i]; local d = A[i][i]; if math.abs(d) < 1e-12 then return nil end; for j = i, n do
            A[i][j] =
                A[i][j] / d
        end; for j = 1, n do I[i][j] = I[i][j] / d end; for j = 1, n do
            if i ~= j then
                local f = A[j][i]; for k = i, n do A[j][k] = A[j][k] - f * A[i][k] end; for k = 1, n do
                    I[j][k] = I[j]
                        [k] - f * I[i][k]
                end
            end
        end
    end
    return I
end

-- === クォータニオン演算 ===
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2 = q_a[1], q_a[2], q_a[3], q_a[4], q_b[1], q_b[2], q_b[3], q_b[4]; local w = w1 *
        w2 - x1 * x2 - y1 * y2 - z1 * z2; local x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2; local y = w1 * y2 - x1 * z2 +
        y1 * w2 + z1 * x2; local z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; return { w, x, y, z }
end

function eulerZYX_to_quaternion(r, y, p)
    local hr, hy, hp = r * 0.5, y * 0.5, p * 0.5; local cr, sr, cy, sy, cp, sp = math.cos(hr), math.sin(hr), math.cos(hy),
        math.sin(hy), math.cos(hp), math.sin(hp); local w = cr * cy * cp + sr * sy * sp; local x = cr * cy * sp -
        sr * sy * cp; local y = cr * sy * cp + sr * cy * sp; local z = sr * cy * cp - cr * sy * sp; return { w, x, y, z }
end

function rotateVectorByQuaternion(v, q)
    local px, py, pz = v[1] or 0, v[2] or 0, v[3] or 0; local p = { 0, px, py, pz }; local qc = { q[1], -q[2], -q[3], -q
    [4] }; local t = multiplyQuaternions(q, p); local p_prime = multiplyQuaternions(t, qc); return { p_prime[2], p_prime
        [3], p_prime[4] }
end

function rotateVectorByInverseQuaternion(v, q)
    local px, py, pz = v[1] or 0, v[2] or 0, v[3] or 0; local p = { 0, px, py, pz }; local qc = { q[1], -q[2], -q[3], -q
    [4] }; local t = multiplyQuaternions(qc, p); local p_prime = multiplyQuaternions(t, q); return { p_prime[2], p_prime
        [3], p_prime[4] }
end

-- === 座標変換 ===
function localToGlobalCoords(l, oP, oQ)
    local gR = rotateVectorByQuaternion(l, oQ); return { x = gR[1] + oP.x, y = gR[2] + oP.y, z = gR[3] + oP.z }
end

function globalToLocalCoords(g, oP, oQ)
    local gR = { g.x - oP.x, g.y - oP.y, g.z - oP.z }; local lV = rotateVectorByInverseQuaternion(gR, oQ); return {
        x =
            lV[1],
        y = lV[2],
        z = lV[3]
    }
end

-- === EKF 関連関数 ===
function calculateAngleDifference(a1, a2)
    local d = a2 - a1; while d <= -PI do d = d + PI2 end; while d > PI do d = d - PI2 end; return d
end

function getObservationJacobianAndPrediction(X, oP)
    local tx, ty, tz = X[1][1], X[3][1], X[5][1]; local rx, ry, rz = tx - oP.x, ty - oP.y, tz - oP.z; local r2 = rx * rx +
        ry * ry + rz * rz; local rh2 = rx * rx + rz * rz; if r2 < 1e-9 then r2 = 1e-9 end; if rh2 < 1e-9 then rh2 = 1e-9 end; local r =
        math.sqrt(r2); local rh = math.sqrt(rh2); local pD = r; local pE = math.asin(clamp(ry / r, -1, 1)); local pA =
        math
        .atan(rx, rz); local h = { { pD }, { pE }, { pA } }; local H = zeros(3, NUM_STATES); H[1][1] = rx / r; H[1][3] =
        ry /
        r; H[1][5] = rz / r; H[2][1] = -rx * ry / (r2 * rh); H[2][3] = rh / r2; H[2][5] = -rz * ry / (r2 * rh); H[3][1] =
        rz /
        rh2; H[3][5] = -rx / rh2; return H, h
end

function extendedKalmanFilterUpdate(target, obs, oP)
    local X, P, lastEps, F, Xp, dt2, dt3, dt4, Qb, af, Q, uf, Pp1, Pp, Z, H, h, R, Y, S1, S, Si, K1, K, KY, Xu, KH, IKH, Pu1, Pu2, Pu, eps, epsM; X =
        target.X; P = target.P; lastEps = target.epsilon or 1; F = MatrixCopy(identityMatrix6x6); F[1][2] = DT; F[3][4] =
        DT; F[5][6] =
        DT; Xp = mul(F, X); dt2 = DT * DT; dt3 = dt2 * DT / 2; dt4 = dt3 * DT / 2; Qb = { { dt4, dt3, 0, 0, 0, 0 }, { dt3, dt2, 0, 0, 0, 0 }, { 0, 0, dt4, dt3, 0, 0 }, { 0, 0, dt3, dt2, 0, 0 }, { 0, 0, 0, 0, dt4, dt3 }, { 0, 0, 0, 0, dt3, dt2 } }; af =
        PROCESS_NOISE_BASE +
        PROCESS_NOISE_ADAPTIVE_SCALE /
        (1 + math.exp(-(lastEps - PROCESS_NOISE_EPSILON_THRESHOLD) * PROCESS_NOISE_EPSILON_SLOPE)); Q =
        scalar(af, Qb); uf = PREDICTION_UNCERTAINTY_FACTOR_BASE; Pp1 = mul(F, P, T(F)); Pp = sum(scalar(uf, Pp1), Q); Z = { { obs.distance }, { obs.elevation }, { obs.azimuth } }; H, h =
        getObservationJacobianAndPrediction(Xp, oP); R = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE); R[1][1] = R[1]
        [1] *
        (obs.distance ^ 2); Y = zeros(3, 1); Y[1][1] = Z[1][1] - h[1][1]; Y[2][1] = calculateAngleDifference(h[2][1],
        Z[2]
        [1]); Y[3][1] = calculateAngleDifference(h[3][1], Z[3][1]); S1 = mul(H, Pp, T(H)); S = sum(S1, R); Si = inv(S); if Si == nil then
        return
            X, P, lastEps, false
    end; K1 = mul(Pp, T(H)); K = mul(K1, Si); KY = mul(K, Y); Xu = sum(Xp, KY); KH = mul(K, H); IKH =
        sub(identityMatrix6x6, KH); Pu1 = mul(IKH, Pp, T(IKH)); Pu2 = mul(K, R, T(K)); Pu = sum(Pu1, Pu2); eps = 1; epsM =
        mul(T(Y), Si, Y); if epsM and epsM[1] and epsM[1][1] then eps = epsM[1][1] end; return Xu, Pu, eps, true
end

function initializeFilterState(obs, tick)
    local Xi, Pi, Ri, pvs; Xi = { { obs.globalX }, { 0 }, { obs.globalY }, { 0 }, { obs.globalZ }, { 0 } }; Pi = zeros(
        NUM_STATES, NUM_STATES); Ri = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE); Ri[1][1] = Ri[1][1] *
        (obs.distance ^ 2); pvs = 10; Pi[1][1] = (Ri[3][3] + Ri[1][1]) *
        pvs; Pi[3][3] = (Ri[2][2] + Ri[1][1]) * pvs; Pi[5][5] = (Ri[3][3] + Ri[1][1]) * pvs; Pi[2][2] =
        INITIAL_VELOCITY_VARIANCE; Pi[4][4] = INITIAL_VELOCITY_VARIANCE; Pi[6][6] = INITIAL_VELOCITY_VARIANCE; return {
        X =
            Xi,
        P = Pi,
        epsilon = 1,
        lastTick = tick,
        lastSeenTick = tick
    }
end

-- === その他 ===
function clamp(v, min_v, max_v) return math.max(min_v, math.min(v, max_v)) end

--------------------------------------------------------------------------------
-- ヘルパー関数 (ここまで)
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    -- 関数のローカル変数宣言
    local oP, oE, oQ, initReq, initTgt, initValid, echoFlags, currentObs, i, dist_in, A_in, E_in, cosE, sinE, cosA, sinA, tlpv, tg, rgv, od, oge, oga, assocObs, minEps, bestIdx, Xap, dtp, Fp, i_obs, obs, eps_try, ok_try, Xu, Pu, eps_up, ok_up, tsls, predX, predD

    currentTick = currentTick + 1
    isTracking = false

    -- 1. 入力読み込み
    oP = { x = input.getNumber(19), y = input.getNumber(20), z = input.getNumber(21) }
    oE = { Pitch = input.getNumber(22), Yaw = input.getNumber(23), Roll = input.getNumber(24) }
    oQ = eulerZYX_to_quaternion(oE.Roll, oE.Yaw, oE.Pitch)
    if oQ == nil then oQ = { 1, 0, 0, 0 } end

    initReq = input.getBool(8)
    debug.log("initReq = " .. tostring(initReq))
    initTgt = { x = input.getNumber(25), y = input.getNumber(26), z = input.getNumber(27) }
    initValid = not (initTgt.x == 0 and initTgt.y == 0 and initTgt.z == 0)

    echoFlags = {}
    currentObs = {} -- このTickの観測リストを初期化

    for i = 1, MAX_TARGETS do
        echoFlags[i] = input.getBool(i)                -- ch 2-7
        if echoFlags[i] then
            dist_in = input.getNumber((i - 1) * 3 + 1) -- ch 1, 4, 7, 10, 13, 16
            A_in = input.getNumber((i - 1) * 3 + 2)    -- ch 2, 5, 8, 11, 14, 17 (Radian)
            E_in = input.getNumber((i - 1) * 3 + 3)    -- ch 3, 6, 9, 12, 15, 18 (Radian)

            if dist_in > 0.1 then
                -- 観測値生成
                cosE = math.cos(E_in); sinE = math.sin(E_in); cosA = math.cos(A_in); sinA = math.sin(A_in)
                tlpv = { dist_in * cosE * sinA, dist_in * sinE, dist_in * cosE * cosA } -- Local Pos Vec
                tg = localToGlobalCoords(tlpv, oP, oQ)                                  -- Target Global Pos
                rgv = vectorSub(tg, oP)                                                 -- Relative Global Vec
                od = vectorMagnitude(rgv)                                               -- Obs Dist
                oge = 0; oga = 0                                                        -- Obs Global Elev, Azim
                if od > 1e-6 then
                    oge = math.asin(clamp(rgv[2] / od, -1, 1))
                    oga = math.atan(rgv[1], rgv[3])
                end
                table.insert(currentObs, {
                    distance = od,
                    azimuth = oga,
                    elevation = oge,
                    globalX = tg.x,
                    globalY = tg.y,
                    globalZ = tg.z,
                    channel = i
                })
            end
        end
    end

    -- 2. EKF 初期化 & 更新
    assocObs = nil
    if #currentObs > 0 then
        if trackedTarget == nil or initReq then
            debug.log("init")
            if initValid then
                local minSq = math.huge; local bestObs = nil
                for _, obs in ipairs(currentObs) do
                    local dx = obs.globalX - initTgt.x; local dy = obs.globalY - initTgt.y; local dz = obs.globalZ -
                        initTgt.z
                    local d2 = dx * dx + dy * dy + dz * dz; if d2 < minSq then
                        minSq = d2; bestObs = obs
                    end
                end
                if bestObs ~= nil then
                    trackedTarget = initializeFilterState(bestObs, currentTick)
                    if trackedTarget ~= nil then
                        isTracking = true; assocObs = bestObs
                    end
                end
            end
        else -- EKF 更新
            minEps = DATA_ASSOCIATION_EPSILON_THRESHOLD + 1; bestIdx = -1; Xap = nil
            dtp = (currentTick - trackedTarget.lastTick) * DT
            if dtp >= 0 then
                Fp = MatrixCopy(identityMatrix6x6); Fp[1][2] = dtp; Fp[3][4] = dtp; Fp[5][6] = dtp
                Xap = mul(Fp, trackedTarget.X)
            end
            if Xap ~= nil then
                for i_obs, obs in ipairs(currentObs) do
                    _, _, eps_try, ok_try = extendedKalmanFilterUpdate(
                        { X = Xap, P = trackedTarget.P, epsilon = trackedTarget.epsilon },
                        { distance = obs.distance, azimuth = obs.azimuth, elevation = obs.elevation }, oP)
                    if ok_try and eps_try < minEps then
                        minEps = eps_try; bestIdx = i_obs
                    end
                end
                if bestIdx ~= -1 and minEps <= DATA_ASSOCIATION_EPSILON_THRESHOLD then
                    assocObs = currentObs[bestIdx]
                    Xu, Pu, eps_up, ok_up = extendedKalmanFilterUpdate(trackedTarget,
                        { distance = assocObs.distance, azimuth = assocObs.azimuth, elevation = assocObs.elevation }, oP)
                    if ok_up then
                        debug.log("updated x = " ..
                            trackedTarget.X[1][1] ..
                            ", y = " .. trackedTarget.X[3][1] .. ", z = " .. trackedTarget.X[5][1])
                        trackedTarget.X = Xu; trackedTarget.P = Pu; trackedTarget.epsilon = eps_up
                        trackedTarget.lastTick = currentTick; trackedTarget.lastSeenTick = currentTick
                        isTracking = true
                    end
                end
            end
        end
    end

    -- 3. 目標ロスト判定 & 予測ステップ
    if trackedTarget ~= nil and not isTracking and not (trackedTarget == nil or initReq) then
        tsls = currentTick - (trackedTarget.lastSeenTick or (currentTick - TARGET_LOST_THRESHOLD_TICKS - 1))
        if tsls > TARGET_LOST_THRESHOLD_TICKS then
            trackedTarget = nil
        else
            dtp = (currentTick - trackedTarget.lastTick) * DT
            if dtp > 0 then
                Fp = MatrixCopy(identityMatrix6x6); Fp[1][2] = dtp; Fp[3][4] = dtp; Fp[5][6] = dtp
                predX = mul(Fp, trackedTarget.X)
                -- TODO: Predict covariance P
                if predX then
                    trackedTarget.X = predX; trackedTarget.lastTick = currentTick; isTracking = true
                else
                    trackedTarget = nil
                end
            elseif trackedTarget.X then
                isTracking = true
            end
        end
    end

    -- 4. 出力
    output.setBool(2, isTracking)
    if trackedTarget ~= nil and trackedTarget.X ~= nil then
        predD = LOGIC_DELAY_PREDICTION
        output.setNumber(1, trackedTarget.X[1][1] + trackedTarget.X[2][1] * DT * predD) -- X
        output.setNumber(2, trackedTarget.X[3][1] + trackedTarget.X[4][1] * DT * predD) -- Y
        output.setNumber(3, trackedTarget.X[5][1] + trackedTarget.X[6][1] * DT * predD) -- Z
        output.setNumber(4, trackedTarget.X[2][1])                                      -- Vx
        output.setNumber(5, trackedTarget.X[4][1])                                      -- Vy
        output.setNumber(6, trackedTarget.X[6][1])                                      -- Vz
        output.setNumber(32, trackedTarget.epsilon or 0)                                -- Epsilon
    else
        for i = 1, 6 do output.setNumber(i, 0) end
        output.setNumber(32, 0)
    end
    output.setNumber(7, initTgt.x)
    output.setNumber(8, initTgt.y)
    output.setNumber(9, initTgt.z) -- Initial Target Pos Passthrough
    output.setNumber(10, oP.x)
    output.setNumber(11, oP.y)
    output.setNumber(12, oP.z) -- Own Pos Passthrough
    output.setNumber(13, oE.Pitch)
    output.setNumber(14, oE.Yaw)
    output.setNumber(15, oE.Roll) -- Own Euler Passthrough
end
