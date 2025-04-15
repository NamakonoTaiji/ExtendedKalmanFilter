--[[
  Stormworks EKF Radar Filter - MC3: EKF Tracker (Constant Acceleration Model)
  Based on user's provided EKF code. Modified for 9D Constant Acceleration state.
  Receives processed polar data and self position.
  Performs EKF prediction/update.
]]

--[[ Log/Error Code Mappings (MC3):
  E:z01, E:sc01, E:su01, E:su02, E:mu01, E:mu02, E:tr01 (Helpers)
  E:in01 - inv: Input matrix is not 3x3. E:in02 - inv: Matrix is singular.
  L:init03 - Initializing MC3. L:init04 - MC3 Initialization Complete.
  L:kf01 - KF Predict Step. L:kf02 - KF Update Step.
  E:kf01 - KF Update: Jacobian H calculation failed. E:kf02 - KF Update: Observation func/Innovation calc failed.
  E:kf03 - KF Update: Innovation Covariance S calculation failed. E:kf04 - KF Update: Matrix Inversion S^-1 failed.
  E:kf05 - KF Update: Kalman Gain K calculation failed. E:kf06 - KF Update: State update X calculation failed.
  E:kf07 - KF Update: Covariance update P calculation failed.
  W:kf01 - Skipping KF update (not RDetect or dt=0).
  W:kf02 - Initializing track state from first measurement.
  E:trk01 - Failed to initialize new track (missing initial P?).
  W:init01 - Invalid initial observation, skipping initialization.
]]

--#################################################################
--# 0. コンパクトな行列・ベクトル演算 ヘルパー関数群 (ユーザーコードベース + inv実装)
--#################################################################
function zeros(m, g)
    if not m or not g or m <= 0 or g <= 0 then
        debug.log("E:z01"); return nil
    end; local c = {}; for i = 1, m do
        c[i] = {}; for j = 1, g do c[i][j] = 0 end
    end; return c
end

function scalar(s, M)
    if not s or not M or not M[1] then
        debug.log("E:sc01"); return nil
    end; local r = {}; for i = 1, #M do
        r[i] = {}; for j = 1, #M[1] do
            if M[i] == nil or M[i][j] == nil then
                debug.log("E:sc01"); return nil
            end; r[i][j] = M[i][j] * s
        end
    end; return r
end

function sum(A, B)
    if not A or not B or not A[1] or not B[1] or #A ~= #B or #A[1] ~= #B[1] then
        debug.log("E:su02"); return nil
    end; local r = {}; for i = 1, #A do
        r[i] = {}; for j = 1, #A[1] do
            if A[i] == nil or A[i][j] == nil or B[i] == nil or B[i][j] == nil then
                debug.log("E:su01"); return nil
            end; r[i][j] = A[i][j] + B[i][j]
        end
    end; return r
end

function sub(A, B)
    if not A or not B or not A[1] or not B[1] or #A ~= #B or #A[1] ~= #B[1] then
        debug.log("E:su02"); return nil
    end; local r = {}; for i = 1, #A do
        r[i] = {}; for j = 1, #A[1] do
            if A[i] == nil or A[i][j] == nil or B[i] == nil or B[i][j] == nil then
                debug.log("E:su01"); return nil
            end; r[i][j] = A[i][j] - B[i][j]
        end
    end; return r
end

function mul(A, B)
    if not A or not B or not A[1] or not B[1] or #A[1] ~= #B then
        debug.log("E:mu02"); return nil
    end; local r = {}; for i = 1, #A do r[i] = {} end; for i = 1, #A do
        for j = 1, #B[1] do
            local s = 0; for k = 1, #B do
                if A[i] == nil or A[i][k] == nil or B[k] == nil or B[k][j] == nil then
                    debug.log("E:mu01"); return nil
                end; s = s + A[i][k] * B[k][j]
            end; r[i][j] = s
        end
    end; return r
end

function T(M)
    if not M or not M[1] then
        debug.log("E:tr01"); return nil
    end; local r = {}; for i = 1, #M[1] do
        r[i] = {}; for j = 1, #M do
            if M[j] == nil then
                debug.log("E:tr01"); return nil
            end; r[i][j] = M[j][i]
        end
    end; return r
end

-- 3x3逆行列関数
function inv(M)
    if not M or #M ~= 3 or not M[1] or #M[1] ~= 3 or not M[2] or #M[2] ~= 3 or not M[3] or #M[3] ~= 3 then
        debug.log("E:in01"); return nil
    end; local a = M[1][1]; local b = M[1][2]; local c = M[1][3]; local d = M[2][1]; local e = M[2][2]; local f = M[2]
        [3]; local g = M[3][1]; local h = M[3][2]; local i = M[3][3]; if not a or not b or not c or not d or not e or not f or not g or not h or not i then
        debug.log("E:in01"); return nil
    end; local det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g); if math.abs(det) < 1e-9 then
        debug.log("E:in02"); return nil
    end; local adj = { { (e * i - f * h), (c * h - b * i), (b * f - c * e) }, { (f * g - d * i), (a * i - c * g), (c * d - a * f) }, { (d * h - e * g), (b * g - a * h), (a * e - b * d) } }; local invDet = 1.0 /
        det; local invM = scalar(invDet, adj); return invM
end

--- 行列をログ出力用の文字列に変換するヘルパー関数
function matrixToString(M, fmt)
    if not M then return "nil" end
    fmt = fmt or "%.3f" -- デフォルトのフォーマット
    local rows = {}
    for i = 1, #M do
        local row = {}
        if M[i] then
            for j = 1, #M[i] do
                table.insert(row, string.format(fmt, M[i][j] or 0)) -- nilなら0を表示
            end
        end
        table.insert(rows, "{" .. table.concat(row, ", ") .. "}")
    end
    return "{" .. table.concat(rows, ", ") .. "}"
end

--#################################################################
--# 1. 定数 & モデル定義
--#################################################################
local STATE_DIM = 9
local OBS_DIM = 3

local pi2 = math.pi * 2
local I9 = zeros(STATE_DIM, STATE_DIM);
if I9 then for i = 1, STATE_DIM do I9[i][i] = 1 end else I9 = {} end
-- 観測ノイズ R の基本値 (分散) - 仕様に合わせて修正済
local R_DIST_VAR_FACTOR = (0.02) ^ 2 / 12
local R_ANGLE_VAR = (0.002 * pi2) ^ 2 / 12
local R_DEFAULT = { { R_DIST_VAR_FACTOR * (50 ^ 2), 0, 0 }, { 0, R_ANGLE_VAR, 0 }, { 0, 0, R_ANGLE_VAR } } -- Default R for 50m distance

-- プロセスノイズ Q
local sigma_a = property.getNumber("SigmaA")

local sigma_a_sq = sigma_a * sigma_a
function calculate_Q(dt)
    local dt2 = dt * dt; local dt3 = dt2 * dt; local dt4 = dt3 * dt; local dt5 = dt4 * dt; local q11 = dt5 / 20; local q12 =
        dt4 / 8; local q13 = dt3 / 6; local q22 = dt3 / 3; local q23 = dt2 / 2; local q33 = dt; local Qb = { { q11, q12, q13 }, { q12, q22, q23 }, { q13, q23, q33 } }; Qb =
        scalar(sigma_a_sq, Qb); if not Qb then return zeros(STATE_DIM, STATE_DIM) end; local Q9 = zeros(STATE_DIM,
        STATE_DIM); if not Q9 then return nil end; for axis = 0, 2 do
        for r = 1, 3 do
            for c = 1, 3 do
                Q9[axis * 3 + r][axis * 3 + c] =
                    Qb[r][c]
            end
        end
    end; return Q9
end

-- 状態遷移行列 F (定加速度モデル用)
function calculate_F(dt)
    local dt2h = 0.5 * dt * dt; return { { 1, dt, dt2h, 0, 0, 0, 0, 0, 0 }, { 0, 1, dt, 0, 0, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, dt, dt2h, 0, 0, 0 }, { 0, 0, 0, 0, 1, dt, 0, 0, 0 }, { 0, 0, 0, 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 1, dt, dt2h }, { 0, 0, 0, 0, 0, 0, 0, 1, dt }, { 0, 0, 0, 0, 0, 0, 0, 0, 1 } }
end

local F; local Q

-- 初期共分散行列 P (9x9 - 要調整)
local Initial_P_Matrix = zeros(STATE_DIM, STATE_DIM)
local iposv = 1000 ^ 2; local ivelv = 1000 ^ 2; local iaccv = 100 ^ 2
-- プロパティから読み込み (オプション)
local Q_ADAPT_SCALE_prop = property.getNumber("QAdaptScale")
local Q_ADAPT_EPS_THR_prop = property.getNumber("QAdaptEpsThr")
local Q_ADAPT_STEEPNESS_prop = property.getNumber("QAdaptSteep")
if Q_ADAPT_SCALE_prop then
    Q_ADAPT_SCALE = Q_ADAPT_SCALE_prop; debug.log("I:qsc")
end
if Q_ADAPT_EPS_THR_prop then
    Q_ADAPT_EPS_THR = Q_ADAPT_EPS_THR_prop; debug.log("I:qet")
end
if Q_ADAPT_STEEPNESS_prop then
    Q_ADAPT_STEEPNESS = Q_ADAPT_STEEPNESS_prop; debug.log("I:qst")
end
if not Q_ADAPT_SCALE_prop or not Q_ADAPT_EPS_THR_prop or not Q_ADAPT_STEEPNESS_prop then debug.log("W:init04") end

if Initial_P_Matrix then
    for i = 1, STATE_DIM do
        if (i - 1) % 3 == 0 then
            Initial_P_Matrix[i][i] =
                iposv
        elseif (i - 1) % 3 == 1 then
            Initial_P_Matrix[i][i] = ivelv
        else
            Initial_P_Matrix[i][i] = iaccv
        end
    end
else
    Initial_P_Matrix = {}
end
-- 入力チャンネル定義 (MC1から - ループ用のみ)
local IN_DIST_CH     = function(i) return (i - 1) * 3 + 1 end -- Num Ch 1, 4, ...
local IN_G_AZIM_CH   = function(i) return (i - 1) * 3 + 2 end -- Num Ch 2, 5, ...
local IN_G_ELEV_CH   = function(i) return (i - 1) * 3 + 3 end -- Num Ch 3, 6, ...
local IN_NEW_DATA_CH = function(i) return 6 + i end           -- Bool Ch 7, 8, ...
-- 出力チャンネル定義 (モニターや他のMCへ - ループ用のみ)
local OUT_ACTIVE_CH  = function(i) return i end               -- Bool Ch 1-6
local OUT_POS_X_CH   = function(i) return (i - 1) * 6 + 1 end -- Num Ch 1, 7, ...
local OUT_POS_Y_CH   = function(i) return (i - 1) * 6 + 2 end -- Num Ch 2, 8, ...
local OUT_POS_Z_CH   = function(i) return (i - 1) * 6 + 3 end -- Num Ch 3, 9, ...

local OUT_VEL_X_CH   = function(i) return (i - 1) * 6 + 4 end -- Num Ch 4, 10, ..
local OUT_VEL_Y_CH   = function(i) return (i - 1) * 6 + 5 end -- Num Ch 5, 11, ..
local OUT_VEL_Z_CH   = function(i) return (i - 1) * 6 + 6 end -- Num Ch 6, 12, ..

--#################################################################
--# 2. グローバル変数 / 状態変数
--#################################################################
local X              = zeros(STATE_DIM, 1)
local P              = Initial_P_Matrix or zeros(STATE_DIM, STATE_DIM)
local isInit         = true
local epsilon        = 0
local prev_epsilon   = 0
local tickCounter    = 0

--#################################################################
--# 3. EKF コア関数
--#################################################################

--- 観測関数 h(x): 状態ベクトルから観測値（距離、仰角、方位角）を計算 (GPS座標系基準)
-- @param x_vec table 状態ベクトル (9x1) {x,vx,ax, y,vy,ay, z,vz,az} - 目標の絶対座標 (East, North, Up)
-- @param self_pos table 自機位置 {sx, sy, sz} (East, North, Up) - 自機の絶対座標
-- @return table or nil 観測ベクトル (3x1) {dist}, {elev}, {azim} - 自機から見た相対的な極座標
function observationFunction(x_vec, self_pos)
    if not x_vec or not self_pos or not x_vec[1] or not x_vec[4] or not x_vec[7] or not self_pos[1] or not self_pos[2] or not self_pos[3] then return nil end
    -- 目標の絶対位置 (East, North, Up)
    local target_x = x_vec[1][1]; local target_y = x_vec[4][1]; local target_z = x_vec[7][1]
    -- 自機位置 (East, North, Up)
    local self_x = self_pos[1]; local self_y = self_pos[2]; local self_z = self_pos[3]

    -- 自機から目標への相対ベクトル (East, North, Up)
    local rel_x = target_x - self_x -- Relative East
    local rel_y = target_y - self_y -- Relative North
    local rel_z = target_z - self_z -- Relative Up

    -- 相対ベクトルから距離、仰角、方位角を計算
    local r_sq = rel_x * rel_x + rel_y * rel_y + rel_z * rel_z
    if r_sq < 1e-9 then return { { 0 }, { 0 }, { 0 } } end -- 目標が自機と同一地点の場合、距離0、角度0を返す (エラーよりはまし)
    local r = math.sqrt(r_sq)                              -- Distance

    local rh_sq = rel_x * rel_x + rel_y * rel_y            -- Horizontal distance squared
    local rh = math.sqrt(rh_sq > 1e-9 and rh_sq or 1e-9)   -- Horizontal distance (avoid zero division)

    -- 仰角 (Elevation): asin(Up / Distance) - Range [-pi/2, pi/2]
    local elev = math.asin(rel_z / r)

    -- 方位角 (Azimuth): atan(East / North) - Range [-pi, pi], North=0, East=pi/2
    local azim = math.atan(rel_x, rel_y) -- Using math.atan(y, x) equivalent

    return { { r }, { elev }, { azim } }
end

--- ヤコビアン行列 H = ∂h/∂X の計算 (3x9 for CA model) ***[修正済み]***
-- @param x_vec table 状態ベクトル (9x1) - 目標の絶対座標
-- @param self_pos table 自機位置 {sx, sy, sz} (East, North, Up) - 自機の絶対座標
-- @return table or nil ヤコビアン行列 (3x9)
function calculateJacobianH(x_vec, self_pos)
    if not x_vec or not self_pos or not x_vec[1] or not x_vec[4] or not x_vec[7] or not self_pos[1] or not self_pos[2] or not self_pos[3] then return nil end
    -- 目標の絶対位置 (East, North, Up)
    local target_x = x_vec[1][1]; local target_y = x_vec[4][1]; local target_z = x_vec[7][1]
    -- 自機位置 (East, North, Up)
    local self_x = self_pos[1]; local self_y = self_pos[2]; local self_z = self_pos[3]

    -- 自機から目標への相対ベクトル (East, North, Up)
    local x = target_x - self_x -- Relative East
    local y = target_y - self_y -- Relative North
    local z = target_z - self_z -- Relative Up

    -- 必要な値を計算 (ゼロ除算回避含む)
    local r_sq = x * x + y * y + z * z
    if r_sq < 1e-9 then return zeros(OBS_DIM, STATE_DIM) end -- Avoid division by zero, return zero Jacobian
    local r = math.sqrt(r_sq)
    local rh_sq = x * x + y * y
    if rh_sq < 1e-9 then rh_sq = 1e-9 end -- Avoid division by zero for azimuth calculation
    local rh = math.sqrt(rh_sq)
    local r3 = r_sq * r
    local rh2 = rh_sq

    local H = zeros(OBS_DIM, STATE_DIM) -- Initialize 3x9 Jacobian with zeros

    -- 偏微分を計算 (∂h / ∂(target_pos))
    -- h = {dist, elev, azim}
    -- X = {pos_e, vel_e, acc_e, pos_n, vel_n, acc_n, pos_u, vel_u, acc_u}

    -- Row 1: ∂dist/∂X = [∂r/∂x_e, 0, 0, ∂r/∂x_n, 0, 0, ∂r/∂x_u, 0, 0]
    H[1][1] = x / r -- ∂r/∂x_e
    H[1][4] = y / r -- ∂r/∂x_n
    H[1][7] = z / r -- ∂r/∂x_u

    -- Row 2: ∂elev/∂X = [∂e/∂x_e, 0, 0, ∂e/∂x_n, 0, 0, ∂e/∂x_u, 0, 0] where e = asin(z/r)
    -- ∂e/∂x = (1 / sqrt(1 - (z/r)^2)) * ∂(z/r)/∂x
    -- ∂(z/r)/∂x_e = (r*0 - z*(x/r)) / r^2 = -zx / r^3
    -- ∂(z/r)/∂x_n = (r*0 - z*(y/r)) / r^2 = -zy / r^3
    -- ∂(z/r)/∂x_u = (r*1 - z*(z/r)) / r^2 = (r^2 - z^2) / r^3 = (x^2+y^2) / r^3 = rh^2 / r^3
    local term_elev_denom_sq = 1 - (z / r) ^ 2
    if term_elev_denom_sq < 1e-9 then term_elev_denom_sq = 1e-9 end -- Avoid sqrt(0) or negative
    local term_elev = 1 / math.sqrt(term_elev_denom_sq)

    H[2][1] = term_elev * (-z * x / r3) -- ∂e/∂x_e
    H[2][4] = term_elev * (-z * y / r3) -- ∂e/∂x_n
    H[2][7] = term_elev * (rh2 / r3)    -- ∂e/∂x_u

    -- Row 3: ∂azim/∂X = [∂a/∂x_e, 0, 0, ∂a/∂x_n, 0, 0, ∂a/∂x_u, 0, 0] where a = atan(x,y) = atan(East, North)
    -- ∂a/∂x_e = (1 / (1 + (x/y)^2)) * (1/y) = y / (y^2 + x^2) = y / rh^2  <-- Error in formula, should be atan2(y,x) derivative
    -- Correct derivative for atan(y=East, x=North): atan(x_e, x_n)
    -- ∂a/∂x_e = (1 / (1 + (x_e/x_n)^2)) * (1/x_n) = x_n / (x_n^2 + x_e^2) = y / rh2
    -- ∂a/∂x_n = (1 / (1 + (x_e/x_n)^2)) * (-x_e/x_n^2) = -x_e / (x_n^2 + x_e^2) = -x / rh2
    -- ∂a/∂x_u = 0
    H[3][1] = y / rh2  -- ∂a/∂x_e
    H[3][4] = -x / rh2 -- ∂a/∂x_n
    H[3][7] = 0        -- ∂a/∂x_u

    return H
end

--- EKF 更新ステップ (Joseph form for covariance update) ***[修正済み]***
-- @param F_current table 状態遷移行列 (9x9)
-- @param Q_current table プロセスノイズ共分散 (9x9)
-- @param R_obs table 観測ノイズ共分散 (3x3)
-- @param Z_obs table 実際の観測ベクトル (3x1) {dist}, {elev}, {azim}
-- @param X_state table 現在の状態ベクトル (9x1)
-- @param P_cov table 現在の共分散行列 (9x9)
-- @param current_self_pos table 現在の自機位置 {sx, sy, sz} <-- ***追加***
-- @return table, table, number or nil,nil,nil 更新後の X, P, epsilon
function EKF_Update(F_current, Q_current, R_obs, Z_obs, X_state, P_cov, current_self_pos) -- 引数追加
    -- === 予測 ===
    local X_pred = mul(F_current, X_state)

    local P_pred = sum(mul(F_current, mul(P_cov, T(F_current))), Q_current)
    if not X_pred or not P_pred then return nil, nil, nil end

    -- === 更新 ===
    -- 観測予測 h(x_pred) - 自機位置を渡す
    local Z_pred = observationFunction(X_pred, current_self_pos)
    if not Z_pred then
        return nil, nil, nil
    end

    local Y = sub(Z_obs, Z_pred) -- イノベーション Y = Z_obs - h(x_pred)
    if not Y then
        return nil, nil, nil
    end
    if Y[3][1] then Y[3][1] = (Y[3][1] + math.pi) % pi2 - math.pi end -- 方位角の差を正規化

    -- ヤコビアン H = ∂h/∂X at X_pred - 自機位置を渡す
    local H = calculateJacobianH(X_pred, current_self_pos)
    if not H then
        debug.log("E:kf01"); return nil, nil, nil
    end
    -- (以降の計算は変更なし、ただし H が正しく計算されていることが前提)
    local PHT = mul(P_pred, T(H)); if not PHT then
        debug.log("E:kf03"); return nil, nil, nil
    end
    local HPHT = mul(H, PHT); if not HPHT then
        debug.log("E:kf03"); return nil, nil, nil
    end
    local S = sum(HPHT, R_obs); if not S then
        debug.log("E:kf03"); return nil, nil, nil
    end
    local S_inv = inv(S); if not S_inv then
        debug.log("E:kf04"); return nil, nil, nil
    end
    local epsilon_val = 0; local Y_T = T(Y); local temp_eps = mul(Y_T, S_inv); local eps_mat = mul(temp_eps, Y)
    if eps_mat and eps_mat[1] and eps_mat[1][1] then epsilon_val = eps_mat[1][1] end
    local K = mul(PHT, S_inv); if not K then
        debug.log("E:kf05"); return nil, nil, nil
    end
    local KY = mul(K, Y); if not KY then
        debug.log("E:kf06"); return nil, nil, nil
    end
    local X_new = sum(X_pred, KY); if not X_new then
        debug.log("E:kf06"); return nil, nil, nil
    end
    local KH = mul(K, H); if not KH then
        debug.log("E:kf07"); return nil, nil, nil
    end
    local I_KH = sub(I9, KH); if not I_KH then
        debug.log("E:kf07"); return nil, nil, nil
    end
    local P1 = mul(mul(I_KH, P_pred), T(I_KH)); if not P1 then
        debug.log("E:kf07"); return nil, nil, nil
    end
    local KRT = mul(K, mul(R_obs, T(K))); if not KRT then
        debug.log("E:kf07"); return nil, nil, nil
    end
    local P_new = sum(P1, KRT); if not P_new then
        debug.log("E:kf07"); return nil, nil, nil
    end

    return X_new, P_new, epsilon_val
end

--#################################################################
--# 4. メインループ (onTick)
--#################################################################
function onTick()
    tickCounter = tickCounter + 1

    -- === 入力読み取り (対象は目標1のみとする) ===
    local target_index = 1

    -- MC1 からのデータ
    local obs_dist = input.getNumber(IN_DIST_CH(target_index))
    local obs_azim = input.getNumber(IN_G_AZIM_CH(target_index))
    local obs_elev = input.getNumber(IN_G_ELEV_CH(target_index))
    local newDataAvailable = input.getBool(IN_NEW_DATA_CH(target_index)) -- New Data Flag (Bool Ch 7)

    -- 時間関連情報
    local n_ticks = input.getNumber(19) -- <<< 修正: 直接数値を指定 >>> Detection Interval (ticks) from MC1
    local dt = 1 / 60
    if n_ticks and n_ticks > 0 then dt = n_ticks / 60 end

    -- 自機位置
    local self_x = input.getNumber(20) -- <<< 修正: 直接数値を指定 >>> East
    local self_y = input.getNumber(21) -- <<< 修正: 直接数値を指定 >>> North
    local self_z = input.getNumber(22) -- <<< 修正: 直接数値を指定 >>> Up
    local current_self_pos = { self_x, self_y, self_z }
    -- === EKF 計算 ===
    F = calculate_F(dt)
    local Q_base = calculate_Q(dt)
    -- --- 適応的Qの計算 ---
    local Q_adaptive = Q_base     -- デフォルトは基本Q
    if Q_base and not isInit then -- 初期化前や基本Q計算失敗時は適応しない
        -- シグモイド関数でスケーリング係数を計算
        -- q_scale = 1 + Scale / (1 + exp(-(prev_epsilon - Threshold) * Steepness))
        local exp_arg = -(prev_epsilon - Q_ADAPT_EPS_THR) * Q_ADAPT_STEEPNESS
        -- math.expの引数が大きすぎるとinfになるのを防ぐ (例: 700程度でオーバーフローの可能性)
        if exp_arg > 700 then exp_arg = 700 end
        local sigmoid_denom = 1 + math.exp(exp_arg)
        local q_scale = 1 + Q_ADAPT_SCALE / sigmoid_denom

        Q_adaptive = scalar(q_scale, Q_base)
        if not Q_adaptive then
            debug.log("E:adapQ") -- 適応Q計算失敗
            Q_adaptive = Q_base  -- 失敗したら基本Qを使う
        end
        -- Dl("AdapQ Scale: ".. string.format("%.2f", q_scale)) -- デバッグ用
    end

    -- --- 予測ステップ ---
    local X_pred = X -- 更新がない場合、現在の状態が予測値になる
    local P_pred = P
    if not isInit then
        -- Dl("L:p") -- Predict step log (optional)
        X_pred = mul(F, X)
        -- <<< 修正: 適応的なQ (Q_adaptive) を使用 >>>
        P_pred = sum(mul(F, mul(P, T(F))), Q_adaptive)
        if not X_pred or not P_pred then
            debug.log("E:kfp")
            isInit = true          -- 予測失敗時はリセット
            X_pred = X; P_pred = P -- リセット前の値を使うか、ゼロにするか？一旦維持
        end
    end

    -- --- 更新ステップ (新しいデータがある場合のみ) ---
    local current_epsilon = 0 -- 今回の更新でのepsilonを保持 (prev_epsilonとは別)
    if newDataAvailable then
        local obsZ = { { obs_dist }, { obs_elev }, { obs_azim } }
        if obs_dist and obs_dist > 1e-6 and obs_elev and obs_azim then
            -- 適応的観測ノイズRの計算
            local R_calculated = R_DEFAULT
            if n_ticks and n_ticks > 0 and obs_dist > 1e-6 then
                local current_dist_var = R_DIST_VAR_FACTOR * (obs_dist ^ 2)
                local R_base = { { current_dist_var, 0, 0 }, { 0, R_ANGLE_VAR, 0 }, { 0, 0, R_ANGLE_VAR } }
                local scale_factor = 1.0
                scale_factor = math.max(scale_factor, 0.01) -- 下限設定
                R_calculated = { { current_dist_var, 0, 0 }, { 0, R_ANGLE_VAR, 0 }, { 0, 0, R_ANGLE_VAR } }
                if not R_calculated then R_calculated = R_DEFAULT end
            end

            if isInit then
                -- Dl("I:init") -- Initializing filter
                local r = obsZ[1][1]; local el = obsZ[2][1]; local az = obsZ[3][1]
                local cp = math.cos(el); local sp = math.sin(el); local ct = math.cos(az); local st = math.sin(az)
                local dX = r * cp * st; local dY = r * cp * ct; local dZ = r * sp
                local initX = self_x + dX; local initY = self_y + dY; local initZ = self_z + dZ
                X = { { initX }, { 0 }, { 0 }, { initY }, { 0 }, { 0 }, { initZ }, { 0 }, { 0 } }
                P = Initial_P_Matrix or zeros(STATE_DIM, STATE_DIM)
                isInit = false
                prev_epsilon = 0 -- 初期化時は prev_epsilon をリセット
                current_epsilon = 0
            else
                -- EKF更新 (予測された X_pred, P_pred を渡す)
                local X_new, P_new, eps_new = EKF_Update(F, Q_adaptive, R_calculated, obsZ, X, P, current_self_pos)
                if X_new and P_new and eps_new then
                    X = X_new                      -- 状態を更新
                    P = P_new
                    current_epsilon = eps_new      -- 今回計算されたepsilon
                    prev_epsilon = current_epsilon -- 次の予測ステップのために保存
                    debug.log(string.format("epsilon:%.2f", current_epsilon))
                else
                    debug.log("E:kfu")
                    -- 更新失敗時は状態 X, P を予測ステップの結果(X_pred, P_pred)のままにする
                    X = X_pred
                    P = P_pred
                    current_epsilon = 0 -- 更新失敗時のepsilonは0とする
                    prev_epsilon = 0    -- 次の予測に影響しないようにリセット
                end
            end
        else
            debug.log("W:obs")
            prev_epsilon = 0 -- 無効な観測値の場合もリセット
            current_epsilon = 0
        end
    else
        -- 新しいデータがない場合
        if not isInit then
            -- 状態 X, P は予測ステップの結果のまま
            X = X_pred
            P = P_pred
            -- prev_epsilon は前の値を維持 (予測だけでは更新しない)
            current_epsilon = prev_epsilon -- 現在のepsilonとしては前の値を表示？あるいは0？ 0が安全か。
            current_epsilon = 0
        end
        -- isInit が true の場合は何もしない
    end

    -- === 出力 ===
    local outputX, outputY, outputZ = 0, 0, 0
    -- local outputVX, outputVY, outputVZ = 0, 0, 0
    local outputActive = false
    if not isInit then
        outputX = X[1][1]; outputVX = X[2][1]
        outputY = X[4][1]; outputVY = X[5][1]
        outputZ = X[7][1]; outputVZ = X[8][1]
        outputActive = true
    end

    output.setBool(OUT_ACTIVE_CH(target_index), outputActive) -- Bool Ch 1
    output.setNumber(OUT_POS_X_CH(target_index), outputX)     -- Num Ch 1
    output.setNumber(OUT_POS_Y_CH(target_index), outputY)     -- Num Ch 2
    output.setNumber(OUT_POS_Z_CH(target_index), outputZ)     -- Num Ch 3

    output.setNumber(OUT_VEL_X_CH(target_index), outputVX)    -- Num Ch 4
    output.setNumber(OUT_VEL_Y_CH(target_index), outputVY)    -- Num Ch 5
    output.setNumber(OUT_VEL_Z_CH(target_index), outputVZ)    -- Num Ch 6
    output.setNumber(32, current_epsilon)                     -- Num Ch 32 (Epsilon)

    --複数目標対応はしていないのでクリア処理は不要
end
