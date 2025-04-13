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

-- ショートハンド (ユーザーコードより)
local iN = input.getNumber; local iB = input.getBool; local oN = output.setNumber; local oB = output.setBool
local M = math; local sqrt = M.sqrt; local abs = M.abs; local pi = M.pi; local pi2 = pi * 2; local asin = M.asin; local atan =
    M.atan; local Dl = debug.log

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
    end; local det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g); if abs(det) < 1e-9 then
        debug.log("E:in02"); return nil
    end; local adj = { { (e * i - f * h), (c * h - b * i), (b * f - c * e) }, { (f * g - d * i), (a * i - c * g), (c * d - a * f) }, { (d * h - e * g), (b * g - a * h), (a * e - b * d) } }; local invDet = 1.0 /
        det; local invM = scalar(invDet, adj); return invM
end

--#################################################################
--# 1. 定数 & モデル定義
--#################################################################
local STATE_DIM = 9; local OBS_DIM = 3; local MAX_TRACKS = 1
local I9 = zeros(STATE_DIM, STATE_DIM); if I9 then for i = 1, STATE_DIM do I9[i][i] = 1 end else I9 = {} end
local R_diag_dist_sq = (0.02 * 50) ^ 2; local R_diag_angle_sq = (0.002 * pi2) ^ 2
local R = { { R_diag_dist_sq, 0, 0 }, { 0, R_diag_angle_sq, 0 }, { 0, 0, R_diag_angle_sq } }
local sigma_a = 0.5; local sigma_a_property = property.getNumber("SigmaA"); if sigma_a_property and sigma_a_property > 0 then
    sigma_a = sigma_a_property; debug.log('MC3 Init: Read SigmaA = ' .. sigma_a .. ' from property.')
else
    debug.log('W:init03')
end; local sigma_a_sq = sigma_a * sigma_a
local function calculate_Q(dt)
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
local Q; local function calculate_F(dt)
    local dt2h = 0.5 * dt * dt; return { { 1, dt, dt2h, 0, 0, 0, 0, 0, 0 }, { 0, 1, dt, 0, 0, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 1, dt, dt2h, 0, 0, 0 }, { 0, 0, 0, 0, 1, dt, 0, 0, 0 }, { 0, 0, 0, 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 1, dt, dt2h }, { 0, 0, 0, 0, 0, 0, 0, 1, dt }, { 0, 0, 0, 0, 0, 0, 0, 0, 1 } }
end
local F; local Initial_P_Matrix = zeros(STATE_DIM, STATE_DIM); local iposv = 100; local ivelv = 100; local iaccv = 10; if Initial_P_Matrix then
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
local IN_DIST_CH = 1; local IN_G_AZIM_CH = 2; local IN_G_ELEV_CH = 3; local IN_DETECT_CH = 2; local IN_SELF_X_CH = 11; local IN_SELF_Y_CH = 13; local IN_SELF_Z_CH = 12; local IN_DT_N_CH = 10
local OUT_ACTIVE_CH = function(i) return i end; local OUT_POS_X_CH = function(i) return (i - 1) * 6 + 1 end; local OUT_POS_Y_CH = function(
    i)
    return (i - 1) * 6 + 2
end; local OUT_POS_Z_CH = function(i) return (i - 1) * 6 + 3 end; local OUT_VEL_X_CH = function(
    i)
    return (i - 1) * 6 + 4
end; local OUT_VEL_Y_CH = function(i) return (i - 1) * 6 + 5 end; local OUT_VEL_Z_CH = function(
    i)
    return (i - 1) * 6 + 6
end; local OUT_EPSILON_CH = 32

--#################################################################
--# 2. グローバル変数 / 状態変数
--#################################################################
local X = zeros(STATE_DIM, 1)
local P = Initial_P_Matrix or zeros(STATE_DIM, STATE_DIM)
local isInit = true
local epsilon = 0
local tickCounter = 0

--#################################################################
--# 3. EKF コア関数
--#################################################################

--- 観測関数 h(x): 状態ベクトルから観測値（距離、仰角、方位角）を計算 ***[修正済み]***
-- @param x_vec table 状態ベクトル (9x1) {x,vx,ax, y,vy,ay, z,vz,az} - 目標の絶対座標
-- @param self_pos table 自機位置 {sx, sy, sz} (East, North, Up) - 自機の絶対座標
-- @return table or nil 観測ベクトル (3x1) {dist}, {elev}, {azim} - 自機から見た相対的な極座標
function observationFunction(x_vec, self_pos)
    if not x_vec or not self_pos or not x_vec[1] or not x_vec[4] or not x_vec[7] then return nil end
    -- 状態ベクトルから目標の絶対位置を抽出
    local target_x = x_vec[1][1]; local target_y = x_vec[4][1]; local target_z = x_vec[7][1]
    -- 自機位置を取得
    local self_x = self_pos[1]; local self_y = self_pos[2]; local self_z = self_pos[3]
    -- *** 自機から目標への相対ベクトルを計算 ***
    local rel_x = target_x - self_x
    local rel_y = target_y - self_y
    local rel_z = target_z - self_z
    -- *** 相対ベクトルから距離、仰角、方位角を計算 ***
    local r_sq = rel_x * rel_x + rel_y * rel_y + rel_z * rel_z
    if r_sq < 1e-9 then return nil end -- 自機と目標が同一地点の場合
    local r = sqrt(r_sq)
    local rh_sq = rel_x * rel_x + rel_y * rel_y
    local rh = sqrt(rh_sq > 1e-9 and rh_sq or 1e-9) -- 水平距離
    local dist = r
    local elev = asin(rel_z / r)                    -- 仰角
    local azim = atan(rel_x, rel_y)                 -- 方位角 (atan(East, North))
    return { { dist }, { elev }, { azim } }
end

--- ヤコビアン行列 H = ∂h/∂X の計算 (3x9 for CA model) ***[修正済み]***
-- @param x_vec table 状態ベクトル (9x1) - 目標の絶対座標
-- @param self_pos table 自機位置 {sx, sy, sz} (East, North, Up) - 自機の絶対座標
-- @return table or nil ヤコビアン行列 (3x9)
function calculateJacobianH(x_vec, self_pos)
    if not x_vec or not self_pos or not x_vec[1] or not x_vec[4] or not x_vec[7] then return nil end
    -- 状態ベクトルから目標の絶対位置を抽出
    local target_x = x_vec[1][1]; local target_y = x_vec[4][1]; local target_z = x_vec[7][1]
    -- 自機位置を取得
    local self_x = self_pos[1]; local self_y = self_pos[2]; local self_z = self_pos[3]
    -- *** 自機から目標への相対ベクトルを計算 ***
    local x = target_x - self_x -- 相対X (East)
    local y = target_y - self_y -- 相対Y (North)
    local z = target_z - self_z -- 相対Z (Up)
    -- 必要な値を計算 (ゼロ除算回避含む)
    local r_sq = x * x + y * y + z * z; if r_sq < 1e-9 then return nil end; local r = sqrt(r_sq)
    local rh_sq = x * x + y * y; if rh_sq < 1e-9 then return nil end; local rh = sqrt(rh_sq)
    local r2 = r_sq; local r3 = r2 * r; local rh2 = rh_sq

    local H = zeros(OBS_DIM, STATE_DIM) -- 3x9

    -- 偏微分を計算 (∂h / ∂(target_pos)) - 計算式自体は相対座標を使えば同じ
    -- Row 1: ∂dist/∂X = [∂r/∂x, 0, 0, ∂r/∂y, 0, 0, ∂r/∂z, 0, 0]
    H[1][1] = x / r; H[1][4] = y / r; H[1][7] = z / r
    -- Row 2: ∂elev/∂X = [∂e/∂x, 0, 0, ∂e/∂y, 0, 0, ∂e/∂z, 0, 0] (e = asin(z/r))
    local term_elev_denom = sqrt(1 - (z / r) ^ 2); if abs(z / r) > 0.9999 then term_elev_denom = 1e-5 end
    local term_elev = 1 / term_elev_denom
    H[2][1] = term_elev * (-z * x / r3); H[2][4] = term_elev * (-z * y / r3); H[2][7] = term_elev * (rh2 / r3)
    -- Row 3: ∂azim/∂X = [∂a/∂x, 0, 0, ∂a/∂y, 0, 0, ∂a/∂z, 0, 0] (a = atan(x,y))
    H[3][1] = y / rh2; H[3][4] = -x / rh2; H[3][7] = 0

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
    debug.log("L:kf01")
    local X_pred = mul(F_current, X_state)
    local P_pred = sum(mul(F_current, mul(P_cov, T(F_current))), Q_current)
    if not X_pred or not P_pred then return nil, nil, nil end

    -- === 更新 ===
    debug.log("L:kf02")
    -- 観測予測 h(x_pred) - 自機位置を渡す
    local Z_pred = observationFunction(X_pred, current_self_pos)
    if not Z_pred then
        debug.log("E:kf02"); return nil, nil, nil
    end

    local Y = sub(Z_obs, Z_pred) -- イノベーション Y = Z_obs - h(x_pred)
    if not Y then
        debug.log("E:kf02"); return nil, nil, nil
    end
    if Y[3][1] then Y[3][1] = (Y[3][1] + pi) % pi2 - pi end -- 方位角の差を正規化

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

    -- === 入力読み取り ===
    local obs_dist = iN(IN_DIST_CH); local obs_azim = iN(IN_G_AZIM_CH); local obs_elev = iN(IN_G_ELEV_CH)
    local RDetect = iB(1); local n_ticks = iN(19)
    local self_x = iN(20); local self_y = iN(22); local self_z = iN(21)
    local current_self_pos = { self_x, self_y, self_z } -- テーブルに格納
    debug.log(string.format("Self Pos Input: X(11)=%.2f, Y(13)=%.2f, Z(12)=%.2f", self_x, self_y, self_z))
    -- === 計算 ===
    local dt = 1 / 60; if n_ticks and n_ticks > 0 then dt = n_ticks / 60 end
    F = calculate_F(dt); Q = calculate_Q(dt)
    local obsZ = { { obs_dist }, { obs_elev }, { obs_azim } }

    if RDetect and dt > 0 then
        if isInit then
            debug.log("W:kf02")
            local r = obsZ[1][1]; local el = obsZ[2][1]; local az = obsZ[3][1]
            if r and el and az then
                local cp = M.cos(el); local sp = M.sin(el); local ct = M.cos(az); local st = M.sin(az)
                local dX = r * cp * st; local dY = r * cp * ct; local dZ = r * sp
                local initX = self_x + dX; local initY = self_y + dY; local initZ = self_z + dZ
                if Initial_P_Matrix then P = Initial_P_Matrix else P = zeros(STATE_DIM, STATE_DIM) end
                X = { { initX }, { 0 }, { 0 }, { initY }, { 0 }, { 0 }, { initZ }, { 0 }, { 0 } }
                isInit = false
            else
                debug.log("W:init01")
            end
        else
            -- EKF 更新ステップ呼び出し - *** 自機位置を渡すように変更 ***
            local X_new, P_new, eps_new = EKF_Update(F, Q, R, obsZ, X, P, current_self_pos)
            if X_new and P_new and eps_new then
                X = X_new; P = P_new; epsilon = eps_new
            else
                isInit = true
            end
        end
    else
        if not isInit then
            debug.log("W:kf01"); isInit = true
        end
    end

    -- === 出力 ===
    local outputX = 0; local outputY = 0; local outputZ = 0; local outputVX = 0; local outputVY = 0; local outputVZ = 0
    if not isInit then
        outputX = X[1][1]; outputVX = X[2][1]; outputY = X[4][1]; outputVY = X[5][1]; outputZ = X[7][1]; outputVZ = X[8]
            [1]
    end
    oB(OUT_ACTIVE_CH(1), not isInit and RDetect)
    oN(OUT_POS_X_CH(1), outputX); oN(OUT_POS_Y_CH(1), outputY); oN(OUT_POS_Z_CH(1), outputZ)
    oN(OUT_VEL_X_CH(1), outputVX); oN(OUT_VEL_Y_CH(1), outputVY); oN(OUT_VEL_Z_CH(1), outputVZ)
    oN(OUT_EPSILON_CH, epsilon)
    for i = 2, MAX_TRACKS do
        oB(OUT_ACTIVE_CH(i), false); local bc = OUT_POS_X_CH(i); for j = 0, 5 do oN(bc + j, 0) end
    end
end

--[[ (初期化はトップレベル変数定義で行われる) ]]
