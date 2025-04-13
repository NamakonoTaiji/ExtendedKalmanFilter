--[[
  Stormworks UKF Radar Filter - MC1: Preprocessor (Readability Improved)
  Handles observation buffering, averaging, and angle conversion.
  Uses user-provided rotation logic (Code Example 1). No input filtering.
  Outputs processed data (Avg Dist, Global Azimuth, Global Elevation) to MC2.
]]

--[[ Log/Error Code Mappings (MC1):
  E:va01 - vectorAdd: Dimension mismatch.
  E:vs01 - vectorSubtract: Dimension mismatch.
  L:ndc01 - New detection cycle started.
  L:ao01 - Averaged observation calculated for channel X.
  E:ao01 - averageObservations: Invalid or empty buffer provided.
  L:gc01 - Global angles calculated for channel X.
  E:gc01 - rotateLocalToGlobal_UserA: Calculation failed (likely nil input).
  E:ga01 - Global angle calculation failed (e.g., sqrt domain error).
  W:ga01 - Ground distance is zero in global angle calculation.
  W:ac01 - Angle conversion failed (rotate func returned nil).
]]

--#################################################################
--# 1. 定数定義 (トップレベルで定義)
--#################################################################
local MAX_TARGETS_INPUT      = 6
local PI                     = math.pi
local HALF_PI                = PI / 2
local PI2                    = PI * 2

-- 入力チャンネル定義
local RADAR_DIST_CH          = function(i) return (i - 1) * 4 + 1 end
local RADAR_AZIM_CH          = function(i) return (i - 1) * 4 + 2 end
local RADAR_ELEV_CH          = function(i) return (i - 1) * 4 + 3 end
local RADAR_ELAPSED_CH       = function(i) return (i - 1) * 4 + 4 end
local RADAR_DETECT_CH        = function(i) return i end

-- 出力チャンネル定義 (MC2へ)
local OUT_DIST_CH            = function(i) return (i - 1) * 3 + 1 end
local OUT_G_AZIM_CH          = function(i) return (i - 1) * 3 + 2 end
local OUT_G_ELEV_CH          = function(i) return (i - 1) * 3 + 3 end
local OUT_DETECT_CH          = function(i) return i end

--#################################################################
--# 2. グローバル変数 / 状態変数 (トップレベルで初期化)
--#################################################################
local observationBuffer      = {}
local tickCounter            = 0

-- 固定された更新間隔 (tick数)
local DetectionIntervalTicks = 1 -- デフォルト値
-- *** プロパティから名前 "EffectiveRange" で直接読み込む ***
local EffectiveRange         = property.getNumber("EffectiveRange")
if EffectiveRange and EffectiveRange > 0 then
    DetectionIntervalTicks = math.max(math.floor(EffectiveRange / 2000 + 0.5), 1)
    debug.log("MC1 Init: EffectiveRange=" .. EffectiveRange .. ", DetectionInterval=" .. DetectionIntervalTicks ..
        " ticks")
else
    -- *** 警告メッセージも修正 ***
    debug.log('W:init01') -- Warning: Radar Range Property "EffectiveRange" not set or invalid.
end

--#################################################################
--# 2. ベクトル演算 ヘルパー関数群
--#################################################################

--- vectorAdd: ベクトル同士の加算
function vectorAdd(vecA, vecB)
    local d1 = #vecA; local d2 = #vecB
    if d1 ~= d2 then
        debug.log("E:va01"); return nil
    end
    local r = {}
    for i = 1, d1 do
        r[i] = vecA[i] + vecB[i]
    end
    return r
end

--- vectorSubtract: ベクトル同士の減算 (vecA - vecB)
function vectorSubtract(vecA, vecB)
    local d1 = #vecA; local d2 = #vecB
    if d1 ~= d2 then
        debug.log("E:vs01"); return nil
    end
    local r = {}
    for i = 1, d1 do
        r[i] = vecA[i] - vecB[i]
    end
    return r
end

--- vectorScale: ベクトルのスカラー倍
function vectorScale(vec, scalar)
    local r = {}; local d = #vec
    for i = 1, d do
        r[i] = vec[i] * scalar
    end
    return r
end

--#################################################################
--# 3. 補助関数
--#################################################################

--- 観測値バッファから最小値と最大値の平均（ミッドレンジ）を計算する関数
-- @param buffer table 観測値 {r={}, theta={}, phi={}} のリストを含むテーブル
-- @return table or nil 平均値 {r=avg_r, theta=avg_t, phi=avg_p}, 計算不能な場合は nil
function averageObservations(buffer)
    -- 入力バッファと内部リストの基本的な検証
    if not buffer or not buffer.r or not buffer.theta or not buffer.phi or #buffer.r == 0 then
        debug.log("E:ao01"); return nil
    end
    local count = #buffer.r
    if #buffer.theta ~= count or #buffer.phi ~= count then
        debug.log("E:ao01"); return nil -- Inconsistent lengths
    end

    -- 最初の要素で最小値・最大値を初期化
    local min_r = buffer.r[1]; local max_r = buffer.r[1]
    local min_t = buffer.theta[1]; local max_t = buffer.theta[1]
    local min_p = buffer.phi[1]; local max_p = buffer.phi[1]

    -- nil チェック (最初の要素がnilの場合)
    if min_r == nil or min_t == nil or min_p == nil then
        debug.log("E:ao02"); return nil -- Buffer contains nil
    end

    -- 2番目以降の要素をループして最小値・最大値を更新
    for i = 2, count do
        local current_r = buffer.r[i]
        local current_t = buffer.theta[i]
        local current_p = buffer.phi[i]

        -- nil や非数値が含まれていたらエラー
        if type(current_r) ~= "number" or type(current_t) ~= "number" or type(current_p) ~= "number" then
            debug.log("E:ao02"); return nil -- Buffer contains non-numeric or nil
        end

        min_r = math.min(min_r, current_r); max_r = math.max(max_r, current_r)
        min_t = math.min(min_t, current_t); max_t = math.max(max_t, current_t)
        min_p = math.min(min_p, current_p); max_p = math.max(max_p, current_p)
    end

    -- 最小値と最大値の平均（ミッドレンジ）を計算して返す
    return {
        r = (min_r + max_r) / 2,
        theta = (min_t + max_t) / 2,
        phi = (min_p + max_p) / 2
    }
end

--- ローカル直交座標ベクトルをワールド座標系ベクトルに回転させる関数 (ユーザー提供コード1ベース)
-- @param pitch number X軸オイラー角 (ラジアン)
-- @param yaw   number Y軸オイラー角 (ラジアン)
-- @param roll  number Z軸オイラー角 (ラジアン)
-- @param Lx number ローカルX座標 (Right)
-- @param Ly number ローカルY座標 (Up)
-- @param Lz number ローカルZ座標 (Forward)
-- @return table or nil ワールド座標系での相対ベクトル {x=dX(East), y=dY(Altitude/Up), z=dZ(North)}
function rotateLocalToGlobal_UserA(pitch, yaw, roll, Lx, Ly, Lz)
    if not pitch or not yaw or not roll or not Lx or not Ly or not Lz then
        debug.log("E:gc01"); return nil
    end
    local the = pitch; local phi = yaw; local psi = roll;
    local cY = math.cos(phi); local sY = math.sin(phi); local cP = math.cos(the); local sP = math.sin(the); local cR =
        math.cos(psi); local sR = math.sin(psi)
    local dX = cY * cR * Lx + (sP * sY * cR - cP * sR) * Ly + (cP * sY * cR + sP * sR) * Lz
    local dY_Alt = cY * sR * Lx + (sP * sY * sR + cP * cR) * Ly + (cP * sY * sR - sP * cR) * Lz
    local dZ_North = -sY * Lx + sP * cY * Ly + cP * cY * Lz
    return { x = dX, y = dY_Alt, z = dZ_North }
end

--#################################################################
--# 4. メインループ (onTick) - Stormworks が毎フレーム呼び出す関数
--#################################################################
function onTick()
    tickCounter                      = tickCounter + 1

    -- === 4.1 入力読み取り ===
    local yaw_val                    = input.getNumber(28)
    local pitch_val                  = input.getNumber(29)
    local roll_val                   = input.getNumber(30)
    local self_x                     = input.getNumber(25)
    local self_y                     = input.getNumber(26)
    local self_z                     = input.getNumber(27)
    local self_orient                = { yaw = yaw_val, pitch = pitch_val, roll = roll_val }

    local current_observations_local = {}
    local is_new_cycle               = false
    local detected_flags             = {}
    for i = 1, MAX_TARGETS_INPUT do
        local is_detected = input.getBool(RADAR_DETECT_CH(i)); detected_flags[i] = is_detected
        if is_detected then
            local r, t, p, e = input.getNumber(RADAR_DIST_CH(i)), input.getNumber(RADAR_AZIM_CH(i)),
                input.getNumber(RADAR_ELEV_CH(i)), input.getNumber(RADAR_ELAPSED_CH(i))
            current_observations_local[i] = { r = r, theta = t, phi = p, elapsed = e }; if e == 0 then is_new_cycle = true end
        else
            if observationBuffer[i] then observationBuffer[i] = nil end
        end
    end

    -- === 4.2 平均化 & グローバル角度計算 ===
    local processed_data_for_output = {}
    if is_new_cycle then
        debug.log("L:ndc01")
        for i = 1, MAX_TARGETS_INPUT do
            if observationBuffer[i] and observationBuffer[i].r and #observationBuffer[i].r > 0 then
                local avg_obs = averageObservations(observationBuffer[i])
                if avg_obs then
                    debug.log("L:ao01")
                    local theta_rad = (avg_obs.theta or 0) * PI2; local phi_rad = (avg_obs.phi or 0) * PI2; local r_avg =
                        avg_obs.r or 0
                    local cp = math.cos(phi_rad); local sp = math.sin(phi_rad); local ct = math.cos(theta_rad); local st =
                        math.sin(theta_rad)
                    local localX_R = r_avg * cp * st; local localY_U = r_avg * sp; local localZ_F = r_avg * cp * ct
                    local GV_relative = rotateLocalToGlobal_UserA(self_orient.pitch, self_orient.yaw, self_orient.roll,
                        localX_R, localY_U, localZ_F)
                    if GV_relative then
                        local dX = GV_relative.x; local dY_North = GV_relative.z; local dZ_Up = GV_relative.y
                        local ground_dist_sq = dX ^ 2 + dY_North ^ 2; local ground_dist = 0; local theta_global_rad = 0; local phi_global_rad = 0
                        if ground_dist_sq > 1e-9 then
                            ground_dist = math.sqrt(ground_dist_sq); theta_global_rad = math.atan(dX, dY_North); phi_global_rad =
                                math.atan(dZ_Up, ground_dist)
                        else
                            if dZ_Up >= 0 then phi_global_rad = HALF_PI else phi_global_rad = -HALF_PI end; theta_global_rad = 0; debug
                                .log("W:ga01")
                        end
                        processed_data_for_output[i] = { dist = r_avg, g_theta = theta_global_rad, g_phi = phi_global_rad }; debug
                            .log("L:gc01")
                    else
                        debug.log("E:gc01")
                    end
                end; observationBuffer[i] = nil
            end
            ::continue_loop::
        end
    end
    for ci, obs_local in pairs(current_observations_local) do
        if not observationBuffer[ci] then observationBuffer[ci] = { r = {}, theta = {}, phi = {} } end; table.insert(
            observationBuffer[ci].r, obs_local.r); table.insert(observationBuffer[ci].theta, obs_local.theta); table
            .insert(
                observationBuffer[ci].phi, obs_local.phi)
    end

    -- === 4.3 出力書き込み (MC2へ) ===
    for i = 1, MAX_TARGETS_INPUT do
        local data = processed_data_for_output[i]; local base_num_ch = OUT_DIST_CH(i); local bool_ch = OUT_DETECT_CH(i); output
            .setBool(bool_ch, detected_flags[i] or false)
        if data and detected_flags[i] then
            output.setNumber(base_num_ch + 0, data.dist); output.setNumber(base_num_ch + 1, data.g_theta); output
                .setNumber(base_num_ch + 2, data.g_phi)
        else
            --[[output.setNumber(base_num_ch + 0, 0); output.setNumber(base_num_ch + 1, 0); output.setNumber(base_num_ch + 2,
                0)]]
        end
    end
    -- 更新間隔(tick数)を専用チャンネルに出力
    output.setNumber(19, DetectionIntervalTicks)
    output.setNumber(20, self_x); output.setNumber(21, self_y); output.setNumber(22, self_z)
end

--[[ (初期化はトップレベル変数定義で行われる) ]]
