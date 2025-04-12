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
--# 0. 定数定義
--#################################################################
local MAX_TARGETS_INPUT = 6 -- 同時に処理する最大目標数
local PI                = math.pi
local HALF_PI           = PI / 2
local PI2               = PI * 2

-- 入力チャンネル定義
-- レーダー入力 (目標 i = 1 to MAX_TARGETS_INPUT)
-- Note: base_ch = (i - 1) * 4
local RADAR_DIST_CH     = function(i) return (i - 1) * 4 + 1 end -- 1, 5, 9, ...
local RADAR_AZIM_CH     = function(i) return (i - 1) * 4 + 2 end -- 2, 6, 10, ...
local RADAR_ELEV_CH     = function(i) return (i - 1) * 4 + 3 end -- 3, 7, 11, ...
local RADAR_ELAPSED_CH  = function(i) return (i - 1) * 4 + 4 end -- 4, 8, 12, ...
local RADAR_DETECT_CH   = function(i) return i end               -- Bool 1-6

-- 自機姿勢入力 (物理センサーからの再マッピング想定)
local SELF_YAW_CH       = 28
local SELF_PITCH_CH     = 29
local SELF_ROLL_CH      = 30

-- 出力チャンネル定義 (MC2へ)
-- Note: base_out_ch = (i - 1) * 3 + 1
local OUT_DIST_CH       = function(i) return (i - 1) * 3 + 1 end -- 1, 4, 7, ...
local OUT_G_AZIM_CH     = function(i) return (i - 1) * 3 + 2 end -- 2, 5, 8, ...
local OUT_G_ELEV_CH     = function(i) return (i - 1) * 3 + 3 end -- 3, 6, 9, ...
local OUT_DETECT_CH     = function(i) return i end               -- Bool 1-6

--#################################################################
--# 1. グローバル変数 / 状態変数 (トップレベルで初期化)
--#################################################################
local observationBuffer = {} -- 観測値バッファ
local lastDetectionTick = {} -- 最終探知更新tick
local tickCounter       = 0  -- tickカウンター

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

--- 観測値バッファから平均値を計算する関数
function averageObservations(buffer)
    if not buffer or not buffer.r or not buffer.theta or not buffer.phi or #buffer.r == 0 then
        debug.log("E:ao01"); return nil
    end
    local count = #buffer.r
    if #buffer.theta ~= count or #buffer.phi ~= count then
        debug.log("E:ao01"); return nil
    end
    local sum_r, sum_t, sum_p = 0, 0, 0
    for i = 1, count do
        sum_r = sum_r + (buffer.r[i] or 0)
        sum_t = sum_t + (buffer.theta[i] or 0)
        sum_p = sum_p + (buffer.phi[i] or 0)
    end
    if count == 0 then return { r = 0, theta = 0, phi = 0 } end
    return {
        r = sum_r / count,
        theta = sum_t / count,
        phi = sum_p / count
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
    local yaw_val                    = input.getNumber(SELF_YAW_CH)   -- 自機ヨー
    local pitch_val                  = input.getNumber(SELF_PITCH_CH) -- 自機ピッチ
    local roll_val                   = input.getNumber(SELF_ROLL_CH)  -- 自機ロール
    local self_orient                = { yaw = yaw_val, pitch = pitch_val, roll = roll_val }

    local current_observations_local = {}    -- 現在のtickでのレーダー観測値
    local is_new_cycle               = false -- 探知更新サイクルかどうかのフラグ
    local detected_flags             = {}    -- 各チャンネルの検出状態

    for i = 1, MAX_TARGETS_INPUT do
        local is_detected = input.getBool(RADAR_DETECT_CH(i))
        detected_flags[i] = is_detected
        if is_detected then
            local r = input.getNumber(RADAR_DIST_CH(i))
            local t = input.getNumber(RADAR_AZIM_CH(i))
            local p = input.getNumber(RADAR_ELEV_CH(i))
            local e = input.getNumber(RADAR_ELAPSED_CH(i))
            current_observations_local[i] = { r = r, theta = t, phi = p, elapsed = e }
            if e == 0 then
                is_new_cycle = true
                lastDetectionTick[i] = tickCounter
            end
        else
            -- 非検出の場合、対応するバッファをクリア
            if observationBuffer[i] then
                observationBuffer[i] = nil
            end
        end
    end

    -- === 4.2 平均化 & グローバル角度計算 ===
    local processed_data_for_output = {} -- MC2への出力データ {dist, g_theta, g_phi}

    if is_new_cycle then
        debug.log("L:ndc01") -- 新しい探知サイクル開始ログ
        for i = 1, MAX_TARGETS_INPUT do
            if observationBuffer[i] and observationBuffer[i].r and #observationBuffer[i].r > 0 then
                -- 観測値の平均化
                local avg_obs = averageObservations(observationBuffer[i])
                if avg_obs then
                    debug.log("L:ao01") -- 平均化完了ログ

                    -- ローカル極座標 -> ローカル直交座標 (X=R, Y=U, Z=Fwd)
                    local theta_rad = (avg_obs.theta or 0) * PI2
                    local phi_rad = (avg_obs.phi or 0) * PI2
                    local r_avg = avg_obs.r or 0
                    local cp = math.cos(phi_rad); local sp = math.sin(phi_rad)
                    local ct = math.cos(theta_rad); local st = math.sin(theta_rad)
                    local localX_R = r_avg * cp * st
                    local localY_U = r_avg * sp
                    local localZ_F = r_avg * cp * ct

                    -- ローカル直交ベクトルをグローバル相対ベクトルに回転
                    local GV_relative = rotateLocalToGlobal_UserA(self_orient.pitch, self_orient.yaw, self_orient.roll,
                        localX_R, localY_U, localZ_F)

                    if GV_relative then
                        -- グローバル相対ベクトルからグローバル角度を計算
                        local dX = GV_relative.x; local dY_North = GV_relative.z; local dZ_Up = GV_relative.y
                        local ground_dist_sq = dX ^ 2 + dY_North ^ 2
                        local ground_dist = 0
                        local theta_global_rad = 0
                        local phi_global_rad = 0

                        if ground_dist_sq > 1e-9 then                      -- ゼロ除算回避
                            ground_dist = math.sqrt(ground_dist_sq)
                            theta_global_rad = math.atan(dX, dY_North)     -- atan(East, North)
                            phi_global_rad = math.atan(dZ_Up, ground_dist) -- atan(Up, HorizontalDist)
                        else                                               -- 真上/真下の場合
                            if dZ_Up >= 0 then phi_global_rad = HALF_PI else phi_global_rad = -HALF_PI end
                            theta_global_rad = 0
                            debug.log("W:ga01") -- 警告: 水平距離ゼロ
                        end
                        -- 計算結果を保存
                        processed_data_for_output[i] = { dist = r_avg, g_theta = theta_global_rad, g_phi = phi_global_rad }
                        debug.log("L:gc01") -- グローバル角度計算完了ログ
                    else
                        debug.log("E:gc01") -- 回転計算失敗ログ
                    end
                end
                -- 処理が終わったのでバッファをクリア
                observationBuffer[i] = nil
            end
            ::continue_loop:: -- goto 用ラベル (Lua 5.1 では goto はないが、コメントとして残す)
            -- Note: Lua 5.1 does not have goto. The loop structure needs adjustment if continue is needed.
            --       In this case, the if/end structure implicitly continues.
        end
    end

    -- === 4.3 現在の観測値をバッファに蓄積 ===
    for ci, obs_local in pairs(current_observations_local) do
        if not observationBuffer[ci] then
            observationBuffer[ci] = { r = {}, theta = {}, phi = {} }
        end
        table.insert(observationBuffer[ci].r, obs_local.r)
        table.insert(observationBuffer[ci].theta, obs_local.theta)
        table.insert(observationBuffer[ci].phi, obs_local.phi)
    end

    -- === 4.4 出力書き込み (MC2へ) ===
    for i = 1, MAX_TARGETS_INPUT do
        local data = processed_data_for_output[i]
        local out_num_ch_base = OUT_DIST_CH(i)                  -- Base channel for number output
        local out_bool_ch = OUT_DETECT_CH(i)                    -- Channel for boolean output

        output.setBool(out_bool_ch, detected_flags[i] or false) -- 検出状態

        if data and detected_flags[i] then
            -- 新しい計算結果があれば出力
            output.setNumber(out_num_ch_base + 0, data.dist)    -- 平均距離
            output.setNumber(out_num_ch_base + 1, data.g_theta) -- グローバル方位角 (rad)
            output.setNumber(out_num_ch_base + 2, data.g_phi)   -- グローバル仰角 (rad)
        else
            -- 新しい計算結果がない場合や非検出の場合は 0 を出力
            -- (MC2側で Bool=true かつ Dist > 0 かどうかで有効なデータか判断)
            output.setNumber(out_num_ch_base + 0, 0)
            output.setNumber(out_num_ch_base + 1, 0)
            output.setNumber(out_num_ch_base + 2, 0)
        end
    end
end

--[[ (初期化はトップレベル変数定義で行われる) ]]
