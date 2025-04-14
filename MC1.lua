--[[
  Stormworks UKF Radar Filter - MC1: Preprocessor (Readability Improved)
  Handles observation buffering, averaging, and angle conversion.
  Uses user-provided rotation logic (Code Example 1). No input filtering.
  Outputs processed data (Avg Dist, Global Azimuth, Global Elevation) to MC2.
]]

--[[ Log/Error Code Mappings (MC1):
  E:va01 - vectorAdd: Dimension mismatch.
  E:vs01 - vectorSubtract: Dimension mismatch.
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
local MAX_TARGETS_INPUT       = 6
local PI                      = math.pi
local HALF_PI                 = PI / 2
local PI2                     = PI * 2

-- 入力チャンネル定義
local RADAR_DIST_CH           = function(i) return (i - 1) * 4 + 1 end
local RADAR_AZIM_CH           = function(i) return (i - 1) * 4 + 2 end
local RADAR_ELEV_CH           = function(i) return (i - 1) * 4 + 3 end
local RADAR_ELAPSED_CH        = function(i) return (i - 1) * 4 + 4 end
local RADAR_DETECT_CH         = function(i) return i end

-- 出力チャンネル定義 (MC2へ)
local OUT_DIST_CH             = function(i) return (i - 1) * 3 + 1 end
local OUT_NEW_DATA_FLAG_CH    = function(i) return 6 + i end -- 例: Bool Ch 7-12
--#################################################################
--# 2. グローバル変数 / 状態変数 (トップレベルで初期化)
--#################################################################
local observationBuffer       = {}
local tickCounter             = 0

-- <<< 修正: 2段階の状態保持 >>>
-- stateForProcessing: 現在処理中のバッファに対応する自機状態 (1サイクル前のもの)
local stateForProcessing      = {
    orient = { yaw = 0, pitch = 0, roll = 0 },
    pos_gps = { x = 0, y = 0, z = 0 }
}
-- latestStateAtCycleStart: 最新のサイクル開始時の自機状態 (次の処理で使用)
local latestStateAtCycleStart = {
    orient = { yaw = 0, pitch = 0, roll = 0 },
    pos_gps = { x = 0, y = 0, z = 0 }
}

-- 固定された更新間隔 (tick数)
local DetectionIntervalTicks  = 1 -- デフォルト値
-- *** プロパティから名前 "EffectiveRange" で直接読み込む ***
local EffectiveRange          = property.getNumber("EffectiveRange")
if EffectiveRange and EffectiveRange > 0 then
    DetectionIntervalTicks = math.max(math.floor(EffectiveRange / 2000 + 0.5), 1)
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
    local cY = math.cos(yaw); local sY = math.sin(yaw); local cP = math.cos(pitch); local sP = math.sin(pitch); local cR =
        math.cos(roll); local sR = math.sin(roll)
    local dX = cY * cR * Lx + (sP * sY * cR - cP * sR) * Ly + (cP * sY * cR + sP * sR) * Lz
    local dY_Alt = cY * sR * Lx + (sP * sY * sR + cP * cR) * Ly + (cP * sY * sR - sP * cR) * Lz
    local dZ_North = -sY * Lx + sP * cY * Ly + cP * cY * Lz
    return { x = dX, y = dY_Alt, z = dZ_North }
end

--#################################################################
--# 4. メインループ (onTick) - Stormworks が毎フレーム呼び出す関数
--#################################################################
function onTick()
    tickCounter = tickCounter + 1

    -- === 4.1 入力読み取り (現在の自機状態とレーダー観測) ===
    local current_yaw = input.getNumber(28)
    local current_pitch = input.getNumber(29)
    local current_roll = input.getNumber(30)
    -- Physics Sensor 出力からGPS座標系 (East, North, Up) に読み替え
    local current_self_x_east = input.getNumber(25)
    local current_self_y_north = input.getNumber(27) -- Physics Sensor Z is North
    local current_self_z_up = input.getNumber(26)    -- Physics Sensor Y is Up
    local current_orient = { yaw = current_yaw, pitch = current_pitch, roll = current_roll }
    local current_pos_gps = { x = current_self_x_east, y = current_self_y_north, z = current_self_z_up }

    local current_observations_local = {}
    local is_new_cycle = false    -- このtickが新しいサイクルの開始点か？
    local detected_this_tick = {} -- このtickで検出されたか？ (出力用)

    for i = 1, MAX_TARGETS_INPUT do
        local is_detected = input.getBool(RADAR_DETECT_CH(i))
        detected_this_tick[i] = is_detected -- 現在の検出状態を記録

        if is_detected then
            local r, t, p, e = input.getNumber(RADAR_DIST_CH(i)), input.getNumber(RADAR_AZIM_CH(i)),
                input.getNumber(RADAR_ELEV_CH(i)), input.getNumber(RADAR_ELAPSED_CH(i))

            -- elapsed time が nil でないことを確認 (念のため)
            if e ~= nil then
                current_observations_local[i] = { r = r, theta = t, phi = p, elapsed = e }
                -- 新しいサイクルの開始を判定 (elapsed が 0 になった瞬間)
                if e == 0 then
                    is_new_cycle = true
                end
            end
        else
            -- 検出が途切れたらその目標のバッファをクリア
            if observationBuffer[i] then
                observationBuffer[i] = nil
                -- debug.log("Target " .. i .. " lost, buffer cleared.")
            end
        end
    end

    -- === 4.2 過去サイクルのデータ処理 (新しいサイクルの開始時に実行) ===
    local processed_data_for_output = {} -- このtickでMC2に出力するデータ
    local new_data_flag_for_output = {}  -- このtickでMC2に出力する更新フラグ

    if is_new_cycle then
        for i = 1, MAX_TARGETS_INPUT do
            -- 過去のサイクルでデータがバッファリングされていれば処理
            if observationBuffer[i] and observationBuffer[i].r and #observationBuffer[i].r > 0 then
                local avg_obs = averageObservations(observationBuffer[i])
                if avg_obs then
                    local theta_rad = (avg_obs.theta or 0) * PI2
                    local phi_rad = (avg_obs.phi or 0) * PI2
                    local r_avg = avg_obs.r or 0

                    -- ローカル直交座標 (Right, Up, Forward) に変換
                    local cp = math.cos(phi_rad); local sp = math.sin(phi_rad)
                    local ct = math.cos(theta_rad); local st = math.sin(theta_rad)
                    local localX_R = r_avg * cp * st -- Right
                    local localY_U = r_avg * sp      -- Up
                    local localZ_F = r_avg * cp * ct -- Forward

                    -- <<< 修正: 1サイクル前の姿勢情報 (stateForProcessing.orient) を使用 >>>
                    local GV_relative = rotateLocalToGlobal_UserA(
                        stateForProcessing.orient.pitch, stateForProcessing.orient.yaw, stateForProcessing.orient.roll,
                        localX_R, localY_U, localZ_F
                    )

                    if GV_relative then
                        -- グローバル角度 (East-North平面からの仰角、NorthからのEast向き方位角) を計算
                        local dX_East = GV_relative.x
                        local dY_North = GV_relative.z -- 注意: rotateLocalToGlobal_UserAの出力 Z は North
                        local dZ_Up = GV_relative.y    -- 注意: rotateLocalToGlobal_UserAの出力 Y は Altitude/Up

                        local ground_dist_sq = dX_East ^ 2 + dY_North ^ 2
                        local ground_dist = 0
                        local theta_global_rad = 0 -- Azimuth from North (East is positive)
                        local phi_global_rad = 0   -- Elevation from Horizon

                        if ground_dist_sq > 1e-9 then
                            ground_dist = math.sqrt(ground_dist_sq)
                            -- atan2(y, x) -> atan2(East, North)
                            theta_global_rad = math.atan(dX_East, dY_North)
                            -- atan2(y, x) -> atan2(Up, GroundDist)
                            phi_global_rad = math.atan(dZ_Up, ground_dist)
                        else
                            -- Target is directly above or below
                            if dZ_Up >= 0 then phi_global_rad = HALF_PI else phi_global_rad = -HALF_PI end
                            theta_global_rad = 0 -- Azimuth is undefined/zero
                            -- debug.log("W:ga01 - Ground distance zero Ch" .. i)
                        end
                        -- 処理結果を保存
                        processed_data_for_output[i] = { dist = r_avg, g_theta = theta_global_rad, g_phi = phi_global_rad }
                        new_data_flag_for_output[i] = true -- 新しいデータがあることを示す
                    else
                        debug.log("E:gc01r" .. i)
                        new_data_flag_for_output[i] = false
                    end -- end if GV_relative
                else
                    -- averageObservations が失敗した場合
                    debug.log("E:ao01a" .. i)
                    new_data_flag_for_output[i] = false
                end -- end if avg_obs

                -- 処理が終わったので、この目標のバッファをクリア
                observationBuffer[i] = nil
            else
                -- このサイクルでは処理するデータがなかった (前サイクルで未検出だったなど)
                new_data_flag_for_output[i] = false
            end -- end if buffer exists
        end     -- end for loop (target processing)

        -- <<< 修正: 次のサイクルの処理のために状態を更新 >>>
        -- stateForProcessing を latestStateAtCycleStart の値で更新
        stateForProcessing = latestStateAtCycleStart
        -- latestStateAtCycleStart を現在の値で更新
        latestStateAtCycleStart = { orient = current_orient, pos_gps = current_pos_gps }
    else
        -- is_new_cycle が false の場合、データ処理は行わない
        for i = 1, MAX_TARGETS_INPUT do
            new_data_flag_for_output[i] = false -- 新しいデータは出力されない
        end
    end                                         -- end if is_new_cycle
    -- === 4.3 観測値をバッファに追加 (毎tick行う) ===
    for ci, obs_local in pairs(current_observations_local) do
        -- バッファが存在しない場合は作成 (検出開始時 or ロスト後再検出時)
        if not observationBuffer[ci] then
            observationBuffer[ci] = { r = {}, theta = {}, phi = {} }
            -- debug.log("Target " .. ci .. " detected, buffer created.")
        end
        table.insert(observationBuffer[ci].r, obs_local.r)
        table.insert(observationBuffer[ci].theta, obs_local.theta)
        table.insert(observationBuffer[ci].phi, obs_local.phi)
    end

    -- === 4.4 出力書き込み (MC2へ) ===
    for i = 1, MAX_TARGETS_INPUT do
        local data = processed_data_for_output[i]
        local new_data_available = new_data_flag_for_output[i] or false

        -- 数値チャンネル出力 (データがある場合のみ意味を持つ値を出力)
        local base_num_ch = OUT_DIST_CH(i)
        if new_data_available and data then
            output.setNumber(base_num_ch + 0, data.dist)
            output.setNumber(base_num_ch + 1, data.g_theta) -- Global Azimuth (rad)
            output.setNumber(base_num_ch + 2, data.g_phi)   -- Global Elevation (rad)
        else
            -- データがない場合は 0 を出力 (MC2側でフラグを見て無視する想定)
            output.setNumber(base_num_ch + 0, 0)
            output.setNumber(base_num_ch + 1, 0)
            output.setNumber(base_num_ch + 2, 0)
        end

        -- Boolチャンネル出力 (MC2への更新トリガー)
        output.setBool(OUT_NEW_DATA_FLAG_CH(i), new_data_available)
    end

    -- 更新間隔(tick数)を出力 (これは定数なので毎tick出力しても良い)
    output.setNumber(19, DetectionIntervalTicks)

    -- 自機位置 (EKF更新時の位置として stateForProcessing.pos_gps を使う)
    output.setNumber(20, stateForProcessing.pos_gps.x) -- East
    output.setNumber(21, stateForProcessing.pos_gps.y) -- North (GPS Y)
    output.setNumber(22, stateForProcessing.pos_gps.z) -- Up (GPS Z)
end
