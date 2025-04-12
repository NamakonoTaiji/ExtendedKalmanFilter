--[[
  Stormworks UKF Radar Filter - MC1: Preprocessor
  Handles observation buffering, averaging, and angle conversion.
  Outputs processed data to MC2 via composite signal.
  Sensor and Radar use the same local frame (Z-fwd, Y-up, X-right).
  Orientation input uses correct channels and ZYX Euler order.
]]

--[[ Log/Error Code Mappings (MC1): (省略) ]]
--[[ ... Log/Error Codes ... ]]

--#################################################################
--# 0. 行列・ベクトル演算 ヘルパー関数群 (MC1で必要なため定義)
--#################################################################
--- vectorAdd: ベクトル同士の加算
function vectorAdd(vecA, vecB)
    local dimA = #vecA; local dimB = #vecB
    if dimA ~= dimB then
        debug.log("E:va01"); return nil
    end
    local r = {}; for i = 1, dimA do r[i] = vecA[i] + vecB[i] end; return r
end

--- vectorSubtract: ベクトル同士の減算 (vecA - vecB)
function vectorSubtract(vecA, vecB)
    local dimA = #vecA; local dimB = #vecB
    if dimA ~= dimB then
        debug.log("E:vs01"); return nil
    end
    local r = {}; for i = 1, dimA do r[i] = vecA[i] - vecB[i] end; return r
end

--- vectorScale: ベクトルのスカラー倍
function vectorScale(vec, scalar)
    local r = {}; local dim = #vec; for i = 1, dim do r[i] = vec[i] * scalar end; return r
end

--- matrixVectorMultiply: 行列とベクトルの乗算 (mat * vec)
function matrixVectorMultiply(mat, vec)
    if not mat or #mat == 0 or not mat[1] or #mat[1] == 0 or not vec or #vec == 0 then
        debug.log("E:mv01"); return nil
    end
    local nr = #mat; local nc = #mat[1]; local vd = #vec
    if nc ~= vd then
        debug.log("E:mv02"); return nil
    end
    local rV = {}; for i = 1, nr do
        rV[i] = 0; for j = 1, nc do
            if not mat[i] or mat[i][j] == nil then
                debug.log("E:mv03"); return nil
            end; if vec[j] == nil then
                debug.log("E:mv04"); return nil
            end; rV[i] = rV[i] + mat[i][j] * vec[j]
        end
    end; return rV
end

--- matrixMultiply: 行列同士の乗算 (matA * matB)
function matrixMultiply(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:mm01"); return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if cA ~= rB then
        debug.log("E:mm02"); return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for k = 1, cB do
            local s = 0; for j = 1, cA do
                if not matA[i] or matA[i][j] == nil or not matB[j] or matB[j][k] == nil then
                    debug.log("E:mm03"); return nil
                end; s = s + matA[i][j] * matB[j][k]
            end; rM[i][k] = s
        end
    end; return rM
end

--- matrixTranspose: 行列の転置
function matrixTranspose(mat)
    if not mat or #mat == 0 or (mat[1] and #mat[1] == 0) then
        debug.log("E:mt01"); return nil
    end
    local r = #mat; local c = #mat[1] or 0; local rM = {}; for j = 1, c do rM[j] = {} end
    for i = 1, r do
        local cCL = #mat[i] or 0; if cCL ~= c and i > 1 then debug.log("W:mt01") end
        for j = 1, c do
            if mat[i] and mat[i][j] ~= nil then
                if not rM[j] then rM[j] = {} end; rM[j][i] = mat[i][j]
            else
                if not rM[j] then rM[j] = {} end; rM[j][i] = nil
            end
        end
    end; return rM
end

--#################################################################
--# 1. 定数定義 (トップレベルで定義)
--#################################################################
local MAX_TARGETS_INPUT = 6
local PI = math.pi

--#################################################################
--# 2. グローバル変数 / 状態変数 (トップレベルで初期化)
--#################################################################
local observationBuffer = {}
local lastDetectionTick = {}
local tickCounter = 0

--#################################################################
--# 3. 補助関数
--#################################################################

--- 観測値バッファから平均値を計算する関数
function averageObservations(buffer)
    -- (実装は変更なし)
    if not buffer or not buffer.r or not buffer.theta or not buffer.phi or #buffer.r == 0 then
        debug.log("E:ao01"); return nil
    end
    local count = #buffer.r; if #buffer.theta ~= count or #buffer.phi ~= count then
        debug.log("E:ao01"); return nil
    end
    local sum_r, sum_t, sum_p = 0, 0, 0; for i = 1, count do
        sum_r = sum_r + (buffer.r[i] or 0); sum_t = sum_t + (buffer.theta[i] or 0); sum_p = sum_p + (buffer.phi[i] or 0)
    end
    if count == 0 then return { r = 0, theta = 0, phi = 0 } end; return {
        r = sum_r / count,
        theta = sum_t / count,
        phi =
            sum_p / count
    }
end

--- ローカル角度をグローバル基準角度に変換する関数 (ZYX Euler Order)
function angleLocalToGlobal(theta_local, phi_local, self_orientation)
    if not self_orientation then
        debug.log("E:ac02"); return nil, nil
    end

    -- 1. 入力角度をラジアンに変換
    local theta_rad = (theta_local or 0) * 2 * PI
    local phi_rad = (phi_local or 0) * 2 * PI

    -- 2. ローカル座標系(Z前, X右, Y上)での方向ベクトル(長さ1)を計算
    local cos_phi = math.cos(phi_rad); local sin_phi = math.sin(phi_rad)
    local cos_theta = math.cos(theta_rad); local sin_theta = math.sin(theta_rad)
    local x_L = cos_phi * sin_theta; local y_L = sin_phi; local z_L = cos_phi * cos_theta
    local LV = { x_L, y_L, z_L }

    -- 3. 自機姿勢(ラジアン)を取得 (ch4=Pitch, ch5=Yaw, ch6=Roll)
    local roll = self_orientation.roll or 0   -- Z軸回転 (Roll) - ch6 からの値
    local pitch = self_orientation.pitch or 0 -- X軸回転 (Pitch) - ch4 からの値
    local yaw = self_orientation.yaw or 0     -- Y軸回転 (Yaw) - ch5 からの値

    -- 4. 回転行列(ローカル->グローバル)を計算 (ZYX Euler Order)
    -- R_local_to_world = Rz(yaw) * Ry(pitch) * Rx(roll)
    -- (注: ZYXオーダーは通常、最初にZ(Yaw), 次に新しいY(Pitch), 最後に新しいX(Roll)で回転)
    -- (回転行列の定義は convention によって異なる場合があるため注意)
    local cY = math.cos(yaw); local sY = math.sin(yaw)     -- Yaw (around Y)
    local cP = math.cos(pitch); local sP = math.sin(pitch) -- Pitch (around X)
    local cR = math.cos(roll); local sR = math.sin(roll)   -- Roll (around Z)

    -- 各軸周りの回転行列 (World to Body を仮定)
    -- Rz = {{cR, sR, 0}, {-sR, cR, 0}, {0, 0, 1}} -- Roll around Z
    -- Ry = {{cP, 0, -sP}, {0, 1, 0}, {sP, 0, cP}} -- Pitch around X (間違いやすい: これはY回転)
    -- Rx = {{1, 0, 0}, {0, cP, sP}, {0, -sP, cP}} -- Pitch around X
    -- Ry = {{cY, 0, -sY}, {0, 1, 0}, {sY, 0, cY}} -- Yaw around Y
    -- Rz = {{cR, sR, 0}, {-sR, cR, 0}, {0, 0, 1}} -- Roll around Z

    -- ZYX オーダー (Intrinsic: Z -> new Y -> new X) での Body to World 回転行列
    -- R = Rz(yaw) * Ry(pitch) * Rx(roll)
    -- Z軸周りヨー回転
    local Rz = { { cY, -sY, 0 }, { sY, cY, 0 }, { 0, 0, 1 } }
    -- Y軸周りピッチ回転
    local Ry = { { cP, 0, sP }, { 0, 1, 0 }, { -sP, 0, cP } }
    -- X軸周りロール回転
    local Rx = { { 1, 0, 0 }, { 0, cR, -sR }, { 0, sR, cR } }

    -- 回転行列を計算: Rz * Ry * Rx
    local tempMat = matrixMultiply(Rz, Ry); if not tempMat then
        debug.log("E:ac03"); return nil, nil
    end
    local RotMat = matrixMultiply(tempMat, Rx); if not RotMat then
        debug.log("E:ac03"); return nil, nil
    end

    -- 5. ローカル方向ベクトルを回転行列でグローバル方向ベクトルに変換
    local GV_relative = matrixVectorMultiply(RotMat, LV); if not GV_relative then
        debug.log("E:ac03"); return nil, nil
    end

    -- 6. グローバル方向ベクトルからグローバル角度(ラジアン)を計算
    local dX = GV_relative[1]; local dY = GV_relative[2]; local dZ = GV_relative[3]
    local ground_dist = math.sqrt(dX ^ 2 + dY ^ 2)
    local theta_global_rad = math.atan(dX, dY)        -- atan(East, North)
    local phi_global_rad = math.atan(dZ, ground_dist) -- atan(Up, HorizontalDist)
    return theta_global_rad, phi_global_rad
end

--#################################################################
--# 4. メインループ (onTick) - Stormworks が毎フレーム呼び出す関数
--#################################################################
function onTick()
    tickCounter                      = tickCounter + 1

    -- === 4.1 入力読み取り ===
    -- 自機姿勢 (物理センサー ch4=Pitch, ch5=Yaw, ch6=Roll を想定し、
    -- Lua入力 ch 28, 29, 30 に Composite Read で接続されていると仮定)
    local yaw_val                    = input.getNumber(28) -- Yaw value (from sensor ch 5)
    local pitch_val                  = input.getNumber(29) -- Pitch value (from sensor ch 4)
    local roll_val                   = input.getNumber(30) -- Roll value (from sensor ch 6)
    local self_orient                = { yaw = yaw_val, pitch = pitch_val, roll = roll_val }

    -- レーダー情報 (チャンネル 1-24, Bool 1-6)
    local current_observations_local = {}
    local is_new_cycle               = false
    local detected_flags             = {}
    for i = 1, MAX_TARGETS_INPUT do
        local base_ch = (i - 1) * 4; local is_detected = input.getBool(i); detected_flags[i] = is_detected
        if is_detected then
            local r, t, p, e = input.getNumber(base_ch + 1), input.getNumber(base_ch + 2), input.getNumber(base_ch + 3),
                input.getNumber(base_ch + 4)
            current_observations_local[i] = { r = r, theta = t, phi = p, elapsed = e }; if e == 0 then
                is_new_cycle = true; lastDetectionTick[i] = tickCounter
            end
        else
            if observationBuffer[i] then observationBuffer[i] = nil end
        end
    end

    -- === 4.2 平均化 & 角度変換 ===
    local processed_data_for_output = {}
    if is_new_cycle then
        debug.log("L:ndc01")
        for i = 1, MAX_TARGETS_INPUT do
            if observationBuffer[i] and observationBuffer[i].r and #observationBuffer[i].r > 0 then
                local avg_obs = averageObservations(observationBuffer[i])
                if avg_obs then
                    debug.log("L:ao01"); local global_theta, global_phi = angleLocalToGlobal(avg_obs.theta, avg_obs.phi,
                        self_orient)
                    if global_theta and global_phi then
                        processed_data_for_output[i] = { dist = avg_obs.r, g_theta = global_theta, g_phi = global_phi }; debug
                            .log("L:ac01")
                    else
                        debug.log("W:ac01")
                    end
                end; observationBuffer[i] = nil
            end
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
        local data = processed_data_for_output[i]; local base_num_ch = (i - 1) * 3 + 1; local bool_ch = i; output
            .setBool(bool_ch, detected_flags[i] or false)
        if data and detected_flags[i] then
            output.setNumber(base_num_ch + 0, data.dist); output.setNumber(base_num_ch + 1, data.g_theta); output
                .setNumber(base_num_ch + 2, data.g_phi)
        else
            output.setNumber(base_num_ch + 0, 0); output.setNumber(base_num_ch + 1, 0); output.setNumber(base_num_ch + 2,
                0)
        end
    end
end

--[[ (初期化はトップレベル変数定義で行われる) ]]
