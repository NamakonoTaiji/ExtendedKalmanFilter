--[[
  Stormworks UKF Radar Filter - Main Script Structure
  (Based on User Requirements and Discussions)

  This script outlines the overall structure for the UKF filter.
  It includes helper functions for matrix/vector operations and placeholders
  for complex UKF calculations and other logic like data association.
]]
--[[
  Stormworks UKF実装のための基本的な行列・ベクトル演算関数群
  (ログ出力は debug.log("CODE") 形式)
]]

--[[ Log/Error Code Mappings (for Helper Functions):
  E:va01 - vectorAdd: Dimension mismatch.
  E:vs01 - vectorSubtract: Dimension mismatch.
  E:mv01 - matrixVectorMultiply: Empty/invalid matrix or vector.
  E:mv02 - matrixVectorMultiply: Dimension mismatch (mat cols vs vec dim).
  E:mv03 - matrixVectorMultiply: Invalid matrix element access.
  E:mv04 - matrixVectorMultiply: Invalid vector element access.
  E:mm01 - matrixMultiply: Empty/invalid matrices.
  E:mm02 - matrixMultiply: Dimension mismatch (matA cols vs matB rows).
  E:mm03 - matrixMultiply: Invalid matrix element access during multiplication.
  E:mt01 - matrixTranspose: Empty/invalid matrix.
  W:mt01 - matrixTranspose: Inconsistent column count (Warning).
  E:ma01 - matrixAdd: Empty/invalid matrices.
  E:ma02 - matrixAdd: Dimension mismatch.
  E:ma03 - matrixAdd: Invalid matrix element access during addition.
  E:ms01 - matrixSubtract: Empty/invalid matrices.
  E:ms02 - matrixSubtract: Dimension mismatch.
  E:ms03 - matrixSubtract: Invalid matrix element access during subtraction.
]]

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

--- matrixAdd: 行列同士の加算
function matrixAdd(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:ma01"); return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if rA ~= rB or cA ~= cB then
        debug.log("E:ma02"); return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for j = 1, cA do
            if matA[i] == nil or matA[i][j] == nil or matB[i] == nil or matB[i][j] == nil then
                debug.log("E:ma03"); return nil
            end; rM[i][j] = matA[i][j] + matB[i][j]
        end
    end; return rM
end

--- matrixSubtract: 行列同士の減算 (matA - matB)
function matrixSubtract(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:ms01"); return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if rA ~= rB or cA ~= cB then
        debug.log("E:ms02"); return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for j = 1, cA do
            if matA[i] == nil or matA[i][j] == nil or matB[i] == nil or matB[i][j] == nil then
                debug.log("E:ms03"); return nil
            end; rM[i][j] = matA[i][j] - matB[i][j]
        end
    end; return rM
end

--#################################################################
--# 2. UKF / システム 定数定義
--#################################################################

-- UKF パラメータ (最初は推奨値を使用)
local ALPHA = 0.01 -- シグマポイントの広がり (0.001 - 1)
local BETA = 2     -- 分布の事前情報 (正規分布なら 2)
local KAPPA = 0    -- 補助パラメータ (0 or 3-L)

-- 状態ベクトルの次元数 L (例: X, Y, Z, VX, VY, VZ -> 6次元)
local STATE_DIM = 6

-- 観測ベクトルの次元数 M (例: X, Y, Z -> 3次元)
local OBS_DIM = 3

-- UKF計算用パラメータ λ (lambda)
local LAMBDA = ALPHA ^ 2 * (STATE_DIM + KAPPA) - STATE_DIM

-- ゲームのタイムステップ (tick = 1/60秒)
local DT = 1 / 60

-- 同時に追跡する最大目標数 (Lua入力制限を考慮)
local MAX_TRACKS = 6 -- レーダー(24ch) + 自機(6ch) = 30ch < 32ch

-- データアソシエーション用: 観測と予測の最大許容距離 (要調整)
local ASSOCIATION_THRESHOLD_SQ = 100 * 100 -- 距離の2乗で比較 (sqrtを避けるため)


--#################################################################
--# 3. 複雑な行列演算 (プレースホルダー / TODO)
--#################################################################

--- (TODO) 行列の逆行列を計算する関数
-- @param mat number[][] 正方行列
-- @return number[][] or nil 逆行列、計算不能な場合は nil
function matrixInverse(mat)
    debug.log("Error: matrixInverse function not implemented yet.")
    if not mat or #mat == 0 or #mat ~= #mat[1] then return nil end
    local dim = #mat
    local identity = {}
    for i = 1, dim do
        identity[i] = {}; for j = 1, dim do identity[i][j] = (i == j and 1 or 0) end
    end
    return identity -- ダミー
end

--- (TODO) 行列のコレスキー分解 (下三角行列 L を求める P = L*L^T)
-- @param mat number[][] 対称な正定値行列 P
-- @return number[][] or nil 下三角行列 L、分解不能な場合は nil
function matrixCholesky(mat)
    debug.log("Error: matrixCholesky function not implemented yet.")
    if not mat or #mat == 0 or #mat ~= #mat[1] then return nil end
    local dim = #mat
    local identity = {}
    for i = 1, dim do
        identity[i] = {}; for j = 1, dim do identity[i][j] = (i == j and 1 or 0) end
    end
    return identity -- ダミー
end

--#################################################################
--# 4. UKF コア関数 (プレースホルダー / TODO)
--#################################################################

--- (TODO) シグマポイントを生成する関数
-- @param x number[] 現在の状態ベクトル (L次元)
-- @param P number[][] 現在の共分散行列 (LxL)
-- @return table or nil シグマポイントのセット (テーブル、例: {points={}, Wm={}, Wc={}})
function generateSigmaPoints(x, P)
    debug.log("Error: generateSigmaPoints function not implemented yet.")
    -- ... (実装は省略) ...
    local dummy_points = {}; for i = 0, 2 * STATE_DIM do dummy_points[i] = x end
    local dummy_Wm = {}; local dummy_Wc = {}; dummy_Wm[0] = 1.0; dummy_Wc[0] = 1.0
    for i = 1, 2 * STATE_DIM do
        dummy_Wm[i] = 0; dummy_Wc[i] = 0
    end
    return { points = dummy_points, Wm = dummy_Wm, Wc = dummy_Wc } -- ダミー
end

--- (TODO) UKF 予測ステップ
-- @param track table 追跡中の目標情報 (x, P を含む)
-- @param F number[][] プロセスモデル遷移行列
-- @param Q number[][] プロセスノイズ共分散行列
-- @return table or nil 予測された状態 x_pred と共分散 P_pred を含むテーブル
function ukfPredict(track, F, Q)
    debug.log("LOG: ukfPredict called for track ID " .. (track and track.id or "N/A"))
    local x = track.x; local P = track.P
    local sigmaData = generateSigmaPoints(x, P)
    if not sigmaData then return nil end
    local predictedSigmaPoints = {}; for i = 0, 2 * STATE_DIM do
        predictedSigmaPoints[i] = matrixVectorMultiply(F, sigmaData.points[i]); if not predictedSigmaPoints[i] then return nil end
    end
    local x_pred = x; local P_yy = P -- ダミーUT
    debug.log("Warning: ukfPredict - Unscented Transform for prediction not implemented.")
    local P_pred = matrixAdd(P_yy, Q)
    if not P_pred then return nil end
    debug.log("LOG: ukfPredict finished for track ID " .. (track and track.id or "N/A") .. " (using dummy UT).")
    return { x = x_pred, P = P_pred }
end

--- (TODO) UKF 更新ステップ
-- @param predicted table 予測された状態 x_pred と共分散 P_pred
-- @param z number[] 実際の観測値 (平均化・座標変換済み)
-- @param H number[][] 観測モデル行列 (今回は線形)
-- @param R number[][] 観測ノイズ共分散行列 (平均化・近似済み)
-- @return table or nil 更新された状態 x と共分散 P を含むテーブル
function ukfUpdate(predicted, z, H, R)
    debug.log("LOG: ukfUpdate called.")
    local x_pred = predicted.x; local P_pred = predicted.P
    debug.log("Warning: ukfUpdate - UKF update logic (UT or simplified) not fully implemented.")
    -- 線形KFの更新式を参考にダミー実装 (UKFとは異なる)
    local H_x_pred = matrixVectorMultiply(H, x_pred)
    if not H_x_pred then return nil end
    local y = vectorSubtract(z, H_x_pred)
    if not y then return nil end
    local PHT = matrixMultiply(P_pred, matrixTranspose(H))
    if not PHT then return nil end
    local H_PHT = matrixMultiply(H, PHT)
    if not H_PHT then return nil end
    local S = matrixAdd(H_PHT, R)
    if not S then return nil end
    local S_inv = matrixInverse(S) -- S^-1 (TODO: 逆行列実装)
    if not S_inv then
        debug.log("Error: ukfUpdate - Failed to invert S."); return nil
    end
    local K = matrixMultiply(PHT, S_inv) -- カルマンゲイン K
    if not K then return nil end
    local K_y = matrixVectorMultiply(K, y)
    if not K_y then return nil end
    local x_new = vectorAdd(x_pred, K_y) -- 状態更新
    if not x_new then return nil end
    local KS = matrixMultiply(K, S)
    if not KS then return nil end
    local KSKT = matrixMultiply(KS, matrixTranspose(K)) -- K*S*K^T
    if not KSKT then return nil end
    local P_new = matrixSubtract(P_pred, KSKT)          -- 共分散更新
    if not P_new then return nil end
    debug.log("LOG: ukfUpdate finished (using dummy KF-like update).")
    return { x = x_new, P = P_new }
end

--#################################################################
--# 5. その他の補助関数 (プレースホルダー / TODO)
--#################################################################

--- (TODO) ローカル極座標をグローバル直交座標に変換
function localToGlobal(r, theta_local, phi_local, self_pos, self_orientation)
    debug.log("Error: localToGlobal function not implemented yet.")
    -- 1. ローカル極座標 -> ローカル直交座標 (Z前, X右, Y上)
    -- 2. 自機姿勢(オイラー角ZYX)から回転行列を計算
    -- 3. ローカル直交座標を回転行列でグローバル相対ベクトルに変換
    -- 4. 自機グローバル位置に相対ベクトルを加算
    return { 0, 0, 0 } -- ダミー
end

--- (TODO) 観測値バッファから平均値を計算
function averageObservations(buffer)
    debug.log("Error: averageObservations function not implemented yet.")
    -- 1. バッファ内の距離、ローカル方位角、ローカル仰角をそれぞれ単純平均
    -- 2. 平均値を返す (例: {r=avg_r, theta=avg_theta, phi=avg_phi})
    return { r = 0, theta = 0, phi = 0 } -- ダミー
end

--- (TODO) データアソシエーション (GNN: Global Nearest Neighbor)
function associateMeasurements(active_tracks_with_pred, observations_global)
    debug.log("Error: associateMeasurements function not implemented yet.")
    -- 1. 各アクティブトラックの予測位置を取得 (active_tracks_with_pred から)
    -- 2. 各観測値と各予測位置の間の距離（2乗距離）を計算
    -- 3. 距離行列を作成
    -- 4. 最小距離のペアを繰り返し選択（閾値チェックあり）
    -- 5. 対応付け結果 (例: {trackID1=obsIndex1, trackID2=obsIndex3, ...}) と、
    --    未対応の観測値リスト、未対応のトラックリストを返す
    local associations = {}        -- ダミー
    local unassociated_obs = {}    -- ダミー
    local unassociated_tracks = {} -- ダミー
    -- ダミー: 最初の観測を最初のトラックに割り当て
    if #active_tracks_with_pred > 0 and #observations_global > 0 then
        local first_track_id = active_tracks_with_pred[1].id -- 仮
        local first_obs_key = next(observations_global)      -- observations_global はキーがチャンネル番号のテーブル
        if first_track_id and first_obs_key then
            associations[first_track_id] = first_obs_key
        end
    end
    return associations, unassociated_obs, unassociated_tracks -- ダミー
end

--#################################################################
--# 6. 初期化処理
--#################################################################

-- グローバル変数 or 初期化関数内で実行
local tracks = {}            -- アクティブなトラック情報 (例: tracks[trackID] = {id=trackID, x=stateVec, P=covMat, age=0, active=true, last_update_tick=0})
local observationBuffer = {} -- 観測値バッファ (例: buffer[channel_idx] = {r={}, theta={}, phi={}})
local lastDetectionTick = {} -- 各チャンネルの最後の探知更新tick
local tickCounter = 0
local nextTrackID = 1        -- 新規トラックに割り当てるID

-- プロセスモデル F (例: 等速直線運動) - tick毎に適用
local F_matrix = {
    { 1, 0, 0, DT, 0, 0 }, { 0, 1, 0, 0, DT, 0 }, { 0, 0, 1, 0, 0, DT },
    { 0, 0, 0, 1,  0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 }
}
-- プロセスノイズ Q (要調整) - 加速度などの不確かさ
local Q_matrix = { -- 例: 小さな値を対角に配置
    { 0.01, 0, 0,    0, 0, 0 }, { 0, 0.01, 0, 0, 0, 0 },
    { 0,    0, 0.01, 0, 0, 0 }, { 0, 0, 0, 0.01, 0, 0 },
    { 0, 0, 0, 0, 0.01, 0 }, { 0, 0, 0, 0, 0, 0.01 }
}
-- 観測モデル H (状態から位置を取り出す)
local H_matrix = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 } }
-- 観測ノイズ R (平均化・座標変換・正規分布近似後、要計算・調整)
local R_matrix = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } -- 仮の値


--#################################################################
--# 7. メインループ (onTick)
--#################################################################

function onTick()
    tickCounter = tickCounter + 1

    -- === 7.1 入力読み取り ===
    -- 自機センサー情報 (チャンネル 25-30 を使用)
    local self_x = input.getNumber(25)     -- ワールドX座標 (East)
    local self_y = input.getNumber(26)     -- ワールドY座標 (North)
    local self_z = input.getNumber(27)     -- ワールドZ座標 (Up)
    local self_yaw = input.getNumber(28)   -- ヨー (ラジアン, ZYX順)
    local self_pitch = input.getNumber(29) -- ピッチ (ラジアン, ZYX順)
    local self_roll = input.getNumber(30)  -- ロール (ラジアン, ZYX順)
    -- チャンネル 31, 32 は空き

    local self_pos = { self_x, self_y, self_z }
    local self_orientation = { yaw = self_yaw, pitch = self_pitch, roll = self_roll }

    -- レーダー情報
    local detected_channel_indices = {}   -- 現在検出中のチャンネルインデックスリスト (1-6)
    local current_observations_local = {} -- 各チャンネルのローカル観測値 {r, theta, phi, elapsed}
    local is_new_detection_cycle = false  -- 探知更新タイミングかどうかのフラグ

    for i = 1, MAX_TRACKS do
        local base_ch = (i - 1) * 4
        local is_detected = input.getBool(i) -- オンオフ入力チャンネル 1-6

        if is_detected then
            table.insert(detected_channel_indices, i)
            local r = input.getNumber(base_ch + 1)       -- 数値入力チャンネル 1, 5, 9, ...
            local theta = input.getNumber(base_ch + 2)   -- 数値入力チャンネル 2, 6, 10, ...
            local phi = input.getNumber(base_ch + 3)     -- 数値入力チャンネル 3, 7, 11, ...
            local elapsed = input.getNumber(base_ch + 4) -- 数値入力チャンネル 4, 8, 12, ...

            current_observations_local[i] = { r = r, theta = theta, phi = phi, elapsed = elapsed }

            if elapsed == 0 then
                is_new_detection_cycle = true
                lastDetectionTick[i] = tickCounter -- 記録はチャンネルごとに行う方が後で役立つかも
            end
        else
            if observationBuffer[i] then observationBuffer[i] = nil end
        end
    end

    -- === 7.2 観測値の蓄積と平均化 ===
    local averaged_observations_local = {} -- キーはチャンネル番号 i
    if is_new_detection_cycle then
        debug.log("LOG: New detection cycle started at tick " .. tickCounter)
        for i = 1, MAX_TRACKS do
            if observationBuffer[i] and observationBuffer[i].r and #observationBuffer[i].r > 0 then
                averaged_observations_local[i] = averageObservations(observationBuffer[i]) -- TODO 実装
                debug.log("LOG: Averaged observation calculated for channel " .. i)
                observationBuffer[i] = nil                                                 -- バッファクリア
            end
        end
    end

    -- 現在の観測値をバッファに蓄積
    for channel_idx, obs_local in pairs(current_observations_local) do
        if not observationBuffer[channel_idx] then
            observationBuffer[channel_idx] = { r = {}, theta = {}, phi = {} }
        end
        table.insert(observationBuffer[channel_idx].r, obs_local.r)
        table.insert(observationBuffer[channel_idx].theta, obs_local.theta)
        table.insert(observationBuffer[channel_idx].phi, obs_local.phi)
    end

    -- === 7.3 グローバル座標への変換 ===
    local observations_global = {} -- キーはチャンネル番号 i
    for channel_idx, avg_obs_local in pairs(averaged_observations_local) do
        local global_pos = localToGlobal(avg_obs_local.r, avg_obs_local.theta, avg_obs_local.phi, self_pos,
            self_orientation) -- TODO 実装
        if global_pos then
            observations_global[channel_idx] = global_pos
            debug.log("LOG: Observation from channel " .. channel_idx .. " converted to global.")
        end
    end

    -- === 7.4 データアソシエーション ===
    local active_tracks_with_pred = {} -- 予測ステップ後のアクティブトラック情報 {id=..., x=..., P=..., predicted_pos=...}
    for trackID, trackData in pairs(tracks) do
        if trackData.active then
            -- UKF予測を先に実行しておく必要がある
            local predicted = ukfPredict(trackData, F_matrix, Q_matrix) -- TODO 実装
            if predicted then
                trackData.predicted_x = predicted.x
                trackData.predicted_P = predicted.P
                table.insert(active_tracks_with_pred, {
                    id = trackID,
                    x = trackData.x,                                                   -- 現在の状態 (予測前)
                    P = trackData.P,                                                   -- 現在の共分散 (予測前)
                    predicted_x = predicted.x,                                         -- 予測後の状態
                    predicted_P = predicted.P,                                         -- 予測後の共分散
                    predicted_pos = { predicted.x[1], predicted.x[2], predicted.x[3] } -- 予測位置
                })
                debug.log("LOG: UKF Predict done for track " .. trackID)
            else
                debug.log("Error: UKF Predict failed for track " .. trackID)
                trackData.active = false -- 予測失敗したら非アクティブ化
            end
        end
    end

    -- 観測値リストを作成 (データアソシエーション用)
    local obs_list_for_assoc = {} -- {index=channel_idx, pos={x,y,z}} のリスト
    for obs_idx, obs_pos in pairs(observations_global) do
        table.insert(obs_list_for_assoc, { index = obs_idx, pos = obs_pos })
    end

    -- データアソシエーション実行 (TODO: associateMeasurements 関数を実装)
    local associations, unassociated_obs, unassociated_track_ids
    associations, unassociated_obs, unassociated_track_ids = associateMeasurements(active_tracks_with_pred,
        obs_list_for_assoc)
    debug.log("LOG: Data association performed.")


    -- === 7.5 UKF 更新 ループ ===
    local updated_tracks_temp = {} -- 更新後のトラック情報を一時保存するテーブル {trackID = {x=..., P=..., age=..., last_update_tick=...}}

    -- アソシエーション結果に基づいて更新を実行
    for trackID, obs_channel_index in pairs(associations) do
        local trackData = nil
        -- active_tracks_with_pred から該当トラック情報を取得
        for _, trk in ipairs(active_tracks_with_pred) do
            if trk.id == trackID then
                trackData = trk; break;
            end
        end

        if trackData and observations_global[obs_channel_index] then
            local z = observations_global[obs_channel_index] -- 対応する観測値
            local predicted = { x = trackData.predicted_x, P = trackData.predicted_P }

            -- UKF 更新ステップ実行 (TODO: ukfUpdate 関数を実装)
            local updated = ukfUpdate(predicted, z, H_matrix, R_matrix)

            if updated then
                updated_tracks_temp[trackID] = {
                    x = updated.x,
                    P = updated.P,
                    age = tracks[trackID].age + 1, -- age は元のトラック情報から
                    last_update_tick = tickCounter
                }
                debug.log("LOG: UKF Update done for track " ..
                    trackID .. " with observation from channel " .. obs_channel_index)
            else
                debug.log("Error: UKF Update failed for track " .. trackID)
                -- エラー処理 (元のトラック情報を非アクティブ化など)
                if tracks[trackID] then tracks[trackID].active = false end
            end
        else
            debug.log("Warning: Skipping update for track " .. trackID .. " due to missing data (track or observation).")
            -- 対応付けはされたがデータがない場合もトラックを非アクティブにするか？検討要
            if tracks[trackID] then tracks[trackID].active = false end
        end
    end

    -- 更新されなかったトラックの処理 (予測結果をそのまま使う)
    for _, trackID in ipairs(unassociated_track_ids) do
        local trackData = tracks[trackID]
        if trackData and trackData.active and trackData.predicted_x then
            updated_tracks_temp[trackID] = {
                x = trackData.predicted_x,
                P = trackData.predicted_P,
                age = trackData.age + 1,
                last_update_tick = trackData.last_update_tick -- 更新されなかったので tick はそのまま
            }
            debug.log("LOG: Track " .. trackID .. " propagated without update.")
        end
    end

    -- tracks テーブルを更新後の情報で置き換え
    for trackID, updateData in pairs(updated_tracks_temp) do
        if tracks[trackID] then
            tracks[trackID].x = updateData.x
            tracks[trackID].P = updateData.P
            tracks[trackID].age = updateData.age
            tracks[trackID].last_update_tick = updateData.last_update_tick
            tracks[trackID].predicted_x = nil -- 一時変数をクリア
            tracks[trackID].predicted_P = nil
        else
            -- updated_tracks_temp に含まれるが tracks にないケースは通常ないはず
            debug.log("Warning: Track ID " .. trackID .. " found in updates but not in main tracks table.")
        end
    end


    -- === 7.6 トラック管理 ===
    -- 新規トラックの開始 (TODO)
    for _, obs_data in ipairs(unassociated_obs) do
        local obs_index = obs_data.index
        local obs_pos = obs_data.pos
        debug.log("LOG: Unassociated observation from channel " .. obs_index .. ". Initialize new track? (TODO)")
        -- ここで obs_pos を使って新しいトラックを tracks テーブルに追加するロジック
        -- 例: local newID = nextTrackID; tracks[newID] = {id=newID, x={obs_pos[1], obs_pos[2], obs_pos[3], 0, 0, 0}, P=Initial_P, age=1, active=true, last_update_tick=tickCounter}; nextTrackID = nextTrackID + 1
    end

    -- ロストトラックの削除 (TODO)
    local lost_threshold = 300 -- 例: 5秒 (300 tick) 更新がなければロストとみなす
    local tracks_to_delete = {}
    for trackID, trackData in pairs(tracks) do
        if trackData.active and (tickCounter - trackData.last_update_tick > lost_threshold) then
            debug.log("LOG: Track " .. trackID .. " lost (no update for " .. lost_threshold .. " ticks). Deleting.")
            -- tracks[trackID] = nil -- または active=false にする
            table.insert(tracks_to_delete, trackID)
        end
    end
    for _, trackID in ipairs(tracks_to_delete) do
        tracks[trackID] = nil -- テーブルから削除
    end


    -- === 7.7 出力書き込み ===
    -- フィルター結果 (tracks テーブル内の x ベクトルなど) を
    -- 数値/オンオフチャンネルに出力する (TODO)
    local output_channel_num = 1  -- 出力用数値チャンネルの開始番号 (仮)
    local output_channel_bool = 1 -- 出力用オンオフチャンネルの開始番号 (仮)
    local count = 0
    for trackID, trackData in pairs(tracks) do
        if trackData.active and count < MAX_TRACKS then -- 出力も最大数まで
            count = count + 1
            output.setBool(output_channel_bool, true)
            output.setNumber(output_channel_num + 0, trackData.x[1]) -- X
            output.setNumber(output_channel_num + 1, trackData.x[2]) -- Y
            output.setNumber(output_channel_num + 2, trackData.x[3]) -- Z
            output.setNumber(output_channel_num + 3, trackData.x[4]) -- VX
            output.setNumber(output_channel_num + 4, trackData.x[5]) -- VY
            output.setNumber(output_channel_num + 5, trackData.x[6]) -- VZ
            -- 必要なら他の情報も出力 (例: track ID, age など)
            output_channel_num = output_channel_num + 6              -- 次の目標用にチャンネルをずらす
            output_channel_bool = output_channel_bool + 1
        end
    end
    -- 出力されなかったチャンネルをクリア
    for i = count + 1, MAX_TRACKS do
        output.setBool(output_channel_bool + (i - count - 1), false)
        local base_ch = output_channel_num + (i - count - 1) * 6
        for j = 0, 5 do output.setNumber(base_ch + j, 0) end
    end
end

--#################################################################
--# 8. 初期化呼び出し (必要に応じて)
--#################################################################
-- この関数はStormworksによって自動的に呼び出されることはないかもしれません。
-- グローバルスコープで初期化するか、onTick内で初回のみ実行するなどの工夫が必要。
function initialize()
    debug.log("Initializing UKF Script...")
    tracks = {}
    observationBuffer = {}
    lastDetectionTick = {}
    tickCounter = 0
    nextTrackID = 1
    -- 必要なら R_matrix や Q_matrix をプロパティ等から読み込んで設定
    debug.log("Initialization Complete.")
end

-- スクリプトロード時に一度だけ初期化を実行する例 (onTick 内で初回判定する方が確実かも)
-- initialize()

-- onTick 内で初回実行時に初期化する例
local isInitialized = false
function onTick()
    if not isInitialized then
        initialize()
        isInitialized = true
    end
    -- ... (以降は通常の onTick 処理) ...
    tickCounter = tickCounter + 1
    -- ...
end
