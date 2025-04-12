--[[
  Stormworks UKF Radar Filter - Main Script Structure
  (Based on User Requirements and Discussions)

  This script outlines the overall structure for the UKF filter.
  It includes helper functions for matrix/vector operations and placeholders
  for complex UKF calculations and other logic like data association.
  Uses debug.log() with shortened codes for output.
]]

--[[ Log/Error Code Mappings:
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
  E:mi01 - matrixInverse: Not implemented.
  E:mc01 - matrixCholesky: Not implemented.
  E:gs01 - generateSigmaPoints: Not implemented.
  L:up01 - ukfPredict: Called.
  W:up01 - ukfPredict: Unscented Transform not implemented (dummy used).
  L:up02 - ukfPredict: Finished (dummy UT).
  E:up01 - ukfPredict: Failed (e.g., sigma point generation failed).
  L:uu01 - ukfUpdate: Called.
  W:uu01 - ukfUpdate: UKF update logic not fully implemented (dummy used).
  E:uu01 - ukfUpdate: Failed to invert S.
  E:uu02 - ukfUpdate: Failed (e.g., matrix operation failed).
  L:uu02 - ukfUpdate: Finished (dummy KF-like).
  E:ltg01 - localToGlobal: Not implemented.
  E:ao01 - averageObservations: Not implemented.
  E:da01 - associateMeasurements: Not implemented.
  L:da01 - Data association performed.
  L:init01 - Initializing script.
  L:init02 - Initialization complete.
  L:ndc01 - New detection cycle started.
  L:ao01 - Averaged observation calculated for channel X.
  L:ltg01 - Observation from channel X converted to global.
  W:uu02 - Skipping update for track X due to missing data.
  L:trk01 - Track X propagated without update.
  W:trk01 - Track ID X found in updates but not in main tracks table.
  L:trk02 - Unassociated observation from channel X. Initialize new track? (TODO)
  L:trk03 - Track X lost (no update). Deleting.
]]

--#################################################################
--# 1. 行列・ベクトル演算 ヘルパー関数群
--#################################################################

function vectorAdd(vecA, vecB)
    local dimA = #vecA; local dimB = #vecB
    if dimA ~= dimB then
        debug.log("E:va01")
        return nil
    end
    local r = {}; for i = 1, dimA do r[i] = vecA[i] + vecB[i] end; return r
end

function vectorSubtract(vecA, vecB)
    local dimA = #vecA; local dimB = #vecB
    if dimA ~= dimB then
        debug.log("E:vs01")
        return nil
    end
    local r = {}; for i = 1, dimA do r[i] = vecA[i] - vecB[i] end; return r
end

function vectorScale(vec, scalar)
    local r = {}; local dim = #vec; for i = 1, dim do r[i] = vec[i] * scalar end; return r
end

function matrixVectorMultiply(mat, vec)
    if not mat or #mat == 0 or not mat[1] or #mat[1] == 0 or not vec or #vec == 0 then
        debug.log("E:mv01")
        return nil
    end
    local nr = #mat; local nc = #mat[1]; local vd = #vec
    if nc ~= vd then
        debug.log("E:mv02")
        return nil
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

function matrixMultiply(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:mm01")
        return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if cA ~= rB then
        debug.log("E:mm02")
        return nil
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

function matrixTranspose(mat)
    if not mat or #mat == 0 or (mat[1] and #mat[1] == 0) then
        debug.log("E:mt01")
        return nil
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

function matrixAdd(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:ma01")
        return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if rA ~= rB or cA ~= cB then
        debug.log("E:ma02")
        return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for j = 1, cA do
            if matA[i] == nil or matA[i][j] == nil or matB[i] == nil or matB[i][j] == nil then
                debug.log("E:ma03"); return nil
            end; rM[i][j] = matA[i][j] + matB[i][j]
        end
    end; return rM
end

function matrixSubtract(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:ms01")
        return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if rA ~= rB or cA ~= cB then
        debug.log("E:ms02")
        return nil
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
local ALPHA = 0.01; local BETA = 2; local KAPPA = 0; local STATE_DIM = 6; local OBS_DIM = 3
local LAMBDA = ALPHA ^ 2 * (STATE_DIM + KAPPA) - STATE_DIM; local DT = 1 / 60; local MAX_TRACKS = 6
local ASSOCIATION_THRESHOLD_SQ = 100 * 100

--#################################################################
--# 3. 複雑な行列演算 (プレースホルダー / TODO)
--#################################################################
function matrixInverse(mat)
    debug.log("E:mi01"); if not mat or #mat == 0 or #mat ~= #mat[1] then return nil end
    local d = #mat; local id = {}; for i = 1, d do
        id[i] = {}; for j = 1, d do id[i][j] = (i == j and 1 or 0) end
    end; return id
end

function matrixCholesky(mat)
    debug.log("E:mc01"); if not mat or #mat == 0 or #mat ~= #mat[1] then return nil end
    local d = #mat; local id = {}; for i = 1, d do
        id[i] = {}; for j = 1, d do id[i][j] = (i == j and 1 or 0) end
    end; return id
end

--#################################################################
--# 4. UKF コア関数 (プレースホルダー / TODO)
--#################################################################
function generateSigmaPoints(x, P)
    debug.log("E:gs01")
    local dP = {}; for i = 0, 2 * STATE_DIM do dP[i] = x end; local dWm = {}; local dWc = {}; dWm[0] = 1.0; dWc[0] = 1.0
    for i = 1, 2 * STATE_DIM do
        dWm[i] = 0; dWc[i] = 0
    end; return { points = dP, Wm = dWm, Wc = dWc }
end

function ukfPredict(track, F, Q)
    debug.log("L:up01") --(track and track.id or "N/A"))
    local x = track.x; local P = track.P; local sD = generateSigmaPoints(x, P); if not sD then return nil end
    local pSP = {}; for i = 0, 2 * STATE_DIM do
        pSP[i] = matrixVectorMultiply(F, sD.points[i]); if not pSP[i] then return nil end
    end
    local x_pred = x; local P_yy = P; debug.log("W:up01")
    local P_pred = matrixAdd(P_yy, Q); if not P_pred then return nil end
    debug.log("L:up02") --(track and track.id or "N/A"))
    return { x = x_pred, P = P_pred }
end

function ukfUpdate(predicted, z, H, R)
    debug.log("L:uu01")
    local x_pred = predicted.x; local P_pred = predicted.P; debug.log("W:uu01")
    local Hxp = matrixVectorMultiply(H, x_pred); if not Hxp then return nil end
    local y = vectorSubtract(z, Hxp); if not y then return nil end
    local PHT = matrixMultiply(P_pred, matrixTranspose(H)); if not PHT then return nil end
    local HPHT = matrixMultiply(H, PHT); if not HPHT then return nil end
    local S = matrixAdd(HPHT, R); if not S then return nil end
    local Si = matrixInverse(S); if not Si then
        debug.log("E:uu01"); return nil
    end
    local K = matrixMultiply(PHT, Si); if not K then return nil end
    local Ky = matrixVectorMultiply(K, y); if not Ky then return nil end
    local xn = vectorAdd(x_pred, Ky); if not xn then return nil end
    local KS = matrixMultiply(K, S); if not KS then return nil end
    local KSKT = matrixMultiply(KS, matrixTranspose(K)); if not KSKT then return nil end
    local Pn = matrixSubtract(P_pred, KSKT); if not Pn then return nil end
    debug.log("L:uu02")
    return { x = xn, P = Pn }
end

--#################################################################
--# 5. その他の補助関数 (プレースホルダー / TODO)
--#################################################################
function localToGlobal(r, theta_local, phi_local, self_pos, self_orientation)
    debug.log("E:ltg01"); return { 0, 0, 0 }
end

function averageObservations(buffer)
    debug.log("E:ao01"); return { r = 0, theta = 0, phi = 0 }
end

function associateMeasurements(active_tracks_with_pred, observations_global)
    debug.log("E:da01")
    local a = {}; local uo = {}; local ut = {}
    if #active_tracks_with_pred > 0 and next(observations_global) ~= nil then
        local tid = active_tracks_with_pred[1].id; local ok = next(observations_global)
        if tid and ok then a[tid] = ok end
    end; return a, uo, ut
end

--#################################################################
--# 6. 初期化処理
--#################################################################
local tracks = {}; local observationBuffer = {}; local lastDetectionTick = {}; local tickCounter = 0; local nextTrackID = 1
local F_matrix = { { 1, 0, 0, DT, 0, 0 }, { 0, 1, 0, 0, DT, 0 }, { 0, 0, 1, 0, 0, DT }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }
local Q_matrix = { { 0.01, 0, 0, 0, 0, 0 }, { 0, 0.01, 0, 0, 0, 0 }, { 0, 0, 0.01, 0, 0, 0 }, { 0, 0, 0, 0.01, 0, 0 }, { 0, 0, 0, 0, 0.01, 0 }, { 0, 0, 0, 0, 0, 0.01 } }
local H_matrix = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 } }
local R_matrix = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } -- 仮

--#################################################################
--# 7. メインループ (onTick)
--#################################################################
function onTick_MainLogic() -- onTickのメイン処理を別関数に
    tickCounter = tickCounter + 1
    -- === 7.1 入力 ===
    local sx = input.getNumber(25); local sy = input.getNumber(26); local sz = input.getNumber(27)
    local syaw = input.getNumber(28); local spitch = input.getNumber(29); local sroll = input.getNumber(30)
    local spos = { sx, sy, sz }; local sorient = { yaw = syaw, pitch = spitch, roll = sroll }
    local detected_ch_idx = {}; local current_obs_local = {}; local is_new_cycle = false
    for i = 1, MAX_TRACKS do
        local bc = (i - 1) * 4; local det = input.getBool(i)
        if det then
            table.insert(detected_ch_idx, i); local r, t, p, e = input.getNumber(bc + 1), input.getNumber(bc + 2),
                input.getNumber(bc + 3), input.getNumber(bc + 4)
            current_obs_local[i] = { r = r, theta = t, phi = p, elapsed = e }; if e == 0 then
                is_new_cycle = true; lastDetectionTick[i] = tickCounter
            end
        else
            if observationBuffer[i] then observationBuffer[i] = nil end
        end
    end
    -- === 7.2 平均化 ===
    local avg_obs_local = {}; if is_new_cycle then
        debug.log("L:ndc01")
        for i = 1, MAX_TRACKS do
            if observationBuffer[i] and observationBuffer[i].r and #observationBuffer[i].r > 0 then
                avg_obs_local[i] = averageObservations(observationBuffer[i]); debug.log("L:ao01"); observationBuffer[i] = nil
            end
        end
    end
    for ci, ol in pairs(current_obs_local) do
        if not observationBuffer[ci] then observationBuffer[ci] = { r = {}, theta = {}, phi = {} } end; table.insert(
            observationBuffer[ci].r, ol.r); table.insert(observationBuffer[ci].theta, ol.theta); table.insert(
            observationBuffer[ci].phi, ol.phi)
    end
    -- === 7.3 座標変換 ===
    local obs_global = {}; for ci, aol in pairs(avg_obs_local) do
        local gp = localToGlobal(aol.r, aol.theta, aol.phi, spos, sorient); if gp then
            obs_global[ci] = gp; debug.log("L:ltg01")
        end
    end
    -- === 7.4 データアソシエーション & 7.5 UKF 予測/更新 (密結合している) ===
    local active_trks_pred = {}; -- UKF予測結果を含むアクティブトラックリスト
    for tid, td in pairs(tracks) do
        if td.active then
            local pred = ukfPredict(td, F_matrix, Q_matrix);
            if pred then
                td.predicted_x = pred.x; td.predicted_P = pred.P; table.insert(active_trks_pred,
                    { id = tid, x = td.x, P = td.P, predicted_x = pred.x, predicted_P = pred.P, predicted_pos = { pred.x[1], pred.x[2], pred.x[3] } });
            else
                debug.log("E:up01"); td.active = false
            end
        end
    end
    local obs_list_assoc = {}; for oi, op in pairs(obs_global) do table.insert(obs_list_assoc, { index = oi, pos = op }) end
    local assoc, unassoc_obs, unassoc_trks = associateMeasurements(active_trks_pred, obs_list_assoc); debug.log("L:da01")
    local updated_trks_tmp = {};       -- 更新結果一時保存
    for tid, obs_ci in pairs(assoc) do -- 対応付けられたトラックを更新
        local trkData = nil; for _, trk in ipairs(active_trks_pred) do
            if trk.id == tid then
                trkData = trk; break
            end
        end
        if trkData and obs_global[obs_ci] then
            local z = obs_global[obs_ci]; local pred = { x = trkData.predicted_x, P = trkData.predicted_P }; local updated =
                ukfUpdate(pred, z, H_matrix, R_matrix)
            if updated then
                updated_trks_tmp[tid] = {
                    x = updated.x,
                    P = updated.P,
                    age = tracks[tid].age + 1,
                    last_update_tick =
                        tickCounter
                };
            else
                debug.log("E:uu02"); if tracks[tid] then tracks[tid].active = false end
            end
        else
            debug.log("W:uu02"); if tracks[tid] then tracks[tid].active = false end
        end
    end
    for _, tid in ipairs(unassoc_trks) do -- 更新されなかったトラック (予測値を採用)
        local td = nil; for _, trk in ipairs(active_trks_pred) do
            if trk.id == tid then
                td = trk; break
            end
        end                                                   -- active_tracks_with_predから取得
        if td and tracks[td.id] and tracks[td.id].active then -- 元のtracksテーブルも参照
            updated_trks_tmp[tid] = {
                x = td.predicted_x,
                P = td.predicted_P,
                age = tracks[tid].age + 1,
                last_update_tick =
                    tracks[tid].last_update_tick
            }; debug.log("L:trk01")
        end
    end
    for tid, upd in pairs(updated_trks_tmp) do -- tracksテーブルを更新
        if tracks[tid] then
            tracks[tid].x = upd.x; tracks[tid].P = upd.P; tracks[tid].age = upd.age; tracks[tid].last_update_tick = upd
                .last_update_tick; tracks[tid].predicted_x = nil; tracks[tid].predicted_P = nil;
        else
            debug.log("W:trk01")
        end
    end
    -- === 7.6 トラック管理 ===
    for _, obs_d in ipairs(unassoc_obs) do
        local oi = obs_d.index; local op = obs_d.pos; debug.log("L:trk02") --("LOG: Unassociated observation from channel "..oi..". Initialize new track? (TODO)")
        -- TODO: 新規トラック初期化ロジック
        -- local newID = nextTrackID
        -- tracks[newID] = {id=newID, x={op[1], op[2], op[3], 0, 0, 0}, P=Initial_P_Matrix, age=1, active=true, last_update_tick=tickCounter}
        -- nextTrackID = nextTrackID + 1
    end
    local lost_thresh = 300; local trks_del = {}; for tid, td in pairs(tracks) do
        if td.active and (tickCounter - (td.last_update_tick or 0) > lost_thresh) then
            debug.log("L:trk03"); table.insert(trks_del, tid)
        end
    end; for _, tid in ipairs(trks_del) do tracks[tid] = nil end
    -- === 7.7 出力 ===
    local out_num = 1; local out_bool = 1; local cnt = 0
    for tid, td in pairs(tracks) do
        if td and td.active and cnt < MAX_TRACKS then
            cnt = cnt + 1; output.setBool(out_bool, true); output.setNumber(out_num + 0, td.x[1]); output.setNumber(
                out_num + 1, td.x[2]); output.setNumber(out_num + 2, td.x[3]); output.setNumber(out_num + 3, td.x[4]); output
                .setNumber(out_num + 4, td.x[5]); output.setNumber(out_num + 5, td.x[6]); out_num = out_num + 6; out_bool =
                out_bool + 1
        end
    end
    for i = cnt + 1, MAX_TRACKS do
        output.setBool(out_bool + (i - cnt - 1), false); local bc = out_num + (i - cnt - 1) * 6; for j = 0, 5 do
            output
                .setNumber(bc + j, 0)
        end
    end
end

--#################################################################
--# 8. 初期化呼び出し & onTick ラッパー
--#################################################################
local isInitialized = false
function initialize()
    debug.log("L:init01")
    tracks = {}; observationBuffer = {}; lastDetectionTick = {}; tickCounter = 0; nextTrackID = 1
    -- TODO: Define Initial_P_Matrix (初期共分散行列) in section 6
    debug.log("L:init02")
end

-- Stormworksはこの onTick() を毎フレーム呼び出す
function onTick()
    if not isInitialized then
        initialize()
        isInitialized = true
    end
    onTick_MainLogic() -- メインロジックを呼び出す
end
