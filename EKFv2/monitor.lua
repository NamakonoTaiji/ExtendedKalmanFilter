--[[
  EKF 出力モニター表示スクリプト (5x3 モニター想定)
  入力値、中間計算結果、最終的なローカル座標をログ出力するデバッグ版。
  Missing 'mul' helper function added.
]]

-- ショートハンド
local iN = input.getNumber; local iB = input.getBool; local S = screen; local math = math
local sin, cos, pi, asin, atan, sqrt, tan = math.sin, math.cos, math.pi, math.asin, math.atan, math.sqrt, math.tan
local SF = string.format; local DT = S.drawText; local SC = S.setColor

-- 定数
local MAX_TARGETS = 6; local PI2 = pi * 2; local HALF_PI = pi / 2

--#################################################################
--# 0. ヘルパー関数群
--#################################################################
function vectorSubtract(vA, vB)
    if not vA or not vB or #vA ~= #vB then return nil end; local r = {}; for i = 1, #vA do r[i] = vA[i] - vB[i] end; return
        r
end

function matrixVectorMultiply(mat, vec)
    if not mat or #mat == 0 or not mat[1] or #mat[1] == 0 or not vec or #vec == 0 then return nil end; local nr = #mat; local nc = #
        mat[1]; local vd = #vec; if nc ~= vd then return nil end; local rV = {}; for i = 1, nr do
        rV[i] = 0; for j = 1, nc do
            if not mat[i] or mat[i][j] == nil then return nil end; if vec[j] == nil then return nil end; rV[i] = rV[i] +
                mat[i][j] * vec[j]
        end
    end; return rV
end

function matrixMultiply(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then return nil end; local rA = #
        matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]; if cA ~= rB then return nil end; local rM = {}; for i = 1, rA do
        rM[i] = {}; for k = 1, cB do
            local s = 0; for j = 1, cA do
                if not matA[i] or matA[i][j] == nil or not matB[j] or matB[j][k] == nil then return nil end; s = s +
                    matA[i][j] * matB[j][k]
            end; rM[i][k] = s
        end
    end; return rM
end

function transpose(M)
    if not M or not M[1] then return nil end; local r = {}; for i = 1, #M[1] do
        r[i] = {}; for j = 1, #M do
            if M[j] == nil then return nil end; r[i][j] = M[j][i]
        end
    end; return r
end

function matrixToString(M, fmt)
    if not M then return "nil" end; fmt = fmt or "%.3f"; local rows = {}; for i = 1, #M do
        local row = {}; if M[i] then for j = 1, #M[i] do table.insert(row, SF(fmt, M[i][j] or 0)) end end; table.insert(
            rows, "{" .. table.concat(row, ", ") .. "}")
    end; return "{" .. table.concat(rows, ", ") .. "}"
end

-- *** 追加: 行列乗算関数 'mul' (MC3からコピー) ***
function mul(A, B)
    if not A or not B or not A[1] or not B[1] or #A[1] ~= #B then
        debug.log("E:mu02"); return nil
    end
    local r = {}; for i = 1, #A do r[i] = {} end
    for i = 1, #A do
        for j = 1, #B[1] do
            local s = 0
            for k = 1, #B do
                if A[i] == nil or A[i][k] == nil or B[k] == nil or B[k][j] == nil then
                    debug.log("E:mu01"); return nil
                end
                s = s + A[i][k] * B[k][j]
            end
            r[i][j] = s
        end
    end
    return r
end

--- Body(Local) -> World への回転行列を計算 (ユーザー提供コード1ベース)
function calculateBodyToWorldMatrix(pitch, yaw, roll)
    if not pitch or not yaw or not roll then return nil end
    local cY = math.cos(yaw); local sY = math.sin(yaw); local cP = math.cos(pitch); local sP = math.sin(pitch); local cR =
        math.cos(roll); local sR = math.sin(roll)
    -- 行列の各行は、ワールド座標の基底ベクトル(East, Up, North)がローカル座標でどう表されるかを示す
    local R_B2W = {
        { cY * cR, sP * sY * cR - cP * sR, cP * sY * cR + sP * sR }, -- Row 1 -> Outputs East
        { cY * sR, sP * sY * sR + cP * cR, cP * sY * sR - sP * cR }, -- Row 2 -> Outputs Altitude/Up
        { -sY,     sP * cY,                cP * cY }                 -- Row 3 -> Outputs North
    }
    return R_B2W
end

--#################################################################
--# グローバル変数
--#################################################################
local plotData = {}
local cameraZoomLevel = 1
-- ローパスフィルター用変数は削除
-- local filtered_pitch_mon = 0
-- local filtered_roll_mon = 0

--#################################################################
--# onTick - 入力読み取り & 描画用データ計算
--#################################################################
function onTick()
    -- 自機状態読み取り
    local self_x = iN(20); local self_y = iN(21); local self_z = iN(22)       -- East, Up, North
    local self_pos = { self_x, self_y, self_z }
    local yaw_val = iN(28); local pitch_val = iN(29); local roll_val = iN(30) -- Raw values

    -- 入力値 nil チェック
    if yaw_val == nil or pitch_val == nil or roll_val == nil then
        debug.log("ERROR: Monitor - Orientation input is nil!")
        plotData = {}
        return
    end
    -- *** フィルターは適用しない ***
    local self_orient = { yaw = yaw_val, pitch = pitch_val, roll = roll_val }

    -- カメラズームレベル
    cameraZoomLevel = iN(32) or 1

    -- 回転行列 Body -> World を計算
    local R_B2W = calculateBodyToWorldMatrix(self_orient.pitch, self_orient.yaw, self_orient.roll)
    local R_W2B = nil
    if R_B2W then R_W2B = transpose(R_B2W) end -- World -> Body は転置

    -- EKF出力読み取り & 描画用データ計算
    plotData = {}
    if R_W2B then
        for i = 1, MAX_TARGETS do
            local isActive = iB(i); local local_azim_rad = 0; local local_elev_rad = 0
            local target_x, target_y, target_z = 0, 0, 0

            if isActive then
                local base_ch = (i - 1) * 6 + 1
                target_x = iN(base_ch + 0); target_y = iN(base_ch + 2); target_z = iN(base_ch + 1) -- EKF Output (East, Up, North)
                local target_pos = { target_x, target_y, target_z }

                -- 1. 相対グローバルベクトル (East, Up, North)
                local GV_relative_vec = vectorSubtract(target_pos, self_pos)

                if GV_relative_vec then
                    -- Vector to Column Matrix for multiplication
                    local GV_relative = { { GV_relative_vec[1] }, { GV_relative_vec[2] }, { GV_relative_vec[3] } }

                    -- 2. グローバル相対ベクトル -> ローカルベクトル (Right, Up, Forward)
                    local LV_mat = matrixMultiply(R_W2B, GV_relative) -- (3x3) * (3x1) = 3x1 Matrix

                    if LV_mat and LV_mat[1] and LV_mat[2] and LV_mat[3] then
                        local localX_R = LV_mat[1][1]; local localY_U = LV_mat[2][1]; local localZ_F = LV_mat[3][1]
                        -- *** デバッグログ: 計算されたローカルベクトル ***

                        -- 3. ローカルベクトル -> ローカル角度
                        local dist_sq = localX_R ^ 2 + localY_U ^ 2 + localZ_F ^ 2
                        if dist_sq > 1e-6 then
                            local dist = sqrt(dist_sq)
                            local_elev_rad = asin(localY_U / dist)    -- 仰角 (from horizontal)
                            local_azim_rad = atan(localX_R, localZ_F) -- 方位角 (from forward)
                        end
                    else
                        debug.log("Error: Monitor - Local vector calculation failed for target " .. i)
                    end
                else
                    debug.log("Error: Monitor - Relative global vector calculation failed for target " .. i)
                end
            end
            table.insert(plotData, { id = i, active = isActive, azim = local_azim_rad, elev = local_elev_rad })
        end
    else
        debug.log("Monitor - R_W2B calculation FAILED")
        for i = 1, MAX_TARGETS do table.insert(plotData, { id = i, active = false, azim = 0, elev = 0 }) end
    end
end

--#################################################################
--# onDraw - モニター描画
--#################################################################
function onDraw()
    -- (描画ロジックは変更なし)
    local w = S.getWidth()
    local h = S.getHeight()
    local cw = w / 2
    local ch = h / 2
    local fov = (cameraZoomLevel + 1) / 2
    local fovRad = -2.175 * fov + 2.2
    if fovRad <= 0 then fovRad = pi / 2 end
    local rw = (cw) / math.tan(fovRad / 2)
    local rh = (ch) / math.tan(fovRad / 2)
    SC(255, 255, 255, 100)
    S.drawLine(cw, 0, cw, h)
    S.drawLine(0, ch, w, ch)
    SC(0, 255, 0, 200)
    for i = 1, #plotData do
        local target = plotData[i]
        if target.active then
            local tan_azim = math.tan(target.azim)
            local tan_elev = math.tan(target.elev)
            local echoX = cw + rw * tan_azim * (3 / 5)
            local echoY = ch - rh * tan_elev
            SC(0, 255, 0, 200)
            S.drawCircle(math.floor(echoX + 0.5), math.floor(echoY + 0.5), 3)
            DT(math.floor(echoX + 0.5), math.floor(echoY - 8 + 0.5), target.id)
        end
    end
end
