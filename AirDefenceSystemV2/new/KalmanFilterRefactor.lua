--[[
================================================================================
kalmanfilter.lua (リファクタリング最終版 - 遅延フラグ対応)
================================================================================
機能:
- RadarList1 (ch 1-12) と RadarList2 (ch 13-24) から圧縮済み目標情報 (pack1, pack2) を入力。
- RadarList1 (ch 31) と RadarList2 (ch 32) からの遅延フラグ (0 or 1) を入力。
- Physics Sensor (ch 25-30) から自機情報を入力。
- unpackTargetData でデータを展開 (距離, ローカル角度, レーダーID)。
- localToGlobalCoords でグローバル座標 (Physics Sensor 座標系) に変換。
- EKF (拡張カルマンフィルター) で目標の状態 (位置 X,Y,Z と速度 Vx,Vy,Vz) を推定・更新。
  - dt (観測時間差) は、入力された遅延フラグを考慮して正確に計算。
- データアソシエーション (最近傍法) を実行。
- 新規目標の登録、タイムアウトや離反した目標の削除。
- 追跡中の目標の予測位置をグローバル座標 (Physics Sensor 座標系) で出力。

入力 (コンポジット信号):
- 数値 1-12: RadarList1 からの出力 (pack1, pack2 x 6目標)
- 数値 13-24: RadarList2 からの出力 (pack1, pack2 x 6目標)
- 数値 25: 自機 X座標 (World Position X, Physics Sensor ch 1)
- 数値 26: 自機 Y座標 (World Position Y, Physics Sensor ch 2)
- 数値 27: 自機 Z座標 (World Position Z, Physics Sensor ch 3)
- 数値 28: 自機 オイラー角 X (Pitch, ラジアン, Physics Sensor ch 4)
- 数値 29: 自機 オイラー角 Y (Yaw, ラジアン, Physics Sensor ch 5)
- 数値 30: 自機 オイラー角 Z (Roll, ラジアン, Physics Sensor ch 6)
- 数値 31: RadarList1 からの遅延フラグ (0: 通常, 1: 1Tick前のデータ)
- 数値 32: RadarList2 からの遅延フラグ (0: 通常, 1: 1Tick前のデータ)

出力 (コンポジット信号):
- 数値 1-32: 追跡中の目標の予測グローバル座標 (X, Y, Z のセット x 最大10目標)
  - ch (i*3 - 2): 目標 i の予測 X 座標 (East)
  - ch (i*3 - 1): 目標 i の予測 Y 座標 (Up)
  - ch (i*3)    : 目標 i の予測 Z 座標 (North)

前提:
- RadarList1 は レーダーID 0 (Front) と 2 (Back) のデータを扱う。
- RadarList2 は レーダーID 1 (Right) と 3 (Left) のデータを扱う。
- 座標系は Physics Sensor 座標系 (X:東, Y:上, Z:北, 左手系) を基準とする。
================================================================================
]]

-- Lua標準ライブラリ
local string = string
local table = table
local tonumber = tonumber
local ipairs = ipairs
local pairs = pairs -- データアソシエーションで使用

-- Stormworks API ショートカット
local inputNumber = input.getNumber
local outputNumber = output.setNumber

-- 定数
local PI = math.pi
local PI2 = PI * 2
local MAX_INPUT_TARGETS_RL1 = 6                                 -- RadarList1からの最大目標数
local MAX_INPUT_TARGETS_RL2 = 6                                 -- RadarList2からの最大目標数
local MAX_OUTPUT_CHANNELS = 32                                  -- 出力チャンネル数
local MAX_TRACKED_TARGETS = math.floor(MAX_OUTPUT_CHANNELS / 3) -- 同時に追跡・出力できる最大目標数 (約10)

-- EKF パラメータ
local NUM_STATES = 6                                                        -- 状態変数の数 (x, vx, y, vy, z, vz)
local DATA_ASSOCIATION_THRESHOLD = property.getNumber("D_ASSOC") or 100     -- データアソシエーションの閾値 (epsilon)
local TARGET_TIMEOUT_TICKS = property.getNumber("TIMEOUT") or 70            -- 目標が更新されない場合のタイムアウトtick数 (約1.17秒)
local TARGET_IS_LEAVING_THRESHOLD = property.getNumber("TGT_LEAVING") or -1 -- 目標が離反していると判断する閾値 (接近速度 < -1 m/s ?)
local INITIAL_VELOCITY_VARIANCE = (300 ^ 2)                                 -- 新規目標の初期速度分散 (大きい値に設定)

-- 観測ノイズ共分散行列 R (テンプレート) - レーダーの精度に基づく
-- オリジナルコードの R0 [source: 40] を参考に設定。
local R0_DIST_VAR_FACTOR = (0.02 ^ 2) / 24   -- 距離に対する分散係数 (距離^2に掛ける)
local R0_ANGLE_VAR = ((2e-3 * PI2) ^ 2) / 24 -- 角度の分散 (固定値)
local OBSERVATION_NOISE_MATRIX_TEMPLATE = {
    { R0_DIST_VAR_FACTOR, 0, 0 },            -- 距離誤差分散 (距離に応じてスケール)
    { 0, R0_ANGLE_VAR, 0 },                  -- 仰角誤差分散
    { 0, 0, R0_ANGLE_VAR }                   -- 方位角誤差分散
}

-- プロセスノイズ Q の適応的調整パラメータ (オリジナルコード [source: 41] より)
local PROCESS_NOISE_BASE = 1e-3
local PROCESS_NOISE_ADAPTIVE_SCALE = 1e+4
local PROCESS_NOISE_EPSILON_THRESHOLD = 140
local PROCESS_NOISE_EPSILON_SLOPE = 100
-- 予測誤差の不確かさ増加係数 (オリジナルコード [source: 41] より)
local PREDICTION_UNCERTAINTY_FACTOR_BASE = 1.01

-- 単位行列 I (6x6)
local identityMatrix6x6 = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }

-- グローバル変数 (状態保持)
local targetList = {}                                                           -- 追跡中の目標リスト { id, lastTick, X=stateVector, P=covarianceMatrix, epsilon=lastEpsilon }
local physicsSensorData = { x = 0, y = 0, z = 0, pitch = 0, yaw = 0, roll = 0 } -- 自機情報
local nextTargetId = 1                                                          -- 新規目標に割り当てるユニークID
local currentTick = 0                                                           -- このスクリプト内でのTickカウンター

--------------------------------------------------------------------------------
-- 行列演算ヘルパー関数 (ゼロ行列、コピー、スカラー倍、加算、減算、乗算、転置、逆行列)
--------------------------------------------------------------------------------
-- zeros(rows, cols)
function zeros(rows, cols)
    local m = {}; for r = 1, rows do
        m[r] = {}; for c = 1, cols do m[r][c] = 0 end
    end
    return m
end

-- MatrixCopy(M)
function MatrixCopy(M)
    local N = {}; for r, row in ipairs(M) do N[r] = { table.unpack(row) } end
    return N
end

-- scalar(s, M)
function scalar(s, M)
    local R = zeros(#M, #M[1]); for r = 1, #M do for c = 1, #M[1] do R[r][c] = M[r][c] * s end end
    return R
end

-- sum(A, B)
function sum(A, B)
    local R = zeros(#A, #A[1]); for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] + B[r][c] end end
    return R
end

-- sub(A, B)
function sub(A, B)
    local R = zeros(#A, #A[1]); for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] - B[r][c] end end
    return R
end

-- mul(...) - 可変長引数対応
function mul(...)
    local mats = { ... }; local A = mats[1]; local R; for i = 2, #mats do
        local B = mats[i]; if #A[1] ~= #B then
            debug.log("Error: Matrix mul dim mismatch")
            return nil
        end
        R = zeros(#A, #B[1]); for r = 1, #A do
            for c = 1, #B[1] do
                local sVal = 0; for k = 1, #B do sVal = sVal + A[r][k] * B[k][c] end
                R[r][c] = sVal
            end
        end
        A = R
    end
    return A
end

-- T(M) - 転置
function T(M)
    local rows = #M; local cols = #M[1]; local R = zeros(cols, rows); for r = 1, rows do
        for c = 1, cols do
            R[c][r] = M
                [r][c]
        end
    end
    return R
end

function inv(M)
    if M == nil or #M == 0 or #M[1] == 0 then return nil end; local n = #M; if n ~= #M[1] then return nil end -- 基本チェック
    local aug = {}; for r = 1, n do
        aug[r] = {}; if M[r] == nil then return nil end; for c = 1, n do
            local v = M[r][c]; if v == nil or v ~= v or v == math.huge or v == -math.huge then return nil end; aug[r][c] =
                v
        end
        for c = 1, n do aug[r][n + c] = (r == c) and 1 or 0 end
    end                                                                                   -- 入力チェック
    for r = 1, n do
        local piv = aug[r][r]; if piv == nil or math.abs(piv) < 1e-12 then return nil end -- ピボットチェック
        for c = r, 2 * n do
            if aug[r][c] == nil then return nil end; aug[r][c] = aug[r][c] / piv
        end -- 除算前 nil チェック
        for i = 1, n do
            if i ~= r then
                local f = aug[i][r]; if f == nil then return nil end; for c = r, 2 * n do
                    if aug[i][c] == nil or aug[r][c] == nil then return nil end; aug[i][c] = aug[i][c] - f * aug[r][c]
                end
            end
        end
    end
    local invM = zeros(n, n); for r = 1, n do
        for c = 1, n do
            local v = aug[r][n + c]; if v == nil or v ~= v or v == math.huge or v == -math.huge then
                invM[r][c] = 0
            else
                invM[r][c] =
                    v
            end
        end
    end -- 結果チェック
    return invM
end

--------------------------------------------------------------------------------
-- データ展開関数: unpackTargetData
--------------------------------------------------------------------------------
-- pack1, pack2 を展開し、距離, ローカル方位角(rad), ローカル仰角(rad), レーダーID(0-3) を返す。
-- PackandFilter の packTargetData と対になる処理。
--------------------------------------------------------------------------------
function unpackTargetData(pack1, pack2)
    if pack1 == 0 and pack2 == 0 then
        return 0, 0, 0, -1
    end

    local distance, azimuthRad, elevationRad, radarId
    local signList = { [1] = -1, [2] = 1 } -- Index 1 -> 符号 -1, Index 2 -> 符号 +1

    -- 1. レーダーIDのデコード (変更なし)
    local i = (pack1 > 0) and 2 or 1
    local j = (pack2 > 0) and 2 or 1
    local radarIdMap = { { 1, 2 }, { 3, 4 } }
    local s = radarIdMap[i][j]
    radarId = s - 1

    -- 2. 絶対値を取得
    local absPack1 = math.abs(pack1)
    local absPack2 = math.abs(pack2)

    -- 3. ★修正: 絶対値を直接文字列に変換し、その後で文字列としてゼロ埋め
    local pack1Str = string.format("%.0f", absPack1) -- まず整数文字列に
    local pack2Str = string.format("%.0f", absPack2)

    -- 文字列として右寄せゼロパディングで7桁を保証する
    -- string.format("%07d", tonumber(pack1Str)) よりも安全
    pack1Str = string.format("%7s", pack1Str):gsub(" ", "0")
    pack2Str = string.format("%7s", pack2Str):gsub(" ", "0")

    -- 4. 各パーツを抽出 (ここからは変更なし)
    local aziSignCodeRaw = tonumber(string.sub(pack1Str, 1, 1)) -- 1桁目: 符号コード
    local e_str = string.sub(pack1Str, 2, 5)                    -- 2-5桁目: 方位角小数部4桁
    local distPart1 = string.sub(pack1Str, 6, 7)                -- 6-7桁目: 距離前半2桁

    local eleSignCodeRaw = tonumber(string.sub(pack2Str, 1, 1)) -- 1桁目: 符号コード
    local f_str = string.sub(pack2Str, 2, 5)                    -- 2-5桁目: 仰角小数部4桁
    local distPart2 = string.sub(pack2Str, 6, 7)                -- 6-7桁目: 距離後半/中央2桁
    -- 5. 符号インデックスの決定 (不正値対応)
    local aziSignIndex = (aziSignCodeRaw == 1 or aziSignCodeRaw == 2) and aziSignCodeRaw or 2
    local eleSignIndex = (eleSignCodeRaw == 1 or eleSignCodeRaw == 2) and eleSignCodeRaw or 2

    -- 6. 距離を復元
    distance = tonumber(distPart1 .. distPart2)
    if distance == nil then distance = 0 end

    -- 7. 角度を復元 (ラジアン単位)
    local aziFraction = tonumber("0." .. e_str)
    local eleFraction = tonumber("0." .. f_str)
    if aziFraction == nil then aziFraction = 0 end
    if eleFraction == nil then eleFraction = 0 end

    local aziSignValue = signList[aziSignIndex]
    local eleSignValue = signList[eleSignIndex]

    azimuthRad = aziFraction * aziSignValue * PI2
    elevationRad = eleFraction * eleSignValue * PI2

    -- 8. 角度を [-PI, PI) の範囲に正規化
    azimuthRad = (azimuthRad + PI) % PI2 - PI
    elevationRad = (elevationRad + PI) % PI2 - PI
    return distance, azimuthRad, elevationRad, radarId
end

--------------------------------------------------------------------------------
-- 座標・ベクトル変換関数
--------------------------------------------------------------------------------

--[[
rotateVectorZYX: Z-Y-X オイラー角でベクトルを回転 (ローカル->グローバル)
Physics Sensor のオイラー角 (Z-Y-X Intrinsic, 左手系) に対応。
入力オイラー角は Physics Sensor 出力値をそのまま使う (元コードの反転は不要と判断)。
※もし動作がおかしい場合は、元コードのように符号反転を試す。
]]
function rotateVectorZYX(vector, pitch, yaw, roll)
    -- 回転行列 R = Rx(pitch) * Ry(yaw) * Rz(roll)
    -- Rx
    local RX = { { 1, 0, 0 }, { 0, math.cos(pitch), -math.sin(pitch) }, { 0, math.sin(pitch), math.cos(pitch) } }
    -- Ry
    local RY = { { math.cos(yaw), 0, math.sin(yaw) }, { 0, 1, 0 }, { -math.sin(yaw), 0, math.cos(yaw) } }
    -- Rz
    local RZ = { { math.cos(roll), -math.sin(roll), 0 }, { math.sin(roll), math.cos(roll), 0 }, { 0, 0, 1 } }
    local R = mul(RZ, RY, RX)
    return mul(R, vector)
end

--[[
localToGlobalCoords: レーダーのローカル極座標をグローバル直交座標に変換
出力は Physics Sensor グローバル座標系 (X:東, Y:上, Z:北)
]]
function localToGlobalCoords(dist, locAzi, locEle, rId, ownP)
    -- 1. レーダー基準ローカル直交座標
    local locX = dist * math.cos(locEle) * math.sin(locAzi); local locY = dist * math.sin(locEle); local locZ = dist *
        math.cos(locEle) * math.cos(locAzi);
    local radarLocVec = { { locX }, { locY }, { locZ } };
    -- 2. ヨー回転適用 -> 車両前方基準ローカル座標へ
    local rYOff = 0; if rId == 1 then rYOff = PI / 2 elseif rId == 2 then rYOff = PI elseif rId == 3 then rYOff = -PI / 2 end
    local vehLocVec_rotated = radarLocVec -- ID 0 は回転不要
    if rYOff ~= 0 then
        local cy = math.cos(rYOff); local sy = math.sin(rYOff); local RotY = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } };
        vehLocVec_rotated = mul(RotY, radarLocVec);
    end
    debug.log("vehLocVec_rotated X: " .. vehLocVec_rotated[1][1] .. " Y:" .. vehLocVec_rotated[2][1] .. " Z:" ..
        vehLocVec_rotated[3][1] .. " id:" .. rId)
    vehLocVec_rotated[2][1] = vehLocVec_rotated[2][1] + 2.5 / (rId + 1)
    -- 5. 車両姿勢で回転 -> グローバルな相対ベクトルへ
    local globalRelativeVector = rotateVectorZYX(vehLocVec_rotated, ownP.pitch, ownP.yaw, ownP.roll);

    -- 6. 物理センサーのグローバル位置を加算 -> 最終的な目標グローバル座標
    local gX = globalRelativeVector[1][1] + ownP.x; local gY = globalRelativeVector[2][1] + ownP.y; local gZ =
        globalRelativeVector[3][1] + ownP.z;
    debug.log("gX:" ..
        globalRelativeVector[1][1] .. " gY:" .. globalRelativeVector[2][1] .. " gZ:" .. globalRelativeVector[3][1])
    return gX, gY, gZ
end

--------------------------------------------------------------------------------
-- EKF (拡張カルマンフィルター) 関連関数
--------------------------------------------------------------------------------

--[[
getObservationJacobianAndPrediction: 観測予測値 h とヤコビ行列 H を計算
入力: stateVector (X, 6x1), ownPosition (自機座標 {x,y,z})
出力: H (3x6), h (3x1 {dist, ele, azi})
]]
function getObservationJacobianAndPrediction(stateVector, ownPosition)
    local targetX = stateVector[1][1]; local targetY = stateVector[3][1]; local targetZ = stateVector[5][1]
    local relativeX = targetX - ownPosition.x
    local relativeY = targetY - ownPosition.y -- Physics Sensor Y は Up
    local relativeZ = targetZ - ownPosition.z -- Physics Sensor Z は North

    local r_sq = relativeX ^ 2 + relativeY ^ 2 + relativeZ ^ 2
    local rh_sq = relativeX ^ 2 + relativeZ ^ 2 -- XZ平面 (East-North)
    if r_sq < 1e-9 then r_sq = 1e-9 end
    if rh_sq < 1e-9 then rh_sq = 1e-9 end
    local r = math.sqrt(r_sq)
    local rh = math.sqrt(rh_sq)

    -- 1. 観測予測値 h = [距離, 仰角(Y基準), 方位角(Z基準)]
    local predictedDistance = r
    local predictedElevation = math.asin(relativeY / r)
    local predictedAzimuth = math.atan(relativeX, relativeZ)
    local h = { { predictedDistance }, { predictedElevation }, { predictedAzimuth } }

    -- 2. 観測ヤコビ行列 H = dh/dX (3x6)
    local H = zeros(3, NUM_STATES)
    H[1][1] = relativeX / r; H[1][3] = relativeY / r; H[1][5] = relativeZ / r
    H[2][1] = (-relativeX * relativeY) / (rh_sq * r); H[2][3] = rh / r_sq; H[2][5] = (-relativeZ * relativeY) /
        (rh_sq * r)
    H[3][1] = relativeZ / rh_sq; H[3][3] = 0; H[3][5] = -relativeX / rh_sq
    -- 速度項の偏微分はゼロ (H[row][2,4,6] = 0)

    return H, h
end

--[[
extendedKalmanFilterUpdate: EKF の予測・更新ステップを実行
]]
function extendedKalmanFilterUpdate(stateVector, covariance, observation, ownPosition, dt, lastEpsilon)
    -- 1. 予測ステップ (Predict)
    local F = MatrixCopy(identityMatrix6x6)
    F[1][2] = dt; F[3][4] = dt; F[5][6] = dt
    local X_predicted = mul(F, stateVector)

    local dt2 = dt * dt; local dt3 = dt2 * dt / 2; local dt4 = dt3 * dt / 2;
    local Q_base = { { dt4, dt3, 0, 0, 0, 0 }, { dt3, dt2, 0, 0, 0, 0 }, { 0, 0, dt4, dt3, 0, 0 },
        { 0,   0,   dt3, dt2, 0, 0 }, { 0, 0, 0, 0, dt4, dt3 }, { 0, 0, 0, 0, dt3, dt2 } }
    local adaptiveFactor = PROCESS_NOISE_BASE +
        PROCESS_NOISE_ADAPTIVE_SCALE /
        (1 + math.exp(-(lastEpsilon - PROCESS_NOISE_EPSILON_THRESHOLD) * PROCESS_NOISE_EPSILON_SLOPE))
    local Q_adapted = scalar(adaptiveFactor, Q_base)

    local uncertaintyIncreaseFactor = PREDICTION_UNCERTAINTY_FACTOR_BASE ^ (2 * (dt * 60)) -- tick数換算で計算
    local P_predicted = sum(scalar(uncertaintyIncreaseFactor, mul(F, covariance, T(F))), Q_adapted)

    -- 2. 更新ステップ (Update)
    local Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }
    local H, h = getObservationJacobianAndPrediction(X_predicted, ownPosition)
    local R = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R[1][1] = R[1][1] * (observation.distance ^ 2) -- 距離の2乗に比例

    local Y = sub(Z, h)
    Y[3][1] = (Y[3][1] + PI) % PI2 - PI -- 方位角正規化
    Y[2][1] = (Y[2][1] + PI) % PI2 - PI -- 仰角正規化

    local S = sum(mul(H, P_predicted, T(H)), R)
    local S_inv = inv(S)
    if S_inv == nil then return stateVector, covariance, lastEpsilon end -- 逆行列計算失敗
    local K = mul(P_predicted, T(H), S_inv)

    local X_updated = sum(X_predicted, mul(K, Y))
    local I_minus_KH = sub(identityMatrix6x6, mul(K, H))
    local P_updated = sum(mul(I_minus_KH, P_predicted, T(I_minus_KH)), mul(K, R, T(K)))
    local epsilon = mul(T(Y), S_inv, Y)[1][1]

    return X_updated, P_updated, epsilon
end

--------------------------------------------------------------------------------
-- メインループ: onTick
--------------------------------------------------------------------------------
function onTick()
    currentTick = currentTick + 1 -- 内部Tickカウンター更新

    -- 1. 自機情報と遅延フラグを取得
    physicsSensorData.x = inputNumber(25)
    physicsSensorData.y = inputNumber(26)
    physicsSensorData.z = inputNumber(27)
    physicsSensorData.pitch = inputNumber(28)
    physicsSensorData.yaw = inputNumber(29)
    physicsSensorData.roll = inputNumber(30)
    local isDelayed1 = (inputNumber(31) == 1) -- RadarList1 (ID 0, 2) 遅延フラグ
    local isDelayed2 = (inputNumber(32) == 1) -- RadarList2 (ID 1, 3) 遅延フラグ

    local currentObservations = {}            -- このtickで有効な観測データリスト

    -- 2. 入力データ読み込み、展開、変換、Tick情報付与
    if (inputNumber(1) ~= 0 or inputNumber(13) ~= 0) then
        for i = 1, MAX_INPUT_TARGETS_RL1 + MAX_INPUT_TARGETS_RL2 do
            local isDelayed
            if i > MAX_INPUT_TARGETS_RL1 then
                isDelayed = isDelayed2
            else
                isDelayed = isDelayed1
            end
            local pack1 = inputNumber(i * 2 - 1)
            local pack2 = inputNumber(i * 2)
            --debug.log("pack1:" .. pack1 .. " pack2:" .. pack2)
            if pack1 ~= 0 or pack2 ~= 0 then
                local dist, localAziRad, localEleRad, rId = unpackTargetData(pack1, pack2)
                if rId ~= -1 and dist > 0 then
                    local gX, gY, gZ = localToGlobalCoords(dist, localAziRad, localEleRad, rId, physicsSensorData)
                    -- 観測Tickを決定 (遅延フラグを考慮)
                    local observationTick = isDelayed and (currentTick - 1) or currentTick
                    table.insert(currentObservations, {
                        distance = dist,
                        azimuth = localAziRad,
                        elevation = localEleRad,
                        radarId = rId,
                        globalX = gX,
                        globalY = gY,
                        globalZ = gZ,
                        obsTick = observationTick
                    })
                end
            end
        end
    end
    -- 3. データアソシエーションとEKF更新
    local assignedObservationIndices = {}
    local updatedTargetIds = {}

    if #currentObservations > 0 then
        for targetId, currentTarget in pairs(targetList) do
            local bestMatchObsIndex = -1
            local minEpsilon = DATA_ASSOCIATION_THRESHOLD + 1
            local matchedState, matchedCovariance, matchedEpsilon, matchedObsTick

            for j = 1, #currentObservations do
                if not assignedObservationIndices[j] then
                    local observation = currentObservations[j]
                    -- dt (時間差) を計算 (観測Tick - 前回の更新Tick)
                    local dt_ticks = observation.obsTick - currentTarget.lastTick
                    if dt_ticks > 0 then
                        local dt_sec = dt_ticks / 60.0
                        -- EKF更新試算
                        local X_post, P_post, epsilon = extendedKalmanFilterUpdate(
                            currentTarget.X, currentTarget.P, observation, physicsSensorData, dt_sec,
                            currentTarget.epsilon
                        )
                        -- 最良マッチを更新
                        if epsilon < minEpsilon then
                            minEpsilon = epsilon
                            bestMatchObsIndex = j
                            matchedState = X_post; matchedCovariance = P_post; matchedEpsilon = epsilon; matchedObsTick =
                                observation.obsTick;
                        end
                    end
                end
            end

            -- 最良マッチが閾値以下なら目標を更新
            if bestMatchObsIndex ~= -1 and minEpsilon <= DATA_ASSOCIATION_THRESHOLD then
                targetList[targetId].X = matchedState
                targetList[targetId].P = matchedCovariance
                targetList[targetId].epsilon = matchedEpsilon
                targetList[targetId].lastTick = matchedObsTick -- 観測Tickで更新
                assignedObservationIndices[bestMatchObsIndex] = true
                updatedTargetIds[targetId] = true
            end
        end
    end

    -- 4. 新規目標の登録
    for j = 1, #currentObservations do
        if not assignedObservationIndices[j] then -- 未割り当ての観測
            local newObs = currentObservations[j]
            local X_init = { { newObs.globalX }, { 0 }, { newObs.globalY }, { 0 }, { newObs.globalZ }, { 0 } }
            local P_init = zeros(NUM_STATES, NUM_STATES)
            local init_pos_var_factor = 10
            local pos_var_x = OBSERVATION_NOISE_MATRIX_TEMPLATE[3][3] * init_pos_var_factor
            local pos_var_y = OBSERVATION_NOISE_MATRIX_TEMPLATE[2][2] * init_pos_var_factor
            local pos_var_z = pos_var_x
            P_init[1][1] = pos_var_x * (newObs.distance ^ 2); P_init[3][3] = pos_var_y * (newObs.distance ^ 2); P_init[5][5] =
                pos_var_z * (newObs.distance ^ 2);
            P_init[2][2] = INITIAL_VELOCITY_VARIANCE; P_init[4][4] = INITIAL_VELOCITY_VARIANCE; P_init[6][6] =
                INITIAL_VELOCITY_VARIANCE;
            local newId = nextTargetId
            targetList[newId] = { id = newId, lastTick = newObs.obsTick, X = X_init, P = P_init, epsilon = 1 }
            nextTargetId = nextTargetId + 1
            updatedTargetIds[newId] = true
        end
    end

    -- 5. 古い目標、離反する目標の削除
    local targetIdsToDelete = {}
    for targetId, target in pairs(targetList) do
        local ticksSinceLastUpdate = currentTick - target.lastTick
        local isTimeout = ticksSinceLastUpdate >= TARGET_TIMEOUT_TICKS
        local relativePosX = target.X[1][1] - physicsSensorData.x; local relativePosY = target.X[3][1] -
            physicsSensorData.y; local relativePosZ = target.X[5][1] - physicsSensorData.z;
        local targetVx = target.X[2][1]; local targetVy = target.X[4][1]; local targetVz = target.X[6][1];
        local relativePosMagSq = relativePosX ^ 2 + relativePosY ^ 2 + relativePosZ ^ 2; local isLeaving = false;
        if relativePosMagSq > 1 then
            local relativePosMag = math.sqrt(relativePosMagSq);
            local closingSpeed = -(relativePosX * targetVx + relativePosY * targetVy + relativePosZ * targetVz) /
                relativePosMag;
            if closingSpeed < TARGET_IS_LEAVING_THRESHOLD then isLeaving = true end
        end
        if isTimeout or isLeaving then table.insert(targetIdsToDelete, targetId) end
    end
    for _, id in ipairs(targetIdsToDelete) do targetList[id] = nil end

    -- 6. 出力チャンネルクリアと目標座標の書き込み
    for i = 1, MAX_OUTPUT_CHANNELS do outputNumber(i, 0) end
    local outputCount = 0
    for targetId, target in pairs(targetList) do
        if outputCount < MAX_TRACKED_TARGETS then
            -- 現在Tickでの予測位置を出力
            local predX, predY, predZ
            local dt_pred_sec = (currentTick - target.lastTick) / 60.0
            if dt_pred_sec >= 0 then -- 念のためチェック
                predX = target.X[1][1] + target.X[2][1] * dt_pred_sec
                predY = target.X[3][1] + target.X[4][1] * dt_pred_sec
                predZ = target.X[5][1] + target.X[6][1] * dt_pred_sec
            else
                predX = target.X[1][1] -- dtが負なら前回位置
                predY = target.X[3][1]
                predZ = target.X[5][1]
            end
            outputNumber(outputCount * 3 + 1, predX) -- X (East)
            outputNumber(outputCount * 3 + 2, predY) -- Y (Up)
            outputNumber(outputCount * 3 + 3, predZ) -- Z (North)
            outputCount = outputCount + 1
        else
            break
        end
    end
end
