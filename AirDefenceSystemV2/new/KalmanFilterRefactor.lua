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
- オンオフ 1-10: 追跡中の目標の更新フラグ(目標更新時の1tickだけオンを出力する)
  - ch 1: 目標 1 の更新フラグ
  - ch 2: 目標 2 の更新フラグ
  - ...
  - ch 10: 目標 10 の更新フラグ

前提:
- RadarList1 は レーダーID 0 (Front) と 2 (Back) のデータを扱う。
- RadarList2 は レーダーID 1 (Right) と 3 (Left) のデータを扱う。
- 座標系は Physics Sensor 座標系 (X:東, Y:上, Z:北, 左手系) を基準とする。
================================================================================
]]

-- Stormworks API ショートカット
inputNumber = input.getNumber
outputNumber = output.setNumber
outputBool = output.setBool
-- 定数 (文字数削減のため定数はローカル宣言を削除)
PI = math.pi
PI2 = PI * 2
MAX_INPUT_TARGETS_RL1 = 6                                     -- RadarList1からの最大目標数
MAX_INPUT_TARGETS_RL2 = 6                                     -- RadarList2からの最大目標数
MAX_TRACKED_TARGETS = 10                                      -- 同時に追跡・出力できる最大目標数
LOGIC_DELAY = 8 + property.getNumber("n")                     -- 暫定ロジック遅延 1ステップ前のデータが送られてくるので探知間隔も足す必要があるかもしれない。
-- EKF パラメータ
NUM_STATES = 6                                                -- 状態変数の数 (x, vx, y, vy, z, vz)
DATA_ASSOCIATION_THRESHOLD = property.getNumber("D_ASOC")     -- データアソシエーションの閾値 (epsilon)
TARGET_TIMEOUT_TICKS = property.getNumber("T_OUT")            -- 目標が更新されない場合のタイムアウトtick数 (約1.17秒)
TARGET_IS_LEAVING_THRESHOLD = property.getNumber("TGT_LVING") -- 目標が離反していると判断する閾値 (接近速度 < -1 m/s ?)
INITIAL_VELOCITY_VARIANCE = (300 ^ 2)                         -- 新規目標の初期速度分散 (大きい値に設定)

-- 観測ノイズ共分散行列 R (テンプレート) - レーダーの精度に基づく
-- オリジナルコードの R0 [source: 40] を参考に設定。
R0_DIST_VAR_FACTOR = (0.02 ^ 2) / 24   -- 距離に対する分散係数 (距離^2に掛ける)
R0_ANGLE_VAR = ((2e-3 * PI2) ^ 2) / 24 -- 角度の分散 (固定値)
OBSERVATION_NOISE_MATRIX_TEMPLATE = {
    { R0_DIST_VAR_FACTOR, 0, 0 },      -- 距離誤差分散 (距離に応じてスケール)
    { 0, R0_ANGLE_VAR, 0 },            -- 仰角誤差分散
    { 0, 0, R0_ANGLE_VAR }             -- 方位角誤差分散
}

-- プロセスノイズ Q の適応的調整パラメータ (オリジナルコード [source: 41] より)
PROCESS_NOISE_BASE = property.getNumber("P_BASE")
PROCESS_NOISE_ADAPTIVE_SCALE = property.getNumber("P_ADPT")
PROCESS_NOISE_EPSILON_THRESHOLD = property.getNumber("P_NOISE_EPS_THRS")
PROCESS_NOISE_EPSILON_SLOPE = property.getNumber("P_NOISE_EPS_SLOPE")
-- 予測誤差の不確かさ増加係数 (オリジナルコード [source: 41] より)
PREDICTION_UNCERTAINTY_FACTOR_BASE = property.getNumber("PRED_UNCERTAINTY_FACT")

-- 敵対判定パラメータ
HOSTILE_IDENTIFICATION_THRESHOLD = property.getNumber("IDENTI_THRS")       -- 同定成功回数の閾値
HOSTILE_CLOSING_SPEED_THRESHOLD = property.getNumber("TGT_CLOSING_SPD")    -- 接近速度の閾値 (m/s) - 必要に応じて調整してください
HOSTILE_RECENT_UPDATES_THRESHOLD = property.getNumber("TGT_RECENT_UPDATE") -- 閾値超えを要求する直近の更新回数



-- 単位行列 I (6x6)
identityMatrix6x6 = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }

-- グローバル変数 (状態保持)
targetList = {}                                                           -- 追跡中の目標リスト { id, lastTick, X=stateVector, P=covarianceMatrix, epsilon=lastEpsilon }
physicsSensorData = { x = 0, y = 0, z = 0, pitch = 0, yaw = 0, roll = 0 } -- 自機情報
currentTick = 0                                                           -- このスクリプト内でのTickカウンター
nextInternalId = 1                                                        -- これは増え続ける内部ID
assignedOutputIds = {}                                                    -- 使用中のOutput IDを管理するセット (例: assignedOutputIds[3] = true なら ID 3 は使用中)

--------------------------------------------------------------------------------
-- 行列演算ヘルパー関数 (ゼロ行列、コピー、スカラー倍、加算、減算、乗算、転置、逆行列)
--------------------------------------------------------------------------------
-- zeros(rows, cols)
function zeros(rows, cols)
    local m = {}
    for r = 1, rows do
        m[r] = {}
        for c = 1, cols do m[r][c] = 0 end
    end
    return m
end

-- MatrixCopy(M)
function MatrixCopy(M)
    local N = {}
    for r, row in ipairs(M) do N[r] = { table.unpack(row) } end
    return N
end

-- scalar(s, M)
function scalar(s, M)
    local R = zeros(#M, #M[1])
    for r = 1, #M do for c = 1, #M[1] do R[r][c] = M[r][c] * s end end
    return R
end

-- sum(A, B)
function sum(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] + B[r][c] end end
    return R
end

-- sub(A, B)
function sub(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do for c = 1, #A[1] do R[r][c] = A[r][c] - B[r][c] end end
    return R
end

-- mul(...) - 可変長引数対応
function mul(...)
    local mats, A, R, B, sVal
    mats = { ... }
    A = mats[1]
    for i = 2, #mats do
        B = mats[i]
        if #A[1] ~= #B then
            -- debug.log("Error: Matrix mul dim mismatch")
            return nil
        end
        R = zeros(#A, #B[1])
        for r = 1, #A do
            for c = 1, #B[1] do
                sVal = 0
                for k = 1, #B do sVal = sVal + A[r][k] * B[k][c] end
                R[r][c] = sVal
            end
        end
        A = R
    end
    return A
end

-- T(M) - 転置
function T(M)
    local rows, cols, R
    rows = #M
    cols = #M[1]
    R = zeros(cols, rows)
    for r = 1, rows do
        for c = 1, cols do
            R[c][r] = M[r][c]
        end
    end
    return R
end

function inv(M)
    if M == nil or #M == 0 or #M[1] == 0 then return nil end
    local n = #M
    if n ~= #M[1] then return nil end -- 基本チェック
    local aug = {}
    for r = 1, n do
        aug[r] = {}
        if M[r] == nil then return nil end
        for c = 1, n do
            local v = M[r][c]
            if v == nil or v ~= v or v == math.huge or v == -math.huge then return nil end
            aug[r][c] = v
        end
        for c = 1, n do aug[r][n + c] = (r == c) and 1 or 0 end
    end -- 入力チェック
    for r = 1, n do
        local piv = aug[r][r]
        if piv == nil or math.abs(piv) < 1e-12 then return nil end -- ピボットチェック
        for c = r, 2 * n do
            if aug[r][c] == nil then return nil end
            aug[r][c] = aug[r][c] / piv
        end -- 除算前 nil チェック
        for i = 1, n do
            if i ~= r then
                local f = aug[i][r]
                if f == nil then return nil end
                for c = r, 2 * n do
                    if aug[i][c] == nil or aug[r][c] == nil then return nil end
                    aug[i][c] = aug[i][c] - f * aug[r][c]
                end
            end
        end
    end
    local invM = zeros(n, n)
    for r = 1, n do
        for c = 1, n do
            local v = aug[r][n + c]
            if v == nil or v ~= v or v == math.huge or v == -math.huge then
                invM[r][c] = 0
            else
                invM[r][c] = v
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
    local distance, azimuthRad, elevationRad, radarId, signList, i, j, radarIdMap, s, absPack1, absPack2, pack1Str, pack2Str, aziSignCodeRaw, e_str, distPart1
    local eleSignCodeRaw, f_str, distPart2, aziSignIndex, eleSignIndex, aziFraction, eleFraction, aziSignValue, eleSignValue

    signList = { [1] = -1, [2] = 1 } -- Index 1 -> 符号 -1, Index 2 -> 符号 +1

    -- 1. レーダーIDのデコード (変更なし)
    i = (pack1 > 0) and 2 or 1
    j = (pack2 > 0) and 2 or 1
    radarIdMap = { { 1, 2 }, { 3, 4 } }
    s = radarIdMap[i][j]
    radarId = s - 1

    -- 2. 絶対値を取得
    absPack1 = math.abs(pack1)
    absPack2 = math.abs(pack2)

    -- 3. 絶対値を直接文字列に変換し、その後で文字列としてゼロ埋め
    pack1Str = string.format("%.0f", absPack1) -- まず整数文字列に
    pack2Str = string.format("%.0f", absPack2)

    -- 文字列として右寄せゼロパディングで7桁を保証する
    -- string.format("%07d", tonumber(pack1Str)) よりも安全
    pack1Str = string.format("%7s", pack1Str):gsub(" ", "0")
    pack2Str = string.format("%7s", pack2Str):gsub(" ", "0")

    -- 4. 各パーツを抽出 (ここからは変更なし)
    aziSignCodeRaw = tonumber(string.sub(pack1Str, 1, 1)) -- 1桁目: 符号コード
    e_str = string.sub(pack1Str, 2, 5)                    -- 2-5桁目: 方位角小数部4桁
    distPart1 = string.sub(pack1Str, 6, 7)                -- 6-7桁目: 距離前半2桁

    eleSignCodeRaw = tonumber(string.sub(pack2Str, 1, 1)) -- 1桁目: 符号コード
    f_str = string.sub(pack2Str, 2, 5)                    -- 2-5桁目: 仰角小数部4桁
    distPart2 = string.sub(pack2Str, 6, 7)                -- 6-7桁目: 距離後半/中央2桁
    -- 5. 符号インデックスの決定 (不正値対応)
    aziSignIndex = (aziSignCodeRaw == 1 or aziSignCodeRaw == 2) and aziSignCodeRaw
    eleSignIndex = (eleSignCodeRaw == 1 or eleSignCodeRaw == 2) and eleSignCodeRaw

    -- 6. 距離を復元
    distance = tonumber(distPart1 .. distPart2)
    if distance == nil then distance = 0 end

    -- 7. 角度を復元 (ラジアン単位)
    aziFraction = tonumber("0." .. e_str)
    eleFraction = tonumber("0." .. f_str)
    if aziFraction == nil then aziFraction = 0 end
    if eleFraction == nil then eleFraction = 0 end

    aziSignValue = signList[aziSignIndex]
    eleSignValue = signList[eleSignIndex]

    azimuthRad = aziFraction * aziSignValue * PI2
    elevationRad = eleFraction * eleSignValue * PI2

    -- 8. 角度を [-PI, PI) の範囲に正規化
    azimuthRad = (azimuthRad + PI) % PI2 - PI
    elevationRad = (elevationRad + PI) % PI2 - PI
    return distance, azimuthRad, elevationRad, radarId
end

--------------------------------------------------------------------------------
-- 接近速度計算関数
--------------------------------------------------------------------------------
-- 入力: target (目標テーブル), ownPos (自機位置テーブル {x,y,z})
-- 出力: closingSpeed (スカラー値, m/s)
function calculateClosingSpeed(target, ownPos)
    local relativePosX, relativePosY, relativePosZ, targetVx, targetVy, targetVz, relativePosMagSq, relativePosMag, closingSpeed
    if not target or not target.X then return 0 end -- 安全チェック
    relativePosX = target.X[1][1] - ownPos.x
    relativePosY = target.X[3][1] - ownPos.y
    relativePosZ = target.X[5][1] - ownPos.z
    targetVx = target.X[2][1]
    targetVy = target.X[4][1]
    targetVz = target.X[6][1]
    relativePosMagSq = relativePosX ^ 2 + relativePosY ^ 2 + relativePosZ ^ 2

    relativePosMag = math.sqrt(relativePosMagSq)
    -- 接近速度 = -(相対位置ベクトル・目標速度ベクトル) / 相対距離
    closingSpeed = -(relativePosX * targetVx + relativePosY * targetVy + relativePosZ * targetVz) / relativePosMag
    return closingSpeed
end

--------------------------------------------------------------------------------
-- 敵対判定関数
--------------------------------------------------------------------------------
-- 入力: target (目標テーブル)
-- 出力: なし (target.is_hostile を直接更新)
function checkHostileCondition(target)
    -- すでに敵対判定済みなら何もしない
    if target.is_hostile then return end

    -- 1. 同定回数チェック
    if target.identification_count >= HOSTILE_IDENTIFICATION_THRESHOLD then
        -- 2. 直近の接近速度チェック
        local highSpeedCount = 0
        if #target.recent_closing_speeds >= HOSTILE_RECENT_UPDATES_THRESHOLD then
            for _, speed in ipairs(target.recent_closing_speeds) do
                if speed > HOSTILE_CLOSING_SPEED_THRESHOLD then
                    highSpeedCount = highSpeedCount + 1
                end
            end
            -- 連続して閾値を超えたかチェック
            if highSpeedCount >= HOSTILE_RECENT_UPDATES_THRESHOLD then
                target.is_hostile = true
            end
        end
    end
end

-- ★ Output ID を割り当てる関数
function assignOutputId(target)
    if target.outputId ~= nil then return true end -- すでに割り当て済みなら何もしない
    for id = 1, MAX_TRACKED_TARGETS do
        if not assignedOutputIds[id] then          -- 空いているIDを見つけたら
            target.outputId = id                   -- ターゲットに記録
            assignedOutputIds[id] = true           -- 使用中にマーク
            -- debug.log("Assigned Output ID " .. id .. " to internal ID " .. target.internalId)
            return true                            -- 割り当て成功
        end
    end
    -- debug.log("Could not assign Output ID to internal ID " .. target.internalId .. " - All slots full.")
    return false -- 割り当て失敗 (満杯)
end

function releaseOutputId(target)
    if target.outputId ~= nil then
        -- debug.log("Releasing Output ID " .. target.outputId .. " from internal ID " .. target.internalId)
        assignedOutputIds[target.outputId] = nil -- 使用中マークを解除
        target.outputId = nil                    -- ターゲットの記録も消す
    end
end

------------------------
--- クォータニオン演算関数
------------------------
-- 二つのクォータニオン q_a = {w, x, y, z}, q_b = {w, x, y, z} の積を計算する関数
-- q_result = q_a * q_b
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2, w_result, x_result, y_result, z_result
    w1, x1, y1, z1 = q_a[1], q_a[2], q_a[3], q_a[4]
    w2, x2, y2, z2 = q_b[1], q_b[2], q_b[3], q_b[4]

    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return { w_result, x_result, y_result, z_result }
end

-- ZYXオイラー角 (Roll: phi, Yaw: psi, Pitch: theta) からクォータニオン q = (w, x, y, z) を計算する関数
-- 入力角度はラジアン単位
function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z
    -- オイラー角の半分を計算
    half_roll = roll * 0.5
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5

    -- 角度の半分のcosとsinを事前計算
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)

    -- クォータニオンの成分を計算
    w = cr * cy * cp + sr * sy * sp
    x = cr * cy * sp - sr * sy * cp -- X成分
    y = cr * sy * cp + sr * cy * sp -- Y成分
    z = sr * cy * cp - cr * sy * sp -- Z成分

    -- クォータニオンをテーブルとして返す (wが最初の要素)
    -- または {x=x, y=y, z=z, w=w} のような形式でも良い
    return { w, x, y, z }
end

-- ベクトル v = {x, y, z} をクォータニオン q = {w, x, y, z} で回転させる関数 (標準版: p' = q p q*)
function rotateVectorByQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, p_prime
    -- ベクトルを純粋クォータニオン p = (0, vx, vy, vz) に変換
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    p = { 0, px, py, pz }

    -- 回転クォータニオン q
    q = quaternion

    -- 回転クォータニオンの共役 q* = (w, -x, -y, -z) を計算
    q_conj = { q[1], -q[2], -q[3], -q[4] }

    -- p' = q * p * q* を計算 (標準的な順序)
    p_prime = multiplyQuaternions(multiplyQuaternions(q, p), q_conj) -- ★ 標準の掛け算順序

    -- p' のベクトル部 {x, y, z} を回転後のベクトルとして返す
    return { p_prime[2], p_prime[3], p_prime[4] }
end

--------------------------------------------------------------------------------
-- 座標・ベクトル変換関数
--------------------------------------------------------------------------------

--[[
rotateVectorZYX: Z-Y-X オイラー角でベクトルを回転 (ローカル->グローバル)
Physics Sensor のオイラー角 (Z-Y-X Intrinsic, 左手系) に対応。
入力オイラー角は Physics Sensor 出力値をそのまま使う (元コードの反転は不要と判断)。
※もし動作がおかしい場合は、元コードのように符号反転を試す。

function rotateVectorZYX(vector, pitch, yaw, roll)
    -- 回転行列 R = Rx(pitch) * Ry(yaw) * Rz(roll)

    -- Rz
    local RZ = { { math.cos(roll), -math.sin(roll), 0 }, { math.sin(roll), math.cos(roll), 0 }, { 0, 0, 1 } }
    -- Ry
    local RY = { { math.cos(yaw), 0, math.sin(yaw) }, { 0, 1, 0 }, { -math.sin(yaw), 0, math.cos(yaw) } }
    -- Rx
    local RX = { { 1, 0, 0 }, { 0, math.cos(pitch), -math.sin(pitch) }, { 0, math.sin(pitch), math.cos(pitch) } }
    local R = mul(RZ, RY, RX)


    -- 展開したものを使用して計算コスト削減
    local R = { { math.cos(roll) * math.cos(yaw), math.cos(roll) * math.sin(yaw) * math.sin(pitch) - math.sin(roll) * math.cos(pitch), math.cos(roll) * math.sin(yaw) * math.cos(pitch) + math.sin(roll) * math.sin(pitch) },
        { math.sin(roll) * math.cos(yaw), math.sin(roll) * math.sin(yaw) * math.sin(pitch) + math.cos(roll) * math.cos(pitch), math.sin(roll) * math.sin(yaw) * math.cos(pitch) - math.cos(roll) * math.sin(pitch) },
        { -math.sin(yaw),                 math.cos(yaw) * math.sin(pitch),                                                     math.cos(yaw) * math.cos(pitch) } }

    return mul(R, vector)
end
]]
--[[
localToGlobalCoords: レーダーのローカル極座標をグローバル直交座標に変換
出力は Physics Sensor グローバル座標系 (X:東, Y:上, Z:北)
]]
function localToGlobalCoords(dist, locAzi, locEle, rId, ownP)
    local locX, locY, locZ, radarLocVec, rYOff, cy, sy, RotY, gX, gY, gZ, vehLocVec_rotated, rotation_quaternion, globalRelativeVector
    -- 1. レーダー基準ローカル直交座標
    locX = dist * math.cos(locEle) * math.sin(locAzi)
    locY = dist * math.sin(locEle)
    locZ = dist * math.cos(locEle) * math.cos(locAzi)
    radarLocVec = { { locX }, { locY }, { locZ } }
    -- 2. ヨー回転適用 -> 車両前方基準ローカル座標へ
    rYOff = 0
    if rId == 1 then rYOff = PI / 2 elseif rId == 2 then rYOff = PI elseif rId == 3 then rYOff = -PI / 2 end
    vehLocVec_rotated = radarLocVec -- ID 0 は回転不要
    if rYOff ~= 0 then
        cy = math.cos(rYOff)
        sy = math.sin(rYOff)
        RotY = { { cy, 0, sy }, { 0, 1, 0 }, { -sy, 0, cy } }
        vehLocVec_rotated = mul(RotY, radarLocVec)
    end

    vehLocVec_rotated[2][1] = vehLocVec_rotated[2][1] + 2.5 / (rId + 1) -- レーダーのY軸オフセット
    -- 5. 車両姿勢で回転 -> グローバルな相対ベクトルへ
    -- 2. オイラー角から回転クォータニオンを生成
    vehLocVec_rotated = { vehLocVec_rotated[1][1], vehLocVec_rotated[2][1], vehLocVec_rotated[3][1] }
    rotation_quaternion = eulerZYX_to_quaternion(ownP.roll, ownP.yaw, ownP.pitch)
    globalRelativeVector = rotateVectorByQuaternion(vehLocVec_rotated, rotation_quaternion)

    -- 6. 物理センサーのグローバル位置を加算 -> 最終的な目標グローバル座標
    gX = globalRelativeVector[1] + ownP.x
    gY = globalRelativeVector[2] + ownP.y
    gZ = globalRelativeVector[3] + ownP.z
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
    local targetrX, targetrY, targetrZ, targetX, targetY, targetZ, relativeX, relativeY, relativeZ, r_sq, rh_sq, r, rh
    local predictedDistance, predictedElevation, predictedAzimuth, H, h
    targetX = stateVector[1][1]
    targetY = stateVector[3][1]
    targetZ = stateVector[5][1]
    relativeX = targetX - ownPosition.x
    relativeY = targetY - ownPosition.y -- Physics Sensor Y は Up
    relativeZ = targetZ - ownPosition.z -- Physics Sensor Z は North

    r_sq = relativeX ^ 2 + relativeY ^ 2 + relativeZ ^ 2
    rh_sq = relativeX ^ 2 + relativeZ ^ 2 -- XZ平面 (East-North)
    if r_sq < 1e-9 then r_sq = 1e-9 end
    if rh_sq < 1e-9 then rh_sq = 1e-9 end
    r = math.sqrt(r_sq)
    rh = math.sqrt(rh_sq)

    -- 1. 観測予測値 h = [距離, 仰角(Y基準), 方位角(Z基準)]
    predictedDistance = r
    predictedElevation = math.asin(math.max(-1.0, math.min(1.0, relativeY / r)))
    predictedAzimuth = math.atan(relativeX, relativeZ)
    h = { { predictedDistance }, { predictedElevation }, { predictedAzimuth } }

    -- 2. 観測ヤコビ行列 H = dh/dX (3x6)
    H = zeros(3, NUM_STATES)
    H[1][1] = relativeX / r
    H[1][3] = relativeY / r
    H[1][5] = relativeZ / r
    H[2][1] = (-relativeX * relativeY) / (rh_sq * r)
    H[2][3] = rh / r_sq
    H[2][5] = (-relativeZ * relativeY) /
        (rh_sq * r)
    H[3][1] = relativeZ / rh_sq
    H[3][3] = 0
    H[3][5] = -relativeX / rh_sq
    -- 速度項の偏微分はゼロ (H[row][2,4,6] = 0)

    return H, h
end

--[[
extendedKalmanFilterUpdate: EKF の予測・更新ステップを実行
]]
function extendedKalmanFilterUpdate(stateVector, covariance, observation, ownPosition, dt, lastEpsilon)
    function CalculateAngleDifference(current, set)
        return (set - current + PI * 3) % PI2 - PI
    end

    local F, X_predicted, P_predicted, Z, H, h, R, Y, S, S_inv, K, X_updated, P_updated, epsilon, dt2, dt3, dt4, Q_base, Q_adapted, uncertaintyIncreaseFactor, adaptiveFactor, I_minus_KH

    -- 1. 予測ステップ (Predict)
    F = MatrixCopy(identityMatrix6x6)
    F[1][2] = dt
    F[3][4] = dt
    F[5][6] = dt
    X_predicted = mul(F, stateVector)

    dt2 = dt * dt
    dt3 = dt2 * dt / 2
    dt4 = dt3 * dt / 2
    Q_base = { { dt4, dt3, 0, 0, 0, 0 }, { dt3, dt2, 0, 0, 0, 0 }, { 0, 0, dt4, dt3, 0, 0 },
        { 0,   0,   dt3, dt2, 0, 0 }, { 0, 0, 0, 0, dt4, dt3 }, { 0, 0, 0, 0, dt3, dt2 } }
    adaptiveFactor = PROCESS_NOISE_BASE +
        PROCESS_NOISE_ADAPTIVE_SCALE /
        (1 + math.exp(-(lastEpsilon - PROCESS_NOISE_EPSILON_THRESHOLD) * PROCESS_NOISE_EPSILON_SLOPE))
    Q_adapted = scalar(adaptiveFactor, Q_base)

    uncertaintyIncreaseFactor = PREDICTION_UNCERTAINTY_FACTOR_BASE ^ (2 * (dt * 60)) -- tick数換算で計算
    P_predicted = sum(scalar(uncertaintyIncreaseFactor, mul(F, covariance, T(F))), Q_adapted)

    -- 2. 更新ステップ (Update)
    Z = { { observation.distance }, { observation.elevation }, { observation.azimuth } }

    H, h = getObservationJacobianAndPrediction(X_predicted, ownPosition)
    R = MatrixCopy(OBSERVATION_NOISE_MATRIX_TEMPLATE)
    R[1][1] = R[1][1] * (observation.distance ^ 2)       -- 距離の2乗に比例
    Y = zeros(3, 1)
    Y[1][1] = Z[1][1] - h[1][1]                          -- 距離はそのまま引き算
    -- CalculateAngleDifference(予測角度, 観測角度) で角度差を計算
    Y[2][1] = CalculateAngleDifference(h[2][1], Z[2][1]) -- 仰角の差 (正規化済み)
    Y[3][1] = CalculateAngleDifference(h[3][1], Z[3][1]) -- 方位角の差 (正規化済み)

    S = sum(mul(H, P_predicted, T(H)), R)
    S_inv = inv(S)
    if S_inv == nil then return stateVector, covariance, lastEpsilon end -- 逆行列計算失敗
    K = mul(P_predicted, T(H), S_inv)
    X_updated = sum(X_predicted, mul(K, Y))
    I_minus_KH = sub(identityMatrix6x6, mul(K, H))
    P_updated = sum(mul(I_minus_KH, P_predicted, T(I_minus_KH)), mul(K, R, T(K)))
    epsilon = mul(T(Y), S_inv, Y)[1][1]
    return X_updated, P_updated, epsilon
end

--------------------------------------------------------------------------------
-- デバッグ用: 行列の内容をログに出力する関数
--------------------------------------------------------------------------------
-- M: ログ出力したい行列 (テーブルのテーブル形式)
-- matrixName: (オプション) ログに出力する際の見出しとなる行列の名前 (文字列)
-- decimalPlaces: (オプション) 表示する小数点以下の桁数 (数値、デフォルトは 3)
-- fieldWidth: (オプション) 各数値が表示されるフィールドの最小幅 (数値、デフォルトは 10、右寄せ)
--[[
function logMatrix(M, matrixName, decimalPlaces, fieldWidth)
    -- デフォルト値の設定
    local dp = decimalPlaces or 3 -- 小数点以下桁数
    local fw = fieldWidth or 10   -- 表示幅

    -- 行列名の出力 (指定されている場合)
    if matrixName then
        debug.log("--- Matrix: " .. matrixName .. " ---")
    end

    -- 入力が行列 (テーブルのテーブル) か、空でないか基本的なチェック
    if type(M) ~= "table" or #M == 0 or type(M[1]) ~= "table" or #M[1] == 0 then
        debug.log("Invalid or empty matrix provided to logMatrix.")
        if matrixName then debug.log("------------------------") end
        return
    end

    -- 各行をループして整形された文字列を作成し、ログ出力
    local rows = #M
    local cols = #M[1] -- 列数は最初の行に基づいて決定

    for r = 1, rows do
        local rowString = "["         -- 行の開始を示す括弧
        if type(M[r]) == "table" then -- 行がテーブルであることを確認
            for c = 1, cols do
                local value = M[r][c] -- 要素を取得
                if type(value) == "number" then
                    -- 数値の場合、指定された桁数と幅でフォーマット
                    local formatSpecifier = "%" .. fw .. "." .. dp .. "f"
                    rowString = rowString .. string.format(formatSpecifier, value)
                else
                    -- 数値以外の場合、文字列に変換して幅を合わせて表示
                    local formatSpecifier = "%" .. fw .. "s"
                    rowString = rowString .. string.format(formatSpecifier, tostring(value))
                end
                -- 要素間にスペースを追加 (最後の要素でなければ)
                if c < cols then
                    rowString = rowString .. " "
                end
            end
        else
            -- 行がテーブルでない場合はエラーメッセージ
            rowString = rowString .. " Invalid row data (not a table)"
        end
        rowString = rowString .. " ]" -- 行の終了を示す括弧
        debug.log(rowString)          -- 整形された行をログに出力
    end

    -- 行列名の出力 (指定されている場合)
    if matrixName then
        debug.log("------------------------")
    end
end
]]
--------------------------------------------------------------------------------
-- メインループ: onTick
--------------------------------------------------------------------------------
function onTick()
    local isDelayed1, isDelayed2
    for internalId, target in pairs(targetList) do
        target.isUpdated = false
    end
    currentTick = currentTick + 1 -- 内部Tickカウンター更新

    -- 1. 自機情報と遅延フラグを取得
    physicsSensorData.x = inputNumber(25)
    physicsSensorData.y = inputNumber(26)
    physicsSensorData.z = inputNumber(27)
    physicsSensorData.pitch = inputNumber(28)
    physicsSensorData.yaw = inputNumber(29)
    physicsSensorData.roll = inputNumber(30)
    isDelayed1 = (inputNumber(31) == 1) -- RadarList1 (ID 0, 2) 遅延フラグ
    isDelayed2 = (inputNumber(32) == 1) -- RadarList2 (ID 1, 3) 遅延フラグ

    local currentObservations = {}      -- このtickで有効な観測データリスト

    -- 2. 入力データ読み込み、展開、変換、Tick情報付与
    if (inputNumber(1) ~= 0 or inputNumber(13) ~= 0) then
        for i = 1, MAX_INPUT_TARGETS_RL1 + MAX_INPUT_TARGETS_RL2 do
            local isDelayed, pack1, pack2, dist, localAziRad, localEleRad, rId, gX, gY, gZ, globalElevation, globalAzimuth, observationTick
            if i > MAX_INPUT_TARGETS_RL1 then
                isDelayed = isDelayed2
            else
                isDelayed = isDelayed1
            end
            pack1 = inputNumber(i * 2 - 1)
            pack2 = inputNumber(i * 2)

            if pack1 ~= 0 or pack2 ~= 0 then
                dist, localAziRad, localEleRad, rId = unpackTargetData(pack1, pack2)
                if rId ~= -1 and dist > 0 then
                    gX, gY, gZ = localToGlobalCoords(dist, localAziRad, localEleRad, rId, physicsSensorData)
                    globalElevation = math.asin((gY - physicsSensorData.y) / dist)
                    globalAzimuth = math.atan(gX - physicsSensorData.x, gZ - physicsSensorData.z)
                    -- 観測Tickを決定 (遅延フラグを考慮)
                    observationTick = isDelayed and (currentTick - 1) or currentTick
                    table.insert(currentObservations, {
                        distance = dist,
                        azimuth = globalAzimuth,
                        elevation = globalElevation,
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
    local assignedObservationIndices = {} -- Keeps track of which observation was assigned
    if #currentObservations > 0 then
        for internalId, currentTarget in pairs(targetList) do
            local bestMatchObsIndex, bestMatchObsIndex, minEpsilon, matchedState, matchedCovariance, matchedEpsilon, matchedObsTick
            local observation, dt_ticks, dt_sec, X_post, P_post, epsilon, currentClosingSpeed

            bestMatchObsIndex = -1
            minEpsilon = DATA_ASSOCIATION_THRESHOLD + 1

            for j = 1, #currentObservations do
                if not assignedObservationIndices[j] then
                    observation = currentObservations[j]
                    dt_ticks = observation.obsTick - currentTarget.lastTick
                    if dt_ticks > 0 then
                        dt_sec = dt_ticks / 60.0
                        X_post, P_post, epsilon = extendedKalmanFilterUpdate(currentTarget.X, currentTarget.P,
                            observation, physicsSensorData, dt_sec, currentTarget.epsilon)
                        if epsilon < minEpsilon then
                            minEpsilon = epsilon
                            bestMatchObsIndex = j
                            matchedState = X_post
                            matchedCovariance = P_post
                            matchedEpsilon = epsilon
                            matchedObsTick = observation.obsTick
                        end
                    end
                end
            end

            -- ★更新: 最良マッチが見つかった場合の処理
            if bestMatchObsIndex ~= -1 and minEpsilon <= DATA_ASSOCIATION_THRESHOLD then
                -- EKF状態更新
                targetList[internalId].X = matchedState
                targetList[internalId].P = matchedCovariance
                targetList[internalId].epsilon = matchedEpsilon
                targetList[internalId].lastTick = matchedObsTick

                -- 同定成功回数インクリメント
                targetList[internalId].identification_count = targetList[internalId].identification_count + 1
                targetList[internalId].isUpdated = true
                -- 接近速度を計算して記録
                currentClosingSpeed = calculateClosingSpeed(targetList[internalId], physicsSensorData)
                table.insert(targetList[internalId].recent_closing_speeds, currentClosingSpeed)
                -- リストが指定サイズを超えたら古いものを削除
                if #targetList[internalId].recent_closing_speeds > HOSTILE_RECENT_UPDATES_THRESHOLD then
                    table.remove(targetList[internalId].recent_closing_speeds, 1)
                end

                -- 敵対判定実行
                assignedObservationIndices[bestMatchObsIndex] = true
            end
        end
    end

    -- ★ 4. 目標リスト更新ループ (Output ID 管理と削除判定) ★
    local targetIdsToDelete = {}
    for internalId, target in pairs(targetList) do
        local isHostileNow, ticksSinceLastUpdate, isTimeout, closingSpeed, isLeaving
        -- 4.1 敵対判定実行 & Output ID 管理
        checkHostileCondition(target) -- target.is_hostile が更新される可能性
        isHostileNow = target.is_hostile

        if isHostileNow then
            -- 敵対状態なら Output ID 割り当てを試行 (すでに持っていれば assignOutputId は何もしない)
            assignOutputId(target)
        else
            -- 非敵対状態なら Output ID を解放 (すでに解放済みなら releaseOutputId は何もしない)
            releaseOutputId(target)
        end

        -- 4.2 タイムアウト・離反判定
        ticksSinceLastUpdate = currentTick - target.lastTick
        isTimeout = ticksSinceLastUpdate >= TARGET_TIMEOUT_TICKS
        closingSpeed = calculateClosingSpeed(target, physicsSensorData)
        isLeaving = (closingSpeed < TARGET_IS_LEAVING_THRESHOLD)
        if isTimeout or isLeaving then
            table.insert(targetIdsToDelete, internalId)
            -- ★ 削除対象になったら Output ID を解放
            releaseOutputId(target)
            -- debug.log("Target "..internalId.." marked for deletion. Reason: "..(isTimeout and "Timeout" or "Leaving"))
        end
    end

    -- 4.3 目標削除実行
    for _, id_to_delete in ipairs(targetIdsToDelete) do
        -- debug.log("Deleting Target Internal ID: " .. id_to_delete .. ", Output ID: " .. (targetList[id_to_delete] and targetList[id_to_delete].outputId or "N/A"))
        targetList[id_to_delete] = nil -- テーブルからエントリを削除
    end


    -- 5. 新規目標の登録
    for j = 1, #currentObservations do
        if not assignedObservationIndices[j] then -- 観測が既存のターゲットに割り当てられていない場合
            local newObs, X_init, P_init, init_pos_var_factor, pos_var_ele, pos_var_azi, newInternalId
            local newObs = currentObservations[j]
            -- 状態ベクトル(X)と共分散行列(P)の初期化
            X_init = { { newObs.globalX }, { 0 }, { newObs.globalY }, { 0 }, { newObs.globalZ }, { 0 } }
            P_init = zeros(NUM_STATES, NUM_STATES)
            init_pos_var_factor = 10
            -- pos_var_dist = OBSERVATION_NOISE_MATRIX_TEMPLATE[1][1] * init_pos_var_factor * (newObs.distance ^ 2)
            pos_var_ele = OBSERVATION_NOISE_MATRIX_TEMPLATE[2][2] * init_pos_var_factor
            pos_var_azi = OBSERVATION_NOISE_MATRIX_TEMPLATE[3][3] * init_pos_var_factor
            -- 単純化された初期位置の分散 - より良い方法が存在するが、より複雑な計算を必要とする
            P_init[1][1] = pos_var_azi -- プレースホルダーの分散割り当て
            P_init[3][3] = pos_var_ele
            P_init[5][5] = pos_var_azi
            P_init[2][2] = INITIAL_VELOCITY_VARIANCE
            P_init[4][4] = INITIAL_VELOCITY_VARIANCE
            P_init[6][6] = INITIAL_VELOCITY_VARIANCE

            -- targetListに新しいターゲット・エントリーを作成する
            newInternalId = nextInternalId
            targetList[newInternalId] = {
                internalId = newInternalId, -- 内部IDを保存する
                outputId = nil,             -- ★ 出力IDはnilから始まる
                lastTick = newObs.obsTick,
                X = X_init,
                P = P_init,
                epsilon = 1,                -- 初期イプシロン
                identification_count = 0,   -- Hostile check data
                recent_closing_speeds = {}, -- Hostile check data
                is_hostile = false,         -- Hostile check data
                isUpdated = false
            }
            -- debug.log("New target registered. Internal ID: " .. newInternalId)
            nextInternalId = nextInternalId + 1 -- 次の新しい目標のためにインクリメントする
            -- 注：敵対チェックと出力IDの割り当ては、次のティックの更新ループで行われる。
        end
    end


    -- 6. 出力処理 (Output ID ベース)
    -- 最初に出力チャンネルをクリアする
    for i = 1, 32 do
        outputNumber(i, 0)
        outputBool(i, false)
    end

    -- 追跡されたターゲットを反復処理し、割り当てられた出力IDを持つターゲットを出力する。
    for internalId, target in pairs(targetList) do
        -- ★ ターゲットに出力IDが割り当てられているかチェックする
        if target and target.outputId ~= nil then
            local predX, predY, predZ, dt_pred_sec, baseChannel
            -- Calculate predicted position for the current tick
            dt_pred_sec = (currentTick - target.lastTick + LOGIC_DELAY) / 60 -- ロジック遅延を考慮して先読みさせる
            -- Ensure dt is non-negative; if target just updated, dt is 0
            dt_pred_sec = math.max(0, dt_pred_sec)

            predX = target.X[1][1] + target.X[2][1] * dt_pred_sec
            predY = target.X[3][1] + target.X[4][1] * dt_pred_sec
            predZ = target.X[5][1] + target.X[6][1] * dt_pred_sec

            -- Calculate the base channel using the Output ID
            baseChannel = (target.outputId - 1) * 3
            -- Check if baseChannel is within limits (sanity check)
            if baseChannel + 3 <= MAX_TRACKED_TARGETS * 3 then
                -- Output predicted coordinates to the fixed channels
                outputNumber(baseChannel + 1, predX) -- X (East)
                outputNumber(baseChannel + 2, predY) -- Y (Up)
                outputNumber(baseChannel + 3, predZ) -- Z (North)
                outputBool(target.outputId, target.isUpdated)
                -- else -- Debugging: Calculated channel out of bounds
                -- debug.log("Error: Target Output ID "..target.outputId.." results in channel out of bounds.")
            end
        end
    end
end
