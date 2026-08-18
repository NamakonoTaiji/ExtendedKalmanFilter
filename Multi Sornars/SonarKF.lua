--[[
機能:
一次フィルター（Torpedo NewtonDistanceFinder）で同一Ping内の重複反射を
1観測へ統合したデータを受け取り、6状態の線形カルマンフィルターで時系列追跡する。

処理:
1. 距離・方位角・仰角をグローバル座標へ変換
2. 各観測をマハラノビス距離が最小の既存トラックへ関連付け
3. 関連付けできない観測を仮クラスタへ追加
4. 仮クラスタが複数回確認されたら正式トラック化
5. 観測がない間は等速モデルで予測し、距離依存時間後に削除

一次フィルター側で空間的な重複反射を統合済みのため、
このコードでは正式トラック同士の近接統合や重複エコー吸収を行わない。

入力（Composite Number）:
- ch 1～16: 最大8目標、1目標につき2ch
  - 2n-1: 圧縮された目標情報(pack1) [m]
  - 2n: 圧縮された目標情報(pack2) [rad]
- ch 25: 自機グローバル位置 X（東）
- ch 26: 自機グローバル位置 Y（上）
- ch 27: 自機グローバル位置 Z（北）
- ch 28: 自機Pitch [rad]
- ch 29: 自機Yaw [rad]
- ch 30: 自機Roll [rad]

出力:
- bool 1: 正式トラックあり
- num 1～3: 現在時刻へ外挿した推定位置 X,Y,Z
- num 4～6: 推定速度 Vx,Vy,Vz
- num 7～9: 予約（0）
- num 10: 最終観測の目標到達Tick
- num 11: 現在Tickから最終観測Tickまでの遅延
- num 12: トラックID
- num 13: 累積ヒット数
- num 32: 最新のマハラノビス距離 epsilon

座標系:
Physics Sensor座標系（X:東、Y:上、Z:北、左手系）。
状態ベクトルは [x,vx,y,vy,z,vz]。
]]

-- 定数
PI = math.pi
PI2 = PI * 2
DT = 1 / 60           -- 更新時間の基準 (秒)
MAX_RADAR_TARGETS = 8 -- 一次フィルターから受け取る最大目標数

-- カルマンフィルターパラメータ
DATA_ASSOCIATION_EPSILON_THRESHOLD = property.getNumber("D_ASOC_EPS") -- データアソシエーションのε閾値
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")            -- 目標ロスト判定のTick数

INITIAL_VELOCITY_VARIANCE = 10000
PROCESS_NOISE = property.getNumber("PROCESS_NOISE")
LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
DISTANCE_STEP = property.getNumber("DIST_STEP")
-- 観測ノイズ
-- 旧等方モデル v = 100 + distance^2 * 2.025e-5 を、
-- 距離方向と角度方向へ分離して異方性共分散Rを構成する。
-- RANGE_NOISE_VARIANCE: 距離方向の分散 [m^2]
-- ANGLE_NOISE_VARIANCE: 方位角/仰角の分散 [rad^2]
RANGE_NOISE_VARIANCE = property.getNumber("RANGE_NOISE_VAR")
ANGLE_NOISE_VARIANCE = 1.315947253e-5
if RANGE_NOISE_VARIANCE <= 0 then RANGE_NOISE_VARIANCE = 100 end
if ANGLE_NOISE_VARIANCE <= 0 then ANGLE_NOISE_VARIANCE = 2.025e-5 end

OBSERVATION_MATRIX = {
    { 1, 0, 0, 0, 0, 0 },
    { 0, 0, 1, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0 }
}


-- グローバル変数
trackedTargets = {} -- 複数のトラックを保持するテーブル
nextTrackID = 1     -- 新規トラックに割り当てるID
currentTick = 0
trackedTargetsIndex = 0

tentativeClusters = {}
nextClusterID = 1

-- ソナー向けクラスタリング設定
-- 必要に応じてプロパティ化してください。
CLUSTER_CONFIRM_HITS = 3     -- 正式トラック化に必要な独立観測数
CLUSTER_CONFIRM_WINDOW = 360 -- 候補を維持する最大観測時刻差[tick]
CLUSTER_MIN_HIT_GAP = 2      -- 同時刻付近の重複を別ヒットとして数えない
CLUSTER_BASE_GATE = 40       -- 候補クラスタの基礎ゲート[m]
CLUSTER_RANGE_FACTOR = 0.01  -- 距離に比例して広げるゲート
MAX_INITIAL_SPEED = 250      -- クラスタ速度の上限
--------------------------------------------------------------------------------
-- ベクトル演算ヘルパー関数
--------------------------------------------------------------------------------
function vectorMagnitude(v)
    local x, y, z
    x = v[1]
    y = v[2]
    z = v[3]
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

function vectorSub(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1]
    y1 = v1[2]
    z1 = v1[3]
    x2 = v2[1]
    y2 = v2[2]
    z2 = v2[3]
    return { x1 - x2, y1 - y2, z1 - z2 }
end

--------------------------------------------------------------------------------
-- 行列演算ヘルパー関数
--------------------------------------------------------------------------------
function zeros(rows, cols)
    local m = {}
    for r = 1, rows do
        m[r] = {}
        for c = 1, cols do m[r][c] = 0 end
    end
    return m
end

-- 単位行列
identityMatrix6x6 = zeros(6, 6)
for i = 1, 6 do identityMatrix6x6[i][i] = 1 end

function MatrixCopy(M)
    local N = {}
    for r, row in ipairs(M) do
        N[r] = { table.unpack(row) }
    end
    return N
end

function sum(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do
            R[r][c] = A[r][c] + B[r][c]
        end
    end
    return R
end

function sub(A, B)
    local R = zeros(#A, #A[1])
    for r = 1, #A do
        for c = 1, #A[1] do
            R[r][c] = A[r][c] - B[r][c]
        end
    end
    return R
end

function mul(...)
    local mats, A, R, B, sVal = { ... }
    A = mats[1]
    for i = 2, #mats do
        B = mats[i]

        R = zeros(#A, #B[1])
        for r = 1, #A do
            for c = 1, #B[1] do
                sVal = 0
                for k = 1, #B do
                    sVal = sVal + A[r][k] * B[k][c]
                end
                R[r][c] = sVal
            end
        end
        A = R -- 次の乗算のために結果をAに代入
    end
    return A  -- 最終結果
end

function T(M)
    local rows, cols, R
    --
    rows = #M
    cols = #M[1]
    R = zeros(cols, rows)
    for r = 1, rows do
        --
        for c = 1, cols do
            --
            R[c][r] = M[r][c]
        end
    end
    return R
end

-- 3x3行列専用の逆行列関数 (汎用invの代替)
function inv3(m)
    local a, b, c, d, e, f, g, h, i, det, invDet
    a, b, c = m[1][1], m[1][2], m[1][3]
    d, e, f = m[2][1], m[2][2], m[2][3]
    g, h, i = m[3][1], m[3][2], m[3][3]

    -- 行列式 (Sarrus)
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    -- 特異行列判定
    if math.abs(det) < 1e-15 then return nil end

    invDet = 1 / det

    -- クラメルの公式に基づく余因子行列の転置 * (1/det)
    return {
        { (e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet },
        { (f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet },
        { (d * h - e * g) * invDet, (g * b - a * h) * invDet, (a * e - b * d) * invDet }
    }
end

--------------------------------------------------------------------------------
-- クォータニオン演算関数
--------------------------------------------------------------------------------

---3次元ベクトルをクォータニオンで回転させる関数（軽量化版）
---@param vector table {x, y, z} または {1, 2, 3} 形式の回転対象ベクトル
---@param quaternion table {w, x, y, z} (インデックス 1, 2, 3, 4) 形式のクォータニオン
---@param isInverse? boolean true を渡すと逆回転を実行（省略時は false/順回転）
---@return number[] @回転後の 3次元ベクトル {x, y, z}
function rotateVectorByQuaternion(vector, quaternion, isInverse)
    local w, x, y, z = quaternion[1], quaternion[2], quaternion[3], quaternion[4]
    local vx, vy, vz = vector[1] or vector.x, vector[2] or vector.y, vector[3] or vector.z

    -- 外積演算の共通部分（軽量化計算）
    local tx = 2 * (y * vz - z * vy)
    local ty = 2 * (z * vx - x * vz)
    local tz = 2 * (x * vy - y * vx)

    -- 逆回転なら w の符号を反転
    if isInverse then w = -w end

    return {
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx)
    }
end

function localToGlobalCoords(localPosVec, ownGlobalPos, ownOrientationQuat)
    local localX, localY, localZ, globalRelativeVector, globalX, globalY, globalZ
    -- vector は {x, y, z} または {<index 1>, <index 2>, <index 3>} 形式のテーブルを想定

    localX = localPosVec[1]
    localY = localPosVec[2]
    localZ = localPosVec[3]

    globalRelativeVector = rotateVectorByQuaternion({ localX, localY, localZ }, ownOrientationQuat)

    globalX = globalRelativeVector[1] + ownGlobalPos[1]
    globalY = globalRelativeVector[2] + ownGlobalPos[2]
    globalZ = globalRelativeVector[3] + ownGlobalPos[3]
    return { globalX, globalY, globalZ }
end

-- グローバル直交座標からローカル直交座標へ

--- ローカル座標から方位角と仰角へ変換
---@return table azimuthとelevationを返します(ラジアン)

function polarCoordsToLocalCoords(dist, localEleRad, localAziRad)
    local localX, localY, localZ
    localX = dist * math.cos(localEleRad) * math.sin(localAziRad)
    localY = dist * math.sin(localEleRad)
    localZ = dist * math.cos(localEleRad) * math.cos(localAziRad)
    return { localX, localY, localZ }
end

--------------------------------------------------------------------------------
-- カルマンフィルター関連関数
--------------------------------------------------------------------------------
-- グローバル直交座標観測による線形カルマンフィルター
-- 状態 [x,vx,y,vy,z,vz] から位置 [x,y,z] を直接観測する。
function predictStep(currentTarget, dt)
    local X = currentTarget.X
    local P = currentTarget.P
    local F = MatrixCopy(identityMatrix6x6)

    F[1][2] = dt
    F[3][4] = dt
    F[5][6] = dt

    local XP = mul(F, X)
    local d2 = dt * dt
    local d3 = d2 * dt
    local Q = zeros(6, 6)

    for i = 0, 2 do
        local n = i * 2
        Q[n + 1][n + 1] = d3 / 3 * PROCESS_NOISE
        Q[n + 1][n + 2] = d2 / 2 * PROCESS_NOISE
        Q[n + 2][n + 1] = d2 / 2 * PROCESS_NOISE
        Q[n + 2][n + 2] = dt * PROCESS_NOISE
    end

    return XP, sum(mul(F, P, T(F)), Q)
end

-- 直交座標のイノベーションとマハラノビス距離を計算
function observationCovariance(o)
    local a = o.localAzimuthRad
    local e = o.localElevationRad
    local r = o.distance
    local ca, sa = math.cos(a), math.sin(a)
    local ce, se = math.cos(e), math.sin(e)

    -- ローカル極座標の直交単位基底
    local ur = { ce * sa, se, ce * ca }      -- 距離方向
    local ua = { ca, 0, -sa }                -- 方位角方向
    local ue = { -se * sa, ce, -se * ca }   -- 仰角方向

    -- 観測位置をGlobalへ変換したのと同じ姿勢で共分散軸もGlobalへ回す
    ur = rotateVectorByQuaternion(ur, o.orientation)
    ua = rotateVectorByQuaternion(ua, o.orientation)
    ue = rotateVectorByQuaternion(ue, o.orientation)

    local vr = RANGE_NOISE_VARIANCE
    local va = ANGLE_NOISE_VARIANCE * r * r * ce * ce
    local ve = ANGLE_NOISE_VARIANCE * r * r
    local R = zeros(3, 3)

    for i = 1, 3 do
        for j = 1, 3 do
            R[i][j] =
                vr * ur[i] * ur[j] +
                va * ua[i] * ua[j] +
                ve * ue[i] * ue[j]
        end
    end
    return R
end

-- 直交座標のイノベーションとマハラノビス距離を計算
function calculateInnovation(X, P, o)
    local H = OBSERVATION_MATRIX
    local Y = {
        { o.globalX - X[1][1] },
        { o.globalY - X[3][1] },
        { o.globalZ - X[5][1] }
    }

    local R = observationCovariance(o)
    local S = sum(mul(H, P, T(H)), R)
    local Si = inv3(S)

    if not Si then
        return math.huge, nil, nil, nil
    end

    local E = mul(T(Y), Si, Y)

    return E[1][1], Y, Si, H, S
end

-- カルマン更新ステップ
---@return table X_updated, table P_updated
function updateStep(X_predicted, P_predicted, Y, S_inv, H)
    local K, X_updated, P_updated
    K = mul(P_predicted, T(H), S_inv)
    X_updated = sum(X_predicted, mul(K, Y))
    P_updated = mul(sub(identityMatrix6x6, mul(K, H)), P_predicted)
    return X_updated, P_updated
end

-- フィルター状態を初期化
function initializeFilterState(c, id)
    local o, X, P, R
    o = c.lastObservation

    X = {
        { o.globalX }, { c.vx },
        { o.globalY }, { c.vy },
        { o.globalZ }, { c.vz }
    }

    P = zeros(6, 6)
    R = observationCovariance(o)

    -- 旧実装と同じく初期位置共分散は観測分散の5倍。
    -- Global XYZの相関もそのまま状態Pへ入れる。
    local pi = { 1, 3, 5 }
    for i = 1, 3 do
        for j = 1, 3 do
            P[pi[i]][pi[j]] = R[i][j] * 5
        end
    end

    P[2][2] = INITIAL_VELOCITY_VARIANCE
    P[4][4] = INITIAL_VELOCITY_VARIANCE
    P[6][6] = INITIAL_VELOCITY_VARIANCE

    return {
        id = id,
        X = X,
        P = P,
        epsilon = -1,
        lastTick = o.targetReachedTick,
        lastSeenTick = o.targetReachedTick,
        lastReceiveTick = currentTick,
        hits = c.hits,
    }
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function removeExpiredTentativeClusters()
    for id, c in pairs(tentativeClusters) do
        local timeout = math.max(
            CLUSTER_CONFIRM_WINDOW,
            c.distance * 120 / 1480 + 120
        )
        if currentTick - c.lastReceiveTick > timeout then
            tentativeClusters[id] = nil
        end
    end
end

function addTentativeObservation(obs)
    local bestID = nil
    local bestScore = math.huge
    local duplicateFound = false
    local duplicateScore = math.huge

    for id, c in pairs(tentativeClusters) do
        local dtTicks = obs.targetReachedTick - c.lastTick
        local totalTicks = obs.targetReachedTick - c.firstTick
        local confirmWindow = math.max(
            CLUSTER_CONFIRM_WINDOW,
            c.distance * 120 / 1480 + 120
        )
        -- 古い観測、確認期間外のクラスタは除外
        if dtTicks >= 0 and totalTicks <= confirmWindow then
            local dt = dtTicks * DT

            local baseGate =
                CLUSTER_BASE_GATE +
                obs.distance * CLUSTER_RANGE_FACTOR

            ----------------------------------------------------
            -- 同時刻付近
            -- 同一目標の重複観測として扱い、hitには加算しない
            ----------------------------------------------------
            if dtTicks < CLUSTER_MIN_HIT_GAP then
                local dx = obs.globalX - c.x
                local dy = obs.globalY - c.y
                local dz = obs.globalZ - c.z

                local err2 =
                    dx * dx +
                    dy * dy +
                    dz * dz

                local gate2 = baseGate * baseGate

                if err2 <= gate2 then
                    local score = err2 / gate2

                    if score < duplicateScore then
                        duplicateScore = score
                        duplicateFound = true
                    end
                end

                ----------------------------------------------------
                -- 時間的に独立した観測
                ----------------------------------------------------
            else
                local px = c.x
                local py = c.y
                local pz = c.z
                local gate

                if c.hits < 2 then
                    ------------------------------------------------
                    -- 1点しかないので速度不明
                    -- 最大速度まで許容
                    ------------------------------------------------
                    gate =
                        baseGate +
                        MAX_INITIAL_SPEED * dt
                else
                    ------------------------------------------------
                    -- 2点以上あるので速度予測を使用
                    ------------------------------------------------
                    px = px + c.vx * dt
                    py = py + c.vy * dt
                    pz = pz + c.vz * dt

                    -- 速度が分かった後はゲートを狭める
                    gate =
                        baseGate +
                        20 * dt
                end

                local dx = obs.globalX - px
                local dy = obs.globalY - py
                local dz = obs.globalZ - pz

                local err2 =
                    dx * dx +
                    dy * dy +
                    dz * dz

                local gate2 = gate * gate

                if err2 <= gate2 then
                    -- ゲート幅で正規化して比較
                    local score = err2 / gate2

                    if score < bestScore then
                        bestScore = score
                        bestID = id
                    end
                end
            end
        end
    end


    ------------------------------------------------------------
    -- 最も整合するものが同時刻付近の重複だった場合
    ------------------------------------------------------------
    if duplicateFound and duplicateScore <= bestScore then
        return nil
    end


    ------------------------------------------------------------
    -- 一致クラスタなし → 新規候補
    ------------------------------------------------------------
    if bestID == nil then
        local id = nextClusterID
        nextClusterID = id + 1

        tentativeClusters[id] = {
            id = id,

            x = obs.globalX,
            y = obs.globalY,
            z = obs.globalZ,

            firstX = obs.globalX,
            firstY = obs.globalY,
            firstZ = obs.globalZ,

            firstTick = obs.targetReachedTick,
            lastTick = obs.targetReachedTick,
            lastReceiveTick = currentTick,

            vx = 0,
            vy = 0,
            vz = 0,

            distance = obs.distance,

            hits = 1,

            lastObservation = obs
        }

        return nil
    end


    ------------------------------------------------------------
    -- 既存候補クラスタ更新
    ------------------------------------------------------------
    local c = tentativeClusters[bestID]

    -- 最初の観測から現在までを使って速度推定
    local dt =
        (obs.targetReachedTick - c.firstTick) * DT

    if dt > 0 then
        local vx = (obs.globalX - c.firstX) / dt
        local vy = (obs.globalY - c.firstY) / dt
        local vz = (obs.globalZ - c.firstZ) / dt
        local speed =
            math.sqrt(
                vx * vx +
                vy * vy +
                vz * vz
            )
        -- 異常な初期速度を制限
        if speed > MAX_INITIAL_SPEED then
            local k =
                MAX_INITIAL_SPEED / speed
            vx = vx * k
            vy = vy * k
            vz = vz * k
        end

        c.vx = vx
        c.vy = vy
        c.vz = vz
    end


    c.x = obs.globalX
    c.y = obs.globalY
    c.z = obs.globalZ

    c.lastTick =
        obs.targetReachedTick
    c.lastReceiveTick =
        currentTick
    c.lastObservation =
        obs
    c.distance =
        obs.distance
    c.hits =
        c.hits + 1

    ------------------------------------------------------------
    -- 正式トラック化
    ------------------------------------------------------------
    if c.hits >= CLUSTER_CONFIRM_HITS then
        tentativeClusters[bestID] = nil
        return c
    end

    return nil
end

--============================================================
-- UNPACK
--
-- packed1, packed2 : packTarget()から受信した値
-- currentTick      : 受信側の現在tick
-- transportDelay   : 送信Lua→受信Lua間の固定遅延 [tick]
--                    不要なら0または省略
--
-- return:
-- distance
-- azimuth
-- elevation
-- targetReachedTick
--
-- データなし(packed1=0, packed2=0)の場合 nil
--============================================================
function unpackTarget(
    packed1,
    packed2,
    transportDelay
)
    local pi = math.pi
    local pi2 = pi * 2

    transportDelay = transportDelay or 0


    ----------------------------------------------------------
    -- データなし
    ----------------------------------------------------------
    if packed1 == 0 and packed2 == 0 then
        return nil
    end


    ----------------------------------------------------------
    -- Number伝送後の値を整数へ
    ----------------------------------------------------------
    packed1 = math.floor(packed1 + 0.5)
    packed2 = math.floor(packed2 + 0.5)


    ----------------------------------------------------------
    -- packed1 抽出
    ----------------------------------------------------------

    -- distance 12bit
    local d =
        packed1 % 4096

    -- age 11bit
    local age =
        math.floor(packed1 / 4096) % 2048

    -- azimuth 上位1bit
    local aHigh =
        math.floor(packed1 / 8388608) % 2


    ----------------------------------------------------------
    -- packed2 抽出
    ----------------------------------------------------------

    -- azimuth 下位12bit
    local aLow =
        packed2 % 4096

    -- elevation 12bit
    local e =
        math.floor(packed2 / 4096) % 4096


    ----------------------------------------------------------
    -- 方位角13bitを結合
    ----------------------------------------------------------
    local a =
        aHigh * 4096 + aLow


    ----------------------------------------------------------
    -- 実値へ復元
    ----------------------------------------------------------

    -- 距離
    local distance =
        d * DISTANCE_STEP

    -- 方位角
    local azimuth =
        a / 8192
        * pi2
        - pi

    -- 仰角
    local elevation =
        e / 4095
        * pi
        - pi / 2


    ----------------------------------------------------------
    -- targetReachedTick復元
    --
    -- ageは「送信時点で何tick前だったか」なので、
    -- 通信経路の遅延も追加で引く
    ----------------------------------------------------------
    local targetReachedTick =
        currentTick
        - age
        - transportDelay


    return
        distance,
        azimuth,
        elevation,
        targetReachedTick
end

--------------------------------------------------------------------------------
-- メイン処理 (onTick)
--------------------------------------------------------------------------------
function onTick()
    currentTick = currentTick + 1

    ownGlobalPos = {
        input.getNumber(26),
        input.getNumber(27),
        input.getNumber(28)
    }

    ownOrientation = { input.getNumber(29), input.getNumber(30), input.getNumber(31), input.getNumber(32) }

    if ownOrientation == nil then ownOrientation = { 1, 0, 0, 0 } end

    local currentObservations = {}

    for i = 1, MAX_RADAR_TARGETS do
        local localPos, globalPos, relative, distCheck
        local p1, p2 = input.getNumber(i * 2 - 1), input.getNumber(i * 2)
        local dist, localAziRad, localEleRad, targetReachedTick = unpackTarget(p1, p2, 2)
        -- 至近距離目標は取り込まない
        if dist and dist > 50 then
            localPos = polarCoordsToLocalCoords(
                dist,
                localEleRad,
                localAziRad
            )
            globalPos = localToGlobalCoords(
                localPos,
                ownGlobalPos,
                ownOrientation
            )

            if globalPos ~= nil then
                relative = vectorSub(globalPos, ownGlobalPos)
                distCheck = vectorMagnitude(relative)
                if distCheck > 1e-6 then
                    currentObservations[#currentObservations + 1] = {
                        distance = dist,
                        elevation = math.asin(math.max(
                            -1,
                            math.min(1, relative[2] / distCheck)
                        )),
                        azimuth = math.atan(relative[1], relative[3]),
                        localAzimuthRad = localAziRad,
                        localElevationRad = localEleRad,
                        orientation = ownOrientation,
                        globalX = globalPos[1],
                        globalY = globalPos[2],
                        globalZ = globalPos[3],
                        targetReachedTick = targetReachedTick,

                        source = i
                    }
                end
            end
        end
    end

    -- 波面の到達順と同じく、古い観測時刻から逐次処理する。
    table.sort(currentObservations, function(a, b)
        return a.targetReachedTick < b.targetReachedTick
    end)


    for _, obs in ipairs(currentObservations) do
        local bestTrackID, bestEpsilon, bestCache, dtTicks, XPred, PPred
        local duplicateTrack = false

        bestTrackID = nil
        bestEpsilon = math.huge
        bestCache = nil

        -- 各観測を、予測と最も整合する1本の既存トラックへ関連付けする。
        -- 一次側で同一Ping内の重複反射は統合済み。
        for trackID, track in pairs(trackedTargets) do
            dtTicks = obs.targetReachedTick - track.lastTick

            ------------------------------------------------------------
            -- 新しい観測
            ------------------------------------------------------------
            if dtTicks > CLUSTER_MIN_HIT_GAP then
                XPred, PPred =
                    predictStep(track, dtTicks * DT)

                if XPred ~= nil and PPred ~= nil then
                    local epsilon, Y, SInv, H, S =
                        calculateInnovation(XPred, PPred, obs)

--[[                     debug.log(
                        "src:" .. obs.source ..
                        " eps:" .. epsilon ..
                        " dt:" .. dtTicks ..
                        " Y:" ..
                        math.floor(Y[1][1]) .. "," ..
                        math.floor(Y[2][1]) .. "," ..
                        math.floor(Y[3][1]) ..
                        " S:" ..
                        math.floor(S[1][1]) .. "," ..
                        math.floor(S[2][2]) .. "," ..
                        math.floor(S[3][3])
                    ) ]]
                    if epsilon < DATA_ASSOCIATION_EPSILON_THRESHOLD
                        and epsilon < bestEpsilon then
                        bestEpsilon = epsilon
                        bestTrackID = trackID

                        bestCache = {
                            XPred = XPred,
                            PPred = PPred,
                            Y = Y,
                            SInv = SInv,
                            H = H,
                        }
                    end
                end

                ------------------------------------------------------------
                -- 同時刻 / 過去の観測
                --
                -- 別ソナーから遅れて到着した同一目標の可能性を見る。
                -- KFは巻き戻さず、重複なら捨てる。
                ------------------------------------------------------------
            else
                local dt = dtTicks * DT

                -- trackを観測時刻まで等速で逆外挿
                local px =
                    track.X[1][1] +
                    track.X[2][1] * dt

                local py =
                    track.X[3][1] +
                    track.X[4][1] * dt

                local pz =
                    track.X[5][1] +
                    track.X[6][1] * dt

                local dx = obs.globalX - px
                local dy = obs.globalY - py
                local dz = obs.globalZ - pz

                local err2 =
                    dx * dx +
                    dy * dy +
                    dz * dz

                local gate =
                    CLUSTER_BASE_GATE +
                    obs.distance * CLUSTER_RANGE_FACTOR +
                    20 * math.abs(dt)

                if err2 <= gate * gate then
                    duplicateTrack = true
                end
            end
        end

        if bestTrackID ~= nil then
            local track, XUp, PUp
            track = trackedTargets[bestTrackID]

            XUp, PUp = updateStep(
                bestCache.XPred,
                bestCache.PPred,
                bestCache.Y,
                bestCache.SInv,
                bestCache.H
            )

            if XUp then
                -- 速度はKFの位置-速度共分散による更新だけに任せる。
                -- 観測位置差分の手動混合や更新後の速度クリップは行わない。
                track.X = XUp
                track.P = PUp
                track.epsilon = bestEpsilon
                track.lastTick = obs.targetReachedTick
                track.lastReceiveTick = currentTick
                track.lastSeenTick = obs.targetReachedTick
                track.hits = track.hits + 1
            end
        elseif not duplicateTrack then
            local c, id
            c = addTentativeObservation(obs)

            if c then
                id = nextTrackID
                nextTrackID = id + 1
                trackedTargets[id] = initializeFilterState(c, id)
            end
        end
    end

    -- 最終受信から距離依存の待機時間を超えたトラックを削除する。
    local deleteIDs = {}

    for id, track in pairs(trackedTargets) do
        local dx, dy, dz, distance, timeout
        dx = track.X[1][1] - ownGlobalPos[1]
        dy = track.X[3][1] - ownGlobalPos[2]
        dz = track.X[5][1] - ownGlobalPos[3]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        timeout = math.max(
            TARGET_LOST_THRESHOLD_TICKS,
            distance * 120 / 1480 + 180
        )

        if currentTick - track.lastReceiveTick > timeout then
            deleteIDs[#deleteIDs + 1] = id
        end
    end
    for _, id in ipairs(deleteIDs) do
        trackedTargets[id] = nil
    end

    removeExpiredTentativeClusters()

    -- ID順に時分割出力する。
    local sortedTracks = {}
    for _, track in pairs(trackedTargets) do
        sortedTracks[#sortedTracks + 1] = track
    end

    local targetToOutput = nil
    if #sortedTracks > 0 then
        table.sort(sortedTracks, function(a, b) return a.id < b.id end)
        trackedTargetsIndex = trackedTargetsIndex + 1
        if trackedTargetsIndex > #sortedTracks then
            trackedTargetsIndex = 1
        end
        targetToOutput = sortedTracks[trackedTargetsIndex]
    else
        trackedTargetsIndex = 0
    end

    if targetToOutput ~= nil then
        local X, predictionTicks, dt
        X = targetToOutput.X
        predictionTicks =
            currentTick - targetToOutput.lastTick + LOGIC_DELAY
        if predictionTicks < 0 then predictionTicks = 0 end

        dt = predictionTicks * DT

        output.setNumber(1, X[1][1] + X[2][1] * dt)
        output.setNumber(2, X[3][1] + X[4][1] * dt)
        output.setNumber(3, X[5][1] + X[6][1] * dt)
        output.setNumber(4, X[2][1])
        output.setNumber(5, X[4][1])
        output.setNumber(6, X[6][1])
        output.setNumber(10, targetToOutput.lastSeenTick)
        output.setNumber(
            11,
            currentTick - targetToOutput.lastSeenTick
        )
        output.setNumber(12, targetToOutput.id)
        output.setNumber(13, targetToOutput.hits)
        output.setNumber(32, targetToOutput.epsilon)
        output.setBool(1, true)
    else
        for i = 1, 13 do output.setNumber(i, 0) end
        output.setNumber(32, 0)
        output.setBool(1, false)
    end
end