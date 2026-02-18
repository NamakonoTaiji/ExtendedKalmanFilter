---------------------------------------------------------
--- 一次フィルタースクリプト
---------------------------------------------------------


-- カルマンフィルターに送る情報に一次処理を行うスクリプト
-- 仕様 --
-- * ミッドレンジフィルターによりノイズを除去
-- * 観測をクラスタリングする。ローカル座標間の距離が近い目標は同じ目標とみなし、それぞれを平均した位置を観測とする。距離が長いほどクラスタリングさせる距離の閾値が増える
-- * ローカル座標での変換を行い物理センサ基準での観測にする
-- * 最後に座標変換を行った距離、方位角、仰角を出力
-- * 出力は4tick目のみ行い他のtickは常に0を出力
-- * 4レーダ分にそれぞれこのスクリプトを適用し、compositeSwitchBoxによる出力タイミング制御は他のスクリプトで行う
-- * 0tick探知タイミングで自機姿勢情報を記憶しカルマンフィルターへパススルー

-- グローバル座標系はX-EAST,Y-ALTITUDE,Z-NORTH
-- ローカル座標系はX-RIGHT,Y-UP,Z-FORWARD

-- 入力
-- num1-3: レーダー目標1 (距離, 方位角(回転単位), 仰角(回転単位))
-- num4-6: レーダー目標2 ...
-- num19-21: レーダー目標8
-- num22: 探知時間
-- bool31: 目標ロック

-- 出力
-- num1: 距離(m)
-- num2: 方位角(ラジアン)
-- num3: 仰角(ラジアン)
-- これを最大8目標分24chまで繰り返す
-- num25: 自機ワールド座標X
-- num26: 自機ワールド座標Y
-- num27: 自機ワールド座標Z
-- num28: 自機オイラー角X
-- num29: 自機オイラー角Y
-- num30: 自機オイラー角Z
-- bool1~7: 目標を検出中
-- bool30: 目標ロック
-- bool31: compositeSwitchBox1の制御信号
-- bool32: compositeSwitchBox2の制御信号
local DETECTION_MAX = 7
local DETECTION_INTERVAL = 4
local INPUT_CHANNEL_BASE = 3
local OUTPUT_CHANNEL_BASE = 3
local OFFSET_RIGHT = 0
local OFFSET_UP = 0.25
local OFFSET_FORWARD = 0
local PI = math.pi
local PI2 = PI * 2
local MIN_DISTANCE = property.getNumber("MIN_DISTANCE")
local TRACKMODE_ENABLE_CH = property.getNumber("TRACKMODE_ENABLE_CH")
if TRACKMODE_ENABLE_CH == 0 then
    TRACKMODE_ENABLE_CH = 31
end
--------------------------------------------------------------------------------
-- ヘルパー関数 (ベクトル演算)
--------------------------------------------------------------------------------

--- ベクトル加算
function vectorAdd(v1, v2)
    return { x = (v1.x or 0) + (v2.x or 0), y = (v1.y or 0) + (v2.y or 0), z = (v1.z or 0) + (v2.z or 0) }
end

--- ベクトル・スカラー乗算
function vectorScalarMul(s, v)
    return { x = s * (v.x or 0), y = s * (v.y or 0), z = s * (v.z or 0) }
end

--- 2点間の距離 (ローカル座標)
function vectorDistance(v1, v2)
    local dx = (v1.x or 0) - (v2.x or 0)
    local dy = (v1.y or 0) - (v2.y or 0)
    local dz = (v1.z or 0) - (v2.z or 0)
    return math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
end

--- ベクトルの大きさ（原点からの距離）
function vectorMagnitude(v)
    return math.sqrt((v.x or 0) ^ 2 + (v.y or 0) ^ 2 + (v.z or 0) ^ 2)
end

--------------------------------------------------------------------------------
-- フィルター関数
--------------------------------------------------------------------------------

--- ミッドレンジ (maxMin)
---@param numList table 数値のリスト
---@return number (最大値 + 最小値) / 2
function maxMin(numList)
    if #numList == 0 then return 0 end
    local max, min
    max = numList[1]
    min = numList[1]
    for i = 2, #numList do
        if numList[i] > max then
            max = numList[i]
        elseif numList[i] < min then
            min = numList[i]
        end
    end
    return (max + min) / 2
end

--- 1. 時間軸の平滑化 (ミッドレンジフィルター)
-- スイープバッファ [targetIdx][tick] から、ターゲットごとの平均値を計算
---@param sweepBuffer table [targetIdx][tick] = {obs}
---@return table フィルター後のリスト { distance, azimuthRad, elevationRad }
function applyMidRangeFilter(sweepBuffer)
    local filteredTargets = {}

    -- ターゲットインデックス (1～8) ごとにループ
    for targetIdx, tickData in pairs(sweepBuffer) do
        local distList = {}
        local aziList = {}
        local eleList = {}

        -- スイープ(時間軸)を走査
        for tick, obs in pairs(tickData) do
            table.insert(distList, obs.distance)
            table.insert(aziList, obs.azimuthRad)
            table.insert(eleList, obs.elevationRad)
        end

        if #distList > 0 then
            table.insert(filteredTargets, {
                distance = maxMin(distList),
                azimuthRad = maxMin(aziList),
                elevationRad = maxMin(eleList)
            })
        end
    end
    return filteredTargets
end

--- 2. 空間軸の平滑化 (クラスタリング)
-- ミッドレンジフィルター後の観測値リストをクラスタリング
---@param obsList table 観測値 { distance, azimuthRad, elevationRad } のリスト
---@return table クラスタリング・平均化された観測値 { distance, azimuthRad, elevationRad }
function clusterTargets(obsList)
    local CLUSTER_THRESHOLD_BASE = 15      -- 基準閾値 (15m)
    local CLUSTER_THRESHOLD_FACTOR = 0.012 -- 距離係数

    -- 1. 観測値をローカル座標に変換 (計算用)
    local localObsList = {}
    for _, obs in ipairs(obsList) do
        local pos = localAngleDistToLocalCoords(obs.distance, obs.azimuthRad, obs.elevationRad)
        table.insert(localObsList, {
            originalDistance = obs.distance, -- 閾値計算用に元の距離を保持
            pos = pos
        })
    end

    local clusteredList = {}
    local i = 1
    while i <= #localObsList do
        local g = localObsList[i]

        -- 2. グループ化 (基準点g と 仲間Y)
        local clusterPosSum = { x = g.pos.x, y = g.pos.y, z = g.pos.z }
        local clusterCount = 1

        -- 3. 動的閾値 cV の計算
        local cV = (CLUSTER_THRESHOLD_FACTOR * g.originalDistance) + CLUSTER_THRESHOLD_BASE

        local j = i + 1
        while j <= #localObsList do
            local N = localObsList[j]
            -- 3. 空間距離の計算
            local dist = vectorDistance(g.pos, N.pos)

            if dist < cV then
                -- 仲間とみなす
                clusterPosSum = vectorAdd(clusterPosSum, N.pos)
                clusterCount = clusterCount + 1
                table.remove(localObsList, j)
                -- j はデクリメントしない (table.remove で j 番目が次に来るため)
            else
                j = j + 1 -- 次へ
            end
        end

        -- 4. 平均化
        local avgPos = vectorScalarMul(1 / clusterCount, clusterPosSum)

        -- 5. 逆変換 (平均化された座標から 距離, 方位, 仰角 を再計算)
        local avgDist = vectorMagnitude(avgPos)
        local angles = localCoordsToLocalAngle(avgPos)

        -- 6. 結果リストに追加
        table.insert(clusteredList, {
            distance = avgDist,
            azimuthRad = angles.azimuth,
            elevationRad = angles.elevation
        })

        i = i + 1 -- 次の基準点へ
    end

    return clusteredList
end

--- ローカル座標から方位角と仰角へ変換
---@param localPosVec Vector3 変換したいローカル座標(X右方向, Y上方向, Z前方向)
---@return table azimuthとelevationを返します(ラジアン)
function localCoordsToLocalAngle(localPosVec)
    local horizontalDistance, currentLocalAzimuth, currentLocalElevation
    horizontalDistance = math.sqrt(localPosVec.x ^ 2 + localPosVec.z ^ 2)
    currentLocalAzimuth = math.atan(localPosVec.x, localPosVec.z)        -- atan(左右, 前後)
    currentLocalElevation = math.atan(localPosVec.y, horizontalDistance) -- atan(上下, 水平距離)
    return { azimuth = currentLocalAzimuth, elevation = currentLocalElevation }
end

-- ローカル極座標からローカル直交座標へ
---@param dist number 距離
---@param localAziRad number 方位角(ラジアン)
---@param localEleRad number 仰角(ラジアン)
---@return Vector3 ローカル座標(x右方向, y上方向, z前方向)
function localAngleDistToLocalCoords(dist, localAziRad, localEleRad)
    local localX, localY, localZ
    localX = dist * math.cos(localEleRad) * math.sin(localAziRad)
    localY = dist * math.sin(localEleRad)
    localZ = dist * math.cos(localEleRad) * math.cos(localAziRad)
    return { x = localX, y = localY, z = localZ }
end

local sweepDataBuffer = {} -- 1スイープ分のデータ [targetIdx][tick] = {obs}

local ownWorldX, ownWorldY, ownWorldZ, ownEulerX, ownEulerY, ownEulerZ = 0, 0, 0, 0, 0, 0

function onTick()
    -- 0. 出力リセット (毎tickリセットし、t=3の時だけ出力する)
    for i = 1, 32 do output.setNumber(i, 0) end
    for i = 1, 32 do output.setBool(i, false) end

    -- 1. データ収集 (このTickの観測)
    local detectedTargetsThisTick = {}    -- このtickの生データ [targetIdx] = {obs}

    local sweepTime = input.getNumber(22) -- 経過時間

    local isRadarActive = false
    for i = 1, DETECTION_MAX do
        local isDetecting = input.getNumber(i * INPUT_CHANNEL_BASE - 2) > MIN_DISTANCE
        if isDetecting then
            isRadarActive = true
            detectedTargetsThisTick[i] = {
                distance = input.getNumber(i * INPUT_CHANNEL_BASE - 2),
                azimuthRad = input.getNumber(i * INPUT_CHANNEL_BASE - 1) * PI2, -- 回転単位をラジアンに
                elevationRad = input.getNumber(i * INPUT_CHANNEL_BASE) * PI2,   -- 回転単位をラジアンに
                sweepTime = sweepTime
            }
        end
    end

    -- 2. スイープ状態の管理 と データ蓄積
    if isRadarActive then
        if sweepTime == 0 then
            -- スイープ開始 (t=0) -> バッファをリセット
            sweepDataBuffer = {}
        end

        -- データをバッファに蓄積 (t=0, 1, 2, 3 すべて)
        for i, obs in pairs(detectedTargetsThisTick) do
            if sweepDataBuffer[i] == nil then
                sweepDataBuffer[i] = {}
            end
            -- sweepTime をキーとしてデータを格納
            sweepDataBuffer[i][obs.sweepTime] = obs
        end
    else
        -- レーダーOFF -> バッファをリセット
        sweepDataBuffer = {}
    end

    -- 3. スイープ完了時の処理 (t=3 を検知した瞬間に実行)
    -- (DETECTION_INTERVAL = 4 なので、完了は t=3)
    if isRadarActive and sweepTime == (DETECTION_INTERVAL - 1) then
        -- (t=3 のデータは既に 2. でバッファに追加済み)
        if next(sweepDataBuffer) == nil then return end -- バッファが空なら何もしない

        -- 4. ミッドレンジフィルター (時間軸の平滑化)
        -- (sweepDataBuffer には t=0, 1, 2, 3 のデータが揃っている)
        local filteredTargets = applyMidRangeFilter(sweepDataBuffer)

        -- 5. クラスタリング (空間軸の平滑化)
        local clusteredTargets = clusterTargets(filteredTargets)

        -- 6. オフセット適用 & 出力 (仕様: スイープ完了tickのみ出力)
        for i = 1, #clusteredTargets do
            if i > DETECTION_MAX then break end -- 8目標まで

            local target = clusteredTargets[i]

            -- (A) ローカル座標に変換
            local pos = localAngleDistToLocalCoords(target.distance, target.azimuthRad, target.elevationRad)

            -- (B) オフセットを加算
            pos.x = pos.x + OFFSET_RIGHT
            pos.y = pos.y + OFFSET_UP
            pos.z = pos.z + OFFSET_FORWARD

            -- (C) 角度に再変換
            local newDist = vectorMagnitude(pos)
            local newAngles = localCoordsToLocalAngle(pos)

            -- 出力 (ラジアン)
            output.setBool(i, true)
            output.setNumber(i * OUTPUT_CHANNEL_BASE - 2, newDist)
            output.setNumber(i * OUTPUT_CHANNEL_BASE - 1, newAngles.azimuth)
            output.setNumber(i * OUTPUT_CHANNEL_BASE, newAngles.elevation)
            -- 7. パススルー等の出力
            output.setNumber(25, ownWorldX)
            output.setNumber(26, ownWorldY)
            output.setNumber(27, ownWorldZ)
            output.setNumber(28, ownEulerX)
            output.setNumber(29, ownEulerY)
            output.setNumber(30, ownEulerZ)
        end
    end

    if sweepTime == 0 then
        ownWorldX = input.getNumber(25) -- EAST
        ownWorldY = input.getNumber(26) -- ALTITUDE
        ownWorldZ = input.getNumber(27) -- NORTH
        ownEulerX = input.getNumber(28) -- PITCH
        ownEulerY = input.getNumber(29) -- YAW
        ownEulerZ = input.getNumber(30) -- ROLL
    end



    local isTargetLock = input.getBool(TRACKMODE_ENABLE_CH)
    output.setBool(30, isTargetLock)

    local pilotSeatViewAzimuth_Turn = input.getNumber(31)
    local pilotSeatViewElevation_Turn = input.getNumber(32)
    output.setNumber(31, pilotSeatViewAzimuth_Turn)
    output.setNumber(32, pilotSeatViewElevation_Turn)
end
