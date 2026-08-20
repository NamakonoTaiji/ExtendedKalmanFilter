---------------------------------------------------------
--- 一次フィルタースクリプト
---------------------------------------------------------


-- カルマンフィルターに送る情報に一次処理を行うスクリプト
-- 仕様 --
-- * ミッドレンジフィルターによりノイズを除去
-- * 4tick分の観測をローカル直交座標でミッドレンジ平滑化する
-- * 同一レーダーが返す近接目標同士は統合せず、別目標としてカルマンフィルターへ渡す
-- * ローカル座標での変換を行い物理センサ基準での観測にする
-- * 最後に座標変換を行った距離、方位角、仰角を出力
-- * 出力は4tick目のみ行い他のtickは常に0を出力
-- * 4レーダ分にそれぞれこのスクリプトを適用し、compositeSwitchBoxによる出力タイミング制御は他のスクリプトで行う
-- * Radar1の一次フィルターはcompositeSwitchBoxの制御も兼任する
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
-- これを最大7目標分21chまで繰り返す
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
local OFFSET_UP = 0.5
local OFFSET_FORWARD = 0
local PI = math.pi
local PI2 = PI * 2
local MIN_DISTANCE = property.getNumber("MIN_DISTANCE")
local RADAR_ANGLE_OFFSET = 0.25

--------------------------------------------------------------------------------
-- ヘルパー関数 (ベクトル演算)
--------------------------------------------------------------------------------

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

function addSweepSample(cluster, pos)
    table.insert(cluster.x, pos.x)
    table.insert(cluster.y, pos.y)
    table.insert(cluster.z, pos.z)
    local n = #cluster.x
    cluster.cx = cluster.cx + (pos.x - cluster.cx) / n
    cluster.cy = cluster.cy + (pos.y - cluster.cy) / n
    cluster.cz = cluster.cz + (pos.z - cluster.cz) / n
end

--- 時間軸の平滑化 (ローカル直交座標のミッドレンジフィルター)
-- 角度境界を跨ぐ観測でも破綻しないよう、極座標をXYZに変換してから平滑化する
-- 最初のtickの目標数を維持した一対一対応とし、高速目標を別クラスタへ分裂させない
---@param sweepBuffer table [targetIdx][tick] = {obs}
---@return table フィルター後のリスト { distance, azimuthRad, elevationRad }
function applyMidRangeFilter(sweepBuffer)
    local observationsByTick, clusters = {}, {}
    for targetIdx = 1, DETECTION_MAX do
        local tickData = sweepBuffer[targetIdx]
        if tickData then
            for tick, obs in pairs(tickData) do
                local pos = localAngleDistToLocalCoords(obs.distance, obs.azimuthRad, obs.elevationRad)
                observationsByTick[tick] = observationsByTick[tick] or {}
                table.insert(observationsByTick[tick], pos)
            end
        end
    end

    for tick = 0, DETECTION_INTERVAL - 1 do
        local observations = observationsByTick[tick] or {}
        if #clusters == 0 then
            for _, pos in ipairs(observations) do
                local cluster = { x = {}, y = {}, z = {}, cx = pos.x, cy = pos.y, cz = pos.z }
                addSweepSample(cluster, pos)
                table.insert(clusters, cluster)
            end
        else
            local candidates, usedClusters, usedObservations = {}, {}, {}
            for clusterIndex, cluster in ipairs(clusters) do
                for observationIndex, pos in ipairs(observations) do
                    local dx, dy, dz = pos.x - cluster.cx, pos.y - cluster.cy, pos.z - cluster.cz
                    table.insert(candidates, { dx ^ 2 + dy ^ 2 + dz ^ 2, clusterIndex, observationIndex })
                end
            end
            table.sort(candidates, function(a, b) return a[1] < b[1] end)
            for _, candidate in ipairs(candidates) do
                local clusterIndex, observationIndex = candidate[2], candidate[3]
                if not usedClusters[clusterIndex] and not usedObservations[observationIndex] then
                    addSweepSample(clusters[clusterIndex], observations[observationIndex])
                    usedClusters[clusterIndex] = true
                    usedObservations[observationIndex] = true
                end
            end
        end
    end

    local filteredTargets = {}
    for _, cluster in ipairs(clusters) do
        local pos = { x = maxMin(cluster.x), y = maxMin(cluster.y), z = maxMin(cluster.z) }
        local angles = localCoordsToLocalAngle(pos)
        table.insert(filteredTargets, {
            distance = vectorMagnitude(pos),
            azimuthRad = angles.azimuth,
            elevationRad = angles.elevation
        })
    end
    return filteredTargets
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
    for i = 1, DETECTION_MAX * INPUT_CHANNEL_BASE do output.setNumber(i, 0) end
    for i = 1, DETECTION_MAX do output.setBool(i, false) end

    -- 1. データ収集 (このTickの観測)
    local detectedTargetsThisTick = {}    -- このtickの生データ [targetIdx] = {obs}

    local sweepTime = input.getNumber(22) -- 経過時間

    local isRadarActive = false
    for i = 1, DETECTION_MAX do
        local isDetecting = input.getNumber(i * INPUT_CHANNEL_BASE - 2) > MIN_DISTANCE
        if isDetecting then
            isRadarActive = true
            local azimuthRaw = ((input.getNumber(i * INPUT_CHANNEL_BASE - 1) + RADAR_ANGLE_OFFSET))
            if azimuthRaw > 0.5 then azimuthRaw = azimuthRaw - 1 end
            detectedTargetsThisTick[i] = {
                distance = input.getNumber(i * INPUT_CHANNEL_BASE - 2),
                azimuthRad = azimuthRaw * PI2,
                elevationRad = input.getNumber(i * INPUT_CHANNEL_BASE) * PI2, -- 回転単位をラジアンに
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

        -- 5. オフセット適用 & 出力 (仕様: スイープ完了tickのみ出力)
        for i = 1, #filteredTargets do
            if i > DETECTION_MAX then break end -- 7目標まで

            local target = filteredTargets[i]

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
            -- 6. パススルー等の出力
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



    local isTargetLock = input.getBool(31)
    output.setBool(30, isTargetLock)

    local pilotSeatViewAzimuth_Turn = input.getNumber(31)
    local pilotSeatViewElevation_Turn = input.getNumber(32)
    output.setNumber(31, pilotSeatViewAzimuth_Turn)
    output.setNumber(32, pilotSeatViewElevation_Turn)
end
