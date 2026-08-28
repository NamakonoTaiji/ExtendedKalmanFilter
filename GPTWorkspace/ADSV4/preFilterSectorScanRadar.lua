-- 前向きに設置した走査レーダー1基分の4サンプル一次フィルター
-- 入力: 数値1～24=8目標*(距離,方位角turn,仰角turn)、25～30=自機位置姿勢、
-- 32=探知経過時間0～3（31は未使用）
-- 出力: 数値1～24=8目標*(距離,方位角rad,仰角rad)、25～30=探知時自機位置姿勢、
-- 31=レーダーID、32=観測tick
-- Bool1～8=目標有効、Bool31=有効フレーム

local TARGET_MAX = 8
local INTERVAL = 4
local PI2 = math.pi * 2

local MIN_DISTANCE = property.getNumber("MIN_DISTANCE")
local RADAR_ID = property.getNumber("RadarID")
local OFFSET_RIGHT = property.getNumber("OffsetRight")
local OFFSET_UP = property.getNumber("OffsetUp")
local OFFSET_FORWARD = property.getNumber("OffsetForward")
local MIN_SAMPLES = math.max(1, math.floor(property.getNumber("MinSamples")))

local samples = {}
local own = { 0, 0, 0, 0, 0, 0 }
local measurementTick = 0
local currentTick = 0

local function toXYZ(distance, azimuth, elevation)
    local ce = math.cos(elevation)
    return {
        x = distance * ce * math.sin(azimuth),
        y = distance * math.sin(elevation),
        z = distance * ce * math.cos(azimuth)
    }
end

local function toAngles(position)
    local horizontal = math.sqrt(position.x ^ 2 + position.z ^ 2)
    return math.atan(position.x, position.z), math.atan(position.y, horizontal)
end

local function magnitude(position)
    return math.sqrt(position.x ^ 2 + position.y ^ 2 + position.z ^ 2)
end

local function addSample(cluster, position)
    cluster.count = cluster.count + 1
    cluster.x[cluster.count] = position.x
    cluster.y[cluster.count] = position.y
    cluster.z[cluster.count] = position.z
    cluster.cx = cluster.cx + (position.x - cluster.cx) / cluster.count
    cluster.cy = cluster.cy + (position.y - cluster.cy) / cluster.count
    cluster.cz = cluster.cz + (position.z - cluster.cz) / cluster.count
end

local function midrange(values, count)
    local minimum, maximum = values[1], values[1]
    for i = 2, count do
        local value = values[i]
        if value < minimum then minimum = value end
        if value > maximum then maximum = value end
    end
    return (minimum + maximum) * 0.5
end

local function makeCluster(position)
    local cluster = {
        x = {}, y = {}, z = {}, count = 0,
        cx = position.x, cy = position.y, cz = position.z
    }
    addSample(cluster, position)
    return cluster
end

local function filterFrame()
    local clusters = {}

    for elapsed = 0, INTERVAL - 1 do
        local observations = samples[elapsed] or {}

        if #clusters == 0 then
            for _, position in ipairs(observations) do
                clusters[#clusters + 1] = makeCluster(position)
            end
        else
            local candidates = {}
            local usedClusters, usedObservations = {}, {}

            for clusterIndex, cluster in ipairs(clusters) do
                for observationIndex, position in ipairs(observations) do
                    local dx = position.x - cluster.cx
                    local dy = position.y - cluster.cy
                    local dz = position.z - cluster.cz
                    candidates[#candidates + 1] = {
                        distance2 = dx * dx + dy * dy + dz * dz,
                        cluster = clusterIndex,
                        observation = observationIndex
                    }
                end
            end

            table.sort(candidates, function(a, b) return a.distance2 < b.distance2 end)

            for _, candidate in ipairs(candidates) do
                local clusterIndex = candidate.cluster
                local observationIndex = candidate.observation
                if not usedClusters[clusterIndex] and not usedObservations[observationIndex] then
                    addSample(clusters[clusterIndex], observations[observationIndex])
                    usedClusters[clusterIndex] = true
                    usedObservations[observationIndex] = true
                end
            end
        end
    end

    local filtered = {}

    for _, cluster in ipairs(clusters) do
        if cluster.count >= MIN_SAMPLES then
            local position = {
                x = midrange(cluster.x, cluster.count),
                y = midrange(cluster.y, cluster.count),
                z = midrange(cluster.z, cluster.count)
            }
            position.x = position.x + OFFSET_RIGHT
            position.y = position.y + OFFSET_UP
            position.z = position.z + OFFSET_FORWARD
            local azimuth, elevation = toAngles(position)
            filtered[#filtered + 1] = {
                distance = magnitude(position),
                azimuth = azimuth,
                elevation = elevation
            }
        end
    end

    return filtered
end
function onTick()
    currentTick = currentTick + 1

    for channel = 1, 32 do output.setNumber(channel, 0) end
    for channel = 1, 32 do output.setBool(channel, false) end

    local elapsed = math.floor(input.getNumber(32) + 0.5)
    if elapsed < 0 then elapsed = 0 end
    if elapsed >= INTERVAL then elapsed = INTERVAL - 1 end

    if elapsed == 0 then
        samples = {}
        measurementTick = currentTick
        for channel = 1, 6 do own[channel] = input.getNumber(24 + channel) end
    end

    local observations = {}
    for target = 1, TARGET_MAX do
        local base = (target - 1) * 3
        local distance = input.getNumber(base + 1)
        if distance > MIN_DISTANCE then
            observations[#observations + 1] = toXYZ(
                distance,
                input.getNumber(base + 2) * PI2,
                input.getNumber(base + 3) * PI2
            )
        end
    end
    samples[elapsed] = observations

    if elapsed == INTERVAL - 1 and next(samples) then
        local filtered = filterFrame()
        local outputCount = math.min(#filtered, TARGET_MAX)

        for target = 1, outputCount do
            local base = (target - 1) * 3
            local observation = filtered[target]
            output.setNumber(base + 1, observation.distance)
            output.setNumber(base + 2, observation.azimuth)
            output.setNumber(base + 3, observation.elevation)
            output.setBool(target, true)
        end

        if outputCount > 0 then
            for channel = 1, 6 do output.setNumber(24 + channel, own[channel]) end
            output.setNumber(31, RADAR_ID)
            output.setNumber(32, measurementTick)
            output.setBool(31, true)
        end
    end
end
