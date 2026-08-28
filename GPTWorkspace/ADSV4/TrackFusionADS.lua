--[[
4基のセクターEKFが出力するトラックを、全周共通トラックへ統合する。

入力:
  Bool 1   : 選択中セクターEKFの目標有効信号
  Num 1～3 : 目標位置 X, Y, Z
  Num 4～6 : 目標速度 Vx, Vy, Vz
  Num 7～9 : 目標加速度 Ax, Ay, Az
  Num 10   : セクターEKFの最終観測tick
  Num 11   : セクターEKF出力時点の観測経過tick
  Num 12   : セクター内トラックID
  Num 13   : セクター内観測成功回数
  Num 15   : セクターID
  Num 18   : 入力位置・速度が対応する基準tick
  Num 32   : セクターEKFのイプシロンε

出力はADSV3 KalmanFilterADS.luaを踏襲する:
  Bool 1   : 目標を検出中
  Num 1～3 : 推定目標位置 X, Y, Z
  Num 4～6 : 推定目標速度 Vx, Vy, Vz
  Num 7～9 : 推定目標加速度 Ax, Ay, Az
  Num 10   : 統合器が新しい観測を受け取った最終tick
  Num 11   : 最終更新からの経過tick
  Num 12   : 全周共通トラックID
  Num 13   : 統合後の観測成功回数
  Num 32   : 最新のイプシロンε

前提:
  4基のセクターEKF出力をComposite Switchboxで1tickずつ順番に入力する。
  KalmanFilterSector.luaのOUTPUT_HOLD_TICKSは4にする。
]]

local DT = 1 / 60

local function propertyNumberOrDefault(name, defaultValue)
    local value = property.getNumber(name)
    return value > 0 and value or defaultValue
end

local POSITION_GATE = propertyNumberOrDefault("FUSE_POS_GATE", 1200)
local VELOCITY_GATE = propertyNumberOrDefault("FUSE_VEL_GATE", 800)
local GATE_GROWTH_PER_TICK = propertyNumberOrDefault("FUSE_GATE_GROWTH", 4)
local FUSION_GAIN = math.min(1, propertyNumberOrDefault("FUSE_GAIN", 0.5))
local TARGET_LOST_TICKS = math.max(30, propertyNumberOrDefault("T_LOST", 120))
local SOURCE_TIMEOUT_TICKS = math.max(
    TARGET_LOST_TICKS,
    propertyNumberOrDefault("SOURCE_TIMEOUT", 240)
)
local MERGE_INTERVAL_TICKS = math.max(
    1,
    math.floor(propertyNumberOrDefault("MERGE_INTERVAL", 4))
)
local LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
local SOURCE_TICK_OFFSET = property.getNumber("SOURCE_TICK_OFFSET")

local tracks = {}
local sourceToTrack = {}
local nextTrackID = 1
local currentTick = 0
local outputIndex = 0

local function isFinite(value)
    return value == value and value ~= math.huge and value ~= -math.huge
end

local function copyState(state)
    local result = {}
    for index = 1, 9 do result[index] = state[index] end
    return result
end

local function predictState(state, dt)
    local result = copyState(state)
    local halfDt2 = dt * dt * 0.5

    result[1] = state[1] + state[4] * dt + state[7] * halfDt2
    result[2] = state[2] + state[5] * dt + state[8] * halfDt2
    result[3] = state[3] + state[6] * dt + state[9] * halfDt2
    result[4] = state[4] + state[7] * dt
    result[5] = state[5] + state[8] * dt
    result[6] = state[6] + state[9] * dt

    return result
end

local function predictTrackTo(track, tick)
    if track.referenceTick ~= tick then
        track.state = predictState(track.state, (tick - track.referenceTick) * DT)
        track.referenceTick = tick
    end
end

local function stateDistances(first, second)
    local dx = first[1] - second[1]
    local dy = first[2] - second[2]
    local dz = first[3] - second[3]
    local dvx = first[4] - second[4]
    local dvy = first[5] - second[5]
    local dvz = first[6] - second[6]

    return
        math.sqrt(dx * dx + dy * dy + dz * dz),
        math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz)
end

local function makeSourceKey(sectorID, localTrackID)
    return sectorID .. ":" .. localTrackID
end

local function countSources(track)
    local count = 0
    for _ in pairs(track.sources) do count = count + 1 end
    return count
end

local function sectorIsAvailable(track, sectorID, sourceKey)
    local existingKey = track.sectorSources[sectorID]
    return existingKey == nil or existingKey == sourceKey
end

local function associationScore(track, state, sectorID, sourceKey, sourceMeasurementTick)
    if not sectorIsAvailable(track, sectorID, sourceKey) then return nil end

    local positionDistance, velocityDistance = stateDistances(track.state, state)
    local tickDifference = math.abs(track.latestMeasurementTick - sourceMeasurementTick)
    local positionGate = POSITION_GATE + tickDifference * GATE_GROWTH_PER_TICK

    if positionDistance > positionGate or velocityDistance > VELOCITY_GATE then return nil end

    return
        (positionDistance / positionGate) ^ 2 +
        (velocityDistance / VELOCITY_GATE) ^ 2
end


local function createTrack(state, source, epsilon)
    local trackID = nextTrackID
    nextTrackID = nextTrackID + 1

    local track = {
        id = trackID,
        state = copyState(state),
        referenceTick = currentTick,
        lastUpdateTick = currentTick,
        latestMeasurementTick = source.measurementTick,
        hits = math.max(1, source.hits),
        epsilon = epsilon,
        sources = {},
        sectorSources = {}
    }

    track.sources[source.key] = source
    track.sectorSources[source.sectorID] = source.key
    tracks[trackID] = track
    sourceToTrack[source.key] = trackID
    return track
end

local function findAssociation(state, source)
    local bestTrack, bestScore

    for _, track in pairs(tracks) do
        local score = associationScore(
            track,
            state,
            source.sectorID,
            source.key,
            source.measurementTick
        )
        if score and (bestScore == nil or score < bestScore) then
            bestTrack = track
            bestScore = score
        end
    end

    return bestTrack
end

local function blendTrackState(track, state, gain)
    for index = 1, 9 do
        track.state[index] = track.state[index] + (state[index] - track.state[index]) * gain
    end
end

local function updateTrack(track, state, source, epsilon)
    local previousSource = track.sources[source.key]
    local hitIncrease = 0

    if previousSource == nil then
        hitIncrease = math.max(1, source.hits)
    elseif source.hits > previousSource.hits then
        hitIncrease = source.hits - previousSource.hits
    elseif source.measurementTick > previousSource.measurementTick then
        -- 入力側のカウンターを再設定した場合でも新しい観測として扱う
        hitIncrease = 1
    end

    local sourceCount = countSources(track)
    local gain = FUSION_GAIN
    if sourceCount == 0 or (sourceCount == 1 and previousSource ~= nil) then
        gain = 1
    end
    blendTrackState(track, state, gain)

    track.sources[source.key] = source
    track.sectorSources[source.sectorID] = source.key
    sourceToTrack[source.key] = track.id
    track.latestMeasurementTick = math.max(track.latestMeasurementTick, source.measurementTick)
    track.epsilon = epsilon

    if hitIncrease > 0 then
        track.hits = track.hits + hitIncrease
        track.lastUpdateTick = currentTick
    end
end

local function sourcesAreCompatible(first, second)
    for sectorID, sourceKey in pairs(first.sectorSources) do
        local otherKey = second.sectorSources[sectorID]
        if otherKey ~= nil and otherKey ~= sourceKey then return false end
    end
    return true
end

local function mergeTracks(keep, remove)
    local firstWeight = math.max(1, countSources(keep))
    local secondWeight = math.max(1, countSources(remove))
    local totalWeight = firstWeight + secondWeight

    for index = 1, 9 do
        keep.state[index] = (
            keep.state[index] * firstWeight + remove.state[index] * secondWeight
        ) / totalWeight
    end

    for sourceKey, source in pairs(remove.sources) do
        keep.sources[sourceKey] = source
        keep.sectorSources[source.sectorID] = sourceKey
        sourceToTrack[sourceKey] = keep.id
    end

    keep.hits = keep.hits + remove.hits
    keep.lastUpdateTick = math.max(keep.lastUpdateTick, remove.lastUpdateTick)
    keep.latestMeasurementTick = math.max(keep.latestMeasurementTick, remove.latestMeasurementTick)
    keep.epsilon = math.min(keep.epsilon, remove.epsilon)
    tracks[remove.id] = nil
end

local function mergeDuplicateTracks()
    local sorted = {}
    for _, track in pairs(tracks) do sorted[#sorted + 1] = track end
    table.sort(sorted, function(first, second) return first.id < second.id end)

    for firstIndex = 1, #sorted - 1 do
        local first = sorted[firstIndex]
        if tracks[first.id] then
            for secondIndex = firstIndex + 1, #sorted do
                local second = sorted[secondIndex]
                if tracks[second.id] and sourcesAreCompatible(first, second) then
                    local positionDistance, velocityDistance = stateDistances(first.state, second.state)
                    if positionDistance <= POSITION_GATE and velocityDistance <= VELOCITY_GATE then
                        mergeTracks(first, second)
                    end
                end
            end
        end
    end
end

local function removeTrack(trackID)
    local track = tracks[trackID]
    if track == nil then return end

    for sourceKey in pairs(track.sources) do
        if sourceToTrack[sourceKey] == trackID then sourceToTrack[sourceKey] = nil end
    end
    tracks[trackID] = nil
end

local function pruneSourcesAndTracks()
    local deleteTrackIDs = {}

    for trackID, track in pairs(tracks) do
        local deleteSourceKeys = {}
        for sourceKey, source in pairs(track.sources) do
            if currentTick - source.lastReceivedTick > SOURCE_TIMEOUT_TICKS then
                deleteSourceKeys[#deleteSourceKeys + 1] = sourceKey
            end
        end

        for _, sourceKey in ipairs(deleteSourceKeys) do
            local source = track.sources[sourceKey]
            track.sources[sourceKey] = nil
            if source then track.sectorSources[source.sectorID] = nil end
            if sourceToTrack[sourceKey] == trackID then sourceToTrack[sourceKey] = nil end
        end

        if currentTick - track.lastUpdateTick > TARGET_LOST_TICKS or next(track.sources) == nil then
            deleteTrackIDs[#deleteTrackIDs + 1] = trackID
        end
    end

    for _, trackID in ipairs(deleteTrackIDs) do removeTrack(trackID) end
end

local function readInputTrack()
    if not input.getBool(1) then return nil, nil end

    local sectorID = math.floor(input.getNumber(15) + 0.5)
    local localTrackID = math.floor(input.getNumber(12) + 0.5)
    if sectorID <= 0 or localTrackID <= 0 then return nil, nil end

    local state = {
        input.getNumber(1), input.getNumber(2), input.getNumber(3),
        input.getNumber(4), input.getNumber(5), input.getNumber(6),
        input.getNumber(7), input.getNumber(8), input.getNumber(9)
    }
    for index = 1, 9 do
        if not isFinite(state[index]) then return nil, nil end
    end

    local measurementTick = math.floor(input.getNumber(10) + SOURCE_TICK_OFFSET + 0.5)
    local referenceTick = input.getNumber(18)
    if referenceTick <= 0 then
        referenceTick = input.getNumber(10) + input.getNumber(11)
    end
    referenceTick = referenceTick + SOURCE_TICK_OFFSET

    state = predictState(state, (currentTick - referenceTick) * DT)

    local sourceKey = makeSourceKey(sectorID, localTrackID)
    local source = {
        key = sourceKey,
        sectorID = sectorID,
        localTrackID = localTrackID,
        measurementTick = measurementTick,
        hits = math.max(1, math.floor(input.getNumber(13) + 0.5)),
        lastReceivedTick = currentTick,
        referenceTick = currentTick
    }
    local epsilon = input.getNumber(32)
    if not isFinite(epsilon) then epsilon = 0 end

    return { state = state, source = source }, epsilon
end

local function processInputTrack()
    local inputTrack, epsilon = readInputTrack()
    if inputTrack == nil then return end

    local source = inputTrack.source
    local trackID = sourceToTrack[source.key]
    local track = trackID and tracks[trackID] or nil

    if track == nil then track = findAssociation(inputTrack.state, source) end

    if track then
        updateTrack(track, inputTrack.state, source, epsilon)
    else
        createTrack(inputTrack.state, source, epsilon)
    end
end

local function selectOutputTrack()
    local sorted = {}
    for _, track in pairs(tracks) do
        if track.hits > 1 then sorted[#sorted + 1] = track end
    end
    table.sort(sorted, function(first, second) return first.id < second.id end)

    if #sorted == 0 then
        outputIndex = 0
        return nil
    end

    outputIndex = outputIndex + 1
    if outputIndex > #sorted then outputIndex = 1 end
    return sorted[outputIndex]
end

local function writeOutput(track)
    if track == nil then return end

    local outputState = predictState(track.state, LOGIC_DELAY * DT)
    local detectionTickLag = currentTick - track.lastUpdateTick

    for channel = 1, 9 do output.setNumber(channel, outputState[channel]) end
    output.setNumber(10, track.lastUpdateTick)
    output.setNumber(11, detectionTickLag)
    output.setNumber(12, track.id)
    output.setNumber(13, track.hits)
    output.setNumber(32, track.epsilon)
    output.setBool(1, true)
end

function onTick()
    currentTick = currentTick + 1

    for channel = 1, 32 do
        output.setNumber(channel, 0)
        output.setBool(channel, false)
    end

    for _, track in pairs(tracks) do predictTrackTo(track, currentTick) end
    processInputTrack()

    if currentTick % MERGE_INTERVAL_TICKS == 0 then mergeDuplicateTracks() end
    pruneSourcesAndTracks()
    writeOutput(selectOutputTrack())
end
