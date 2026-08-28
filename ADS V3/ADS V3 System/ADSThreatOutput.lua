---KalmanFilterから時分割で受け取った目標情報を選別し、VLSの発射とデータリンクを管理する。
---脅威条件を満たした目標を発射待ちキューへ登録し、ハッチ開放完了後に順次射出する。
--[[
入力:
- bool 1..32: 各VLSセルのハッチ開放完了信号
- num 1-3: 対空目標位置 (X, Y, Z)
- num 4-6: 対空目標速度 (Vx, Vy, Vz)
- num 7: 対空目標ID
- num 9: 目標情報更新フラグ (1 = 更新)
- num 11: 最終観測からの経過Tick数
- num 12: 連続同定回数
- num 13: 飛行中ミサイルから返信されたミサイルID
- num 15-17: 対水上目標位置 (X, Y, Z)
- num 18: 対水上目標ID
- num 19: 対水上発射要求 (1 = On)
- num 27-29: 自機位置 (X, Y, Z)

出力:
- bool 1..32: 各VLSセルの射出パルス（1tick）
- num 1-6: 発射目標の位置・速度 (X, Y, Z, Vx, Vy, Vz)
- num 12: 目標ID
- num 13: ミサイルID
- num 14: ミサイル専用周波数
- num 15: 最終観測からの経過Tick数
- num 16: ハッチ開放ビットマスク（bit 0 = セル1）
- num 29: チャフ射出要求（1 = 1km以内に接近中の脅威あり、0 = なし）
- num 30: 返信を受信する周波数
- num 31: ミサイルが使用するデータリンク周波数
- num 32: 発射管制接続識別信号
]]

local MAX_VLS_CELLS = 32
local AIR_MODE = "air"
local ANTI_SHIP_MODE = "antiShip"
local TICKS_PER_SECOND = 60
local CHAFF_OUTPUT_CHANNEL = 29

local function propertyNumberOrDefault(name, defaultValue)
    local value = property.getNumber(name)
    return value > 0 and value or defaultValue
end

local INTERCEPT_THREAT_SPEED = property.getNumber("INTERCEPT_THREAT_SPEED")
-- XZ平面で「こちらを向いている」と判定する角度のコサイン閾値。
-- 0.0 = 90度以内、0.5 = 60度以内、0.866 = 30度以内。
local THREAT_FACING_THRESHOLD = property.getNumber("THREAT_FACING_THRESHOLD")
local THREAT_HITS_THRESHOLD = property.getNumber("THREAT_HITS_THRESHOLD")
local MISSILE_TIMEOUT_TICKS = propertyNumberOrDefault("MISSILE_TIMEOUT_TICKS", 120)
local MISSILE_INITIAL_CONTACT_TIMEOUT_TICKS = propertyNumberOrDefault("MISSILE_INITIAL_CONTACT_TIMEOUT_TICKS", 300)
local LAUNCH_CANCEL_TICKS = propertyNumberOrDefault("LAUNCH_CANCEL_TICKS", 30)
local DATA_PRELOAD_TICKS = propertyNumberOrDefault("DATA_PRELOAD_TICKS", 5)
local DATA_POSTLAUNCH_HOLD_TICKS = propertyNumberOrDefault("DATA_POSTLAUNCH_HOLD_TICKS", 15)
local CHAFF_TRIGGER_DISTANCE = propertyNumberOrDefault("CHAFF_TRIGGER_DISTANCE", 1000)
local CHAFF_TRACK_TIMEOUT_TICKS = propertyNumberOrDefault("CHAFF_TRACK_TIMEOUT_TICKS", 30)
local BASE_FREQUENCY = property.getNumber("BASE_FREQUENCY")
local ADS_SEND_FREQ = property.getNumber("AirDefenceSystemFreq")
local ANTI_SHIP_FREQ = math.floor(ADS_SEND_FREQ / 2)

local nextMissileID = 1
local launchQueue = {}
local queuedThreatTargets = {} -- 発射待ち中の対空目標ID
local activeMissiles = {}      -- [missileID] = { targetID, freq, launchTick, lastSeenTick, hasBeenSeen }
local threatTargetsID = {}     -- [targetID] = missileID
local threatHitStates = {}     -- [targetID] = { count, lastKalmanHits }
local trackedChaffThreats = {} -- 接近中の脅威 [targetID] = { position, velocity, lastUpdateTick }
local currentRotIndex = 1
local currentTick = 0
local previousAntiShipRequest = false

local function hasAvailableCell()
    return nextMissileID + #launchQueue <= MAX_VLS_CELLS
end

-- KalmanFilter側のhitsが増えた時だけ、脅威条件の連続成立回数を更新する。
local function updateThreatHitCount(targetID, kalmanHits, meetsThreatCondition)
    local state = threatHitStates[targetID]
    if state == nil then
        state = { count = 0, lastKalmanHits = 0 }
        threatHitStates[targetID] = state
    elseif kalmanHits < state.lastKalmanHits then
        state.count = 0
        state.lastKalmanHits = 0
    end

    if kalmanHits > state.lastKalmanHits then
        state.count = meetsThreatCondition and state.count + 1 or 0
    end

    state.lastKalmanHits = kalmanHits
    return state.count
end

local function isIncomingThreat(target, ownPosition)
    local speed = math.sqrt(target.vX ^ 2 + target.vY ^ 2 + target.vZ ^ 2)
    if speed < INTERCEPT_THREAT_SPEED then
        return false
    end

    local horizontalSpeed = math.sqrt(target.vX ^ 2 + target.vZ ^ 2)
    local toOwnX = ownPosition.x - target.x
    local toOwnZ = ownPosition.z - target.z
    local horizontalDistance = math.sqrt(toOwnX ^ 2 + toOwnZ ^ 2)
    if horizontalSpeed <= 0.1 or horizontalDistance <= 0.1 then
        return false
    end

    local dot = target.vX * toOwnX + target.vZ * toOwnZ
    return dot / (horizontalSpeed * horizontalDistance) >= THREAT_FACING_THRESHOLD
end

local function updateTrackedChaffThreat(target, isThreat)
    if not isThreat then
        trackedChaffThreats[target.id] = nil
        return
    end

    trackedChaffThreats[target.id] = {
        x = target.x,
        y = target.y,
        z = target.z,
        vX = target.vX,
        vY = target.vY,
        vZ = target.vZ,
        lastUpdateTick = currentTick
    }
end

local function outputChaffRequest()
    local ownX = input.getNumber(27)
    local ownY = input.getNumber(28)
    local ownZ = input.getNumber(29)
    local nearestDistance = math.huge

    for targetID, threat in pairs(trackedChaffThreats) do
        local ageTicks = currentTick - threat.lastUpdateTick
        if ageTicks > CHAFF_TRACK_TIMEOUT_TICKS then
            trackedChaffThreats[targetID] = nil
        else
            -- 時分割受信の合間は、最後の位置と速度から現在位置を予測する。
            local elapsedSeconds = ageTicks / TICKS_PER_SECOND
            local targetX = threat.x + threat.vX * elapsedSeconds
            local targetY = threat.y + threat.vY * elapsedSeconds
            local targetZ = threat.z + threat.vZ * elapsedSeconds
            local dx = targetX - ownX
            local dy = targetY - ownY
            local dz = targetZ - ownZ
            local distance = math.sqrt(dx ^ 2 + dy ^ 2 + dz ^ 2)
            if distance < nearestDistance then
                nearestDistance = distance
            end
        end
    end

    output.setNumber(CHAFF_OUTPUT_CHANNEL, nearestDistance <= CHAFF_TRIGGER_DISTANCE and 1 or 0)
end

local function copyTargetState(launch, target)
    launch.x = target.x
    launch.y = target.y
    launch.z = target.z
    launch.vX = target.vX or 0
    launch.vY = target.vY or 0
    launch.vZ = target.vZ or 0
    launch.detectionTickLag = target.detectionTickLag or 0
end

local function enqueueLaunch(mode, target)
    local launch = {
        mode = mode,
        targetID = target.id
    }
    copyTargetState(launch, target)

    if mode == AIR_MODE then
        launch.lastTargetConfirmedTick = currentTick
        queuedThreatTargets[target.id] = true
    end

    table.insert(launchQueue, launch)
end

local function updateQueuedAirLaunch(target)
    for _, launch in ipairs(launchQueue) do
        if launch.mode == AIR_MODE and launch.targetID == target.id then
            copyTargetState(launch, target)
            launch.lastTargetConfirmedTick = currentTick
        end
    end
end

-- 対水上要求は立ち上がり時だけ受け付ける。
local function readNewAntiShipTarget()
    local targetID = input.getNumber(18)
    local request = input.getNumber(19) == 1 and targetID ~= 0
    local isNewRequest = request and not previousAntiShipRequest
    previousAntiShipRequest = request

    if not isNewRequest then
        return nil
    end

    -- 発射管制由来を示す+100000と、対水上を示す+90000。
    return {
        id = targetID + 190000,
        x = input.getNumber(15),
        y = input.getNumber(16),
        z = input.getNumber(17),
        vX = 0,
        vY = 0,
        vZ = 0,
        detectionTickLag = 0
    }
end

local function processAirTarget()
    if input.getNumber(9) ~= 1 then
        return
    end

    local target = {
        id = input.getNumber(7) + 100000,
        x = input.getNumber(1),
        y = input.getNumber(2),
        z = input.getNumber(3),
        vX = input.getNumber(4),
        vY = input.getNumber(5),
        vZ = input.getNumber(6),
        detectionTickLag = input.getNumber(11),
        hits = input.getNumber(12)
    }
    local ownPosition = {
        x = input.getNumber(27),
        y = input.getNumber(28),
        z = input.getNumber(29)
    }
    local isThreat = isIncomingThreat(target, ownPosition)
    local threatHits = updateThreatHitCount(target.id, target.hits, isThreat)
    updateTrackedChaffThreat(target, isThreat)

    -- キューに登録済みの目標は、射出まで最新状態へ更新する。
    updateQueuedAirLaunch(target)

    if hasAvailableCell() and isThreat and target.detectionTickLag == 1 and
        threatHits >= THREAT_HITS_THRESHOLD and threatTargetsID[target.id] == nil and
        queuedThreatTargets[target.id] == nil then
        enqueueLaunch(AIR_MODE, target)
    end
end

local function updateMissileReply()
    local missileID = input.getNumber(13)
    local missile = activeMissiles[missileID]
    if missileID > 0 and missile then
        missile.lastSeenTick = currentTick
        missile.hasBeenSeen = true
    end
end

local function outputListeningFrequency()
    local frequencies = {}
    for _, missile in pairs(activeMissiles) do
        table.insert(frequencies, missile.freq)
    end

    local listeningFrequency = 0
    if #frequencies > 0 then
        currentRotIndex = (currentRotIndex % #frequencies) + 1
        listeningFrequency = frequencies[currentRotIndex]
    else
        currentRotIndex = 0
    end
    output.setNumber(30, listeningFrequency)
end

local function removeTimedOutMissiles()
    for missileID, missile in pairs(activeMissiles) do
        local timeoutTicks = missile.hasBeenSeen and MISSILE_TIMEOUT_TICKS or MISSILE_INITIAL_CONTACT_TIMEOUT_TICKS
        local timeoutBaseTick = missile.hasBeenSeen and missile.lastSeenTick or missile.launchTick
        if currentTick - timeoutBaseTick > timeoutTicks then
            threatTargetsID[missile.targetID] = nil
            activeMissiles[missileID] = nil
        end
    end
end

local function cancelLostQueuedTargets()
    local previousQueueHead = launchQueue[1]
    for index = #launchQueue, 1, -1 do
        local launch = launchQueue[index]
        local isLostAirTarget = not launch.fired and launch.mode == AIR_MODE and
            currentTick - launch.lastTargetConfirmedTick > LAUNCH_CANCEL_TICKS
        if isLostAirTarget then
            queuedThreatTargets[launch.targetID] = nil
            table.remove(launchQueue, index)
        end
    end
    return previousQueueHead
end

-- キュー順を物理セル番号、ミサイルID、専用周波数へ対応させる。
local function assignLaunchCells()
    for index, launch in ipairs(launchQueue) do
        local missileID = nextMissileID + index - 1
        launch.cellID = missileID
        launch.missileID = missileID
        launch.freq = BASE_FREQUENCY + missileID
    end
end

local function buildHatchOpenMask()
    local mask = 0
    for _, launch in ipairs(launchQueue) do
        if launch.cellID >= 1 and launch.cellID <= MAX_VLS_CELLS then
            mask = mask + 2 ^ (launch.cellID - 1)
        end
    end
    return mask
end

local function prepareLaunchQueue()
    local previousQueueHead = cancelLostQueuedTargets()
    assignLaunchCells()

    local queueHead = launchQueue[1]
    if queueHead and (queueHead ~= previousQueueHead or queueHead.dataOutputStartTick == nil) then
        queueHead.dataOutputStartTick = currentTick
    end

    output.setNumber(16, buildHatchOpenMask())
end

local function outputLaunchData(launch)
    output.setNumber(1, launch.x)
    output.setNumber(2, launch.y)
    output.setNumber(3, launch.z)
    output.setNumber(4, launch.vX)
    output.setNumber(5, launch.vY)
    output.setNumber(6, launch.vZ)
    output.setNumber(12, launch.targetID)
    output.setNumber(13, launch.missileID)
    output.setNumber(14, launch.freq)
    output.setNumber(15, launch.detectionTickLag)
    output.setNumber(31, launch.mode == AIR_MODE and ADS_SEND_FREQ or ANTI_SHIP_FREQ)
end

local function fireLaunch(launch)
    output.setBool(launch.cellID, true)

    if launch.mode == AIR_MODE then
        threatTargetsID[launch.targetID] = launch.missileID
        activeMissiles[launch.missileID] = {
            targetID = launch.targetID,
            freq = launch.freq,
            launchTick = currentTick,
            lastSeenTick = currentTick,
            hasBeenSeen = false
        }
    end

    queuedThreatTargets[launch.targetID] = nil
    launch.fired = true
    launch.postLaunchDataHoldEndTick = currentTick + DATA_POSTLAUNCH_HOLD_TICKS
end

local function releaseHeldLaunch(launch)
    nextMissileID = launch.missileID + 1
    table.remove(launchQueue, 1)
    if launchQueue[1] then
        launchQueue[1].dataOutputStartTick = currentTick + 1
    end
end

local function processQueueHead()
    local launch = launchQueue[1]
    if not launch then
        return
    end

    outputLaunchData(launch)

    if launch.fired then
        if currentTick >= launch.postLaunchDataHoldEndTick then
            releaseHeldLaunch(launch)
        end
        return
    end

    local isDataPreloaded = currentTick - launch.dataOutputStartTick >= DATA_PRELOAD_TICKS
    if isDataPreloaded and input.getBool(launch.cellID) then
        fireLaunch(launch)
    end
end

function onTick()
    currentTick = currentTick + 1

    -- 射出信号は必ず1tickパルスにする。
    for channel = 1, MAX_VLS_CELLS do
        output.setBool(channel, false)
    end

    updateMissileReply()
    outputListeningFrequency()
    removeTimedOutMissiles()

    -- 対空を先に登録することで、残り1セルの場合も従来どおり対空を優先する。
    local antiShipTarget = readNewAntiShipTarget()
    processAirTarget()
    if antiShipTarget and hasAvailableCell() then
        enqueueLaunch(ANTI_SHIP_MODE, antiShipTarget)
    end

    prepareLaunchQueue()
    processQueueHead()
    outputChaffRequest()

    -- ミサイル側が発射管制との接続を識別するための固定値。
    output.setNumber(32, 114514)
end
