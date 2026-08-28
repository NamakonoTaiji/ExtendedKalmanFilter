--[[
HGV対艦ミサイル用のレーダー走査・脅威追跡スクリプト。
ADSV3 MissileKF.luaのデータリンク、座標変換、追跡構成をHGV用途へ分離したもの。

入力 number:
  1-18 : レーダー目標1-6（各3ch: 距離m、方位turns、仰角turns）
 19-21 : データリンク敵艦速度 Vx,Vy,Vz (m/s)
 22-24 : データリンク敵艦位置 X,Y,Z (m)
 25-27 : 自機位置 X,Y,Z (m)
 28-30 : 自機姿勢 Pitch,Yaw,Roll (rad)
     31 : データリンク経過時間（予約）
     32 : データリンク目標ID
入力 bool 1: 既存レーダー検出信号（bool 32へパススルー）
入力 bool 2: シースキミングモード（出力bool 5へパススルー）

出力 number:
  1-3  : 敵艦位置 X,Y,Z (m)
  4-6  : 敵艦速度 Vx,Vy,Vz (m/s)
  7-9  : 有効SAM位置 X,Y,Z (m、無効時0)
 10-11 : レーダー指向方位・仰角 (turns)
 12    : 有効SAMトラックID（無効時0）
 13-15 : 自機位置パススルー
 16-19 : 自機姿勢クォータニオン w,x,y,z
 20-22 : 有効SAM速度 Vx,Vy,Vz (m/s、無効時0)
 23    : フェーズ 0=待機,1=SEARCH,2=CANDIDATE,3=EVADE,4=TERMINAL
出力 bool:
  1: 有効SAMあり  2: 対水上モード  3: 発射済み
  4: TERMINALラッチ  5: シースキミングモード  32: 入力bool 1パススルー

レーダー出力角は指向ビーム基準ではなくレーダー筐体正面基準である。
角度入力と出力はturns、内部計算はradを使用する。
]]

DT = 1 / 60
TAU = math.pi * 2
TERMINAL_HORIZONTAL_RANGE = property.getNumber("TERMINAL_HORIZONTAL_RANGE")
SEA_SKIM_POPUP_RANGE_M = property.getNumber("SEA_SKIM_POPUP_RANGE")
SAM_MIN_SPEED = 250
RADAR_EFFECTIVE_RANGE = 1900
SCAN_OFFSETS = { -20 / 360, 0, 20 / 360 }

TRACK_GATE_M = property.getNumber("TRACK_GATE_M")
if TRACK_GATE_M <= 0 then TRACK_GATE_M = 150 end
DL_GATE_M = property.getNumber("DL_GATE")
if DL_GATE_M <= 0 then DL_GATE_M = 200 end
TRACK_DELETE_TICKS = property.getNumber("TRACK_DELETE_TICKS")
if TRACK_DELETE_TICKS <= 0 then TRACK_DELETE_TICKS = 60 end
THREAT_LOST_TICKS = property.getNumber("THREAT_LOST_TICKS")
if THREAT_LOST_TICKS <= 0 then THREAT_LOST_TICKS = 3 end
CANDIDATE_MIN_HITS = property.getNumber("CANDIDATE_MIN_HITS")
if CANDIDATE_MIN_HITS <= 0 then CANDIDATE_MIN_HITS = 12 end
CANDIDATE_MAX_TICKS = property.getNumber("CANDIDATE_MAX_TICKS")
if CANDIDATE_MAX_TICKS <= 0 then CANDIDATE_MAX_TICKS = 45 end

tick = 0
tracks = {}
nextTrackID = 1
dataLinkID = nil
hasDataLink = false
launched = false
shipPos = { 0, 0, 0 }
shipVel = { 0, 0, 0 }
previousOwnPos = nil
ownVel = { 0, 0, 0 }
scanIndex = 1
candidateID = nil
candidateStartTick = 0
activeThreatID = nil
threatWasClosing = false
terminalLatched = false
lastPhase = -1

function magnitude(v)
    return math.sqrt(v[1] * v[1] + v[2] * v[2] + v[3] * v[3])
end

function distance(a, b)
    local x, y, z = a[1] - b[1], a[2] - b[2], a[3] - b[3]
    return math.sqrt(x * x + y * y + z * z)
end

function dot(a, b)
    return a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
end

function eulerZYXToQuaternion(roll, yaw, pitch)
    local cr, sr = math.cos(roll * .5), math.sin(roll * .5)
    local cy, sy = math.cos(yaw * .5), math.sin(yaw * .5)
    local cp, sp = math.cos(pitch * .5), math.sin(pitch * .5)
    return {
        cr * cy * cp + sr * sy * sp,
        cr * cy * sp - sr * sy * cp,
        cr * sy * cp + sr * cy * sp,
        sr * cy * cp - cr * sy * sp
    }
end

function rotateByQuaternion(v, q, inverse)
    local w, x, y, z = q[1], q[2], q[3], q[4]
    if inverse then w = -w end
    local tx = 2 * (y * v[3] - z * v[2])
    local ty = 2 * (z * v[1] - x * v[3])
    local tz = 2 * (x * v[2] - y * v[1])
    return {
        v[1] + w * tx + y * tz - z * ty,
        v[2] + w * ty + z * tx - x * tz,
        v[3] + w * tz + x * ty - y * tx
    }
end

function localToGlobal(localPos, ownPos, q)
    local r = rotateByQuaternion(localPos, q, false)
    return { ownPos[1] + r[1], ownPos[2] + r[2], ownPos[3] + r[3] }
end

function globalToLocal(globalPos, ownPos, q)
    return rotateByQuaternion({
        globalPos[1] - ownPos[1],
        globalPos[2] - ownPos[2],
        globalPos[3] - ownPos[3]
    }, q, true)
end

function localToAngles(v)
    local horizontal = math.sqrt(v[1] * v[1] + v[3] * v[3])
    return math.atan(v[1], v[3]) / TAU, math.atan(v[2], horizontal) / TAU
end

function polarToLocal(dist, azTurns, elTurns)
    local az, el = azTurns * TAU, elTurns * TAU
    return {
        dist * math.cos(el) * math.sin(az),
        dist * math.sin(el),
        dist * math.cos(el) * math.cos(az)
    }
end

function wrapTurns(v)
    while v > .5 do v = v - 1 end
    while v <= -.5 do v = v + 1 end
    return v
end

function updateTrackSet(observations)
    local predictions, pairsList = {}, {}
    for id, track in pairs(tracks) do
        predictions[id] = {
            track.p[1] + track.v[1] * DT,
            track.p[2] + track.v[2] * DT,
            track.p[3] + track.v[3] * DT
        }
        for obsIndex, observation in ipairs(observations) do
            local separation = distance(predictions[id], observation)
            if separation <= TRACK_GATE_M then
                pairsList[#pairsList + 1] = { d = separation, id = id, oi = obsIndex }
            end
        end
    end

    table.sort(pairsList, function(a, b) return a.d < b.d end)
    local assignedTracks, assignedObservations = {}, {}
    for _, pair in ipairs(pairsList) do
        if not assignedTracks[pair.id] and not assignedObservations[pair.oi] then
            assignedTracks[pair.id] = pair.oi
            assignedObservations[pair.oi] = true
        end
    end

    for id, track in pairs(tracks) do
        local prediction = predictions[id]
        local observationIndex = assignedTracks[id]
        if observationIndex then
            local observation = observations[observationIndex]
            local alpha, beta = .45, .10
            for axis = 1, 3 do
                local residual = observation[axis] - prediction[axis]
                track.p[axis] = prediction[axis] + alpha * residual
                track.v[axis] = track.v[axis] + beta * residual / DT
            end
            track.lastSeen = tick
            track.misses = 0
            track.hits = track.hits + 1
        else
            track.p = prediction
            track.misses = track.misses + 1
        end
    end

    for observationIndex, observation in ipairs(observations) do
        if not assignedObservations[observationIndex] then
            tracks[nextTrackID] = {
                p = { observation[1], observation[2], observation[3] },
                v = { 0, 0, 0 },
                hits = 1,
                misses = 0,
                lastSeen = tick,
                rejectedUntil = 0
            }
            nextTrackID = nextTrackID + 1
        end
    end

    local deleted = {}
    for id, track in pairs(tracks) do
        if track.misses > TRACK_DELETE_TICKS then deleted[#deleted + 1] = id end
    end
    for _, id in ipairs(deleted) do tracks[id] = nil end
end

function relativeMotion(track, ownPos)
    local relativePosition = {
        track.p[1] - ownPos[1],
        track.p[2] - ownPos[2],
        track.p[3] - ownPos[3]
    }
    local relativeVelocity = {
        track.v[1] - ownVel[1],
        track.v[2] - ownVel[2],
        track.v[3] - ownVel[3]
    }
    return relativePosition, relativeVelocity, dot(relativePosition, relativeVelocity)
end

function isThreat(track, ownPos)
    local speed = magnitude(track.v)
    local toOwn = {
        ownPos[1] - track.p[1],
        ownPos[2] - track.p[2],
        ownPos[3] - track.p[3]
    }
    local toOwnMagnitude = magnitude(toOwn)
    if speed < SAM_MIN_SPEED or toOwnMagnitude < 1 then return false end
    local facing = dot(track.v, toOwn) / (speed * toOwnMagnitude)
    local _, _, relativeDot = relativeMotion(track, ownPos)
    return facing > 0 and relativeDot < 0
end

function chooseCandidate(ownPos)
    local chosen, chosenDistance = nil, math.huge
    for id, track in pairs(tracks) do
        local notShip = distance(track.p, shipPos) > DL_GATE_M
        local recent = track.misses == 0
        if notShip and recent and tick >= track.rejectedUntil then
            local range = distance(track.p, ownPos)
            if range < chosenDistance and range <= RADAR_EFFECTIVE_RANGE then
                chosen, chosenDistance = id, range
            end
        end
    end
    return chosen
end

function pointAt(globalPos, ownPos, q)
    local az, el = localToAngles(globalToLocal(globalPos, ownPos, q))
    return wrapTurns(az), math.max(-.125, math.min(.125, el))
end

function logPhase(phase, shipHorizontalDistance)
    if phase ~= lastPhase then
        debug.log("[HGV_DBG] PHASE_CHANGE tick=" .. tick .. " from=" .. lastPhase ..
            " to=" .. phase .. " ship_horizontal_m=" .. math.floor(shipHorizontalDistance + .5))
        lastPhase = phase
    end
end

function onTick()
    tick = tick + 1
    local seaSkimmingMode = input.getBool(2)

    local ownPos = { input.getNumber(25), input.getNumber(26), input.getNumber(27) }
    local pitch, yaw, roll = input.getNumber(28), input.getNumber(29), input.getNumber(30)
    local ownOrientation = eulerZYXToQuaternion(roll, yaw, pitch)
    if previousOwnPos then
        for axis = 1, 3 do ownVel[axis] = (ownPos[axis] - previousOwnPos[axis]) / DT end
    end
    previousOwnPos = { ownPos[1], ownPos[2], ownPos[3] }

    local receivedID = input.getNumber(32)
    local receivedPos = { input.getNumber(22), input.getNumber(23), input.getNumber(24) }
    local receivedVel = { input.getNumber(19), input.getNumber(20), input.getNumber(21) }
    local decodedID = receivedID
    local prelaunchFrame = receivedID > 100000
    local acceptData = false
    if prelaunchFrame then
        decodedID = receivedID - 100000
        if decodedID > 90000 then decodedID = decodedID - 90000 end
        if not launched and decodedID > 0 then
            dataLinkID = decodedID
            if receivedPos[1] ~= 0 or receivedPos[2] ~= 0 or receivedPos[3] ~= 0 then
                hasDataLink = true
                acceptData = true
            end
        end
    elseif hasDataLink and dataLinkID and decodedID == dataLinkID then
        if not launched then debug.log("[HGV_DBG] LAUNCH tick=" .. tick) end
        launched = true
        acceptData = true
    end

    for axis = 1, 3 do shipPos[axis] = shipPos[axis] + shipVel[axis] * DT end
    if acceptData then
        shipPos = receivedPos
        shipVel = receivedVel
    end

    local dx, dz = shipPos[1] - ownPos[1], shipPos[3] - ownPos[3]
    local shipHorizontalDistance = math.sqrt(dx * dx + dz * dz)
    local terminalRange = seaSkimmingMode and SEA_SKIM_POPUP_RANGE_M or TERMINAL_HORIZONTAL_RANGE
    if launched and hasDataLink and shipHorizontalDistance <= terminalRange then
        terminalLatched = true
    end

    local observations, rawRanges = {}, {}
    for target = 1, 6 do
        local base = (target - 1) * 3
        local range = input.getNumber(base + 1)
        rawRanges[target] = range
        if launched and range > 0 and range <= RADAR_EFFECTIVE_RANGE then
            local localPosition = polarToLocal(
                range,
                input.getNumber(base + 2),
                input.getNumber(base + 3)
            )
            observations[#observations + 1] = localToGlobal(localPosition, ownPos, ownOrientation)
        end
    end
    updateTrackSet(observations)

    if terminalLatched then
        candidateID = nil
        activeThreatID = nil
        threatWasClosing = false
    elseif activeThreatID then
        local threat = tracks[activeThreatID]
        if not threat or tick - threat.lastSeen >= THREAT_LOST_TICKS then
            debug.log("[HGV_DBG] THREAT_LOST tick=" .. tick .. " id=" .. activeThreatID)
            activeThreatID = nil
            threatWasClosing = false
        else
            local _, _, relativeDot = relativeMotion(threat, ownPos)
            if relativeDot < 0 then threatWasClosing = true end
            if threatWasClosing and relativeDot >= 0 then
                debug.log("[HGV_DBG] THREAT_PASSED tick=" .. tick .. " id=" .. activeThreatID)
                activeThreatID = nil
                threatWasClosing = false
            end
        end
    elseif candidateID then
        local candidate = tracks[candidateID]
        if not candidate then
            candidateID = nil
        elseif candidate.hits >= CANDIDATE_MIN_HITS and isThreat(candidate, ownPos) then
            activeThreatID = candidateID
            candidateID = nil
            threatWasClosing = true
            debug.log("[HGV_DBG] THREAT_CONFIRMED tick=" .. tick .. " id=" .. activeThreatID ..
                " speed_mps=" .. math.floor(magnitude(candidate.v) + .5))
        elseif tick - candidateStartTick >= CANDIDATE_MAX_TICKS then
            candidate.rejectedUntil = tick + 60
            debug.log("[HGV_DBG] CANDIDATE_REJECTED tick=" .. tick .. " id=" .. candidateID ..
                " speed_mps=" .. math.floor(magnitude(candidate.v) + .5))
            candidateID = nil
        end
    end

    if launched and not terminalLatched and not activeThreatID and not candidateID then
        candidateID = chooseCandidate(ownPos)
        if candidateID then
            candidateStartTick = tick
            debug.log("[HGV_DBG] THREAT_CANDIDATE tick=" .. tick .. " id=" .. candidateID)
        end
    end

    local phase = 0
    if terminalLatched then
        phase = 4
    elseif activeThreatID then
        phase = 3
    elseif candidateID then
        phase = 2
    elseif launched then
        phase = 1
    end
    logPhase(phase, shipHorizontalDistance)

    local radarAzimuth, radarElevation
    if terminalLatched then
        radarAzimuth, radarElevation = pointAt(shipPos, ownPos, ownOrientation)
    elseif activeThreatID and tracks[activeThreatID] then
        radarAzimuth, radarElevation = pointAt(tracks[activeThreatID].p, ownPos, ownOrientation)
    elseif candidateID and tracks[candidateID] then
        radarAzimuth, radarElevation = pointAt(tracks[candidateID].p, ownPos, ownOrientation)
    else
        radarAzimuth, radarElevation = pointAt(shipPos, ownPos, ownOrientation)
        radarAzimuth = wrapTurns(radarAzimuth + SCAN_OFFSETS[scanIndex])
        -- Keep the ship launch point and an approaching SAM near the horizon in one vertical beam.
        radarElevation = radarElevation * .5
        if launched then scanIndex = scanIndex % #SCAN_OFFSETS + 1 end
    end

    local shipOutputPos, shipOutputVel = shipPos, shipVel
    if terminalLatched then
        local bestShipTrack, bestDistance = nil, DL_GATE_M
        for _, track in pairs(tracks) do
            local separation = distance(track.p, shipPos)
            if separation < bestDistance then
                bestShipTrack, bestDistance = track, separation
            end
        end
        if bestShipTrack then
            shipOutputPos, shipOutputVel = bestShipTrack.p, bestShipTrack.v
        end
    end

    for axis = 1, 3 do
        output.setNumber(axis, shipOutputPos[axis])
        output.setNumber(axis + 3, shipOutputVel[axis])
        output.setNumber(axis + 12, ownPos[axis])
    end
    for axis = 1, 4 do output.setNumber(axis + 15, ownOrientation[axis]) end

    local threat = activeThreatID and tracks[activeThreatID] or nil
    for axis = 1, 3 do
        output.setNumber(axis + 6, threat and threat.p[axis] or 0)
        output.setNumber(axis + 19, threat and threat.v[axis] or 0)
    end
    output.setNumber(10, radarAzimuth)
    output.setNumber(11, radarElevation)
    output.setNumber(12, activeThreatID or 0)
    output.setNumber(23, phase)
    output.setNumber(32, 0)
    output.setBool(1, threat ~= nil)
    output.setBool(2, true)
    output.setBool(3, launched)
    output.setBool(4, terminalLatched)
    output.setBool(5, seaSkimmingMode)
    output.setBool(32, input.getBool(1))

    if launched and tick % 60 == 0 then
        local storedTracks = 0
        for _ in pairs(tracks) do storedTracks = storedTracks + 1 end
        debug.log("[HGV_DBG] STATUS tick=" .. tick .. " phase=" .. phase ..
            " ship_horizontal_m=" .. math.floor(shipHorizontalDistance + .5) ..
            " sea_skimming=" .. (seaSkimmingMode and 1 or 0) ..
            " radar_az_deg=" .. (math.floor(radarAzimuth * 3600 + .5) / 10) ..
            " radar_el_deg=" .. (math.floor(radarElevation * 3600 + .5) / 10) ..
            " observations=" .. #observations .. " stored_tracks=" .. storedTracks ..
            " r1_m=" .. math.floor(rawRanges[1] + .5) .. " r2_m=" .. math.floor(rawRanges[2] + .5) ..
            " r3_m=" .. math.floor(rawRanges[3] + .5) .. " r4_m=" .. math.floor(rawRanges[4] + .5) ..
            " r5_m=" .. math.floor(rawRanges[5] + .5) .. " r6_m=" .. math.floor(rawRanges[6] + .5))
    end
end
