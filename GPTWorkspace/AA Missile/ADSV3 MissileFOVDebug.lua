--[[
================================================================================
ADSV3 MissileFOVDebug.lua
================================================================================
終末レーダーのFOV余裕と観測ロストを、KF本体とは別のLuaブロックで記録する。
外部出力は使用せず、DebugView++へdebug.logだけを送る。

数値入力:
- 1-12: 終末レーダー目標1-4（距離m、方位角turns、仰角turns）
- 13-15: KF出力の推定目標位置 X, Y, Z（世界座標）
- 16-18: KF出力の推定目標速度 Vx, Vy, Vz（m/s）
- 19-21: データリンク目標位置 X, Y, Z（世界座標）
- 22-24: 自機位置 X, Y, Z（同出力13-15を再配置。実速度・実加速度の差分にも使用）
- 25-28: 自機姿勢クォータニオン w, x, y, z（同出力16-19を再配置）
- 29-30: レーダー指向方位角・仰角 turns（同出力10-11を再配置）
- 31: データリンクが現在送信中の目標ID（ADSV3 MissileKF.lua入力32と同じ信号）
- 32: KF出力のプライマリトラックID（ADSV3 MissileKF.lua出力12）

bool入力:
- 1: 終末レーダー有効（ADSV3 Missile.lua出力bool 1）
- 2: KFが有効な追跡目標を出力中（ADSV3 MissileKF.lua出力bool 1）

出力: なし
================================================================================
]]

PI2 = math.pi * 2
DT = 1 / 60
MAX_RADAR_TARGETS = 4

DEBUG_LOG_ENABLED = true
DEBUG_HORIZONTAL_FOV_DEG = 18
DEBUG_VERTICAL_FOV_DEG = 86.4
DEBUG_VERTICAL_FOV_OFFSET_DEG = 0
DEBUG_RADAR_COMMAND_DELAY_TICKS = 1
DEBUG_DELAY_SCAN_MAX_TICKS = 3
DEBUG_KF_STATE_LOG_ENABLED = true

debugTick = 0
debugCommandHistory = {}
debugMonitorStartTick = nil
debugPreviousOutsideFOV = false
debugFirstFOVExitLogged = false
debugEverRadarObserved = false
debugCurrentLossTicks = 0
debugMaximumLossTicks = 0
debugReacquired = false
debugMinimumHorizontalMargin = math.huge
debugMinimumVerticalMargin = math.huge
debugTrackingID = nil
debugDataLinkPosition = { 0, 0, 0 }
debugDataLinkVelocity = { 0, 0, 0 }
debugLastReceivedPosition = nil
debugLastDataLinkUpdateTick = nil
debugHasTargetData = false
debugWaitingForTargetLogged = false
debugLastRadarObservation = nil
debugPreviousKFPosition = nil
debugPreviousKFVelocity = nil
debugPreviousKFTrackID = nil
debugPreviousOwnPosition = nil
debugPreviousOwnVelocity = nil

function rotateVectorByInverseQuaternion(vector, quaternion)
    local w, x, y, z = -quaternion[1], quaternion[2], quaternion[3], quaternion[4]
    local vx, vy, vz = vector[1], vector[2], vector[3]
    local tx = 2 * (y * vz - z * vy)
    local ty = 2 * (z * vx - x * vz)
    local tz = 2 * (x * vy - y * vx)
    return {
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx)
    }
end

function wrapTurns(value)
    return (value + 0.5) % 1 - 0.5
end

function rounded(value, scale)
    if value >= 0 then
        return math.floor(value * scale + 0.5) / scale
    end
    return math.ceil(value * scale - 0.5) / scale
end

function calculateDelayMargins(targetAzimuth, targetElevation, observationTick)
    local result = {}
    local horizontalHalfFOV = DEBUG_HORIZONTAL_FOV_DEG / 720
    local verticalHalfFOV = DEBUG_VERTICAL_FOV_DEG / 720
    local verticalOffset = DEBUG_VERTICAL_FOV_OFFSET_DEG / 360
    for delay = 0, DEBUG_DELAY_SCAN_MAX_TICKS do
        local commandTick = observationTick - delay
        local command = debugCommandHistory[commandTick]
        if command then
            local horizontalError = wrapTurns(targetAzimuth - command[1])
            local verticalError = targetElevation - (command[2] + verticalOffset)
            result[delay] = {
                commandTick = commandTick,
                horizontalError = horizontalError,
                verticalError = verticalError,
                horizontalMargin = horizontalHalfFOV - math.abs(horizontalError),
                verticalMargin = verticalHalfFOV - math.abs(verticalError)
            }
        end
    end
    return result
end

function logDelayScan(event, observation, monitorTick)
    local message = "[AA_FOV] DELAY_SCAN event=" .. event ..
        " tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
        " observation_tick=" .. observation.tick ..
        " observation_age_ticks=" .. (debugTick - observation.tick) ..
        " radar_distance_m=" .. rounded(observation.distance, 10)
    for delay = 0, DEBUG_DELAY_SCAN_MAX_TICKS do
        local candidate = observation.delayMargins[delay]
        if candidate then
            message = message .. " d" .. delay .. "_command_tick=" .. candidate.commandTick ..
                " d" .. delay .. "_h_error_turn=" .. rounded(candidate.horizontalError, 100000) ..
                " d" .. delay .. "_v_error_turn=" .. rounded(candidate.verticalError, 100000) ..
                " d" .. delay .. "_h_margin_turn=" .. rounded(candidate.horizontalMargin, 100000) ..
                " d" .. delay .. "_v_margin_turn=" .. rounded(candidate.verticalMargin, 100000)
        end
    end
    debug.log(message)
end

function updateTrackedDataLinkTarget(receivedTargetID, receivedPosition)
    if debugHasTargetData then
        debugDataLinkPosition = {
            debugDataLinkPosition[1] + debugDataLinkVelocity[1] * DT,
            debugDataLinkPosition[2] + debugDataLinkVelocity[2] * DT,
            debugDataLinkPosition[3] + debugDataLinkVelocity[3] * DT
        }
    end

    local decodedTargetID = receivedTargetID
    local isPrelaunchFrame = receivedTargetID > 100000
    local acceptPosition = false

    if isPrelaunchFrame then
        decodedTargetID = receivedTargetID - 100000
        if decodedTargetID > 90000 then decodedTargetID = decodedTargetID - 90000 end
        if decodedTargetID > 0 then
            if debugTrackingID ~= decodedTargetID then
                debugLastReceivedPosition = nil
                debugLastDataLinkUpdateTick = nil
                debugDataLinkVelocity = { 0, 0, 0 }
                debugHasTargetData = false
            end
            debugTrackingID = decodedTargetID
            acceptPosition = receivedPosition[1] ~= 0 or receivedPosition[2] ~= 0 or receivedPosition[3] ~= 0
        end
    elseif debugTrackingID and decodedTargetID == debugTrackingID then
        acceptPosition = true
    end

    if acceptPosition then
        if debugLastReceivedPosition and debugLastDataLinkUpdateTick then
            local elapsedSeconds = (debugTick - debugLastDataLinkUpdateTick) * DT
            if elapsedSeconds > 0 then
                debugDataLinkVelocity = {
                    (receivedPosition[1] - debugLastReceivedPosition[1]) / elapsedSeconds,
                    (receivedPosition[2] - debugLastReceivedPosition[2]) / elapsedSeconds,
                    (receivedPosition[3] - debugLastReceivedPosition[3]) / elapsedSeconds
                }
            end
        end
        debugDataLinkPosition = { receivedPosition[1], receivedPosition[2], receivedPosition[3] }
        debugLastReceivedPosition = { receivedPosition[1], receivedPosition[2], receivedPosition[3] }
        debugLastDataLinkUpdateTick = debugTick
        debugHasTargetData = true
    end
end

function vectorMagnitude(vector)
    return math.sqrt(vector[1] ^ 2 + vector[2] ^ 2 + vector[3] ^ 2)
end

function lateralMagnitude(vector, lineOfSight, distance)
    if distance < 1e-9 then return vectorMagnitude(vector) end
    local parallel = (vector[1] * lineOfSight[1] + vector[2] * lineOfSight[2] +
        vector[3] * lineOfSight[3]) / distance
    return math.sqrt(math.max(0, vectorMagnitude(vector) ^ 2 - parallel ^ 2))
end

function updateKFStateDebug(kfPosition, kfVelocity, kfTrackID, kfTrackValid,
                            ownPosition, monitorActive, rawObservationCount)
    local previousPosition = debugPreviousKFPosition
    local previousVelocity = debugPreviousKFVelocity
    local previousTrackID = debugPreviousKFTrackID
    local previousOwnPosition = debugPreviousOwnPosition
    local previousOwnVelocity = debugPreviousOwnVelocity
    local ownVelocity, ownAcceleration

    if previousOwnPosition then
        ownVelocity = {}
        for axis = 1, 3 do
            ownVelocity[axis] = (ownPosition[axis] - previousOwnPosition[axis]) / DT
        end
        if previousOwnVelocity then
            ownAcceleration = {}
            for axis = 1, 3 do
                ownAcceleration[axis] = (ownVelocity[axis] - previousOwnVelocity[axis]) / DT
            end
        end
    end

    debugPreviousKFPosition = { kfPosition[1], kfPosition[2], kfPosition[3] }
    debugPreviousKFVelocity = { kfVelocity[1], kfVelocity[2], kfVelocity[3] }
    debugPreviousKFTrackID = kfTrackID
    debugPreviousOwnPosition = { ownPosition[1], ownPosition[2], ownPosition[3] }
    if ownVelocity then debugPreviousOwnVelocity = ownVelocity end

    if not DEBUG_LOG_ENABLED or not DEBUG_KF_STATE_LOG_ENABLED or not monitorActive or
        not previousPosition or not previousVelocity or not ownAcceleration then return end

    local positionResidual, velocityStep, lineOfSight, relativeVelocity = {}, {}, {}, {}
    for axis = 1, 3 do
        positionResidual[axis] = kfPosition[axis] -
            (previousPosition[axis] + previousVelocity[axis] * DT)
        velocityStep[axis] = kfVelocity[axis] - previousVelocity[axis]
        lineOfSight[axis] = kfPosition[axis] - ownPosition[axis]
        relativeVelocity[axis] = kfVelocity[axis] - ownVelocity[axis]
    end
    local distance = vectorMagnitude(lineOfSight)
    local missileSpeed = vectorMagnitude(ownVelocity)
    local targetSpeed = vectorMagnitude(kfVelocity)
    local estimatedClosingSpeed = 0
    if distance > 1e-9 then
        estimatedClosingSpeed = -(relativeVelocity[1] * lineOfSight[1] +
            relativeVelocity[2] * lineOfSight[2] + relativeVelocity[3] * lineOfSight[3]) / distance
    end
    local estimatedVelocityCrossAngle = -1
    if missileSpeed * targetSpeed > 1e-9 then
        local cosine = (ownVelocity[1] * kfVelocity[1] + ownVelocity[2] * kfVelocity[2] +
            ownVelocity[3] * kfVelocity[3]) / (missileSpeed * targetSpeed)
        estimatedVelocityCrossAngle = math.acos(math.max(-1, math.min(1, cosine))) * 360 / PI2
    end
    local lineOfSightElevation = math.atan(lineOfSight[2],
        math.sqrt(lineOfSight[1] ^ 2 + lineOfSight[3] ^ 2)) * 360 / PI2
    local monitorTick = debugMonitorStartTick and debugTick - debugMonitorStartTick or 0
    local trackChanged = previousTrackID ~= nil and kfTrackID ~= previousTrackID
    debug.log("[AA_FOV] KF_STATE tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
        " kf_track_valid=" .. tostring(kfTrackValid) ..
        " kf_track_id=" .. kfTrackID .. " track_changed=" .. tostring(trackChanged) ..
        " raw_observations=" .. rawObservationCount ..
        " kf_distance_m=" .. rounded(distance, 10) ..
        " position_residual_m=" .. rounded(vectorMagnitude(positionResidual), 1000) ..
        " lateral_position_residual_m=" ..
            rounded(lateralMagnitude(positionResidual, lineOfSight, distance), 1000) ..
        " residual_x_m=" .. rounded(positionResidual[1], 1000) ..
        " residual_y_m=" .. rounded(positionResidual[2], 1000) ..
        " residual_z_m=" .. rounded(positionResidual[3], 1000) ..
        " velocity_step_mps=" .. rounded(vectorMagnitude(velocityStep), 10) ..
        " lateral_velocity_step_mps=" ..
            rounded(lateralMagnitude(velocityStep, lineOfSight, distance), 10) ..
        " velocity_step_x_mps=" .. rounded(velocityStep[1], 10) ..
        " velocity_step_y_mps=" .. rounded(velocityStep[2], 10) ..
        " velocity_step_z_mps=" .. rounded(velocityStep[3], 10) ..
        " velocity_x_mps=" .. rounded(kfVelocity[1], 10) ..
        " velocity_y_mps=" .. rounded(kfVelocity[2], 10) ..
        " velocity_z_mps=" .. rounded(kfVelocity[3], 10) ..
        " missile_speed_mps=" .. rounded(missileSpeed, 10) ..
        " missile_velocity_x_mps=" .. rounded(ownVelocity[1], 10) ..
        " missile_velocity_y_mps=" .. rounded(ownVelocity[2], 10) ..
        " missile_velocity_z_mps=" .. rounded(ownVelocity[3], 10) ..
        " est_closing_speed_mps=" .. rounded(estimatedClosingSpeed, 10) ..
        " est_relative_lateral_speed_mps=" ..
            rounded(lateralMagnitude(relativeVelocity, lineOfSight, distance), 10) ..
        " est_velocity_cross_angle_deg=" .. rounded(estimatedVelocityCrossAngle, 10) ..
        " los_elevation_deg=" .. rounded(lineOfSightElevation, 10) ..
        " missile_accel_mps2=" .. rounded(vectorMagnitude(ownAcceleration), 10) ..
        " missile_lateral_accel_mps2=" ..
            rounded(lateralMagnitude(ownAcceleration, lineOfSight, distance), 10) ..
        " missile_accel_x_mps2=" .. rounded(ownAcceleration[1], 10) ..
        " missile_accel_y_mps2=" .. rounded(ownAcceleration[2], 10) ..
        " missile_accel_z_mps2=" .. rounded(ownAcceleration[3], 10))
end

function updateFOVDebug(terminalRadarOn, dataLinkAzimuth, dataLinkElevation, dataLinkDistance,
                        beamAzimuth, beamElevation, radarAzimuth, radarElevation,
                        radarDistance, radarObserved, rawObservationCount, radarMatchScore, dataAgeTicks)
    debugCommandHistory[debugTick] = { beamAzimuth, beamElevation }
    local commandTick = debugTick - DEBUG_RADAR_COMMAND_DELAY_TICKS
    local appliedCommand = debugCommandHistory[commandTick]
    debugCommandHistory[debugTick - DEBUG_DELAY_SCAN_MAX_TICKS - 1] = nil

    local monitorActive = debugMonitorStartTick ~= nil or terminalRadarOn or radarObserved
    if not DEBUG_LOG_ENABLED or not monitorActive or appliedCommand == nil then return end

    local referenceAzimuth = radarObserved and radarAzimuth or dataLinkAzimuth
    local referenceElevation = radarObserved and radarElevation or dataLinkElevation
    local referenceSource = radarObserved and "radar" or "datalink"
    local horizontalHalfFOV = DEBUG_HORIZONTAL_FOV_DEG / 720
    local verticalHalfFOV = DEBUG_VERTICAL_FOV_DEG / 720
    local verticalOffset = DEBUG_VERTICAL_FOV_OFFSET_DEG / 360
    local horizontalError = wrapTurns(referenceAzimuth - appliedCommand[1])
    local verticalError = referenceElevation - (appliedCommand[2] + verticalOffset)
    local horizontalMargin = horizontalHalfFOV - math.abs(horizontalError)
    local verticalMargin = verticalHalfFOV - math.abs(verticalError)
    local outsideHorizontal = horizontalMargin < 0
    local outsideVertical = verticalMargin < 0
    local outsideFOV = outsideHorizontal or outsideVertical

    if debugMonitorStartTick == nil then
        debugMonitorStartTick = debugTick
        debug.log("[AA_FOV] MONITOR_START tick=" .. debugTick ..
            " distance_m=" .. rounded(dataLinkDistance, 10) ..
            " command_delay_ticks=" .. DEBUG_RADAR_COMMAND_DELAY_TICKS ..
            " horizontal_fov_deg=" .. DEBUG_HORIZONTAL_FOV_DEG ..
            " vertical_fov_deg=" .. DEBUG_VERTICAL_FOV_DEG ..
            " vertical_offset_deg=" .. DEBUG_VERTICAL_FOV_OFFSET_DEG ..
            " raw_observations=" .. rawObservationCount ..
            " target_id=" .. tostring(debugTrackingID) ..
            " data_age_ticks=" .. dataAgeTicks)
    end

    local monitorTick = debugTick - debugMonitorStartTick
    debugMinimumHorizontalMargin = math.min(debugMinimumHorizontalMargin, horizontalMargin)
    debugMinimumVerticalMargin = math.min(debugMinimumVerticalMargin, verticalMargin)

    if radarObserved then
        debugLastRadarObservation = {
            tick = debugTick,
            commandTick = commandTick,
            distance = radarDistance,
            azimuth = radarAzimuth,
            elevation = radarElevation,
            horizontalError = horizontalError,
            verticalError = verticalError,
            horizontalMargin = horizontalMargin,
            verticalMargin = verticalMargin,
            delayMargins = calculateDelayMargins(radarAzimuth, radarElevation, debugTick)
        }
    end

    if outsideFOV and not debugPreviousOutsideFOV then
        local axis = outsideHorizontal and outsideVertical and "both" or
            (outsideHorizontal and "horizontal" or "vertical")
        local event = debugFirstFOVExitLogged and "FOV_EXIT" or "FOV_EXIT_FIRST"
        debugFirstFOVExitLogged = true
        debug.log("[AA_FOV] " .. event .. " tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
            " distance_m=" .. rounded(dataLinkDistance, 10) .. " axis=" .. axis ..
            " reference=" .. referenceSource .. " command_tick=" .. commandTick ..
            " h_margin_turn=" .. rounded(horizontalMargin, 100000) ..
            " v_margin_turn=" .. rounded(verticalMargin, 100000) ..
            " h_error_turn=" .. rounded(horizontalError, 100000) ..
            " v_error_turn=" .. rounded(verticalError, 100000) ..
            " radar_match_score=" .. rounded(radarMatchScore, 100000))
    elseif not outsideFOV and debugPreviousOutsideFOV then
        debug.log("[AA_FOV] FOV_REENTER tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
            " distance_m=" .. rounded(dataLinkDistance, 10) .. " reference=" .. referenceSource ..
            " h_margin_turn=" .. rounded(horizontalMargin, 100000) ..
            " v_margin_turn=" .. rounded(verticalMargin, 100000))
    end
    debugPreviousOutsideFOV = outsideFOV

    if radarObserved then
        if debugCurrentLossTicks > 0 then
            debugReacquired = true
            debug.log("[AA_FOV] RADAR_REACQUIRED tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
                " distance_m=" .. rounded(dataLinkDistance, 10) ..
                " loss_ticks=" .. debugCurrentLossTicks .. " outside_fov=" .. tostring(outsideFOV))
            logDelayScan("reacquired", debugLastRadarObservation, monitorTick)
        elseif not debugEverRadarObserved then
            debug.log("[AA_FOV] RADAR_ACQUIRED tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
                " distance_m=" .. rounded(dataLinkDistance, 10) ..
                " raw_observations=" .. rawObservationCount)
        end
        debugEverRadarObserved = true
        debugCurrentLossTicks = 0
    elseif debugEverRadarObserved then
        debugCurrentLossTicks = debugCurrentLossTicks + 1
        debugMaximumLossTicks = math.max(debugMaximumLossTicks, debugCurrentLossTicks)
        if debugCurrentLossTicks == 1 then
            local lastRadar = debugLastRadarObservation
            debug.log("[AA_FOV] RADAR_LOST_START tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
                " distance_m=" .. rounded(dataLinkDistance, 10) .. " outside_fov=" .. tostring(outsideFOV) ..
                " h_margin_turn=" .. rounded(horizontalMargin, 100000) ..
                " v_margin_turn=" .. rounded(verticalMargin, 100000) ..
                " last_radar_tick=" .. lastRadar.tick ..
                " last_radar_age_ticks=" .. (debugTick - lastRadar.tick) ..
                " last_radar_distance_m=" .. rounded(lastRadar.distance, 10) ..
                " last_radar_command_tick=" .. lastRadar.commandTick ..
                " last_radar_h_error_turn=" .. rounded(lastRadar.horizontalError, 100000) ..
                " last_radar_v_error_turn=" .. rounded(lastRadar.verticalError, 100000) ..
                " last_radar_h_margin_turn=" .. rounded(lastRadar.horizontalMargin, 100000) ..
                " last_radar_v_margin_turn=" .. rounded(lastRadar.verticalMargin, 100000) ..
                " datalink_command_tick=" .. commandTick ..
                " datalink_h_error_turn=" .. rounded(horizontalError, 100000) ..
                " datalink_v_error_turn=" .. rounded(verticalError, 100000) ..
                " datalink_h_margin_turn=" .. rounded(horizontalMargin, 100000) ..
                " datalink_v_margin_turn=" .. rounded(verticalMargin, 100000) ..
                " reference_h_delta_turn=" .. rounded(wrapTurns(dataLinkAzimuth - lastRadar.azimuth), 100000) ..
                " reference_v_delta_turn=" .. rounded(dataLinkElevation - lastRadar.elevation, 100000))
            logDelayScan("last_before_loss", lastRadar, monitorTick)
        end
    end

    if monitorTick % 60 == 0 then
        debug.log("[AA_FOV] STATUS tick=" .. debugTick .. " monitor_tick=" .. monitorTick ..
            " distance_m=" .. rounded(dataLinkDistance, 10) .. " reference=" .. referenceSource ..
            " h_margin_turn=" .. rounded(horizontalMargin, 100000) ..
            " v_margin_turn=" .. rounded(verticalMargin, 100000) ..
            " min_h_margin_turn=" .. rounded(debugMinimumHorizontalMargin, 100000) ..
            " min_v_margin_turn=" .. rounded(debugMinimumVerticalMargin, 100000) ..
            " current_loss_ticks=" .. debugCurrentLossTicks ..
            " max_loss_ticks=" .. debugMaximumLossTicks ..
            " reacquired=" .. tostring(debugReacquired) ..
            " raw_observations=" .. rawObservationCount ..
            " target_id=" .. tostring(debugTrackingID) ..
            " data_age_ticks=" .. dataAgeTicks)
    end
end

function onTick()
    debugTick = debugTick + 1

    local receivedPosition = { input.getNumber(19), input.getNumber(20), input.getNumber(21) }
    local receivedTargetID = input.getNumber(31)
    updateTrackedDataLinkTarget(receivedTargetID, receivedPosition)

    if not debugHasTargetData then
        if input.getBool(1) and not debugWaitingForTargetLogged then
            debugWaitingForTargetLogged = true
            debug.log("[AA_FOV] WAIT_TARGET_ID tick=" .. debugTick ..
                " received_id=" .. receivedTargetID)
        end
        return
    end
    debugWaitingForTargetLogged = false

    local dataLinkPosition = debugDataLinkPosition
    local kfPosition = { input.getNumber(13), input.getNumber(14), input.getNumber(15) }
    local kfVelocity = { input.getNumber(16), input.getNumber(17), input.getNumber(18) }
    local kfTrackID = input.getNumber(32)
    local kfTrackValid = input.getBool(2)
    local ownPosition = { input.getNumber(22), input.getNumber(23), input.getNumber(24) }
    local ownQuaternion = {
        input.getNumber(25), input.getNumber(26), input.getNumber(27), input.getNumber(28)
    }
    local relativeWorld = {
        dataLinkPosition[1] - ownPosition[1],
        dataLinkPosition[2] - ownPosition[2],
        dataLinkPosition[3] - ownPosition[3]
    }
    local dataLinkLocal = rotateVectorByInverseQuaternion(relativeWorld, ownQuaternion)
    local horizontalDistance = math.sqrt(dataLinkLocal[1] ^ 2 + dataLinkLocal[3] ^ 2)
    local dataLinkAzimuth = math.atan(dataLinkLocal[1], dataLinkLocal[3]) / PI2
    local dataLinkElevation = math.atan(dataLinkLocal[2], horizontalDistance) / PI2
    local dataLinkDistance = math.sqrt(
        dataLinkLocal[1] ^ 2 + dataLinkLocal[2] ^ 2 + dataLinkLocal[3] ^ 2
    )

    local rawObservationCount = 0
    local radarObserved = false
    local radarAzimuth, radarElevation, radarDistance = 0, 0, 0
    local radarMatchScore = math.huge
    for i = 1, MAX_RADAR_TARGETS do
        local candidateDistance = input.getNumber(i * 3 - 2)
        if candidateDistance > 0 then
            rawObservationCount = rawObservationCount + 1
            local candidateAzimuth = input.getNumber(i * 3 - 1)
            local candidateElevation = input.getNumber(i * 3)
            local horizontalError = wrapTurns(candidateAzimuth - dataLinkAzimuth)
            local verticalError = candidateElevation - dataLinkElevation
            local rangeError = (candidateDistance - dataLinkDistance) / math.max(dataLinkDistance, 1)
            local score = horizontalError ^ 2 + verticalError ^ 2 + rangeError ^ 2
            if score < radarMatchScore then
                radarObserved = true
                radarMatchScore = score
                radarDistance = candidateDistance
                radarAzimuth = candidateAzimuth
                radarElevation = candidateElevation
            end
        end
    end
    if not radarObserved then radarMatchScore = -1 end
    local dataAgeTicks = debugTick - debugLastDataLinkUpdateTick
    local terminalRadarOn = input.getBool(1)

    updateKFStateDebug(
        kfPosition,
        kfVelocity,
        kfTrackID,
        kfTrackValid,
        ownPosition,
        debugMonitorStartTick ~= nil or terminalRadarOn or radarObserved,
        rawObservationCount
    )

    updateFOVDebug(
        terminalRadarOn,
        dataLinkAzimuth,
        dataLinkElevation,
        dataLinkDistance,
        input.getNumber(29),
        input.getNumber(30),
        radarAzimuth,
        radarElevation,
        radarDistance,
        radarObserved,
        rawObservationCount,
        radarMatchScore,
        dataAgeTicks
    )
end
