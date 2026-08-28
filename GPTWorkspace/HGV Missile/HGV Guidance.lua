--[[
HGV対艦ミサイル用の飛行誘導スクリプト。
ADSV3 Missile.luaの座標変換、単追尾、比例航法、重力補償を流用し、
SEARCH / EVADE / TERMINAL のHGV状態へ置き換えたもの。

入力 bool:
  1: 有効SAMあり  2: 対水上モード  3: 発射済み  4: TERMINALラッチ  5: シースキミングモード
入力 number:
  1-3: 敵艦位置 X,Y,Z  4-6: 敵艦速度 Vx,Vy,Vz
  7-9: SAM位置 X,Y,Z   12: SAMトラックID
 13-15: 自機位置 X,Y,Z  16-19: 自機クォータニオン w,x,y,z
 20-22: SAM速度 Vx,Vy,Vz  23: レーダー側フェーズ

出力 bool:
  1: 予約  2: 対水上モード  3: 主レーダーON
  4: 自爆（未採用のため常時false）  5: 衝撃信管
出力 number:
  1: Yawフィン  2: Pitchフィン  3: 敵艦距離のtick差分

VERTICAL_MANEUVER_AMPLITUDE_Mは機体試験で確定するプロパティ。
0または未設定なら上下蛇行を無効化し、高度2000mだけを指令する。
シースキミングモードでは高度20mを飛び、水平距離1000mから高度400mへポップアップし、
目標3D距離500m未満または高度400m到達で敵艦への比例航法へ移る。
]]

DT = 1 / 60
CRUISE_ALTITUDE_M = 2000
TERMINAL_HORIZONTAL_RANGE_M = property.getNumber("TERMINAL_HORIZONTAL_RANGE")
VERTICAL_REVERSAL_TICKS = 90
BEAM_TARGET_DISTANCE_M = 1000
KINEMATICS_LOG_RANGE_M = 500

PN_FIN_STRENGTH = property.getNumber("PN_FIN_STRENGTH")
PPN_FIN_STRENGTH = property.getNumber("PPN_FIN_STRENGTH")
VERTICAL_AMPLITUDE_M = property.getNumber("VERTICAL_MANEUVER_AMPLITUDE_M")
SEA_SKIM_ALTITUDE_M = 20
SEA_SKIM_POPUP_RANGE_M = property.getNumber("SEA_SKIM_POPUP_RANGE")
SEA_SKIM_POPUP_ALTITUDE_M = property.getNumber("SEA_SKIM_POPUP_ALTITUDE")
SEA_SKIM_DIVE_RANGE_M = 700
LOGIC_DELAY = property.getNumber("LOGIC_DELAY")
GRAVITY_COMP_ENABLE = property.getBool("GRAVITY_COMP_ENABLE")
GRAVITY_COMP_GAIN = property.getNumber("GRAVITY_COMP_GAIN")
MAX_FIN_COMMAND = property.getNumber("MAX_FIN_COMMAND")

flightTick = 0
previousOwnPos = nil
previousPreviousOwnPos = nil
oldLOS = { 0, 0, 1 }
pnInitialized = false
terminalLatched = false
beamThreatID = 0
beamSide = 1
oldShipDistance = nil
lastPhase = -1
seaSkimPopupStarted = false
seaSkimDiveLatched = false

function magnitude(v)
    return math.sqrt(v[1] * v[1] + v[2] * v[2] + v[3] * v[3])
end

function normalize(v)
    local m = magnitude(v)
    if m < 1e-9 then return { 0, 0, 1 } end
    return { v[1] / m, v[2] / m, v[3] / m }
end

function cross(a, b)
    return {
        a[2] * b[3] - a[3] * b[2],
        a[3] * b[1] - a[1] * b[3],
        a[1] * b[2] - a[2] * b[1]
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

function globalToLocal(globalPos, ownPos, q)
    return rotateByQuaternion({
        globalPos[1] - ownPos[1],
        globalPos[2] - ownPos[2],
        globalPos[3] - ownPos[3]
    }, q, true)
end

function localAngles(v)
    local horizontal = math.sqrt(v[1] * v[1] + v[3] * v[3])
    return math.atan(v[1], v[3]), math.atan(v[2], horizontal)
end

function gravityCorrection(localVelocity, q)
    if not GRAVITY_COMP_ENABLE then return 0, 0 end
    local speed = magnitude(localVelocity)
    if speed <= .1 then return 0, 0 end
    local velocityDirection = {
        localVelocity[1] / speed,
        localVelocity[2] / speed,
        localVelocity[3] / speed
    }
    local gravity = rotateByQuaternion({ 0, -1, 0 }, q, true)
    local parallel = gravity[1] * velocityDirection[1] +
        gravity[2] * velocityDirection[2] + gravity[3] * velocityDirection[3]
    local correction = {
        -(gravity[1] - parallel * velocityDirection[1]) * GRAVITY_COMP_GAIN,
        -(gravity[2] - parallel * velocityDirection[2]) * GRAVITY_COMP_GAIN,
        -(gravity[3] - parallel * velocityDirection[3]) * GRAVITY_COMP_GAIN
    }
    local turnAxis = cross(correction, velocityDirection)
    return turnAxis[1], -turnAxis[2]
end

function getThreatCourse(ownPos, threatPos, threatVelocity)
    local vx, vz = threatVelocity[1], threatVelocity[3]
    local speed = math.sqrt(vx * vx + vz * vz)
    if speed > 1e-6 then
        return vx / speed, vz / speed
    end
    local rx, rz = threatPos[1] - ownPos[1], threatPos[3] - ownPos[3]
    local range = math.sqrt(rx * rx + rz * rz)
    if range < 1e-6 then return 0, 1 end
    return rx / range, rz / range
end

function getBeamDirection(ownPos, threatPos, threatVelocity, side)
    local courseX, courseZ = getThreatCourse(ownPos, threatPos, threatVelocity)
    return {
        -courseZ * side,
        courseX * side
    }
end

function selectBeamSide(ownPos, shipPos, threatPos, threatVelocity)
    local first = getBeamDirection(ownPos, threatPos, threatVelocity, 1)
    local second = { -first[1], -first[2] }
    local sx, sz = shipPos[1] - ownPos[1], shipPos[3] - ownPos[3]
    local shipRange = math.sqrt(sx * sx + sz * sz)
    if shipRange < 1 then return 1 end
    sx, sz = sx / shipRange, sz / shipRange
    if first[1] * sx + first[2] * sz >= second[1] * sx + second[2] * sz then
        return 1
    end
    return -1
end
function verticalManeuverSign()
    return math.floor((flightTick - 1) / VERTICAL_REVERSAL_TICKS) % 2 == 0 and 1 or -1
end


function purePursuit(target, ownPos, q)
    local localTarget = globalToLocal(target, ownPos, q)
    local azimuth, elevation = localAngles(localTarget)
    return azimuth * PPN_FIN_STRENGTH, elevation * PPN_FIN_STRENGTH
end

function proportionalNavigation(target, ownPos, q)
    local currentLOS = normalize({
        target[1] - ownPos[1],
        target[2] - ownPos[2],
        target[3] - ownPos[3]
    })
    if not pnInitialized then
        oldLOS = currentLOS
        pnInitialized = true
        return 0, 0
    end
    local omegaWorld = cross(oldLOS, currentLOS)
    omegaWorld = { omegaWorld[1] / DT, omegaWorld[2] / DT, omegaWorld[3] / DT }
    local omegaLocal = rotateByQuaternion(omegaWorld, q, true)
    oldLOS = currentLOS
    return omegaLocal[2] * PN_FIN_STRENGTH, -omegaLocal[1] * PN_FIN_STRENGTH
end

function clampCommand(value)
    if MAX_FIN_COMMAND <= 0 then return value end
    return math.max(-MAX_FIN_COMMAND, math.min(MAX_FIN_COMMAND, value))
end

function roundTenth(value)
    if value >= 0 then return math.floor(value * 10 + .5) / 10 end
    return math.ceil(value * 10 - .5) / 10
end

function onTick()
    local launched = input.getBool(3)
    if launched then flightTick = flightTick + 1 end

    local threatActive = input.getBool(1)
    local terminalInput = input.getBool(4)
    local shipPos = { input.getNumber(1), input.getNumber(2), input.getNumber(3) }
    local seaSkimmingMode = input.getBool(5)
    local threatPos = { input.getNumber(7), input.getNumber(8), input.getNumber(9) }
    local threatVel = { input.getNumber(20), input.getNumber(21), input.getNumber(22) }
    local threatID = input.getNumber(12)
    local radarPhase = input.getNumber(23)
    local ownPos = { input.getNumber(13), input.getNumber(14), input.getNumber(15) }
    local q = {
        input.getNumber(16), input.getNumber(17),
        input.getNumber(18), input.getNumber(19)
    }

    local previousPosition = previousOwnPos or ownPos
    local kinematicsReady = previousPreviousOwnPos ~= nil
    local predictedOwnPos, worldVelocity, worldAcceleration = {}, {}, {}
    local delaySquaredHalf = LOGIC_DELAY * LOGIC_DELAY * .5
    for axis = 1, 3 do
        local velocityPerTick = ownPos[axis] - previousPosition[axis]
        local accelerationPerTick2 = 0
        if previousPreviousOwnPos then
            accelerationPerTick2 = velocityPerTick - previousPosition[axis] + previousPreviousOwnPos[axis]
        end
        predictedOwnPos[axis] = ownPos[axis] + velocityPerTick * LOGIC_DELAY +
            accelerationPerTick2 * delaySquaredHalf
        worldVelocity[axis] = velocityPerTick / DT
        worldAcceleration[axis] = accelerationPerTick2 / (DT * DT)
    end
    previousPreviousOwnPos = previousOwnPos
    previousOwnPos = { ownPos[1], ownPos[2], ownPos[3] }

    local dx, dz = shipPos[1] - predictedOwnPos[1], shipPos[3] - predictedOwnPos[3]
    local horizontalDistance = math.sqrt(dx * dx + dz * dz)
    local shipDistance = magnitude({
        shipPos[1] - predictedOwnPos[1],
        shipPos[2] - predictedOwnPos[2],
        shipPos[3] - predictedOwnPos[3]
    })
    local terminalRange = seaSkimmingMode and SEA_SKIM_POPUP_RANGE_M or TERMINAL_HORIZONTAL_RANGE_M
    if launched and (terminalInput or horizontalDistance <= terminalRange) then
        terminalLatched = true
    end

    local phase, target = 0, shipPos
    local beamAspectDeg = -1
    local popupActive = false
    if terminalLatched then
        phase = 4
        if seaSkimmingMode and not seaSkimDiveLatched then
            if shipDistance < SEA_SKIM_DIVE_RANGE_M or ownPos[2] >= SEA_SKIM_POPUP_ALTITUDE_M then
                seaSkimDiveLatched = true
                local reason = shipDistance < SEA_SKIM_DIVE_RANGE_M and "range" or "altitude"
                debug.log("[HGV_DBG] SEA_SKIM_DIVE tick=" .. flightTick ..
                    " reason=" .. reason .. " ship_distance_m=" .. math.floor(shipDistance + .5) ..
                    " altitude_m=" .. math.floor(ownPos[2] + .5))
            else
                popupActive = true
                if not seaSkimPopupStarted then
                    seaSkimPopupStarted = true
                    debug.log("[HGV_DBG] SEA_SKIM_POPUP tick=" .. flightTick ..
                        " ship_horizontal_m=" .. math.floor(horizontalDistance + .5))
                end
            end
        end
        target = popupActive and {
            shipPos[1], SEA_SKIM_POPUP_ALTITUDE_M, shipPos[3]
        } or shipPos
    elseif launched and threatActive then
        phase = 3
        if threatID ~= beamThreatID then
            beamThreatID = threatID
            beamSide = selectBeamSide(predictedOwnPos, shipPos, threatPos, threatVel)
        end
        local beamDirection = getBeamDirection(predictedOwnPos, threatPos, threatVel, beamSide)
        target = {
            predictedOwnPos[1] + beamDirection[1] * BEAM_TARGET_DISTANCE_M,
            seaSkimmingMode and SEA_SKIM_ALTITUDE_M or CRUISE_ALTITUDE_M + verticalManeuverSign() * VERTICAL_AMPLITUDE_M,
            predictedOwnPos[3] + beamDirection[2] * BEAM_TARGET_DISTANCE_M
        }
        local ownHorizontalSpeed = math.sqrt(worldVelocity[1] ^ 2 + worldVelocity[3] ^ 2)
        local threatHorizontalSpeed = math.sqrt(threatVel[1] ^ 2 + threatVel[3] ^ 2)
        if ownHorizontalSpeed > 1e-6 and threatHorizontalSpeed > 1e-6 then
            local cosine = (worldVelocity[1] * threatVel[1] +
                worldVelocity[3] * threatVel[3]) / (ownHorizontalSpeed * threatHorizontalSpeed)
            cosine = math.max(-1, math.min(1, cosine))
            beamAspectDeg = math.acos(cosine) * 180 / math.pi
        end
    elseif launched then
        phase = radarPhase == 2 and 2 or 1
        beamThreatID = 0
        local horizontalDirectionX, horizontalDirectionZ = 0, 1
        if horizontalDistance > 1 then
            horizontalDirectionX = dx / horizontalDistance
            horizontalDirectionZ = dz / horizontalDistance
        end
        local verticalSign = 0
        if phase == 1 and not seaSkimmingMode then
            verticalSign = verticalManeuverSign()
        end
        local cruiseAltitude = seaSkimmingMode and SEA_SKIM_ALTITUDE_M or CRUISE_ALTITUDE_M
        target = {
            predictedOwnPos[1] + horizontalDirectionX * BEAM_TARGET_DISTANCE_M,
            cruiseAltitude + verticalSign * VERTICAL_AMPLITUDE_M,
            predictedOwnPos[3] + horizontalDirectionZ * BEAM_TARGET_DISTANCE_M
        }
    end

    if phase ~= lastPhase then
        debug.log("[HGV_DBG] GUIDANCE_PHASE_CHANGE tick=" .. flightTick ..
            " from=" .. lastPhase .. " to=" .. phase ..
            " ship_horizontal_m=" .. math.floor(horizontalDistance + .5))
        lastPhase = phase
    end

    if launched and threatActive and kinematicsReady then
        local threatOffset = {
            threatPos[1] - ownPos[1],
            threatPos[2] - ownPos[2],
            threatPos[3] - ownPos[3]
        }
        local threatDistance = magnitude(threatOffset)
        if threatDistance <= KINEMATICS_LOG_RANGE_M then
            local los = normalize(threatOffset)
            local alongLOS = worldAcceleration[1] * los[1] +
                worldAcceleration[2] * los[2] + worldAcceleration[3] * los[3]
            local lateralAcceleration = magnitude({
                worldAcceleration[1] - alongLOS * los[1],
                worldAcceleration[2] - alongLOS * los[2],
                worldAcceleration[3] - alongLOS * los[3]
            })
            debug.log("[HGV_DBG] KINEMATICS tick=" .. flightTick .. " phase=" .. phase ..
                " threat_distance_m=" .. roundTenth(threatDistance) ..
                " velocity_x_mps=" .. roundTenth(worldVelocity[1]) ..
                " velocity_y_mps=" .. roundTenth(worldVelocity[2]) ..
                " velocity_z_mps=" .. roundTenth(worldVelocity[3]) ..
                " accel_x_mps2=" .. roundTenth(worldAcceleration[1]) ..
                " accel_y_mps2=" .. roundTenth(worldAcceleration[2]) ..
                " accel_z_mps2=" .. roundTenth(worldAcceleration[3]) ..
                " accel_mps2=" .. roundTenth(magnitude(worldAcceleration)) ..
                " actual_lateral_accel_mps2=" .. roundTenth(lateralAcceleration))
        end
    end

    local yawCommand, pitchCommand
    if terminalLatched and not popupActive then
        yawCommand, pitchCommand = proportionalNavigation(target, predictedOwnPos, q)
    else
        pnInitialized = false
        yawCommand, pitchCommand = purePursuit(target, predictedOwnPos, q)
        oldLOS = normalize({
            target[1] - predictedOwnPos[1],
            target[2] - predictedOwnPos[2],
            target[3] - predictedOwnPos[3]
        })
    end

    if launched then
        local localVelocity = rotateByQuaternion(worldVelocity, q, true)
        local gravityPitch, gravityYaw = gravityCorrection(localVelocity, q)
        pitchCommand = pitchCommand + gravityPitch
        yawCommand = yawCommand + gravityYaw
    end
    yawCommand = clampCommand(yawCommand)
    pitchCommand = clampCommand(pitchCommand)

    local approachPerTick = oldShipDistance and shipDistance - oldShipDistance or 0
    oldShipDistance = shipDistance
    local fuse = terminalLatched and launched and shipDistance < 500

    output.setBool(1, false)
    output.setBool(2, true)
    output.setBool(3, launched)
    output.setBool(4, false)
    output.setBool(5, fuse)
    output.setNumber(1, yawCommand)
    output.setNumber(2, pitchCommand)
    output.setNumber(3, approachPerTick)

    if launched and flightTick % 60 == 0 then
        debug.log("[HGV_DBG] GUIDANCE_STATUS tick=" .. flightTick .. " phase=" .. phase ..
            " altitude_m=" .. math.floor(ownPos[2] + .5) ..
            " yaw=" .. yawCommand .. " pitch=" .. pitchCommand ..
            " beam_aspect_deg=" .. (math.floor(beamAspectDeg * 10 + .5) / 10) ..
            " sea_skimming=" .. (seaSkimmingMode and 1 or 0) .. " popup=" .. (popupActive and 1 or 0))
    end
end
