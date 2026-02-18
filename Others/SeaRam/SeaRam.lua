MIN_DETECTION_DISTANCE = property.getNumber("MIN_DETECTION_DISTANCE")
PITCH_OFFSET = property.getNumber("PITCH_OFFSET") / 360 / 2000
SWEEP_SPEED = property.getNumber("SWEEP_SPEED") * 0.001
CLOSINGSPEED_ASSESSMENT_THRESHOLD = property.getNumber("CLOSINGSPEED_ASSESSMENT_THRESHOLD") / 60
CHANNEL_BASE = 4
SHOOTING_INTERVAL = property.getNumber("SHOOTING_INTERVAL")
YAW_ANGLE_LIMIT = property.getNumber("YAW_ANGLE_LIMIT") / 360
PITCH_ANGLE_LIMIT = property.getNumber("PITCH_ANGLE_LIMIT") / 360

local outputPivotX = 0
local outputPivotY = 0
local shootCount = 0
local stt_count = 0
local radarSweepYaw = 0
local radarSweepPitch = 0.1
local status = "SWEEP"
local detectedTargetData = {}
local targetDistanceDeltaTable = {}
local oldDistance = 0

function threatCheck(closestTargetRawDistanceDeltaTable)
    if #closestTargetRawDistanceDeltaTable < 30 then
        return "NOT_ENOUGH"
    end
    local sum = 0
    local avg = 0
    for i = 1, #closestTargetRawDistanceDeltaTable do
        sum = sum + closestTargetRawDistanceDeltaTable[i]
    end

    avg = sum / #closestTargetRawDistanceDeltaTable
    if avg < CLOSINGSPEED_ASSESSMENT_THRESHOLD then
        return "THREAT"
    else
        return "SAFE"
    end
end

function onTick()
    local isDetect = input.getBool(1)
    local targetRawData = {}
    local system = input.getBool(32)
    local closestTargetDistance = 0
    shootCount = shootCount + 1
    local fire = false

    if system then
        if isDetect then
            for i = 1, 8 do
                local distance = input.getNumber(i * CHANNEL_BASE - 3)
                local azimuth = input.getNumber(i * CHANNEL_BASE - 2)
                local elevation = input.getNumber(i * CHANNEL_BASE - 1)
                local isDetected = input.getBool(i)

                if distance > MIN_DETECTION_DISTANCE and math.abs(azimuth) < YAW_ANGLE_LIMIT and math.abs(elevation) < PITCH_ANGLE_LIMIT then
                    table.insert(targetRawData, { distance = distance, azimuth = azimuth, elevation = elevation })
                    count = 0
                end
            end

            if #targetRawData > 0 then
                table.sort(targetRawData, function(a, b) return a.distance < b.distance end)
                closestTargetDistance = targetRawData[1].distance
            end
        end

        if status == "SWEEP" then
            outputPivotX = 0
            oldDistance = 0
            stt_count = 0
            targetDistanceDeltaTable = {}
            radarSweepYaw = radarSweepYaw + SWEEP_SPEED
            if radarSweepYaw >= 0.99 then
                status = "SELECT_TARGET"
            end
            if #targetRawData > 0 then
                table.insert(detectedTargetData,
                    {
                        distance = targetRawData[1].distance,
                        azimuth = targetRawData[1].azimuth,
                        elevation = targetRawData[1].elevation
                    })
            end
        elseif status == "SELECT_TARGET" then
            if #detectedTargetData > 0 then
                table.sort(detectedTargetData, function(a, b) return a.distance < b.distance end)
                oldDistance = 0
                stt_count = 0
                targetDistanceDeltaTable = {}
                sttLockTargetAzimuth = detectedTargetData[1].azimuth
                radarSweepYaw = detectedTargetData[1].azimuth
                status = "SIMPLE_TARGET_TRACKING"
            else
                targetDistanceDeltaTable = {}
                radarSweepYaw = 0
                status = "SWEEP"
            end
        elseif status == "SIMPLE_TARGET_TRACKING" then
            stt_count = stt_count + 1
            if #targetRawData > 0 then
                radarSweepYaw = targetRawData[1].azimuth
                outputPivotX = targetRawData[1].azimuth
                outputPivotY = targetRawData[1].elevation

                if oldDistance == 0 then
                    oldDistance = closestTargetDistance
                else
                    distanceDelta = closestTargetDistance - oldDistance
                    oldDistance = closestTargetDistance
                    table.insert(targetDistanceDeltaTable, distanceDelta)
                end
                targetThreatStatus = threatCheck(targetDistanceDeltaTable)
                if targetThreatStatus == "SAFE" then
                    radarSweepYaw = 0
                    status = "SWEEP"
                    detectedTargetData = {}
                end
                if targetThreatStatus == "THREAT" and shootCount >= SHOOTING_INTERVAL then
                    shootCount = 0
                    fire = true
                end
            elseif stt_count > 50 then
                radarSweepYaw = 0
                status = "SWEEP"
                detectedTargetData = {}
            end
        end
    else
        status = "SWEEP"
        targetDistanceDeltaTable = {}
        detectedTargetData = {}
        oldDistance = 0
        radarSweepYaw = 0
        radarSweepPitch = 0.1
    end

    output.setNumber(1, radarSweepYaw)
    output.setNumber(2, radarSweepPitch)
    output.setNumber(10, closestTargetDistance)
    output.setNumber(11, outputPivotX)
    output.setNumber(12, (outputPivotY + PITCH_OFFSET * closestTargetDistance) * 4)

    output.setBool(1, fire)
end
