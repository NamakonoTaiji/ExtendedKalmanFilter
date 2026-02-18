ROD_LINER_CONNECT_DISTANCE = 7.745
ROD_LINER_CLOSEST_DISTANCE = 0.255
ROD_LINER_NEUTRAL_DISTANCE = 5.75
DRILL_MAX_DIST_FROM_GROUND = 12.75
DRILL_MIN_DIST_FROM_GROUND = 2.5
START_DRILL_DELAY = 100

isReloadedRod = false
isReloadRod = false
isOutputLinerConnectRod = false
isOutputDrillConnectRod = false
rodUpDownMotion = false
rodLinear = 0
drillLiner = 1

startDrillDelayCount = 0
status = "RELOAD"
drillStatus = "DOWN"
function rodLinerControlFunc(status, rodLinearDistance, isConnectedRod)
    if status == "NEUTRAL" then
        if rodLinearDistance <= ROD_LINER_NEUTRAL_DISTANCE - 0.2 then
            return 1, "NEUTRAL_PROCESSING"
        elseif rodLinearDistance >= ROD_LINER_NEUTRAL_DISTANCE + 0.2 then
            return -1, "NEUTRAL_PROCESSING"
        else
            return 0, "NEUTRAL"
        end
    elseif status == "RELOAD" then
        if isConnectedRod and rodLinearDistance < ROD_LINER_CONNECT_DISTANCE then
            return 1, "RELOAD_PROCESSING"
        elseif isConnectedRod and rodLinearDistance > ROD_LINER_CONNECT_DISTANCE then
            return 0, "RELOADED"
        end
        if not isConnectedRod and rodLinearDistance > ROD_LINER_CLOSEST_DISTANCE then
            return -1, "RELOAD_PROCESSING"
        else
            return 0, "RELOAD_PROCESSING"
        end
    end
end

function drillOperationFunc(status, drillGroundDistance)
    if status == "UP" then
        if drillGroundDistance > DRILL_MAX_DIST_FROM_GROUND / 4 then
            return 0, "UP_COMPLETED"
        else
            return 1, "UP_PROCESSING"
        end
    end
    if status == "DOWN" then
        if drillGroundDistance < DRILL_MIN_DIST_FROM_GROUND then
            return 0, "DOWN_COMPLETED"
        else
            return -1, "DOWN_PROCESSING"
        end
    end
end

function onTick()
    rodLinearDistance = input.getNumber(1)
    drillGroundDistance = input.getNumber(2)

    isConnectedRod = input.getBool(1)
    isConnectedDrill = input.getBool(2)
    isStartRefining = input.getBool(3)
    isCheckpoint = input.getBool(4)

    if isStartRefining then
        motorOutput = -0.5
        rodLinear, statusResult = rodLinerControlFunc(status, rodLinearDistance, isConnectedRod)
        debug.log(tostring(status) .. " : " .. tostring(statusResult))
        if not rodUpDownMotion then
            if isReloadedRod == false and drillGroundDistance > DRILL_MAX_DIST_FROM_GROUND then
                isReloadRod = true
            else
                isReloadRod = false
            end

            if isReloadRod and not isConnectedDrill then
                isOutputLinerConnectRod = true
                status = "RELOAD"
                isReloadedRod = statusResult == "RELOADED"
            end

            if isReloadedRod then
                isOutputLinerConnectRod = false
                isOutputDrillConnectRod = true
                drillingOperation = true
            end

            if drillingOperation then
                status = "NEUTRAL"
                startDrillDelayCount = startDrillDelayCount + 1
                if startDrillDelayCount > START_DRILL_DELAY then
                    if drillGroundDistance > DRILL_MIN_DIST_FROM_GROUND then
                        drillLiner = -1
                        if isCheckpoint then
                            rodUpDownMotion = true
                        end
                    elseif drillGroundDistance < DRILL_MIN_DIST_FROM_GROUND then
                        drillLiner = 1
                        isReloadedRod = false
                        isOutputDrillConnectRod = false
                        drillingOperation = false
                        startDrillDelayCount = 0
                    end
                end
            end
        else
            drillLiner, drillStatusResult = drillOperationFunc(drillStatus, drillGroundDistance)
            motorOutput = 0
            drillLiner = -1
            -- if drillStatusResult == "DOWN_COMPLETED" then
            --     drillStatus = "UP"
            -- end
            -- if drillStatusResult == "UP_COMPLETED" then
            --     drillStatus = "DOWN"
            -- end
        end

        output.setNumber(1, rodLinear)
        output.setNumber(2, drillLiner)
        output.setNumber(3, motorOutput)
        output.setBool(1, isOutputLinerConnectRod)
        output.setBool(2, isOutputDrillConnectRod)
    end
end
