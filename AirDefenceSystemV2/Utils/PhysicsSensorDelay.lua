local RADAR_INTERVAL_DELAY = property.getNumber("n")
local LOGIC_DELAY = 4
local selfBuffer = {}
local outputBuffer = { 0, 0, 0, 0, 0, 0 }
function onTick()
    local ownP = {
        input.getNumber(1),
        input.getNumber(2),
        input.getNumber(3),
        input.getNumber(4),
        input.getNumber(5),
        input.getNumber(6),
    }
    local isTargetUpdated = input.getNumber(7) == 0
    table.insert(selfBuffer, ownP)
    if #selfBuffer > RADAR_INTERVAL_DELAY + LOGIC_DELAY then
        table.remove(selfBuffer, 1)
    end

    if #selfBuffer > 0 then
        if isTargetUpdated then
            for i = 1, 6 do
                outputBuffer[i] = selfBuffer[1][i]
            end
        end
        for i = 1, 6 do
            output.setNumber(i, outputBuffer[i])
        end
    else
        for i = 1, 6 do
            output.setNumber(i, 0)
        end
    end
end
