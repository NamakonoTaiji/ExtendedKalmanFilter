local TARGET_DATALINK_DIFF_THRESHOLD_SQ = property.getNumber("Target_DataLink_Diff_Threshold") ^ 2

function onTick()
    local dataLinkX = input.getNumber(1)
    local dataLinkY = input.getNumber(2)
    local dataLinkZ = input.getNumber(3)
    local trackingX = input.getNumber(4)
    local trackingY = input.getNumber(5)
    local trackingZ = input.getNumber(6)
    local isDataLinkUpdated = input.getBool(4)
    if (trackingX ~= 0 or trackingY ~= 0 or trackingZ ~= 0) and isDataLinkUpdated then
        local distanceDiff = (dataLinkX - trackingX) ^ 2 + (dataLinkY - trackingY) ^ 2 + (dataLinkZ - trackingZ) ^ 2
        if distanceDiff < TARGET_DATALINK_DIFF_THRESHOLD_SQ then
            output.setBool(1, true) -- f[^NstO
        end
        -- f[^NstO
    else
        output.setBool(1, false) -- f[^NstO
    end
end
