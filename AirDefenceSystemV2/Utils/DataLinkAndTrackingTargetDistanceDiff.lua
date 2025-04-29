local TARGET_DATALINK_DIFF_THRESHOLD_SQ = property.getNumber("Target_DataLink_Diff_Threshold") ^ 2

function onTick()
    local dataLinkX = input.getBool(1)
    local dataLinkY = input.getBool(2)
    local dataLinkZ = input.getBool(3)
    local trackingX = input.getBool(4)
    local trackingY = input.getBool(5)
    local trackingZ = input.getBool(6)
    local isDataLinkUpdated = input.getBool(4)
    if (trackingX ~= 0 or trackingY ~= 0 or trackingZ ~= 0) and isDataLinkUpdated then
        local distanceDiff = (dataLinkX - trackingX) ^ 2 + (dataLinkY - trackingY) ^ 2 + (dataLinkZ - trackingZ) ^ 2
        if distanceDiff < TARGET_DATALINK_DIFF_THRESHOLD_SQ then
            output.setBool(1, true) -- 再データリンク実行フラグ
        end
        -- 再データリンク実行フラグ
    else
        output.setBool(1, false) -- 再データリンク実行フラグ
    end
end
