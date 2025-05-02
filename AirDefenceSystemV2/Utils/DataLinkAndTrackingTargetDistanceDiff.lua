-- データリンクの強制と目標が有効射程圏内にいるか判定する

local TARGET_DATALINK_DIFF_THRESHOLD_SQ = property.getNumber("Target_DataLink_Diff_Threshold") ^
    2 -- データリンクを強制させる補足中目標とデータリンク目標の距離の閾値
local RADAR_EFFECTIVE_RANGE_SQ = (property.getNumber("RadarEffectiveRange") - 100) ^
    2 -- レーダーの有効範囲.確実にレーダーにとらえるために100m減算
local dataLinkUpdateTimer = 0
function onTick()
    local dataLinkX = input.getNumber(1)
    local dataLinkY = input.getNumber(2)
    local dataLinkZ = input.getNumber(3)
    local trackingX = input.getNumber(4)
    local trackingY = input.getNumber(5)
    local trackingZ = input.getNumber(6)
    local isDataLinkUpdated = input.getBool(4)

    local diffX = (input.getNumber(7) - input.getNumber(10)) ^ 2
    local diffY = (input.getNumber(8) - input.getNumber(11)) ^ 2
    local diffZ = (input.getNumber(9) - input.getNumber(12)) ^ 2
    local distanceSq = diffX + diffY + diffZ
    local isRadarEffectiveRange = distanceSq < RADAR_EFFECTIVE_RANGE_SQ
    output.setBool(2, isRadarEffectiveRange) -- レーダー圏内かどうかを出力
    if isDataLinkUpdated then
        dataLinkUpdateTimer = 0
    else
        dataLinkUpdateTimer = dataLinkUpdateTimer + 1
    end
    local isDataLinkStopped = false
    if dataLinkUpdateTimer > 90 then
        isDataLinkStopped = true
    end
    output.setBool(3, isDataLinkStopped) -- データリンクの更新が停止しているか監視
    if (trackingX ~= 0 or trackingY ~= 0 or trackingZ ~= 0) and isDataLinkUpdated then
        local distanceDiff = (dataLinkX - trackingX) ^ 2 + (dataLinkY - trackingY) ^ 2 + (dataLinkZ - trackingZ) ^ 2
        if distanceDiff < TARGET_DATALINK_DIFF_THRESHOLD_SQ then
            output.setBool(1, false) -- 閾値以内ならデータリンクを強制しない
        else
            output.setBool(1, true)  -- 閾値を超えたらデータリンク強制
        end
        --
    else
        output.setBool(1, false)
    end
end
