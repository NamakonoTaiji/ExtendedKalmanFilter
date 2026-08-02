local isHighSpeed = false -- 高速域に達したか否か
local selfDistructCount = 0
local selfSpeedTooSlow, closingSpeedTooSlow = false, false
local TIMEOUT_TICK = property.getNumber("TIMEOUT_TICK")
counter = 0
function onTick()
    local closingSpeedAverage = input.getNumber(3) -- 符号が正の時に離れている
    local selfSpeed = input.getNumber(1)
    local isLaunch = input.getNumber(2) == 1
    local isAntiShipMode = input.getBool(2)
            local isTimeOut = false
    if isLaunch then
        counter = counter + 1
        if counter > TIMEOUT_TICK then
            isTimeOut = true
        end
        if selfSpeed > 200 then
            isHighSpeed = true
        end
        -- 高速域に達した後50m/sまで減速したら自爆
        if isHighSpeed and selfSpeed < 50 then
            selfSpeedTooSlow = true
        end
        -- 近接速度が遅い状態が続いたら通信切断
        if isHighSpeed and closingSpeedAverage > -1 and counter > 120 then
            selfDistructCount = selfDistructCount + 1
            if selfDistructCount > 20 then
                closingSpeedTooSlow = true
            end
        else
            selfDistructCount = 0
        end
    end
    output.setBool(1, selfSpeedTooSlow and not isAntiShipMode or isTimeOut) -- 自爆
    output.setBool(2, selfSpeedTooSlow or closingSpeedTooSlow) -- 切断
end
