local isHighSpeed = false -- 高速域に達したか否か
local selfDistructCount = 0
local selfSpeedTooSlow,closingSpeedTooSlow = false,false
function onTick()
    local closingSpeedAverage = input.getNumber(3) -- 符号が正の時に離れている
    local selfSpeed = input.getNumber(1)

    
        if selfSpeed > 200 then
            isHighSpeed = true
        end
        -- 高速域に達した後50m/sまで減速したら自爆
        if isHighSpeed and selfSpeed < 50 then
            selfSpeedTooSlow = true
        end
        -- 近接速度が遅い状態が続いたら通信切断
        if isHighSpeed and closingSpeedAverage > -1 then
            selfDistructCount = selfDistructCount + 1
            if selfDistructCount > 20 then
                closingSpeedTooSlow = true
            end
        else
            selfDistructCount = 0
        end
    
    output.setBool(1, selfSpeedTooSlow) -- 自爆
    output.setBool(2, selfSpeedTooSlow or closingSpeedTooSlow) -- 切断
end
