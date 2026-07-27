local selfID = 0
local selfFreq = 0

function onTick()
    local selfIDFromVLSFCS = input.getNumber(13)
    local selfFreqFromVLSFCS = input.getNumber(14)
    local isLaunched = input.getNumber(32) ~= 114514

    if selfIDFromVLSFCS ~= 0 then
        selfID = selfIDFromVLSFCS
    end
    if selfFreqFromVLSFCS ~= 0 then
        selfFreq = selfFreqFromVLSFCS
    end

    if isLaunched then
        output.setNumber(13, selfID)  -- 火器管制へ送信する生存確認ID
        output.setNumber(1, selfFreq) -- 自身の無線機に設定する無線周波数
    else
        output.setNumber(13, 0)
        output.setNumber(1, 0)
    end
end
