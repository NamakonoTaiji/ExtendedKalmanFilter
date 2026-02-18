-- 1ick

--
-- 1ch:

--
-- 1 ~ 4ch: 1~4
local radar1IO = false
local radar2IO = false
local radar3IO = false
local radar4IO = false

local compositeController1 = false
local compositeController2 = false

local isRadarOnFlag = false

local count = 0
function onTick()
    local systemIO = input.getBool(1)

    if systemIO and count % 4 == 0 then
        isRadarOnFlag = true
    elseif not systemIO and count % 4 == 0 then
        isRadarOnFlag = false
    end

    if isRadarOnFlag then
        radar4IO = radar3IO
        radar3IO = radar2IO
        radar2IO = radar1IO
        radar1IO = isRadarOnFlag
    end
    if not isRadarOnFlag then
        if radar4IO and not radar3IO then
            radar4IO = false
        end
        if radar3IO and not radar2IO then
            radar3IO = false
        end
        if radar2IO and not radar1IO then
            radar2IO = false
        end
        if radar1IO and not isRadarOnFlag then
            radar1IO = false
        end
    end

    if isRadarOnFlag then
        compositeController1 = not compositeController1
        if compositeController1 then
            compositeController2 = not compositeController2
        end
    else
        compositeController1 = false
        compositeController2 = false
    end

    count = count + 1

    output.setBool(31, compositeController1)
    output.setBool(32, compositeController2)

    output.setBool(1, radar1IO)
    output.setBool(2, radar2IO)
    output.setBool(3, radar3IO)
    output.setBool(4, radar4IO)
end
