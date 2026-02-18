local compositeController1 = false
local compositeController2 = false

function onTick()
    local isRadarActive = input.getBool(1)
    if isRadarActive then
        compositeController1 = not compositeController1
        if compositeController1 then
            compositeController2 = not compositeController2
        end
    else
        compositeController1 = false
        compositeController2 = false
    end

    output.setBool(31, compositeController1)
    output.setBool(32, compositeController2)
end
