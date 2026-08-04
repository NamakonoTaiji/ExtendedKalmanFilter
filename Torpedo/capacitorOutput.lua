time = 0
FIRST_TIMER = property.getNumber("firstTimer") * 60
DELAY = property.getNumber("DELAY") * 60
outputBoolCount = 0
function onTick()
    onOff = input.getBool(1)

    if onOff then
        time = time + 1
    else
        time = 0
    end

    if time > FIRST_TIMER and outputBoolCount < 1 then
        outputBoolCount = outputBoolCount + 1
        time = 0
    elseif time > DELAY and outputBoolCount >= 1 then
        outputBoolCount = outputBoolCount + 1
        time = 0
    end

    output.setBool(outputBoolCount,true)
end