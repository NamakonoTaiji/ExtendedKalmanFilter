WEP_LIMIT_TEMPERATURE = property.getNumber("WEP_LIMIT_TEMPERATURE")
WEP_THROTTLE = property.getNumber("WEP_THROTTLE")
function onTick()
    inputThrottle = input.getNumber(4)
    temperature = input.getNumber(1)
    engineRPS = input.getNumber(2)
    isSeatAnyone = input.getBool(32)
    outputThrottle = WEP_THROTTLE
    if temperature > WEP_LIMIT_TEMPERATURE then
        outputThrottle = 0.1
    end

    if math["math.abs"](inputThrottle) < 0.01 then
        outputThrottle = 0.03
    end

    if isSeatAnyone and engineRPS < 2 and math["math.abs"](inputThrottle) > 0 then
        engineStarter = true
    else
        engineStarter = false
    end

    output.setNumber(1, outputThrottle)
    output.setBool(1, engineStarter)
end
