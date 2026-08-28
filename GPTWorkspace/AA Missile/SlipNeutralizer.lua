SLIP_CNTRL = property.getNumber("SLIP_CNTRL")

function onTick()
    local velocityX      = input.getNumber(7)
    local velocityY      = input.getNumber(8)
    local velocityZ      = input.getNumber(9)

    local forwardSpeed   = math.max(math.abs(velocityZ), 1)

    local horizontalSlip = math.atan(velocityX, forwardSpeed)
    local verticalSlip   = math.atan(velocityY, forwardSpeed)

    local yawCommand     = -horizontalSlip * SLIP_CNTRL
    local pitchCommand   = -verticalSlip * SLIP_CNTRL

    output.setNumber(1, yawCommand)
    output.setNumber(2, pitchCommand)
end
