SWEEP_SPEED = property.getNumber("SweepSpeed")
function onTick()
    upCounter = input.getNumber(1)
    sweepX = upCounter * SWEEP_SPEED

    output.setNumber(1, sweepX)
end
