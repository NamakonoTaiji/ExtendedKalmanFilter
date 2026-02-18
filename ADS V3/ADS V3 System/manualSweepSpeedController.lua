SWEEP_SPEED = property.getNumber("SweepSpeed") * 0.0001
function onTick()
    upCounter = input.getNumber(1)
    sweepX = upCounter * SWEEP_SPEED

    output.setNumber(1, sweepX)
end
