function onTick()
    local diffX = (input.getNumber(8) - input.getNumber(26)) ^ 2
    local diffY = (input.getNumber(12) - input.getNumber(27)) ^ 2
    local diffZ = (input.getNumber(16) - input.getNumber(28)) ^ 2
    local distanceSq = diffX + diffY + diffZ
    output.setNumber(1, distanceSq)
end
