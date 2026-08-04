detected = false
timeOutCounter = 0
targetCoords = { 0, 0, 0 }
targetSpd = { 0, 0, 0 }
targetAv = { 0, 0, 0 }
function onTick()
    id = input.getNumber(12)
    selectedId = math.max(input.getNumber(32),1)
    if timeOutCounter > 30 then
        detected = false
    end
    isDetecting = id == selectedId
    if isDetecting then
        detected = true
        targetCoords = { input.getNumber(1), input.getNumber(2), input.getNumber(3) }
        targetSpd = { input.getNumber(4), input.getNumber(5), input.getNumber(6) }
        targetAv = { input.getNumber(7), input.getNumber(8), input.getNumber(9) }

        timeOutCounter = 0
    else
        timeOutCounter = timeOutCounter + 1
    end

    output.setNumber(1, targetCoords[1])
    output.setNumber(2, targetCoords[2])
    output.setNumber(3, targetCoords[3])
    output.setNumber(4, targetSpd[4])
    output.setNumber(5, targetSpd[5])
    output.setNumber(6, targetSpd[6])
    output.setNumber(7, targetAv[7])
    output.setNumber(8, targetAv[8])
    output.setNumber(9, targetAv[9])
    output.setBool(1,detected)
end
