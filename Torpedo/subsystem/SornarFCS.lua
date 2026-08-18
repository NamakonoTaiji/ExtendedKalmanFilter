DT = 1 / 60

detected = false
timeOutCounter = 0

targetCoords = {0, 0, 0}
targetSpd = {0, 0, 0}
targetAv = {0, 0, 0}
targetID = 0

function onTick()
    -- 1～10 = Sonar
    -- 11～20 = Radar
    local sonarId = input.getNumber(10)
    local radarId = input.getNumber(20)

    ----------------------------------------------------------------
    -- TrackManagerから来るencoded IDを復号
    --
    -- Radar n -> n*10
    -- Sonar n -> n*10+1
    ----------------------------------------------------------------
    local encodedId =
        math.floor(input.getNumber(32) + 0.5)

    local source = encodedId % 10
    local selectedId =
        math.floor(encodedId / 10)

    local selectedSonar =
        source == 1 and selectedId ~= 0

    local selectedRadar =
        source == 0 and selectedId ~= 0

    ----------------------------------------------------------------
    -- 現在の時分割packetがLock対象か
    ----------------------------------------------------------------
    local sonarMatch =
        selectedSonar and
        sonarId == selectedId and
        sonarId ~= 0

    local radarMatch =
        selectedRadar and
        radarId == selectedId and
        radarId ~= 0

    ----------------------------------------------------------------
    -- 新しいLock対象packetを受信
    ----------------------------------------------------------------
    if sonarMatch then
        detected = true

        targetCoords = {
            input.getNumber(1),
            input.getNumber(2),
            input.getNumber(3)
        }

        targetSpd = {
            input.getNumber(4),
            input.getNumber(5),
            input.getNumber(6)
        }

        targetAv = {
            input.getNumber(7),
            input.getNumber(8),
            input.getNumber(9)
        }

        targetID = sonarId
        timeOutCounter = 0

    elseif radarMatch then
        detected = true

        targetCoords = {
            input.getNumber(11),
            input.getNumber(12),
            input.getNumber(13)
        }

        targetSpd = {
            input.getNumber(14),
            input.getNumber(15),
            input.getNumber(16)
        }

        targetAv = {
            input.getNumber(17),
            input.getNumber(18),
            input.getNumber(19)
        }

        targetID = radarId
        timeOutCounter = 0

    ----------------------------------------------------------------
    -- packetが来ていない間は速度で補間
    ----------------------------------------------------------------
    elseif detected then
        targetCoords = {
            targetCoords[1] + targetSpd[1] * DT,
            targetCoords[2] + targetSpd[2] * DT,
            targetCoords[3] + targetSpd[3] * DT
        }

        timeOutCounter = timeOutCounter + 1

        if timeOutCounter > 30 then
            detected = false
        end

    else
        targetCoords = {0, 0, 0}
        targetSpd = {0, 0, 0}
        targetAv = {0, 0, 0}
        targetID = 0

        timeOutCounter = timeOutCounter + 1
    end

    ----------------------------------------------------------------
    -- Output
    ----------------------------------------------------------------
    output.setBool(1, detected)

    -- 有効なTrackManager IDが入力されているか
    output.setBool(
        2,
        selectedSonar or selectedRadar
    )

    if detected then
        output.setNumber(1, targetCoords[1])
        output.setNumber(2, targetCoords[2])
        output.setNumber(3, targetCoords[3])

        output.setNumber(4, targetSpd[1])
        output.setNumber(5, targetSpd[2])
        output.setNumber(6, targetSpd[3])

        output.setNumber(7, targetAv[1])
        output.setNumber(8, targetAv[2])
        output.setNumber(9, targetAv[3])

        output.setNumber(12, targetID)

    else
        for i = 1, 12 do
            output.setNumber(i, 0)
        end
    end
end