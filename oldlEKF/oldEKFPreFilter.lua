input.getNumber = input.getNumber
input.getBool = input.getBool
output.setNumber = output.setNumber
output.setBool = output.setBool
screen.drawText = screen.drawText
string.format = string.format
property.getNumber = property.getNumber
math = math
PI = math.pi
PI2 = PI * 2
sin, cos = math.sin, math.cos
maxSamp = -1
maxSampIO = 0
minData = math.huge
maxData = 0
maxAzimuth = 0
minAzimuth = 9
maxElev = 0
minElev = 9
outputR = 0
outputA = 0
outputE = 0
phyX, phyY, phyZ, eulerX, eulerY, eulerZ = 0, 0, 0, 0, 0, 0
function onTick()
    range = input.getNumber(1)
    azimuth = input.getNumber(2)
    elev = input.getNumber(3)

    isDetect = input.getBool(1)
    if not isDetect then
        maxData = 0
        minData = math.huge
        maxAzimuth = -9
        minAzimuth = 9
        maxElev = -9
        minElev = 9
        maxSampIO = 0
        maxSamp = -1
    end

    if isDetect then
        if maxSamp >= input.getNumber(4) then
            maxSampIO = 1
        end
        maxSamp = math.max(maxSamp, input.getNumber(4))
        if input.getNumber(4) == 0 then
            phyX = input.getNumber(8)
            phyY = input.getNumber(12)
            phyZ = input.getNumber(16)
            eulerX = input.getNumber(20)
            eulerY = input.getNumber(24)
            eulerZ = input.getNumber(28)
            maxData = 0
            minData = math.huge
            maxAzimuth = -9
            minAzimuth = 9
            maxElev = -9
            minElev = 9
        end

        maxData = math.max(range, maxData)
        minData = math.min(range, minData)
        maxAzimuth = math.max(azimuth, maxAzimuth)
        minAzimuth = math.min(azimuth, minAzimuth)
        maxElev = math.max(elev, maxElev)
        minElev = math.min(elev, minElev)

        outputR = (maxData + minData) / 2
        outputA = (maxAzimuth + minAzimuth) / 2
        outputE = (maxElev + minElev) / 2
    end
    output.setNumber(1, phyX)
    output.setNumber(2, phyY)
    output.setNumber(3, phyZ)
    output.setNumber(4, eulerX)
    output.setNumber(5, eulerY)
    output.setNumber(6, eulerZ)
    output.setNumber(20, outputR)
    output.setNumber(18, outputA)
    output.setNumber(19, outputE)
    output.setNumber(24, (maxSamp + 1) * maxSampIO)
    output.setBool(1, input.getBool(1))
end

function clamp(v, min, max)
    return math.min(math.max(v, min), max)
end

function STRF(v)
    return string.format("%1.3f", v)
end
