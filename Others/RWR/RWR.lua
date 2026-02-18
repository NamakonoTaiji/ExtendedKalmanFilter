input.getNumber = input.getNumber
output.setNumber = output.setNumber
input.getBool = input.getBool
output.setBool = output.setBool
math = math
PI = math.pi
PI2 = PI * 2

screen = screen

function onTick()
    isRWRDetect = input.getBool(1)
    if isRWRDetect then
        rwrAzimuth = (1 - math.abs(input.getNumber(1)) % 1) * PI2
        debug.log(rwrAzimuth)
    else
        rwrAzimuth = nil
    end
end

function onDraw()
    local h, w, ch, cw
    h = screen.getHeight()
    w = screen.getWidth()
    ch = h / 2
    cw = w / 2
    radius = ch
    screen.setColor(0, 255, 0)
    screen.drawCircle(cw, ch, ch)
    screen.drawTriangle(32, 29, 29, 36, 35, 36)

    if rwrAzimuth then
        echoX = radius * math.sin(rwrAzimuth) + cw
        echoY = radius * math.cos(rwrAzimuth) + ch
        screen.setColor(255, 0, 0)

        screen.drawCircle(echoX, echoY, 5)
    end
end
