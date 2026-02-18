input.getNumber = input.getNumber
input.getBool = input.getBool
output.setNumber = output.setNumber
output.setBool = output.setBool
screen.drawText = screen.drawText
string.format = string.format
math = math
PI = math.pi
PI2 = PI * 2
DT = 1 / 60
sin, cos = math.sin, math.cos
azimuthG, elevG = 0, 0
R = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }
function scalar(a, Mat)
    local r = {}
    for i = 1, #Mat do
        r[i] = {}
        for j = 1, #Mat[1] do
            r[i][j] = Mat[i][j] * a
        end
    end
    return r
end

function ZYXRotate(the, phi, psi)
    RX = { { 1, 0, 0 }, { 0, cos(the), -sin(the) }, { 0, sin(the), cos(the) } }
    RY = { { cos(phi), 0, sin(phi) }, { 0, 1, 0 }, { -sin(phi), 0, cos(phi) } }
    RZ = { { cos(psi), -sin(psi), 0 }, { sin(psi), cos(psi), 0 }, { 0, 0, 1 } }
    matrix = multiplier((RX), multiplier((RY), (RZ)))
    return matrix
end

function onTick()
    azimuth = input.getNumber(18) * PI2
    elev = input.getNumber(19) * PI2
    eulerX = input.getNumber(4)
    eulerY = input.getNumber(5)
    eulerZ = input.getNumber(6)
    eulerMatR = ZYXRotate(-eulerX, -eulerY, -eulerZ)
    Rthetaphi = { { cos(azimuth), -sin(elev) * sin(azimuth), cos(elev) * sin(azimuth) },
        { 0,             cos(elev),                 sin(elev) },
        { -sin(azimuth), -sin(elev) * cos(azimuth), cos(elev) * cos(azimuth) } }
    R = multiplier(transpose(eulerMatR), Rthetaphi)
    elevG = math.asin(R[2][3])
    azimuthG = math.atan(R[1][3], R[3][3])
    output.setNumber(1, input.getNumber(20))
    output.setNumber(2, azimuthG)
    output.setNumber(3, elevG)
    output.setNumber(10, input.getNumber(24))
    output.setNumber(11, input.getNumber(1))
    output.setNumber(12, input.getNumber(2))
    output.setNumber(13, input.getNumber(3))
    output.setBool(2, input.getBool(1))
end

function inv(M)
    local n = #M
    local r = {}
    for i = 1, n do
        r[i] = {}
        for j = 1, n do r[i][j] = (i == j) and 1 or 0 end
    end
    for i = 1, n do
        local pv = M[i][i]
        for j = 1, n do
            M[i][j] = M[i][j] / pv
            r[i][j] = r[i][j] / pv
        end
        for k = 1, n do
            if k ~= i then
                local f = M[k][i]
                for j = 1, n do
                    M[k][j] = M[k][j] - f * M[i][j]
                    r[k][j] = r[k][j] - f * r[i][j]
                end
            end
        end
    end
    return r
end

function multiplier(A, B)
    if #A[1] ~= #B then return nil end
    local r = {}
    for i = 1, #A do r[i] = {} end
    for i = 1, #A do
        for j = 1, #B[1] do
            local s = 0
            for k = 1, #B do s = s + A[i][k] * B[k][j] end
            r[i][j] = s
        end
    end
    return r
end

function transpose(M)
    local r = {}
    for i = 1, #M[1] do
        r[i] = {}
        for j = 1, #M do r[i][j] = M[j][i] end
    end
    return r
end
