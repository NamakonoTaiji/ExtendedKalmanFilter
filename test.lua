iN = input.getNumber
iB = input.getBool
oN = output.setNumber
oB = output.setBool
DT = screen.drawText
SF = string.format
M = math
pi = M.pi
harfpi = 1 / 2 * pi
pi2 = pi * 2
sin, cos = M.sin, M.cos
matrix = {}
function LocaltoWorld(the, phi, psi, Lx, Ly, Lz)
    local matrix
    matrix = {
        x = cos(phi) * cos(psi) * Lx + (sin(the) * sin(phi) * cos(psi) - cos(the) * sin(psi)) * Ly +
            (cos(the) * sin(phi) * cos(psi) + sin(the) * sin(psi)) * Lz,
        y = cos(phi) * sin(psi) * Lx + (sin(the) * sin(phi) * sin(psi) + cos(the) * cos(psi)) * Ly +
            (cos(the) * sin(phi) * sin(psi) - sin(the) * cos(psi)) * Lz,
        z = -sin(phi) * Lx + sin(the) * cos(phi) * Ly + cos(the) * cos(phi) * Lz
    }
    return matrix
end

function onTick()
    isDetect = iB(1)
    azimuth = iN(2) * pi2
    elev = iN(3) * pi2
    dist = iN(1)
    the = iN(4)
    phi = iN(5)
    psi = iN(6)
    if isDetect then
        localX = dist * math.cos(elev) * math.sin(azimuth)
        localY = dist * math.sin(elev)
        localZ = dist * math.cos(elev) * math.cos(azimuth)
        matrix = LocaltoWorld(the, phi, psi, localX, localY, localZ)
    end
end

function onDraw()
    if isDetect then
        DT(0, 0, STRF(matrix.x) .. " GX")
        DT(0, 6, STRF(matrix.y) .. " GY")
        DT(0, 12, STRF(matrix.z) .. " GZ")
        DT(70, 0, STRF(localX) .. " LX")
        DT(70, 6, STRF(localY) .. " LY")
        DT(70, 12, STRF(localZ) .. " LZ")
    end
end

function clamp(v, min, max)
    return math.min(math.max(v, min), max)
end

function STRF(v)
    return SF("%4.3f", v)
end
