iN = input.getNumber
iB = input.getBool
oN = output.setNumber
oB = output.setBool
DT = screen.drawText
SF = string.format
math = math
pi = math.pi
pi2 = pi * 2
sin, cos = math.sin, math.cos
azimuthL, elevL = 0, 0
dt = 1 / 60
R = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }

distance, azimuthL, elevL = 1, 0, 0

function ZYXRotate(the, phi, psi)
    local cY = math.cos(phi); local sY = math.sin(phi); local cP = math.cos(the); local sP = math.sin(the); local cR =
        math.cos(psi); local sR = math.sin(psi)

    -- R_User = [[cYcR, sPsYcR-cPsR, cPsYcR+sPsR], [cYsR, sPsYsR+cPcR, cPsYsR-sPcR], [-sY, sP*cY, cP*cY]]
    -- s R_B2W  R_User B
    -- s Local {Right, Up, Forward} xNg|AWorld {East, Altitude, North} xNgB
    local R_B2W = {
        { cY * cR, sP * sY * cR - cP * sR, cP * sY * cR + sP * sR }, -- Row 1 -> Outputs East
        { cY * sR, sP * sY * sR + cP * cR, cP * sY * sR - sP * cR }, -- Row 2 -> Outputs Altitude/Up
        { -sY,     sP * cY,                cP * cY }                 -- Row 3 -> Outputs North
    }
    return transpose(R_B2W)
end

function onTick()
    xyz = { { iN(1) - iN(7) }, { iN(3) - iN(8) }, { iN(2) - iN(9) } }

    if iN(1) ^ 2 + iN(2) ^ 2 + iN(3) ^ 2 > 0 then
        eulerX = iN(4)
        eulerY = iN(5)
        eulerZ = iN(6)
        eulerMatR = ZYXRotate(eulerX, eulerY, eulerZ)
        --eulerMatDR=ZYXRotate(-avX*dt,-avY*dt,-avZ*dt)
        localXYZ = multiplier(eulerMatR, xyz)
        --localXYZ=multiplier((eulerMatDR),localXYZ)
        distance = math.sqrt(localXYZ[1][1] ^ 2 + localXYZ[2][1] ^ 2 + localXYZ[3][1] ^ 2)
        azimuthL = math.atan(localXYZ[1][1], localXYZ[3][1])
        elevL = math.asin(localXYZ[2][1] / distance)
    end
    oN(1, distance)
    oN(2, azimuthL)
    oN(3, elevL)
    oN(4, iN(7))
end

function onDraw()
    DT(90, 0, STRF(azimuthL) .. " aziL")
    DT(90, 6, STRF(elevL) .. " elevL")
    DT(90, 12, STRF(distance) .. " dist")
end

function clamp(v, min, max)
    return math.min(math.max(v, min), max)
end

function STRF(v)
    return SF("%1.3f", v)
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
