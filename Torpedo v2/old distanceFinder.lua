function f_prime(dtemp, E, A, selfX, selfY, selfZ)
    local cosE_sinA = math.cos(E) * math.sin(A)
    local sinE = math.sin(E)
    local cosE_cosA = math.cos(E) * math.cos(A)

    local term1 = dtemp * cosE_sinA + selfX
    local term2 = dtemp * sinE + selfY
    local term3 = dtemp * cosE_cosA + selfZ

    local numerator = term1 * cosE_sinA + term2 * sinE + term3 * cosE_cosA
    local denominator = math.sqrt(term1 ^ 2 + term2 ^ 2 + term3 ^ 2)

    return numerator / denominator + 1
end

function f(dtemp, E, A, selfX, selfY, selfZ, sSpeed, pingTimeTick)
    return math.sqrt((dtemp * math.cos(E) * math.sin(A) + selfX) ^ 2 +
            (dtemp * math.sin(E) + selfY) ^ 2 +
            (dtemp * math.cos(E) * math.cos(A) + selfZ) ^ 2) +
        dtemp - (sSpeed * pingTimeTick)
end

-- ニュートン法による解の求め方
function NewtonMethod(dtemp, E, A, selfX, selfY, selfZ, sSpeed, pingTimeTick)
    local d = dtemp
    for i = 1, 15 do
        local f_val = f(d, E, A, selfX, selfY, selfZ, sSpeed, pingTimeTick)
        local f_prime_val = f_prime(d, E, A, selfX, selfY, selfZ)
        local d_new = d - f_val / f_prime_val
        if math.abs(d - d_new) < 0.00001 then
            break
        end
        d = d_new
    end
    return d
end

function angleToXYZ(radarDist, radarEle, radarAzi)
    basehypot = radarDist * math.cos(radarEle)
    radarX = radarDist * math.cos(radarEle) * math.sin(radarAzi) --left/right
    radarY = radarDist * math.sin(radarEle)                      --height
    radarZ = radarDist * math.cos(radarEle) * math.cos(radarAzi) --front/back
    radarX = radarX
    radarY = radarY
    radarZ = radarZ
    return radarX, radarY, radarZ
end

pi2 = math.pi * 2
-- パラメータの定義
globalX, globalY, globalZ = 0, 0, 0
initDist = math.sqrt(globalX ^ 2 + globalY ^ 2 + globalZ ^ 2)
--print("initDist:", initDist)
selfX, selfY, selfZ = -0.24, 0.44, 247.37
relX = globalX - selfX
relY = globalY - selfY
relZ = globalZ - selfZ
dist = math.sqrt(relX ^ 2 + relY ^ 2 + relZ ^ 2)
azi = math.tan(relX / relZ)
--print("Acctualdist:", dist)

sSpeed = 1311 / 60
--pingTimeTick = dist / sSpeed + initDist / sSpeed
pingTimeTick = 83
--print("pingTimeTick:", pingTimeTick)
azimuth = 0.0068 * pi2
elev = 0.0006 * pi2

-- 初期値の設定
dist1 = sSpeed * pingTimeTick / 2

-- ニュートン法による解の求め方
d = NewtonMethod(dist1, elev, azimuth, selfX, selfY, selfZ, sSpeed, pingTimeTick)
print("求められたd:", d)
