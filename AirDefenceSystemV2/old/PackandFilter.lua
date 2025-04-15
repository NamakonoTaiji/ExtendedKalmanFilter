iN = input.getNumber
iB = input.getBool
oN = output.setNumber
oB = output.setBool
M = math
pi = M.pi
pi2 = pi * 2
n = property.getNumber("n")

targetN = 6
maxDatas = {}
minDatas = {}
outputR = 0
outputA = 0
outputE = 0

function Pack(a, b, c)
    local d, e, f, x1, x2, bsgn, csgn, cabs, babs, q
    if a == 0 then
        return 0, 0, 0
    end

    if math.log(a, 10) >= 3 then
        a = a + 5 * 10 ^ (math.floor(math.log(a, 10)) - 4)
        q = 0
    else
        q = 1
        a = a + 5 * 10 ^ (math.floor(math.log(a, 10)) - 4)
    end

    bsgn, babs = sign(b)
    csgn, cabs = sign(c)

    babs = babs + 0.00005
    cabs = cabs + 0.00005


    e = string.format("%4f", babs)
    f = string.format("%4f", cabs)
    d = string.sub(string.format("%4f", a), 1, 4 + q)
    e = string.sub(e, string.find(e, "%.") + 1, string.find(e, "%.") + 4)
    f = string.sub(f, string.find(f, "%.") + 1, string.find(f, "%.") + 4)
    x1 = tonumber(bsgn .. e .. string.sub(d, 1, 2))
    x2 = tonumber(csgn .. f .. string.sub(d, 3, 4 + q))
    return x1, x2
end

function sign(a)
    local sgn
    sgn = 2
    if a < 0 then
        sgn = 1
    end
    return tostring(sgn), math.abs(a)
end

function onTick()
    for i = 1, 6 do
        oN(i * 2 - 1, 0)
        oN(i * 2, 0)
    end
    isDetect = iB(1)
    if isDetect then
        if iN(4) == 0 then
            for i = 1, 6 do
                maxDatas[i] = { dis = 0, azi = -1, ele = -1 }
            end

            for i = 1, 6 do
                minDatas[i] = { dis = 9999999, azi = 1, ele = 1 }
            end
        end

        for i = 1, 6 do
            if iB(i) then
                local Rdis, Razi, Rele
                Rdis = iN(i * 4 - 3)
                Razi = iN(i * 4 - 2)
                Rele = iN(i * 4 - 1)
                maxDatas[i] = {
                    dis = M.max(Rdis, maxDatas[i].dis),
                    azi = M.max(Razi, maxDatas[i].azi),
                    ele = M.max(Rele,
                        maxDatas[i].ele)
                }
                minDatas[i] = {
                    dis = M.min(Rdis, minDatas[i].dis),
                    azi = M.min(Razi, minDatas[i].azi),
                    ele = M.min(Rele,
                        minDatas[i].ele)
                }
            end
        end

        if iN(4) + 1 == n then
            for i = #maxDatas, 1, -1 do
                if maxDatas[i].dis == 0 then
                    table.remove(maxDatas, i)
                end
            end
            for i = 1, M.min(6, #maxDatas) do
                if iB(i) then
                    outputR = (maxDatas[i].dis + minDatas[i].dis) / 2
                    outputA = (maxDatas[i].azi + minDatas[i].azi) / 2
                    outputE = (maxDatas[i].ele + minDatas[i].ele) / 2
                    if outputR > 100 then
                        pack1, pack2 = Pack(outputR, outputA, outputE)
                        oN(i * 2 - 1, -pack1) --[[Radar1: -- Radar2: -+ Radar3: +- Radar4 ++]]
                        oN(i * 2, -pack2)
                    end
                end
            end
        end
    end
end
