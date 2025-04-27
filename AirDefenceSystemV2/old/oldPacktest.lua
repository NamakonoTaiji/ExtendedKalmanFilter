PI = math.pi
PI2 = PI * 2
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

function VariableUnpack(a, b)
    local eabs, fabs, d, e, f, s
    local signList = { -1, 1 }
    if a * b == 0 then
        return 0, 0, 0, 0
    end
    local list, i, j = { { 1, 2 }, { 3, 4 } }, 1, 1
    if a > 0 then
        i = 2
    end
    if b > 0 then
        j = 2
    end
    s = list[i][j]
    a = string.format("%7f", math.abs(a))
    b = string.format("%7f", math.abs(b))
    d = tonumber(a:sub(6, 7) .. b:sub(6, -1))
    eabs = tonumber("0." .. a:sub(2, 5))
    fabs = tonumber("0." .. b:sub(2, 5))
    e = eabs * signList[tonumber(a:sub(1, 1))] * PI2
    f = fabs * signList[tonumber(b:sub(1, 1))] * PI2
    e = ((e + PI / 2 * (s - 1)) % PI2 + PI) % PI2 - PI
    return d, e, f, s - 1
end

x1, x2 = Pack(100, 0.1, 0.05)
print(x1, x2)

d, e, f, s = VariableUnpack(x1, x2)
print("distance:" .. d, "azimuth:" .. e, "elevation:" .. f, "id:" .. s)
