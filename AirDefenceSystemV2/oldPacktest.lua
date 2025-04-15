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

x1, x2 = Pack(999, 0.0, 0.1)
print(x1, x2)
