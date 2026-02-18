input.getNumber = input.getNumber
input.getBool = input.getBool
output.setNumber = output.setNumber
output.setBool = output.setBool
math = math
sqrt = math.sqrt
math["math.abs"] = math["math.abs"]
PI = math.pi
PI2 = PI * 2
EXP = math.exp(1)
Dl = debug.log
function zeros(m, g)
    local c = {}
    for _ = 1, m do
        c[_] = {}
        for a = 1, g do
            c[_][a] = 0
        end
    end
    return c
end

function scalar(l, h)
    local b = {}
    for _ = 1, #h do
        b[_] = {}
        for a = 1, #h[1] do
            b[_][a] = h[_][a] * l
        end
    end
    return b
end

function sum(d, f)
    local b = {}
    for _ = 1, #d do
        b[_] = {}
        for a = 1, #d[1] do
            b[_][a] = d[_][a] + f[_][a]
        end
    end
    return b
end

function sub(d, f)
    local b = {}
    for _ = 1, #d do
        b[_] = {}
        for a = 1, #d[1] do
            b[_][a] = d[_][a] - f[_][a]
        end
    end
    return b
end

function mul(d, f)
    if #d[1] ~= #f then return nil end
    local b = {}
    for _ = 1, #d do b[_] = {} end
    for _ = 1, #d do
        for a = 1, #f[1] do
            local i = 0
            for e = 1, #f do i = i + d[_][e] * f[e][a] end
            b[_][a] = i
        end
    end
    return b
end

function inv(c)
    local g = #c
    local b = {}
    for _ = 1, g do
        b[_] = {}
        for a = 1, g do b[_][a] = (_ == a) and 1 or 0 end
    end
    for _ = 1, g do
        local k = c[_][_]
        for a = 1, g do
            c[_][a] = c[_][a] / k
            b[_][a] = b[_][a] / k
        end
        for e = 1, g do
            if e ~= _ then
                local j = c[e][_]
                for a = 1, g do
                    c[e][a] = c[e][a] - j * c[_][a]
                    b[e][a] = b[e][a] - j * b[_][a]
                end
            end
        end
    end
    return b
end

function T(c)
    local b = {}
    for _ = 1, #c[1] do
        b[_] = {}
        for a = 1, #c do b[_][a] = c[a][_] end
    end
    return b
end

predX = zeros(6, 1)
I = { { 1, 0, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } }
X = zeros(6, 1)
P = zeros(6, 6)
R = zeros(3, 3)
obsZ = zeros(3, 1)
EPSILON = 0
isInit = true
dtU = zeros(6, 1)
c = 1
function KF(F, Q, R, Z, X, P, I)
    local K, S, Y
    --pre
    X = mul(F, X)
    P = sum(scalar((1.01) ^ 2 * (1 + (1.01 / (1 + EXP ^ (-(EPSILON - 9) * 5)))), mul(F, mul(P, T(F)))),
        scalar((1 + (1e+9 / (1 + EXP ^ (-(EPSILON - 30) * 100)))), Q))
    --up
    local x, y, z = X[1][1], X[3][1], X[5][1]
    local r, rh = x ^ 2 + y ^ 2 + z ^ 2, x ^ 2 + y ^ 2
    local H = { { x / sqrt(r),   0, y / sqrt(r),             0, z / sqrt(r),  0 },
        { (-x * z) / (sqrt(rh) * r), 0, -y * z / (sqrt(rh) * r), 0, sqrt(rh) / r, 0 },
        { (y / rh),                  0, (-x / rh),               0, 0,            0 } }
    local h = { { sqrt(r) }, { math.asin(z / sqrt(r)) }, { math.atan(x, y) } }
    Y = sub(Z, h)
    Y[3][1] = (Y[3][1] + 3 * PI) % PI2 - PI
    S = sum(R, mul(H, mul(P, T(H))))
    EPSILON = mul(mul(T(Y), inv(S)), Y)[1][1]
    K = mul(mul(P, T(H)), inv(sum(R, mul(H, mul(P, T(H))))))
    X = sum(X, mul(K, Y))
    P = sum(mul(mul(sub(I, mul(K, H)), P), T(sub(I, mul(K, H)))), mul(K, mul(R, T(K))))
    return X, P, EPSILON
end

function onTick()
    n = input.getNumber(10)
    DT = n / 60
    Q = scalar(0.00004,
        { { DT ^ 4 / 4, DT ^ 3 / 2, 0, 0, 0, 0 }, { DT ^ 3 / 2, DT ^ 2, 0, 0, 0, 0 }, { 0, 0, DT ^ 4 / 4, DT ^ 3 / 2, 0, 0 }, { 0, 0, DT ^ 3 / 2, DT ^ 2, 0, 0 }, { 0, 0, 0, 0, DT ^ 4 / 4, DT ^ 3 / 2 }, { 0, 0, 0, 0, DT ^ 3 / 2, DT ^ 2 } })
    F = { { 1, DT, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, DT, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, DT }, { 0, 0, 0, 0, 0, 1 } }
    dtTemp = c / 60
    FTemp = { { 1, dtTemp, 0, 0, 0, 0 }, { 0, 1, 0, 0, 0, 0 }, { 0, 0, 1, dtTemp, 0, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, dtTemp }, { 0, 0, 0, 0, 0, 1 } }
    obsZ = { { input.getNumber(1) }, { input.getNumber(3) }, { input.getNumber(2) } }
    RDetect = input.getBool(2)
    if RDetect and n ~= 0 then
        if isInit then
            X = { { math.sin(obsZ[3][1]) * obsZ[1][1] * math.cos(obsZ[2][1]) }, { 0 },
                { obsZ[1][1] * math.cos(obsZ[2][1]) * math.cos(obsZ[3][1]) }, { 0 },
                { math.sin(obsZ[2][1]) * obsZ[1][1] }, { 0 } }
            P = { { 10, 0, 0, 0, 0, 0 }, { 0, 1000, 0, 0, 0, 0 }, { 0, 0, 10, 0, 0, 0 }, { 0, 0, 0, 1000, 0, 0 }, { 0, 0, 0, 0, 10, 0 }, { 0, 0, 0, 0, 0, 1000 } }
        end

        if c == n then
            isInit = false
            R = { { (0.02 * obsZ[1][1]) ^ 2, 0, 0 }, { 0, (0.002 * PI2) ^ 2, 0 }, { 0, 0, (0.002 * PI2) ^ 2 } }
            R = scalar(1 / (2 * (n + 1) * (n + 2)), R)
            X, P, EPSILON = KF(F, Q, R, obsZ, X, P, I)
            predX = mul(F, X)
            c = 1
        elseif c ~= n then
            R = { { (0.02 * obsZ[1][1]) ^ 2, 0, 0 }, { 0, (0.002 * PI2) ^ 2, 0 }, { 0, 0, (0.002 * PI2) ^ 2 } }
            R = scalar(1 / (2 * (c + 1) * (c + 2)), R)
            Xtemp, Ptemp, EPSILON = KF(F, Q, R, obsZ, X, P, I)
            predX = mul(FTemp, Xtemp)
            c = c + 1
        end
    else
        c = 1
        isInit = true
    end
    output.setNumber(1, predX[1][1] + input.getNumber(11))
    output.setNumber(2, predX[3][1] + input.getNumber(13))
    output.setNumber(3, predX[5][1] + input.getNumber(12))
    output.setNumber(4, predX[2][1])
    output.setNumber(5, predX[4][1])
    output.setNumber(6, predX[6][1])
    output.setNumber(32, EPSILON)
    output.setBool(1, RDetect)
end
