input.getNumber = input.getNumber
output.setNumber = output.setNumber
PI = math.pi
PI2 = PI * 2
c = 0
targetList = {}
signList = { -1, 1 }
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

function MatrixCopy(M)
    local N = {}
    for i, v in ipairs(M) do
        N[i] = { table.unpack(v) }
    end
    return N
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

function mul(...)
    local A, r, s
    for l, B in ipairs({ ... }) do
        if l == 1 then
            A = B
        else
            r = {}
            for n = 1, #A do
                r[n] = {}
            end
            for i = 1, #A do
                for j = 1, #B[1] do
                    s = 0
                    for k = 1, #B do s = s + A[i][k] * B[k][j] end
                    r[i][j] = s
                end
            end
            A = r
        end
    end
    return A
end

function sub(d, f)
    return sum(d, scalar(-1, f))
end

function inv(M)
    local n, r, N, pv, f = #M, {}, MatrixCopy(M)

    for i = 1, n do
        r[i] = {}
        for j = 1, n do r[i][j] = (i == j) and 1 or 0 end
    end
    for i = 1, n do
        pv = N[i][i]
        for j = 1, n do
            N[i][j] = N[i][j] / pv
            r[i][j] = r[i][j] / pv
        end
        for k = 1, n do
            if k ~= i then
                f = N[k][i]
                for j = 1, n do
                    N[k][j] = N[k][j] - f * N[i][j]
                    r[k][j] = r[k][j] - f * r[i][j]
                end
            end
        end
    end
    return r
end

function T(c)
    local b = {}
    for _ = 1, #c[1] do
        b[_] = {}
        for a = 1, #c do b[_][a] = c[a][_] end
    end
    return b
end

Rotate = zeros(3, 3)
I = zeros(6, 6)
for i = 1, 6 do
    I[i][i] = 1
end
R0 = scalar(1 / 24, { { 0.02 ^ 2, 0, 0 }, { 0, (2e-3 * PI2) ^ 2, 0 }, { 0, 0, (2e-3 * PI2) ^ 2 } })

function Get_r_rh(x, y, z)
    local rh = x ^ 2 + z ^ 2
    return rh + y ^ 2, rh
end

function KF(F, Q, R, Z, X, P, I, phyX, phyY, phyZ, epsilon, t)
    local K, S, Y, relativeX, relativeY, relativeZ
    X = mul(F, X)
    relativeX = X[1][1] - phyX
    relativeY = X[3][1] - phyY
    relativeZ = X[5][1] - phyZ
    P = sum(scalar(1.01 ^ (2 * t), mul(F, P, T(F))), scalar((1e-4 + 1e+4 / (1 + math.exp(-(epsilon - 140) * 100))), Q))
    local r, rh, H, h = Get_r_rh(relativeX, relativeY, relativeZ)
    H = { { relativeX / math.sqrt(r),                 0, relativeY / math.sqrt(r), 0, relativeZ / math.sqrt(r),                     0 },
        { (-relativeX * relativeY) / (math.sqrt(rh) * r), 0, math.sqrt(rh) / r,        0, -relativeZ * relativeY / (math.sqrt(rh) * r), 0 },
        { relativeZ / rh,                                 0, 0,                        0, -relativeX / rh,                              0 } }
    h = { { math.sqrt(r) }, { math.asin(relativeY / math.sqrt(r)) }, { math.atan(relativeX, relativeZ) } }
    Y = sub(Z, h)

    Y[3][1] = (Y[3][1] + 3 * PI) % PI2 - PI

    S = sum(R, mul(H, P, T(H)))

    epsilon = mul(T(Y), inv(S), Y)[1][1]
    K = mul(P, T(H), inv(sum(R, mul(H, P, T(H)))))
    X = sum(X, mul(K, Y))

    P = sum(mul(sub(I, mul(K, H)), P, T(sub(I, mul(K, H)))), mul(K, R, T(K)))
    return X, P, epsilon
end

function ZYXRotate(the, phi, psi)
    RX = { { 1, 0, 0 }, { 0, math.cos(the), -math.sin(the) }, { 0, math.sin(the), math.cos(the) } }
    RY = { { math.cos(phi), 0, math.sin(phi) }, { 0, 1, 0 }, { -math.sin(phi), 0, math.cos(phi) } }
    RZ = { { math.cos(psi), -math.sin(psi), 0 }, { math.sin(psi), math.cos(psi), 0 }, { 0, 0, 1 } }
    matrix = mul(RX, RY, RZ)
    return matrix
end

function ValiableUnpack(a, b)
    local eabs, fabs, d, e, f, s
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
    a = string.format("%7f", math["math.abs"](a))
    b = string.format("%7f", math["math.abs"](b))
    d = tonumber(a:sub(6, 7) .. b:sub(6, -1))
    eabs = tonumber("0." .. a:sub(2, 5))
    fabs = tonumber("0." .. b:sub(2, 5))
    e = eabs * signList[tonumber(a:sub(1, 1))] * PI2
    f = fabs * signList[tonumber(b:sub(1, 1))] * PI2
    e = ((e + PI / 2 * (s - 1)) % PI2 + PI) % PI2 - PI
    return d, e, f, s - 1
end

function onTick()
    phyX = input.getNumber(25)
    phyY = input.getNumber(26) + 1
    phyZ = input.getNumber(27)
    c = input.getNumber(31)
    c2 = input.getNumber(32)
    Datas = {}
    for i = 1, 12 do
        if input.getNumber(i * 2 - 1) ~= 0 then
            dist, azi, ele, radarID = ValiableUnpack(input.getNumber(i * 2 - 1), input.getNumber(i * 2))
            --kakuzdohosei
            eulerMatR = ZYXRotate(-input.getNumber(28), -input.getNumber(29), -input.getNumber(30))
            Rthetaphi = { { math.cos(ele) * math.sin(azi) },
                { math.sin(ele) },
                { math.cos(ele) * math.cos(azi) } }
            Rotate = mul(T(eulerMatR), Rthetaphi)
            ele = math.asin(Rotate[2][1])
            azi = math.atan(Rotate[1][1], Rotate[3][1])
            x = dist * math.cos(ele) * math.sin(azi) + phyX       --left/right
            y = dist * math.sin(ele) - (radarID * 2.5) / 4 + phyY --height
            z = dist * math.cos(ele) * math.cos(azi) + phyZ       --front/back
            if i > 6 then
                c = c2
            end
            table.insert(Datas, { dist = dist, azi = azi, ele = ele, x = x, y = y, z = z, c = c })
        end
    end

    if #Datas ~= 0 then           ---Datas配列の中身がある時
        for i = 1, #targetList do ---targetListに何か入ってる場合はDatasと比較して同定
            epsilonMin = math.huge
            for j = 1, #Datas do  ---DatasとtargetList[i]の距離を比較する
                DT = (targetList[i].t - Datas[j].c) / 60
                F = MatrixCopy(I)
                for k = 1, 3 do
                    F[2 * k - 1][2 * k] = DT
                end
                Qlist = { DT ^ 2, DT ^ 4 / 4 }
                Q = zeros(6, 6)
                for k = 1, 6 do
                    Q[k][k] = Qlist[k % 2 + 1]
                    Q[k][k + k % 2 * 2 - 1] = DT ^ 3 / 2
                end
                obsZ = { { Datas[j].dist }, { Datas[j].ele }, { Datas[j].azi } }
                X = MatrixCopy(targetList[i].X)
                P = MatrixCopy(targetList[i].P)
                R = MatrixCopy(R0)
                R[1][1] = R[1][1] * obsZ[1][1] ^ 2
                X, P, EPSILON = KF(F, Q, R, obsZ, X, P, I, phyX, phyY, phyZ, targetList[i].epsilon,
                    targetList[i].t - Datas[j].c)
                if EPSILON < epsilonMin then
                    epsilonMin = EPSILON
                    matchingID = j
                    matchingX = MatrixCopy(X)
                    matchingP = MatrixCopy(P)
                    matchingEpsilon = EPSILON
                end
            end
            if matchingEpsilon <= 100 then                                                                           ---閾値以内ならtargetList[i]をDatas[j]で更新
                --same target update
                targetList[i] = { t = Datas[matchingID].c, X = matchingX, P = matchingP, EPSILON = matchingEpsilon } ---targetList[i]を更新
                table.remove(Datas, matchingID)                                                                      ---更新に使われたDatas[j]は消す
            end
            if #Datas == 0 then                                                                                      ---比較に使うDatasがなくなったらループを出る
                break
            end
        end

        for i = 1, #Datas do ---targetList[i]と同定できなかったDatasは新しい目標として登録
            relativeX, relativeY, relativeZ = Datas[i].x - phyX, Datas[i].y - phyY, Datas[i].z - phyZ
            r, rh = Get_r_rh(relativeX, relativeY, relativeZ)
            H = { { relativeX / math.sqrt(r),                 relativeY / math.sqrt(r), relativeZ / math.sqrt(r) },
                { (-relativeX * relativeY) / (math.sqrt(rh) * r), math.sqrt(rh) / r,        -relativeZ * relativeY / (math.sqrt(rh) * r) },
                { (relativeZ / rh),                               0,                        (-relativeX / rh) } }
            R = MatrixCopy(R0)
            R[1][1] = R[1][1] * Datas[i].dist ^ 2

            Ptemp = mul(inv(H), R, T(inv(H)))

            P = zeros(6, 6)
            for j = 1, 3 do
                P[j * 2 - 1][j * 2 - 1] = Ptemp[j][j]
                P[j * 2][j * 2] = 300 ^ 2 / 3 * 3
            end
            table.insert(targetList,
                {
                    t = Datas[i].c,
                    X = { { Datas[i].x }, { 0 }, { Datas[i].y }, { 0 }, { Datas[i].z }, { 0 } },
                    P =
                        P,
                    epsilon = 1
                })
        end
    end

    for i = #targetList, 1, -1 do
        targetList[i].t = targetList[i].t +
            1                                                                                                                                                                                                                                                                                                                                       ---targetListのtを+1する
        if targetList[i].t >= 70 or -((targetList[i].X[1][1] - phyX) * targetList[i].X[2][1] + (targetList[i].X[3][1] - phyY) * targetList[i].X[4][1] + (targetList[i].X[5][1] - phyZ) * targetList[i].X[6][1]) / math.sqrt((targetList[i].X[1][1] - phyX) ^ 2 + (targetList[i].X[3][1] - phyY) ^ 2 + (targetList[i].X[5][1] - phyZ) ^ 2) < -1 then ---70tick経っても同定できなかったtargetListは消す
            table.remove(targetList, i)
        end
    end
    for i = 1, 32 do
        output.setNumber(i, 0)
    end

    for i = 1, #targetList do
        if targetList[i].epsilon ~= 1 then
            output.setNumber(i * 3 - 2, targetList[i].X[1][1] + targetList[i].X[2][1] * targetList[i].t / 60)
            output.setNumber(i * 3 - 1, targetList[i].X[3][1] + targetList[i].X[4][1] * targetList[i].t / 60)
            output.setNumber(i * 3, targetList[i].X[5][1] + targetList[i].X[6][1] * targetList[i].t / 60)
        end
    end
end
