-- Radar/Sonar Integrated HMD
-- 圧縮前提版: コンパクト版とほぼ同じ構造のまま可読性を戻した版
--
-- SensorTrack = {x,y,z,vx,vy,vz,epsilon,lastSeen,receiveTick}
-- SystemTrack = {
--   systemID, radarID, sonarID, badCount, lastRadarTick, lastSonarTick,
--   createdTick, lastActiveTick, x,y,z,vx,vy,vz,epsilon,displaySource
-- }
-- displaySource:
--   0 = Radar / 1 = Sonar / 2 = Fusion
--   RS表示では、この値をマーカー内部の線で表す。
--
-- Number:
--   Sonar 1-10 / Radar 11-20
--   View 25-26 / Own 27-32
-- Bool:
--   1 Lock
--   2 Sonar position
--   3 Fusion position
--   4 Fusion output ID: OFF Radar / ON Sonar
--   5 Auto fallback
-- Output 1:
--   Radar ID n = n*10
--   Sonar ID n = n*10+1

PI = math.pi
TAU = PI * 2

function prop(name, default)
    local v = property.getNumber(name)
    if v == 0 then return default end
    return v
end

offsetX = property.getNumber("offsetX")
offsetY = property.getNumber("offsetY")
offsetZ = property.getNumber("offsetZ")
alpha = property.getNumber("RGBAlpha")

sensorLost = prop("TARGET_LOST_THRESHOLD_TICKS")
posGate = prop("ASSOC_POS_GATE")
velGate = prop("ASSOC_VEL_GATE")
keepMult = 2
lockMult = 3
confirmCount = math.max(1, math.floor(prop("ASSOC_CONFIRM", 2) + .5))
breakCount = 3
lockBreakCount = breakCount * 2
reacquireCount = confirmCount
candidateTimeout = 120
systemLost = sensorLost * 2
lockInput = false

RADAR = 0
SONAR = 1
MODE_RADAR = 0
MODE_SONAR = 1
MODE_FUSION = 2

radarTracks = {}
sonarTracks = {}
systems = {}
candidateMemory = {}
drawTargets = {}

tick = 0
nextSystemID = 1
lockedSystemID = 0
lockHadTarget = false

viewX = 0
viewY = 0
positionMode = MODE_RADAR
fusionOutputSonar = false
autoFallback = false

-- Euler(ZYX) -> quaternion
function eulerQuaternion(roll, yaw, pitch)
    roll = roll / 2
    yaw = yaw / 2
    pitch = pitch / 2

    local cr, sr, cy, sy, cp, sp
    cr, sr = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    return {
        cr * cy * cp + sr * sy * sp,
        cr * cy * sp - sr * sy * cp,
        cr * sy * cp + sr * cy * sp,
        sr * cy * cp - cr * sy * sp
    }
end

-- inverse=true なら逆回転
function rotate(x, y, z, q, inverse)
    local w, a, b, c, r11, r12, r13, r21, r22, r23, r31, r32, r33
    w, a, b, c = q[1], q[2], q[3], q[4]

    r11 = 1 - 2 * (b * b + c * c)
    r12 = 2 * (a * b - c * w)
    r13 = 2 * (a * c + b * w)

    r21 = 2 * (a * b + c * w)
    r22 = 1 - 2 * (a * a + c * c)
    r23 = 2 * (b * c - a * w)

    r31 = 2 * (a * c - b * w)
    r32 = 2 * (b * c + a * w)
    r33 = 1 - 2 * (a * a + b * b)

    if inverse then
        return
            r11 * x + r21 * y + r31 * z,
            r12 * x + r22 * y + r32 * z,
            r13 * x + r23 * y + r33 * z
    end

    return
        r11 * x + r12 * y + r13 * z,
        r21 * x + r22 * y + r23 * z,
        r31 * x + r32 * y + r33 * z
end

-- HMDが受信したtickから現在までだけ等速外挿
function predict(track)
    local dt = (tick - track[9]) / 60
    if dt < 0 then dt = 0 end

    return
        track[1] + track[4] * dt,
        track[2] + track[5] * dt,
        track[3] + track[6] * dt,
        track[4], track[5], track[6], track[7]
end

-- Position + Velocity gate
function scoreTracks(a, b, mult)
    local ax, ay, az, avx, avy, avz = predict(a)
    local bx, by, bz, bvx, bvy, bvz = predict(b)

    local dx, dy, dz = ax - bx, ay - by, az - bz
    local dvx, dvy, dvz = avx - bvx, avy - bvy, avz - bvz

    local dp2 = dx * dx + dy * dy + dz * dz
    local dv2 = dvx * dvx + dvy * dvy + dvz * dvz

    local pg = posGate * mult
    local vg = velGate * mult
    pg = pg * pg
    vg = vg * vg

    if dp2 > pg or dv2 > vg then return nil end
    return dp2 / pg + dv2 / vg
end

function findSystem(source, id)
    for _, s in pairs(systems) do
        if (source == RADAR and s[2] == id) or
            (source == SONAR and s[3] == id) then
            return s
        end
    end
end

function createSystem(source, id)
    if not id or id < 1 then return nil end

    local s = {
        nextSystemID, 0, 0, 0, -1, -1,
        tick, tick,
        0, 0, 0, 0, 0, 0, 0,
        source
    }

    nextSystemID = nextSystemID + 1

    if source == RADAR then
        s[2] = id
    else
        s[3] = id
    end

    systems[s[1]] = s
    return s
end

function ensureSystem(source, id)
    if id and not findSystem(source, id) then
        createSystem(source, id)
    end
end

function readSensor(base, tracks)
    if input.getNumber(base + 9) ~= 1 then return nil end

    local id = math.floor(input.getNumber(base + 7) + .5)
    if id < 1 then return nil end

    tracks[id] = {
        input.getNumber(base + 1),
        input.getNumber(base + 2),
        input.getNumber(base + 3),
        input.getNumber(base + 4),
        input.getNumber(base + 5),
        input.getNumber(base + 6),
        input.getNumber(base + 8),
        input.getNumber(base + 10),
        tick
    }

    return id
end

function pruneSensorTracks(tracks)
    for id, t in pairs(tracks) do
        if tick - t[9] > sensorLost then
            tracks[id] = nil
        end
    end
end

-- 同じcached dataを毎tick数えない
function confirmCandidate(key, radarTick, sonarTick, need)
    local c = candidateMemory[key]

    if not c or tick - c[4] > candidateTimeout then
        c = { 0, -1, -1, tick }
        candidateMemory[key] = c
    end

    if c[2] ~= radarTick and c[3] ~= sonarTick then
        c[1] = c[1] + 1
        c[2] = radarTick
        c[3] = sonarTick
        c[4] = tick
    end

    return c[1] >= need
end

-- System Trackの表示状態を更新
function updateSystem(s)
    local radar, sonar, x, y, z, vx, vy, vz, epsilon, usedSource
    radar = s[2] > 0 and radarTracks[s[2]] or nil
    sonar = s[3] > 0 and sonarTracks[s[3]] or nil

    if radar or sonar then s[8] = tick end

    -- RSは「同定状態」。
    -- s[16]は、そのSystem Trackの座標を実際に
    -- Radar / Sonar / Fusionのどれから作ったかを表す。
    if s[2] > 0 and s[3] > 0 then
        if positionMode == MODE_FUSION then
            if radar and sonar then
                local rx, ry, rz, rvx, rvy, rvz = predict(radar)
                local sx, sy, sz, svx, svy, svz = predict(sonar)

                x = (rx + sx) / 2
                y = (ry + sy) / 2
                z = (rz + sz) / 2
                vx = (rvx + svx) / 2
                vy = (rvy + svy) / 2
                vz = (rvz + svz) / 2

                -- epsilonは両KF間で直接比較できないため
                -- Fusion位置では表示しない。
                epsilon = 0
                usedSource = MODE_FUSION
            elseif radar then
                x, y, z, vx, vy, vz, epsilon = predict(radar)
                usedSource = MODE_RADAR
            elseif sonar then
                x, y, z, vx, vy, vz, epsilon = predict(sonar)
                usedSource = MODE_SONAR
            end
        elseif positionMode == MODE_RADAR then
            if radar then
                x, y, z, vx, vy, vz, epsilon = predict(radar)
                usedSource = MODE_RADAR
            elseif autoFallback and sonar then
                x, y, z, vx, vy, vz, epsilon = predict(sonar)
                usedSource = MODE_SONAR
            end
        else
            if sonar then
                x, y, z, vx, vy, vz, epsilon = predict(sonar)
                usedSource = MODE_SONAR
            elseif autoFallback and radar then
                x, y, z, vx, vy, vz, epsilon = predict(radar)
                usedSource = MODE_RADAR
            end
        end
    elseif radar then
        x, y, z, vx, vy, vz, epsilon = predict(radar)
        usedSource = MODE_RADAR
    elseif sonar then
        x, y, z, vx, vy, vz, epsilon = predict(sonar)
        usedSource = MODE_SONAR
    end

    if x then
        s[9], s[10], s[11] = x, y, z
        s[12], s[13], s[14] = vx, vy, vz
        s[15] = epsilon or 0
        s[16] = usedSource
    elseif s[9] ~= 0 then
        -- 両センサー失探中は最後のSystem Track速度でcoast。
        s[9] = s[9] + s[12] / 60
        s[10] = s[10] + s[13] / 60
        s[11] = s[11] + s[14] / 60
    end
end

function updateSystems()
    for _, s in pairs(systems) do
        updateSystem(s)
    end

    for id, s in pairs(systems) do
        local radar, sonar
        radar = s[2] > 0 and radarTracks[s[2]]
        sonar = s[3] > 0 and sonarTracks[s[3]]

        if not radar and not sonar and
            tick - s[8] > systemLost then
            systems[id] = nil

            if lockedSystemID == id then
                lockedSystemID = 0
            end
        end
    end
end

-- 既存RSは広いgateで維持。Lock中はさらに広い。
function maintainPairs()
    local split = {}

    for _, s in pairs(systems) do
        if s[2] > 0 and s[3] > 0 then
            local radar, sonar
            radar = radarTracks[s[2]]
            sonar = sonarTracks[s[3]]

            if radar and sonar and
                radar[9] ~= s[5] and
                sonar[9] ~= s[6] then
                local mult =
                    lockedSystemID == s[1]
                    and lockMult or keepMult

                if scoreTracks(radar, sonar, mult) then
                    s[4] = 0
                else
                    s[4] = s[4] + 1
                end

                s[5] = radar[9]
                s[6] = sonar[9]

                local limit =
                    lockedSystemID == s[1]
                    and lockBreakCount or breakCount

                if s[4] >= limit then
                    split[#split + 1] = s
                end
            end
        end
    end

    for _, s in ipairs(split) do
        local keepSonar, radarID, sonarID
        keepSonar = false

        if lockedSystemID == s[1] then
            keepSonar =
                positionMode == MODE_SONAR or
                (positionMode == MODE_FUSION and fusionOutputSonar)
        end

        if keepSonar then
            radarID = s[2]
            s[2] = 0
            createSystem(RADAR, radarID)
        else
            sonarID = s[3]
            s[3] = 0
            createSystem(SONAR, sonarID)
        end

        s[4] = 0
        s[5] = -1
        s[6] = -1
    end
end

function mergeSystems(radarSystem, sonarSystem)
    local keep, remove

    if lockedSystemID == sonarSystem[1] then
        keep, remove = sonarSystem, radarSystem
    elseif lockedSystemID == radarSystem[1] or radarSystem[7] <= sonarSystem[7] then
        keep, remove = radarSystem, sonarSystem
    else
        keep, remove = sonarSystem, radarSystem
    end

    keep[2] = radarSystem[2]
    keep[3] = sonarSystem[3]
    keep[4] = 0
    keep[5] = -1
    keep[6] = -1
    keep[8] = tick

    systems[remove[1]] = nil
    return keep
end

-- R-only / S-onlyをscore順でgreedy 1:1 association
function associateNewPairs()
    local pairsList = {}

    for _, rSystem in pairs(systems) do
        if rSystem[2] > 0 and rSystem[3] == 0 and
            radarTracks[rSystem[2]] then
            for _, sSystem in pairs(systems) do
                if sSystem[3] > 0 and sSystem[2] == 0 and
                    sonarTracks[sSystem[3]] then
                    local score = scoreTracks(
                        radarTracks[rSystem[2]],
                        sonarTracks[sSystem[3]],
                        1
                    )

                    if score then
                        pairsList[#pairsList + 1] = {
                            score, rSystem, sSystem
                        }
                    end
                end
            end
        end
    end

    table.sort(
        pairsList,
        function(a, b) return a[1] < b[1] end
    )
    local usedRadar, usedSonar, rSystem, sSystem, radar, sonar, key
    usedRadar = {}
    usedSonar = {}

    for _, pair in ipairs(pairsList) do
        rSystem = pair[2]
        sSystem = pair[3]

        if systems[rSystem[1]] and
            systems[sSystem[1]] and
            not usedRadar[rSystem[1]] and
            not usedSonar[sSystem[1]] then
            usedRadar[rSystem[1]] = true
            usedSonar[sSystem[1]] = true

            radar = radarTracks[rSystem[2]]
            sonar = sonarTracks[sSystem[3]]
            key = "a" .. rSystem[2] .. ":" .. sSystem[3]

            if confirmCandidate(
                    key, radar[9], sonar[9], confirmCount
                ) then
                mergeSystems(rSystem, sSystem)
            end
        end
    end
end

-- 片側KF Trackが別IDで再出現した時、元RSへ戻す
function reacquireMissing()
    local used, isLocked, radar, sonar, mult, bestScore, bestTrack, bestSystem, missing, reference
    used = {}

    -- pass 1: locked system
    -- pass 2: other systems
    for pass = 1, 2 do
        for _, s in pairs(systems) do
            isLocked = lockedSystemID == s[1]
            missing = nil
            reference = nil
            bestTrack = nil
            bestSystem = nil
            if s[2] > 0 and s[3] > 0 and
                ((pass == 1 and isLocked) or
                    (pass == 2 and not isLocked)) then
                radar = radarTracks[s[2]]
                sonar = sonarTracks[s[3]]

                if not radar and sonar then
                    missing = RADAR
                    reference = sonar
                elseif radar and not sonar then
                    missing = SONAR
                    reference = radar
                end

                if missing ~= nil then
                    mult = isLocked and lockMult or keepMult
                    bestScore = math.huge

                    for _, candidateSystem in pairs(systems) do
                        if candidateSystem[1] ~= s[1] and
                            not used[candidateSystem[1]] then
                            local candidateTrack

                            if missing == RADAR and
                                candidateSystem[2] > 0 and
                                candidateSystem[3] == 0 then
                                candidateTrack =
                                    radarTracks[candidateSystem[2]]
                            elseif missing == SONAR and
                                candidateSystem[3] > 0 and
                                candidateSystem[2] == 0 then
                                candidateTrack =
                                    sonarTracks[candidateSystem[3]]
                            end

                            if candidateTrack then
                                local score = scoreTracks(
                                    reference,
                                    candidateTrack,
                                    mult
                                )

                                if score and score < bestScore then
                                    bestScore = score
                                    bestTrack = candidateTrack
                                    bestSystem = candidateSystem
                                end
                            end
                        end
                    end

                    if bestSystem then
                        used[bestSystem[1]] = true

                        local radarTick, sonarTick, sensorID, key

                        if missing == RADAR then
                            radarTick = bestTrack[9]
                            sonarTick = reference[9]
                        else
                            radarTick = reference[9]
                            sonarTick = bestTrack[9]
                        end

                        sensorID =
                            missing == RADAR
                            and bestSystem[2]
                            or bestSystem[3]

                        key =
                            "q" .. s[1] .. ":" ..
                            missing .. ":" .. sensorID

                        if confirmCandidate(
                                key,
                                radarTick,
                                sonarTick,
                                reacquireCount
                            ) then
                            if missing == RADAR then
                                s[2] = bestSystem[2]
                            else
                                s[3] = bestSystem[3]
                            end

                            systems[bestSystem[1]] = nil
                            s[4] = 0
                            s[5] = -1
                            s[6] = -1
                        end
                    end
                end
            end
        end
    end
end

function encodedOutputID(s)
    if not s then return 0 end
    local radar, sonar
    radar = s[2] > 0 and radarTracks[s[2]]
    sonar = s[3] > 0 and sonarTracks[s[3]]

    if s[2] > 0 and s[3] == 0 then
        return radar and s[2] * 10 or 0
    end

    if s[3] > 0 and s[2] == 0 then
        return sonar and s[3] * 10 + 1 or 0
    end

    if positionMode == MODE_SONAR or positionMode == MODE_FUSION and fusionOutputSonar then
        if sonar then return s[3] * 10 + 1 end
        if autoFallback and radar then return s[2] * 10 end
        --[[     elseif positionMode == MODE_RADAR then
        if radar then return s[2] * 10 end
        if autoFallback and sonar then return s[3] * 10 + 1 end ]]
    else
        if radar then return s[2] * 10 end
        if autoFallback and sonar then return s[3] * 10 + 1 end
    end

    return 0
end

function angleDifference(a, b)
    local d = b - a
    while d <= -PI do d = d + TAU end
    while d > PI do d = d - TAU end
    return d
end

function onTick()
    tick = tick + 1
    local ownX, ownY, ownZ, ownQ, sonarID, radarID
    -- candidate keyの無制限増加防止
    if tick % 3600 == 0 then candidateMemory = {} end

    lockInput = input.getBool(31)

    positionMode =
        input.getBool(3) and MODE_FUSION or
        (input.getBool(2) and MODE_SONAR or MODE_RADAR)

    fusionOutputSonar = input.getBool(4)
    autoFallback = input.getBool(5)

    viewX = input.getNumber(25)
    viewY = input.getNumber(26)

    ownX = input.getNumber(27)
    ownY = input.getNumber(28)
    ownZ = input.getNumber(29)

    ownQ = eulerQuaternion(
        input.getNumber(32),
        input.getNumber(31),
        input.getNumber(30)
    )

    sonarID = readSensor(0, sonarTracks)
    radarID = readSensor(10, radarTracks)

    ensureSystem(SONAR, sonarID)
    ensureSystem(RADAR, radarID)

    pruneSensorTracks(radarTracks)
    pruneSensorTracks(sonarTracks)

    updateSystems()
    maintainPairs()
    reacquireMissing()
    associateNewPairs()
    updateSystems()

    -- HMD表示用local座標cache
    drawTargets = {}

    for _, s in pairs(systems) do
        if s[9] ~= 0 then
            local x, y, z, azimuth, elevation, distance, source
            x, y, z = rotate(
                s[9] - ownX,
                s[10] - ownY,
                s[11] - ownZ,
                ownQ,
                true
            )

            x = x + offsetX
            y = y + offsetY
            z = z + offsetZ

            azimuth = math.atan(x, z)
            elevation = math.atan(
                y,
                math.sqrt(x * x + z * z)
            )
            distance = math.sqrt(x * x + y * y + z * z)

            source =
                s[2] > 0 and s[3] > 0 and "RS" or
                (s[2] > 0 and "R" or "S")

            drawTargets[#drawTargets + 1] = {
                s[1],
                x, y, z,
                azimuth,
                elevation,
                distance,
                s[15],
                source,
                s[16]
            }
        end
    end

    -- LockはSensor IDではなくSystem IDを保持
    if not lockInput then
        lockedSystemID = 0
        lockHadTarget = false
    elseif lockedSystemID == 0 and not lockHadTarget then
        local best, viewAzimuth, viewElevation
        best = .2741556778 -- (30deg)^2
        viewAzimuth = viewX * TAU
        viewElevation = -viewY * TAU

        for _, t in ipairs(drawTargets) do
            local da, de, d2
            da = angleDifference(viewAzimuth, t[5])
            de = t[6] - viewElevation
            d2 = da * da + de * de

            if d2 < best then
                best = d2
                lockedSystemID = t[1]
            end
        end

        if lockedSystemID > 0 then
            lockHadTarget = true
        end
    end

    local outputID = 0

    if lockInput and lockedSystemID > 0 then
        outputID = encodedOutputID(
            systems[lockedSystemID]
        )
    end

    output.setNumber(1, outputID)
end

function onDraw()
    local w, h, cx, cy, horizontalScale, viewQ, projected, lockedOffscreen

    w = screen.getWidth()
    h = screen.getHeight()
    cx = w / 2
    cy = h / 2

    verticalFov = math.rad(58)
    verticalScale = cy / math.tan(verticalFov / 2)

    horizontalScale =
        cx / math.tan(
            math.atan(
                math.tan(verticalFov / 2) * w / h
            )
        )

    viewQ = eulerQuaternion(
        0,
        viewX * TAU,
        -viewY * TAU
    )

    projected = {}

    --------------------------------------------------------------------------
    -- Projection
    --
    -- 通常目標は画面内だけprojectedへ入れる。
    -- Lock中目標だけは画面外でも方向を保持し、
    -- 後でedge indicatorを描画する。
    --------------------------------------------------------------------------
    for _, t in ipairs(drawTargets) do
        local x, y, z, px, py
        x, y, z = rotate(
            t[2], t[3], t[4],
            viewQ,
            true
        )

        if z > .1 then
            px = cx + horizontalScale * x / z
            py = cy - verticalScale * y / z

            if px >= 0 and px < w and
                py >= 0 and py < h then
                projected[#projected + 1] = {
                    t, px, py,
                    t[1] == lockedSystemID
                }
            elseif t[1] == lockedSystemID then
                lockedOffscreen = t
            end
        elseif t[1] == lockedSystemID then
            -- 真後ろ側にいる場合もedge indicatorを残す。
            lockedOffscreen = t
        end
    end

    -- Lock目標を優先し、その後は距離順。
    table.sort(
        projected,
        function(a, b)
            if a[4] ~= b[4] then return a[4] end
            return a[1][7] < b[1][7]
        end
    )

    --------------------------------------------------------------------------
    -- Target markers
    --
    -- R only : 円
    -- S only : 菱形
    -- RS     : 四角
    --
    -- RS内部の短い線:
    --   横線     = Radar位置
    --   縦線     = Sonar位置
    --   十字     = Fusion位置
    --------------------------------------------------------------------------
    local labels = {}
    for _, item in ipairs(projected) do
        local t, x, y, usedSource, changed
        t = item[1]
        x = item[2]
        y = item[3]
        usedSource=t[10]

        screen.setColor(255, 0, 0, alpha)

        if t[9] == "R" then
            screen.drawCircle(x, y, 4)
        elseif t[9] == "S" then
            screen.drawLine(x, y - 4, x + 4, y)
            screen.drawLine(x + 4, y, x, y + 4)
            screen.drawLine(x, y + 4, x - 4, y)
            screen.drawLine(x - 4, y, x, y - 4)
        else
            screen.drawRect(x - 4, y - 4, 8, 8)

            -- RSの現在使用中position source
            if usedSource == MODE_RADAR or
                usedSource == MODE_FUSION then
                screen.drawLine(x - 2, y, x + 2, y)
            end

            if usedSource == MODE_SONAR or
                usedSource == MODE_FUSION then
                screen.drawLine(x, y - 2, x, y + 2)
            end
        end

        -- epsilonは位置誤差半径ではなくinnovation整合度。
        -- Fusion位置ではepsilon=0。
        if t[8] > 0 then
            screen.setColor(255, 255, 0, alpha / 2)
            screen.drawCircle(x, y, t[8] * 10)
        end

        ----------------------------------------------------------------------
        -- Lock brackets
        --
        -- 通常マーカーとは別の四隅BracketでLockを強調する。
        ----------------------------------------------------------------------
        if item[4] then
            screen.drawRect(x - 10, y - 10, 20, 20)
        end

        ----------------------------------------------------------------------
        -- Label
        --
        -- System ID + association state.
        ----------------------------------------------------------------------
        x = item[2] + 7
        y = item[3] - 8

        changed = true

        while changed do
            changed = false

            for _, p in ipairs(labels) do
                if math.abs(x - p[1]) < 32 and
                    math.abs(y - p[2]) < 12 then
                    y = y + 12
                    changed = true
                end
            end
        end

        labels[#labels + 1] = { x, y }

        screen.drawText(
            x, y,
            "ID " .. t[1] .. " " .. t[9]
        )

        screen.drawText(
            x, y + 6,
            math.floor(t[7]) .. "m"
        )
    end

    --[[
    * = 実際に目標をロック中
    L = Lock入力ONだが未ロック
    - = Lock入力OFF
    R = Radar位置
    S = Sonar位置
    F>R = Fusion位置、出力IDはRadar
    F>S = Fusion位置、出力IDはSonar
    A = Auto Fallback ON  ]]
    --------------------------------------------------------------------------
    -- Off-screen lock indicator
    --
    -- Lock目標がHMD外に出た場合、画面端に小さなアイコンを表示する。
    -- 位置は「現在の視線角」と「目標角」の差から求めるため、
    -- 目標が真後ろ側でも方向案内を維持できる。
    --------------------------------------------------------------------------
    if lockedOffscreen then
        local dx, dy, scale, ix, iy
        dx = angleDifference(
            viewX * TAU,
            lockedOffscreen[5]
        )

        dy = -(lockedOffscreen[6] + viewY * TAU)

        -- 完全に0だとscale計算できないため微小値を入れる。
        if math.abs(dx) < .001 then dx = .001 end
        if math.abs(dy) < .001 then dy = .001 end

        scale = math.min(
            (cx - 6) / math.abs(dx),
            (cy - 6) / math.abs(dy)
        )

        ix = cx + dx * scale
        iy = cy + dy * scale

        screen.setColor(255, 0, 0, alpha)

        screen.drawCircle(
            ix, iy, 2
        )
    end

    -- 現在の操作モード表示
    local modeText, lockText

    if positionMode == MODE_FUSION then
        modeText = "F>" .. (fusionOutputSonar and "S" or "R")
    elseif positionMode == MODE_SONAR then
        modeText = "S"
    else
        modeText = "R"
    end
    screen.setColor(0, 255, 0, alpha)
    lockText = lockedSystemID > 0 and "*" or
        (lockInput and "L" or "-")
    screen.drawText(
        2, 2,
        lockText .. " " .. modeText ..
        (autoFallback and " A" or "")
    )
end
