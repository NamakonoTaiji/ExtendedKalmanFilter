--[[
Radar / Sonar Track Manager
HMD描画を別Luaへ分離したTrack管理専用版。

このLuaの担当:
  1. Sonar / Radar KF Track受信
  2. Sensor Track cache
  3. R/S Association
  4. System Track生成・維持・再捕捉
  5. Radar / Sonar / Fusion位置の選択
  6. System Track単位のLock維持
  7. HMD RendererへSystem Trackを時分割送信
  8. 下流装置へSensor Track IDを1chで出力

----------------------------------------------------------------------
INPUT Number
----------------------------------------------------------------------
Sonar
   1-3 : X,Y,Z
   4-6 : Vx,Vy,Vz
   7   : Track ID
   8   : epsilon
   9   : Detect (1=valid)
   10  : lastSeenTick

Radar
   11-13 : X,Y,Z
   14-16 : Vx,Vy,Vz
   17    : Track ID
   18    : epsilon
   19    : Detect (1=valid)
   20    : lastSeenTick

HMD / Own vehicle
   25-26 : HMD view X,Y
   27-29 : own X,Y,Z
   30-32 : pitch,yaw,roll

----------------------------------------------------------------------
INPUT Bool
----------------------------------------------------------------------
   2  : Sonar position mode
   3  : Fusion position mode
        B2=OFF,B3=OFF -> Radar
        B2=ON ,B3=OFF -> Sonar
        B3=ON         -> Fusion
   4  : Fusion output ID preference
        OFF=Radar / ON=Sonar
   5  : Auto fallback

   31 : Lock
        ※現在渡された最新版コードの割り当てをそのまま維持。

----------------------------------------------------------------------
OUTPUT Number
----------------------------------------------------------------------
   1 : 下流装置向け encoded Sensor Track ID
       Radar ID n -> n*10
       Sonar ID n -> n*10+1

   ---- HMD Renderer向け時分割System Track packet ----
   2 : System ID
   3 : X
   4 : Y
   5 : Z
   6 : Vx
   7 : Vy
   8 : Vz
   9 : epsilon

  10 : Association state
       0=R / 1=S / 2=RS

  11 : Display source
       0=Radar / 1=Sonar / 2=Fusion

  12 : Locked System ID
       0=未Lock

  13 : Radar Track ID
  14 : Sonar Track ID

   ---- Rendererが自機姿勢を直接使えるようpass-through ----
  25-26 : HMD view X,Y
  27-29 : own X,Y,Z
  30-32 : pitch,yaw,roll

----------------------------------------------------------------------
OUTPUT Bool
----------------------------------------------------------------------
   1 : System Track packet valid
   2 : Sonar position mode (pass-through)
   3 : Fusion position mode (pass-through)
   4 : Fusion output Sonar (pass-through)
   5 : Auto fallback (pass-through)
  31 : Lock input (pass-through)

----------------------------------------------------------------------
時分割について
----------------------------------------------------------------------
System TrackをSystem ID昇順で1tickに1件ずつ送る。
Renderer側はSystem IDをkeyにcacheし、receive tickからVxyzで補間する。

System Trackが消滅するとpacketも来なくなるため、Renderer側では
一定時間packet更新の無いSystem IDを削除する。
]]

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

sensorLost = prop("TARGET_LOST_THRESHOLD_TICKS", 120)
posGate = prop("ASSOC_POS_GATE", 250)
velGate = prop("ASSOC_VEL_GATE", 60)
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

tick = 0
nextSystemID = 1
lockedSystemID = 0
lockHadTarget = false
packetCursor = 0

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

--====================================================================
-- Lock acquisition
--
-- 描画はRendererへ移したが、Lock acquisitionはTrack Managerに残す。
-- これによりRenderer -> Managerのフィードバック配線が不要になる。
-- System Trackのglobal座標を自機localへ変換し、HMD視線から30deg以内で
-- 最も近い角度のSystem TrackをLockする。
--====================================================================
function acquireLock(ownX, ownY, ownZ, ownQ)
    if lockedSystemID ~= 0 or lockHadTarget then return end

    local best = .2741556778 -- (30deg)^2
    local viewAzimuth = viewX * TAU
    local viewElevation = -viewY * TAU

    for _, s in pairs(systems) do
        if s[9] ~= 0 then
            local x, y, z, azimuth, elevation, da, de, d2

            x, y, z = rotate(
                s[9] - ownX,
                s[10] - ownY,
                s[11] - ownZ,
                ownQ,
                true
            )

            -- HMD基準位置offset。
            -- Renderer側でも同じoffsetを設定する。
            x = x + offsetX
            y = y + offsetY
            z = z + offsetZ

            azimuth = math.atan(x, z)
            elevation = math.atan(
                y,
                math.sqrt(x * x + z * z)
            )

            da = angleDifference(
                viewAzimuth,
                azimuth
            )

            de = elevation - viewElevation
            d2 = da * da + de * de

            if d2 < best then
                best = d2
                lockedSystemID = s[1]
            end
        end
    end

    if lockedSystemID > 0 then
        lockHadTarget = true
    end
end

--====================================================================
-- System Track packet selection
--
-- pairs()の順序には依存せず、
-- packetCursorより大きい最小System IDを選ぶ。
-- 末尾まで来たら最小System IDへwrapする。
--====================================================================
function nextPacketSystem()
    local nextID

    for id, s in pairs(systems) do
        if s[9] ~= 0 and
            id > packetCursor and
            (not nextID or id < nextID) then
            nextID = id
        end
    end

    if not nextID then
        for id, s in pairs(systems) do
            if s[9] ~= 0 and
                (not nextID or id < nextID) then
                nextID = id
            end
        end
    end

    if nextID then
        packetCursor = nextID
        return systems[nextID]
    end

    packetCursor = 0
end

--====================================================================
-- Renderer packet output
--====================================================================
function outputSystemPacket(s)
    local valid = s ~= nil

    output.setBool(1, valid)

    if not valid then
        -- 出力値が前packetのまま残らないよう明示的に0へ戻す。
        for ch = 2, 14 do
            output.setNumber(ch, 0)
        end
        return
    end

    local state

    if s[2] > 0 and s[3] > 0 then
        state = 2 -- RS
    elseif s[3] > 0 then
        state = 1 -- S
    else
        state = 0 -- R
    end

    output.setNumber(2, s[1])

    output.setNumber(3, s[9])
    output.setNumber(4, s[10])
    output.setNumber(5, s[11])

    output.setNumber(6, s[12])
    output.setNumber(7, s[13])
    output.setNumber(8, s[14])

    output.setNumber(9, s[15])

    output.setNumber(10, state)
    output.setNumber(11, s[16] or state)

    output.setNumber(12, lockedSystemID)
    output.setNumber(13, s[2])
    output.setNumber(14, s[3])
end

function onTick()
    tick = tick + 1

    local ownX, ownY, ownZ, ownQ
    local sonarID, radarID
    local outputID = 0

    ------------------------------------------------------------------
    -- Candidate memory maintenance
    ------------------------------------------------------------------
    if tick % 3600 == 0 then
        candidateMemory = {}
    end

    ------------------------------------------------------------------
    -- Operator inputs
    --
    -- 最新版コードではLockがBool31へ移されているため、そのまま使用。
    ------------------------------------------------------------------
    lockInput = input.getBool(31)

    positionMode =
        input.getBool(2) and MODE_FUSION or
        (input.getBool(1) and MODE_SONAR or MODE_RADAR)

    fusionOutputSonar = input.getBool(3)
    autoFallback = input.getBool(4)

    ------------------------------------------------------------------
    -- HMD / own vehicle state
    ------------------------------------------------------------------
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

    ------------------------------------------------------------------
    -- Sensor Track reception
    ------------------------------------------------------------------
    sonarID = readSensor(0, sonarTracks)
    radarID = readSensor(10, radarTracks)

    ensureSystem(SONAR, sonarID)
    ensureSystem(RADAR, radarID)

    pruneSensorTracks(radarTracks)
    pruneSensorTracks(sonarTracks)

    ------------------------------------------------------------------
    -- Track-to-Track association
    ------------------------------------------------------------------
    updateSystems()

    maintainPairs()
    reacquireMissing()
    associateNewPairs()

    -- AssociationによってSystem構成が変わるため再更新。
    updateSystems()

    ------------------------------------------------------------------
    -- Lock
    --
    -- Lock inputを離した時だけ完全解除。
    -- Lock対象Systemが消滅しても押しっぱなしなら別Systemへ乗換えない。
    ------------------------------------------------------------------
    if not lockInput then
        lockedSystemID = 0
        lockHadTarget = false
    else
        acquireLock(
            ownX, ownY, ownZ,
            ownQ
        )
    end

    ------------------------------------------------------------------
    -- Existing downstream target-ID output
    ------------------------------------------------------------------
    if lockInput and lockedSystemID > 0 then
        outputID = encodedOutputID(
            systems[lockedSystemID]
        )
    end

    output.setNumber(1, outputID)

    ------------------------------------------------------------------
    -- Time-divided System Track packet for HMD Renderer
    ------------------------------------------------------------------
    outputSystemPacket(
        nextPacketSystem()
    )

    ------------------------------------------------------------------
    -- HMD / own state pass-through
    --
    -- RendererをこのComposite出力1本だけで動かせるようにする。
    ------------------------------------------------------------------
    output.setNumber(25, viewX)
    output.setNumber(26, viewY)

    output.setNumber(27, ownX)
    output.setNumber(28, ownY)
    output.setNumber(29, ownZ)

    output.setNumber(30, input.getNumber(30))
    output.setNumber(31, input.getNumber(31))
    output.setNumber(32, input.getNumber(32))

    ------------------------------------------------------------------
    -- Operator mode pass-through
    ------------------------------------------------------------------
    -- Rendererへの出力

    output.setBool(2, input.getBool(1)) -- Sonar mode
    output.setBool(3, input.getBool(2)) -- Fusion mode
    output.setBool(4, input.getBool(3)) -- Fusion output Sonar
    output.setBool(5, input.getBool(4)) -- Auto fallback

    output.setBool(31, lockInput)
end
