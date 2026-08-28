--[[
Radar / Sonar HMD Renderer
Track ManagerのComposite出力をそのまま入力へ接続する。

このLuaの担当:
  1. Track Managerから時分割System Track packetを受信
  2. System IDごとにcache
  3. packet間をVx/Vy/Vzで等速補間
  4. 自機姿勢/HMD視線から画面へ投影
  5. R / S / RSマーカー描画
  6. RSで現在使用中の位置ソースを表示
  7. Lock表示
  8. 画面外Lock方向インジケータ
  9. ラベル重複回避
 10. 操作モード表示
 11. Lock目標の距離 / 近接速度表示

======================================================================
INPUT Number
======================================================================
Track Manager出力chと同じ番号をそのまま読む。

  1 : 下流装置向けencoded Sensor Track ID
      ※Rendererでは現在未使用

  2 : System ID
  3 : X
  4 : Y
  5 : Z
  6 : Vx
  7 : Vy
  8 : Vz
  9 : epsilon

 10 : Association state
      0 = Radar only
      1 = Sonar only
      2 = Radar + Sonar

 11 : Display source
      0 = Radar
      1 = Sonar
      2 = Fusion

  12 : Locked System ID
  13 : Radar Track ID
  14 : Sonar Track ID

 24 : Active sonar effective range (m)

  25-26 : HMD view X,Y
 27-29 : Own X,Y,Z
 30-32 : pitch,yaw,roll

======================================================================
INPUT Bool
======================================================================
  1 : System Track packet valid

  2 : Sonar position mode
  3 : Fusion position mode
  4 : Fusion output ID preference
      OFF=Radar / ON=Sonar
  5 : Auto fallback

 31 : Lock input

HMD_RENDER_TRACK_TIMEOUT
  HMD Track cache保持時間。default = 600 tick

HMD_LOCKED_TRACK_TIMEOUT
  Lock目標のHMD Track cache保持時間。default = 1200 tick

======================================================================
Renderer Track cache
======================================================================
track = {
  [1]  X
  [2]  Y
  [3]  Z
  [4]  Vx
  [5]  Vy
  [6]  Vz
  [7]  epsilon
  [8]  Association state
  [9]  Display source
  [10] Radar Track ID
  [11] Sonar Track ID
  [12] packet receive tick
}

Track Managerの座標は既にKF側で現在時刻付近まで予測されている。
そのためRendererではpacketを受信してからの経過時間だけをVxyzで補間する。
]]

PI = math.pi
TAU = PI * 2

RADAR = 0
SONAR = 1
FUSION = 2

function prop(name, default)
    local v = property.getNumber(name)
    if v == 0 then return default end
    return v
end


--[[
Map Rendererへ移行済みの旧HMD map描画。

--====================================================================
-- Tactical map HMD
--====================================================================

function hmdMapZoom()
    return hmdMapZoomValue
end

function hmdMapCursor()
    if hmdMapCursorTurns <= 0 then
        return 0, 0
    end

    local x = viewX / hmdMapCursorTurns
    local y = -viewY / hmdMapCursorTurns

    x = math.max(-1, math.min(1, x))
    y = math.max(-1, math.min(1, y))

    return x, y
end

function drawMapBracket(x, y, radius, length)
    screen.drawLine(
        x - radius, y - radius,
        x - radius + length, y - radius
    )
    screen.drawLine(
        x - radius, y - radius,
        x - radius, y - radius + length
    )

    screen.drawLine(
        x + radius, y - radius,
        x + radius - length, y - radius
    )
    screen.drawLine(
        x + radius, y - radius,
        x + radius, y - radius + length
    )

    screen.drawLine(
        x - radius, y + radius,
        x - radius + length, y + radius
    )
    screen.drawLine(
        x - radius, y + radius,
        x - radius, y + radius - length
    )

    screen.drawLine(
        x + radius, y + radius,
        x + radius - length, y + radius
    )
    screen.drawLine(
        x + radius, y + radius,
        x + radius, y + radius - length
    )
end

function drawMapUnit(x, y, association, displaySource)
    if association == RADAR then
        screen.drawCircle(x, y, 3)
    elseif association == SONAR then
        screen.drawLine(x, y - 4, x + 4, y)
        screen.drawLine(x + 4, y, x, y + 4)
        screen.drawLine(x, y + 4, x - 4, y)
        screen.drawLine(x - 4, y, x, y - 4)
    else
        screen.drawRect(x - 4, y - 4, 8, 8)

        if displaySource == RADAR or
            displaySource == FUSION then
            screen.drawLine(x - 2, y, x + 2, y)
        end

        if displaySource == SONAR or
            displaySource == FUSION then
            screen.drawLine(x, y - 2, x, y + 2)
        end
    end
end

function drawMapView()
    local w = screen.getWidth()
    local h = screen.getHeight()
    local centerX = w / 2
    local centerY = h / 2
    local mapZoom = hmdMapZoom()
    local cursorNX, cursorNY = hmdMapCursor()
    local cursorX = centerX + cursorNX * centerX
    local cursorY = centerY + cursorNY * centerY
    local gatePixels = hmdMapLockRadius * centerY
    local candidateID = 0
    local candidateDistance2 = gatePixels * gatePixels
    local lockedRange
    local lockedClosingSpeed
    local visibleTracks = {}

    screen.drawMap(
        ownX,
        ownZ,
        mapZoom
    )

    -- アクティブソーナーの現在捜索範囲。
    if activeSonarRange > 0 then
        local rangePixels =
            activeSonarRange /
            (mapZoom * 1000) * w

        screen.setColor(
            0,
            120,
            255,
            math.floor(alpha * 0.35 + 0.5)
        )
        screen.drawCircle(
            centerX,
            centerY,
            rangePixels
        )
    end

    for id, t in pairs(tracks) do
        local gx, gy, gz = predictTrack(t)
        local dx = gx - ownX
        local dy = gy - ownY
        local dz = gz - ownZ
        local distance =
            math.sqrt(
                dx * dx +
                dy * dy +
                dz * dz
            )

        if id == lockedSystemID then
            lockedRange = distance

            if distance > 0.001 then
                local relativeVx = t[4] - ownVx
                local relativeVy = t[5] - ownVy
                local relativeVz = t[6] - ownVz

                lockedClosingSpeed = -(
                    dx * relativeVx +
                    dy * relativeVy +
                    dz * relativeVz
                ) / distance
            else
                lockedClosingSpeed = 0
            end
        end

        local sx, sy =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                gx,
                gz
            )

        if sx >= 0 and sx < w and
            sy >= 0 and sy < h then
            visibleTracks[#visibleTracks + 1] = {
                id,
                sx,
                sy,
                t
            }

            if lockedSystemID == 0 then
                local cursorDx = sx - cursorX
                local cursorDy = sy - cursorY
                local cursorDistance2 =
                    cursorDx * cursorDx +
                    cursorDy * cursorDy

                if cursorDistance2 < candidateDistance2 then
                    candidateDistance2 = cursorDistance2
                    candidateID = id
                end
            end
        end
    end

    table.sort(
        visibleTracks,
        function(a, b) return a[1] < b[1] end
    )

    -- System Tracks。赤=通常、水色=Lock候補、黄=Lock中。
    for _, item in ipairs(visibleTracks) do
        local id = item[1]
        local x = item[2]
        local y = item[3]
        local t = item[4]

        if id == lockedSystemID then
            screen.setColor(255, 220, 0, alpha)
        elseif id == candidateID then
            screen.setColor(0, 220, 255, alpha)
        else
            screen.setColor(255, 40, 40, alpha)
        end

        drawMapUnit(
            x,
            y,
            t[8],
            t[9]
        )

        if id == lockedSystemID then
            drawMapBracket(x, y, 9, 3)
        elseif id == candidateID then
            drawMapBracket(x, y, 7, 2)
        end

        screen.drawText(
            x + 6,
            y - 3,
            id
        )
    end

    -- 自機。
    screen.setColor(0, 255, 80, alpha)
    screen.drawCircle(centerX, centerY, 4)
    screen.drawLine(
        centerX - 2,
        centerY,
        centerX + 2,
        centerY
    )
    screen.drawLine(
        centerX,
        centerY - 2,
        centerX,
        centerY + 2
    )

    -- 首振りで操作するMap cursorとLock gate。
    screen.setColor(0, 255, 0, alpha)
    screen.drawCircle(
        cursorX,
        cursorY,
        gatePixels
    )
    screen.drawLine(
        cursorX - 7,
        cursorY,
        cursorX - 2,
        cursorY
    )
    screen.drawLine(
        cursorX + 2,
        cursorY,
        cursorX + 7,
        cursorY
    )
    screen.drawLine(
        cursorX,
        cursorY - 7,
        cursorX,
        cursorY - 2
    )
    screen.drawLine(
        cursorX,
        cursorY + 2,
        cursorX,
        cursorY + 7
    )

    local modeText

    if positionMode == FUSION then
        modeText =
            "F>" ..
            (fusionOutputSonar and "S" or "R")
    elseif positionMode == SONAR then
        modeText = "S"
    else
        modeText = "R"
    end

    local lockText =
        lockedSystemID > 0 and "*" or
        (lockInput and "L" or "-")

    local statusText =
        "MAP " ..
        lockText ..
        " " ..
        modeText ..
        (autoFallback and " A" or "")

    local activeRangeValue =
        math.floor(activeSonarRange + 0.5)

    local activeText =
        "Active: " ..
        tostring(activeRangeValue) ..
        "m"

    local statusWidth =
        math.max(
            #statusText,
            #activeText
        ) * 4 + 4

    screen.setColor(0, 0, 0, 180)
    screen.drawRectF(0, 0, statusWidth, 15)
    screen.setColor(0, 255, 0, alpha)
    screen.drawText(2, 2, statusText)
    screen.drawText(2, 8, activeText)

    if lockedSystemID > 0 then
        local rangeText =
            lockedRange and
            ("RNG " .. formatDistance(lockedRange)) or
            "RNG ---"

        local closingText =
            lockedClosingSpeed and
            ("VC " .. formatClosingSpeed(lockedClosingSpeed)) or
            "VC ---"

        local infoWidth =
            math.max(
                #rangeText,
                #closingText
            ) * 4 + 4

        local infoX = math.max(0, w - infoWidth)

        screen.setColor(0, 0, 0, 180)
        screen.drawRectF(infoX, 0, infoWidth, 15)
        screen.setColor(0, 255, 0, alpha)
        screen.drawText(infoX + 2, 2, rangeText)
        screen.drawText(infoX + 2, 8, closingText)
    end

    local zoomValue =
        math.floor(mapZoom * 10 + 0.5) /
        10

    local zoomText =
        tostring(zoomValue) ..
        "km"

    screen.setColor(255, 255, 255, alpha)
    screen.drawText(2, h - 6, zoomText)
    screen.drawText(w / 2 - 2, 2, "N")
end
]]

function onDraw()
    drawDirectView()
end

-- Track ManagerのLock判定と同じ値を設定すること。
offsetX = property.getNumber("offsetX")
offsetY = property.getNumber("offsetY")
offsetZ = property.getNumber("offsetZ")

alpha = prop("RGBAlpha", 255)
verticalFov = math.rad(prop("HMD_FOV_HEIGHT", 58))

-- Track数が増えて時分割packetの1周が長くなっても、
-- 更新待ちのTrackを画面から消さない。
trackTimeout = prop("HMD_RENDER_TRACK_TIMEOUT", 180)
lockedTrackTimeout = prop("HMD_LOCKED_TRACK_TIMEOUT", 300)

tracks = {}
tick = 0

viewX = 0
viewY = 0

ownX = 0
ownY = 0
ownZ = 0

ownVx = 0
ownVy = 0
ownVz = 0
ownPositionValid = false

pitch = 0
yaw = 0
roll = 0

lockedSystemID = 0

lockInput = false
positionMode = RADAR
fusionOutputSonar = false
autoFallback = false
activeSonarRange = 0


-- Direct-view helpers.
--====================================================================
-- Orientation
--====================================================================

-- Euler Z-Y-X -> Quaternion
function eulerQuaternion(rollAngle, yawAngle, pitchAngle)
    rollAngle = rollAngle / 2
    yawAngle = yawAngle / 2
    pitchAngle = pitchAngle / 2

    local cr, sr, cy, sy, cp, sp
    cr, sr = math.cos(rollAngle), math.sin(rollAngle)
    cy, sy = math.cos(yawAngle), math.sin(yawAngle)
    cp, sp = math.cos(pitchAngle), math.sin(pitchAngle)

    return {
        cr*cy*cp + sr*sy*sp,
        cr*cy*sp - sr*sy*cp,
        cr*sy*cp + sr*cy*sp,
        sr*cy*cp - cr*sy*sp
    }
end


-- Quaternionを回転行列へ展開してvectorを回転。
-- inverse=trueなら逆回転。
function rotate(x, y, z, q, inverse)
    local w, a, b, c
    local r11, r12, r13
    local r21, r22, r23
    local r31, r32, r33

    w, a, b, c = q[1], q[2], q[3], q[4]

    r11 = 1 - 2*(b*b + c*c)
    r12 = 2*(a*b - c*w)
    r13 = 2*(a*c + b*w)

    r21 = 2*(a*b + c*w)
    r22 = 1 - 2*(a*a + c*c)
    r23 = 2*(b*c - a*w)

    r31 = 2*(a*c - b*w)
    r32 = 2*(b*c + a*w)
    r33 = 1 - 2*(a*a + b*b)

    if inverse then
        return
            r11*x + r21*y + r31*z,
            r12*x + r22*y + r32*z,
            r13*x + r23*y + r33*z
    end

    return
        r11*x + r12*y + r13*z,
        r21*x + r22*y + r23*z,
        r31*x + r32*y + r33*z
end


function angleDifference(from, to)
    local d = to - from

    while d <= -PI do d = d + TAU end
    while d > PI do d = d - TAU end

    return d
end


--====================================================================
-- Track cache
--====================================================================

function receiveTrackPacket()
    if not input.getBool(1) then return end

    local id = math.floor(input.getNumber(2) + .5)
    if id < 1 then return end

    tracks[id] = {
        input.getNumber(3),
        input.getNumber(4),
        input.getNumber(5),

        input.getNumber(6),
        input.getNumber(7),
        input.getNumber(8),

        input.getNumber(9),

        math.floor(input.getNumber(10) + .5),
        math.floor(input.getNumber(11) + .5),

        math.floor(input.getNumber(13) + .5),
        math.floor(input.getNumber(14) + .5),

        tick
    }
end


function pruneTracks()
    for id, t in pairs(tracks) do
        local timeout =
            id == lockedSystemID and
            lockedTrackTimeout or
            trackTimeout

        if tick - t[12] > timeout then
            tracks[id] = nil
        end
    end
end


-- packet受信後に経過した時間だけ等速補間する。
function predictTrack(t)
    local dt = (tick - t[12]) / 60

    if dt < 0 then dt = 0 end

    return
        t[1] + t[4]*dt,
        t[2] + t[5]*dt,
        t[3] + t[6]*dt
end


-- HMD固定表示用の距離文字列。
function formatDistance(distance)
    if distance >= 10000 then
        local text = math.floor(distance / 100) / 10
        return tostring(text .. "km")
    end

    return math.floor(distance + .5) .. "m"
end


-- 正のVCは接近、負のVCは離隔を表す。
function formatClosingSpeed(speed)
    local rounded = math.floor(math.abs(speed) + .5)
    local sign = speed >= 0 and "+" or "-"

    return sign .. rounded .. "m/s"
end


-- Direct-view label collision avoidance.
--====================================================================
-- Label collision avoidance
--====================================================================

function clamp(v, minimum, maximum)
    if v < minimum then return minimum end
    if v > maximum then return maximum end
    return v
end


function overlapArea(a, b)
    local left = math.max(a[1], b[1])
    local right = math.min(a[1] + a[3], b[1] + b[3])
    local top = math.max(a[2], b[2])
    local bottom = math.min(a[2] + a[4], b[2] + b[4])

    if right <= left or bottom <= top then
        return 0
    end

    return (right - left) * (bottom - top)
end


-- マーカーの右上/右下/左上/左下を試し、
-- 既存ラベルとの重なり面積が最小の位置を選ぶ。
function findLabelPosition(
    markerX, markerY,
    labelWidth, labelHeight,
    screenWidth, screenHeight,
    occupied
)
    local positions = {
        {markerX + 7, markerY - labelHeight - 2},
        {markerX + 7, markerY + 3},
        {markerX - labelWidth - 7, markerY - labelHeight - 2},
        {markerX - labelWidth - 7, markerY + 3}
    }

    local best
    local bestScore = math.huge

    for _, p in ipairs(positions) do
        local rect = {
            clamp(
                p[1],
                0,
                math.max(0, screenWidth - labelWidth)
            ),

            clamp(
                p[2],
                0,
                math.max(0, screenHeight - labelHeight)
            ),

            labelWidth,
            labelHeight
        }

        local score = 0

        for _, other in ipairs(occupied) do
            score = score + overlapArea(rect, other)
        end

        if score < bestScore then
            best = rect
            bestScore = score

            if score == 0 then break end
        end
    end

    occupied[#occupied + 1] = best

    return best[1], best[2]
end


--====================================================================
-- Stormworks tick
--====================================================================

function onTick()
    tick = tick + 1

    receiveTrackPacket()
    pruneTracks()

    -- Track Managerからそのままpass-throughされているchを読む。
    viewX = input.getNumber(25)
    viewY = input.getNumber(26)

    local newOwnX = input.getNumber(27)
    local newOwnY = input.getNumber(28)
    local newOwnZ = input.getNumber(29)

    if ownPositionValid then
        local vx = (newOwnX - ownX) * 60
        local vy = (newOwnY - ownY) * 60
        local vz = (newOwnZ - ownZ) * 60
        local speed2 = vx*vx + vy*vy + vz*vz

        -- スポーン/テレポート時の巨大な差分速度を除外。
        if speed2 < 1000000 then
            ownVx = vx
            ownVy = vy
            ownVz = vz
        else
            ownVx = 0
            ownVy = 0
            ownVz = 0
        end
    else
        ownPositionValid = true
    end

    ownX = newOwnX
    ownY = newOwnY
    ownZ = newOwnZ

    pitch = input.getNumber(30)
    yaw = input.getNumber(31)
    roll = input.getNumber(32)

    lockedSystemID =
        math.floor(input.getNumber(12) + .5)

    lockInput = input.getBool(31)
    activeSonarRange = input.getNumber(24)

    positionMode =
        input.getBool(3) and FUSION or
        (input.getBool(2) and SONAR or RADAR)

    fusionOutputSonar = input.getBool(4)
    autoFallback = input.getBool(5)
end


-- Direct-view renderer.
--====================================================================
-- HMD draw
--====================================================================

function drawDirectView()
    local w = screen.getWidth()
    local h = screen.getHeight()

    local centerX = w / 2
    local centerY = h / 2

    local verticalScale =
        centerY / math.tan(verticalFov / 2)

    local horizontalFov =
        2 * math.atan(
            math.tan(verticalFov / 2) *
            w / h
        )

    local horizontalScale =
        centerX / math.tan(horizontalFov / 2)

    local ownQ = eulerQuaternion(
        roll,
        yaw,
        pitch
    )

    local viewQ = eulerQuaternion(
        0,
        viewX * TAU,
        -viewY * TAU
    )

    -- projected item:
    -- {
    --   [1] System ID
    --   [2] screen X
    --   [3] screen Y
    --   [4] distance
    --   [5] epsilon
    --   [6] association state
    --   [7] display source
    --   [8] locked
    --   [9] local azimuth
    --   [10] local elevation
    -- }
    local projected = {}

    local lockedDirection
    local lockedRange
    local lockedClosingSpeed

    ------------------------------------------------------------------
    -- Global System Track -> own local -> HMD projection
    ------------------------------------------------------------------
    for id, t in pairs(tracks) do
        local gx, gy, gz = predictTrack(t)

        local globalDx = gx - ownX
        local globalDy = gy - ownY
        local globalDz = gz - ownZ
        local globalDistance =
            math.sqrt(
                globalDx*globalDx +
                globalDy*globalDy +
                globalDz*globalDz
            )

        if id == lockedSystemID then
            lockedRange = globalDistance

            if globalDistance > 0.001 then
                local relativeVx = t[4] - ownVx
                local relativeVy = t[5] - ownVy
                local relativeVz = t[6] - ownVz

                -- 相対速度の視線方向成分。
                -- range rateの符号を反転し、接近を正とする。
                lockedClosingSpeed = -(
                    globalDx * relativeVx +
                    globalDy * relativeVy +
                    globalDz * relativeVz
                ) / globalDistance
            else
                lockedClosingSpeed = 0
            end
        end

        local x, y, z = rotate(
            globalDx,
            globalDy,
            globalDz,
            ownQ,
            true
        )

        -- Track ManagerのLock判定と同じoffset。
        x = x + offsetX
        y = y + offsetY
        z = z + offsetZ

        local distance =
            math.sqrt(x*x + y*y + z*z)

        local azimuth =
            math.atan(x, z)

        local elevation =
            math.atan(
                y,
                math.sqrt(x*x + z*z)
            )

        -- HMD view方向へさらに逆回転。
        local vx, vy, vz = rotate(
            x, y, z,
            viewQ,
            true
        )

        if vz > 0.1 then
            local px =
                centerX +
                horizontalScale * vx / vz

            local py =
                centerY -
                verticalScale * vy / vz

            if px >= 0 and px < w and
               py >= 0 and py < h then

                projected[#projected + 1] = {
                    id,
                    px,
                    py,
                    distance,
                    t[7],
                    t[8],
                    t[9],
                    id == lockedSystemID,
                    azimuth,
                    elevation
                }

            elseif id == lockedSystemID then
                lockedDirection = {
                    azimuth,
                    elevation
                }
            end

        elseif id == lockedSystemID then
            -- 真後ろ側でもedge indicatorは残す。
            lockedDirection = {
                azimuth,
                elevation
            }
        end
    end


    ------------------------------------------------------------------
    -- Lock targetを最優先、その後は近距離順。
    -- Label配置もこの順なのでLockラベルが優先される。
    ------------------------------------------------------------------
    table.sort(
        projected,
        function(a, b)
            if a[8] ~= b[8] then
                return a[8]
            end

            return a[4] < b[4]
        end
    )


    ------------------------------------------------------------------
    -- Top-left mode status
    --
    -- * : 実際にSystem TrackをLock中
    -- L : Lock入力ONだが未Lock
    -- - : Lock入力OFF
    --
    -- R / S / F>R / F>S
    -- A : Auto fallback ON
    ------------------------------------------------------------------
    local modeText

    if positionMode == FUSION then
        modeText =
            "F>" ..
            (fusionOutputSonar and "S" or "R")

    elseif positionMode == SONAR then
        modeText = "S"

    else
        modeText = "R"
    end

    local lockText =
        lockedSystemID > 0 and "*" or
        (lockInput and "L" or "-")

    local statusText =
        lockText ..
        " " ..
        modeText ..
        (autoFallback and " A" or "")

    screen.setColor(0, 255, 0, alpha)
    screen.drawText(2, 2, statusText)

    local activeRangeValue =
        math.floor(activeSonarRange + 0.5)

    local activeText =
        "Active: " ..
        tostring(activeRangeValue) ..
        "m"

    screen.drawText(2, 8, activeText)


    ------------------------------------------------------------------
    -- Locked target range / closing speed
    --
    -- VCは接近時に正、離隔時に負。
    -- Lock目標が画面外でも右上に固定表示する。
    ------------------------------------------------------------------
    local lockInfoRect

    if lockedSystemID > 0 then
        local rangeText =
            lockedRange and
            ("RNG " .. formatDistance(lockedRange)) or
            "RNG ---"

        local closingText =
            lockedClosingSpeed and
            ("VC " .. formatClosingSpeed(lockedClosingSpeed)) or
            "VC ---"

        local infoWidth =
            math.max(#rangeText, #closingText) * 4

        local infoX = math.max(0, w - infoWidth - 2)

        screen.setColor(0, 255, 0, alpha)
        screen.drawText(infoX, 2, rangeText)
        screen.drawText(infoX, 8, closingText)

        lockInfoRect = {
            infoX,
            0,
            infoWidth + 2,
            15
        }
    end


    ------------------------------------------------------------------
    -- Label collision memory
    --
    -- mode status部分もoccupiedとして予約しておく。
    ------------------------------------------------------------------
    local occupied = {
        {
            0,
            0,
            math.max(
                #statusText,
                #activeText
            ) * 4 + 4,
            14
        }
    }

    if lockInfoRect then
        occupied[#occupied + 1] = lockInfoRect
    end


    ------------------------------------------------------------------
    -- Target marker + label
    ------------------------------------------------------------------
    for _, item in ipairs(projected) do
        local id = item[1]
        local x = item[2]
        local y = item[3]

        local distance = item[4]
        local epsilon = item[5]

        local associationState = item[6]
        local displaySource = item[7]
        local selected = item[8]

        screen.setColor(255, 0, 0, alpha)

        --------------------------------------------------------------
        -- Sensor / Association marker
        --
        -- Radar only : circle
        -- Sonar only : diamond
        -- RS         : square
        --
        -- RS内部:
        --   horizontal = Radar position
        --   vertical   = Sonar position
        --   cross      = Fusion position
        --------------------------------------------------------------
        if associationState == RADAR then
            screen.drawCircle(x, y, 4)

        elseif associationState == SONAR then
            screen.drawLine(x, y-4, x+4, y)
            screen.drawLine(x+4, y, x, y+4)
            screen.drawLine(x, y+4, x-4, y)
            screen.drawLine(x-4, y, x, y-4)

        else
            screen.drawRect(x-4, y-4, 8, 8)

            if displaySource == RADAR or
               displaySource == FUSION then
                screen.drawLine(
                    x-2, y,
                    x+2, y
                )
            end

            if displaySource == SONAR or
               displaySource == FUSION then
                screen.drawLine(
                    x, y-2,
                    x, y+2
                )
            end
        end


        --------------------------------------------------------------
        -- epsilon
        --
        -- 位置誤差半径ではなくKF innovation整合度。
        -- Fusion位置ではManager側から0が来る。
        --------------------------------------------------------------
        if epsilon > 0 then
            screen.setColor(
                255, 255, 0,
                alpha / 2
            )

            screen.drawCircle(
                x,
                y,
                epsilon * 10
            )
        end


        --------------------------------------------------------------
        -- Lock brackets
        --------------------------------------------------------------
        if selected then
            local r = 10
            local l = 3

            screen.setColor(
                255, 0, 0,
                alpha
            )

            -- upper left
            screen.drawLine(x-r, y-r, x-r+l, y-r)
            screen.drawLine(x-r, y-r, x-r, y-r+l)

            -- upper right
            screen.drawLine(x+r, y-r, x+r-l, y-r)
            screen.drawLine(x+r, y-r, x+r, y-r+l)

            -- lower left
            screen.drawLine(x-r, y+r, x-r+l, y+r)
            screen.drawLine(x-r, y+r, x-r, y+r-l)

            -- lower right
            screen.drawLine(x+r, y+r, x+r-l, y+r)
            screen.drawLine(x+r, y+r, x+r, y+r-l)
        end


        --------------------------------------------------------------
        -- Label text
        --------------------------------------------------------------
        local sourceText

        if associationState == RADAR then
            sourceText = "R"
        elseif associationState == SONAR then
            sourceText = "S"
        else
            sourceText = "RS"
        end

        local idText =
            "ID " .. id .. " " .. sourceText

        local distanceText

        if distance >= 10000 then
            distanceText =
                math.floor(distance / 100) / 10
            distanceText = distanceText.."k"
        else
            distanceText =
                math.floor(distance) ..
                "m"
        end

        -- Stormworks font:
        -- width 4px / height 5px
        local labelWidth =
            math.max(
                #idText,
                #distanceText
            ) * 4

        local labelHeight = 11

        local labelX, labelY =
            findLabelPosition(
                x, y,
                labelWidth,
                labelHeight,
                w, h,
                occupied
            )


        --------------------------------------------------------------
        -- Marker -> Label leader line
        --------------------------------------------------------------
        screen.setColor(
            0, 180, 100,
            alpha / 2
        )

        screen.drawLine(
            x, y,
            labelX,
            labelY + 5
        )


        --------------------------------------------------------------
        -- Label
        --------------------------------------------------------------
        screen.setColor(
            0, 120, 255,
            alpha
        )

        screen.drawText(
            labelX,
            labelY,
            idText
        )

        screen.setColor(
            0, 255, 0,
            alpha
        )

        screen.drawText(
            labelX,
            labelY + 6,
            distanceText
        )
    end


    ------------------------------------------------------------------
    -- Off-screen Lock indicator
    --
    -- HMD視線との差をFOVで正規化し、画面端へ三角形を配置する。
    ------------------------------------------------------------------
    if lockedDirection then
        local viewAzimuth =
            viewX * TAU

        local viewElevation =
            -viewY * TAU

        local dx =
            angleDifference(
                viewAzimuth,
                lockedDirection[1]
            ) /
            (horizontalFov / 2)

        local dy =
            (
                lockedDirection[2] -
                viewElevation
            ) /
            (verticalFov / 2)

        if math.abs(dx) < 0.001 then dx = 0.001 end
        if math.abs(dy) < 0.001 then dy = 0.001 end

        local margin = 6

        local scale =
            math.min(
                (centerX - margin) /
                math.abs(dx),

                (centerY - margin) /
                math.abs(dy)
            )

        local ix =
            centerX + dx * scale

        local iy =
            centerY + dy * scale

        local length =
            math.sqrt(dx*dx + dy*dy)

        local nx = dx / length
        local ny = dy / length

        -- directionに直交するvector
        local px = -ny
        local py = nx

        screen.setColor(
            255, 0, 0,
            alpha
        )

        screen.drawTriangleF(
            ix + nx*4,
            iy + ny*4,

            ix - nx*3 + px*3,
            iy - ny*3 + py*3,

            ix - nx*3 - px*3,
            iy - ny*3 - py*3
        )
    end
end
