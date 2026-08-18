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

-- Track ManagerのLock判定と同じ値を設定すること。
offsetX = property.getNumber("offsetX")
offsetY = property.getNumber("offsetY")
offsetZ = property.getNumber("offsetZ")

alpha = prop("RGBAlpha", 255)
verticalFov = math.rad(prop("HMD_FOV_HEIGHT", 58))

-- System Track packetがこのtick数来なければRenderer cacheから削除。
-- Track Managerは通常1周数tick程度なので90tickなら十分余裕がある。
trackTimeout = prop("RENDER_TRACK_TIMEOUT", 90)

tracks = {}
tick = 0

viewX = 0
viewY = 0

ownX = 0
ownY = 0
ownZ = 0

pitch = 0
yaw = 0
roll = 0

lockedSystemID = 0

lockInput = false
positionMode = RADAR
fusionOutputSonar = false
autoFallback = false


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
        if tick - t[12] > trackTimeout then
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

    ownX = input.getNumber(27)
    ownY = input.getNumber(28)
    ownZ = input.getNumber(29)

    pitch = input.getNumber(30)
    yaw = input.getNumber(31)
    roll = input.getNumber(32)

    lockedSystemID =
        math.floor(input.getNumber(12) + .5)

    lockInput = input.getBool(31)

    positionMode =
        input.getBool(3) and FUSION or
        (input.getBool(2) and SONAR or RADAR)

    fusionOutputSonar = input.getBool(4)
    autoFallback = input.getBool(5)
end


--====================================================================
-- HMD draw
--====================================================================

function onDraw()
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

    ------------------------------------------------------------------
    -- Global System Track -> own local -> HMD projection
    ------------------------------------------------------------------
    for id, t in pairs(tracks) do
        local gx, gy, gz = predictTrack(t)

        local x, y, z = rotate(
            gx - ownX,
            gy - ownY,
            gz - ownZ,
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

        if vz > .1 then
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


    ------------------------------------------------------------------
    -- Label collision memory
    --
    -- mode status部分もoccupiedとして予約しておく。
    ------------------------------------------------------------------
    local occupied = {
        {
            0,
            0,
            #statusText * 4 + 4,
            8
        }
    }


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

        if math.abs(dx) < .001 then dx = .001 end
        if math.abs(dy) < .001 then dy = .001 end

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
