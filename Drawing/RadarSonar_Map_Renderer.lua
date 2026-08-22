--[[
Radar / Sonar Tactical Map Renderer

RadarSonar_TrackManager.lua のComposite出力を
HMD Rendererと並列に接続して使用する。

======================================================================
INPUT Number
======================================================================

Track Manager出力をそのまま入力。

  1 : Encoded Sensor Track ID（Lock表示の補助判定）

  2 : System ID

  3 : X
  4 : Y
  5 : Z

  6 : Vx
  7 : Vy
  8 : Vz

  9 : epsilon

 10 : Association state
      0 = Radar
      1 = Sonar
      2 = Radar + Sonar

 11 : Display source
      0 = Radar
      1 = Sonar
      2 = Fusion

 12 : Locked System ID

 13 : Radar Track ID
 14 : Sonar Track ID

 15 : Merge detection X
 16 : Merge detection Y (altitude)
 17 : Merge detection Z
 18 : Number of merges

 19 : Active sonar effective range (m)

 22 : HMD Map lock radius (screen half-height ratio)

 25-26 : HMD view X,Y

 27 : Own X
 28 : Own Y
 29 : Own Z

======================================================================
INPUT Bool
======================================================================

  1 : System Track Packet Valid
  2 : Pinger shot pulse
  9 : Map zoom in
 10 : Map zoom out

======================================================================
表示
======================================================================

・マップは常に自機中心
・北が上、東が右
・自機位置を緑で表示
・目標位置を赤で表示
・Lock目標は黄色Bracket
・Merge detectionはマージ数に応じて色分け
    1 : 半透明緑
    2 : 緑
    3以上 : 赤

Association marker:
  Radar only : Circle
  Sonar only : Diamond
  RS         : Square

速度ベクトル:
  現在速度のまま VECTOR_TIME 秒進んだ位置まで線を引く。

目標:
  Track Managerから送られるVx/Vyを使用。

自機:
  N27/N28の位置差分からVx/Vyを推定。

======================================================================
Properties
======================================================================

VECTOR_TIME
  ベクトルが何秒先を表すか。
  default = 10

VECTOR_MAX_PIXELS
  速度ベクトルの最大描画長。
  default = 35

OWN_VEL_SMOOTH
  自機速度推定の平滑化係数。
  default = 0.15

RENDER_TRACK_TIMEOUT
  Track packet timeout。
  default = 90 tick

LOCKED_RENDER_TRACK_TIMEOUT
  Lock対象のTrack packet timeout。
  default = 600 tick

MERGE_MARKER_TIMEOUT
  Merge detection markerの表示時間。
  default = 60 tick

SONAR_WAVE_SPEED
  ソーナー波紋の伝播速度。単位 m/s。
  default = 1480

SONAR_RIPPLE_LIMIT
  同時に保持する波紋数の上限。
  default = 32

RGBAlpha
  default = 255

MAP_ZOOM_INDEX
  初期zoom level。1-8。default = 1

HMD_MAP_CURSOR_ANGLE
  Map端までカーソルを動かす首振り角。単位 deg。
  default = 45

HMD_MAP_LOCK_RADIUSはTrackManager入力Number 22へ接続し、
Map RendererにはNumber 22として渡す。
]]

RADAR = 0
SONAR = 1
FUSION = 2

function prop(name, default)
    local v = property.getNumber(name)

    if v == 0 then
        return default
    end

    return v
end

--====================================================================
-- Properties
--====================================================================

zoomLevels = {
    1,
    2,
    5,
    10,
    20,
    30,
    50,
    90,
}

zoomIndex = math.max(
    1,
    math.min(
        #zoomLevels,
        math.floor(prop("MAP_ZOOM_INDEX", 1) + 0.5)
    )
)
mapZoom = zoomLevels[zoomIndex]

mapCursorTurns = prop("HMD_MAP_CURSOR_ANGLE", 45) / 360

vectorTime = prop("VECTOR_TIME", 10)
vectorMaxPixels = prop("VECTOR_MAX_PIXELS", 35)

ownVelSmooth = prop("OWN_VEL_SMOOTH", .15)

HMD_MAP_LOCK_RADIUS = prop("HMD_MAP_LOCK_RADIUS", 0.1)
trackTimeout = prop("RENDER_TRACK_TIMEOUT", 90)
lockedTrackTimeout = prop("LOCKED_RENDER_TRACK_TIMEOUT", 600)
mergeMarkerTimeout = prop("MERGE_MARKER_TIMEOUT", 60)
sonarWaveSpeed = 1480
sonarRippleLimit = math.max(
    1,
    math.floor(prop("SONAR_RIPPLE_LIMIT", 32) + 0.5)
)

alpha = prop("RGBAlpha", 255)


--====================================================================
-- State
--====================================================================

tracks = {}
mergeMarkers = {}
sonarRipples = {}

tick = 0

ownX = 0
ownZ = 0
ownZ = 0

ownVx = 0
ownVz = 0

ownPositionValid = false

lockedSystemID = 0
cursorCandidateID = 0
pingerShot = false
activeSonarRange = 0

zoomInPrev = false
zoomOutPrev = false
viewX = 0
viewY = 0
--====================================================================
-- Track reception
--====================================================================

function receiveTrackPacket()
    if not input.getBool(1) then
        return
    end

    local id =
        math.floor(
            input.getNumber(2) + .5
        )

    if id < 1 then
        return
    end

    tracks[id] = {
        input.getNumber(3), -- X
        input.getNumber(4), -- Y
        input.getNumber(5), -- Z

        input.getNumber(6), -- Vx
        input.getNumber(7), -- Vy
        input.getNumber(8), -- Vz

        input.getNumber(9), -- epsilon

        math.floor(
            input.getNumber(10) + .5
        ), -- association

        math.floor(
            input.getNumber(11) + .5
        ), -- display source

        math.floor(
            input.getNumber(13) + .5
        ), -- radar ID

        math.floor(
            input.getNumber(14) + .5
        ),   -- sonar ID

        tick -- receive tick
    }
end

-- ch15-17の世界座標とch18のマージ数を受信する。
-- merges=0はデータなし。単独観測は1として半透明緑で表示する。
function receiveMergeMarker()
    local merges =
        math.floor(
            input.getNumber(18) + .5
        )

    if merges < 1 then
        return
    end

    mergeMarkers[#mergeMarkers + 1] = {
        input.getNumber(15), -- X
        input.getNumber(16), -- Y (altitude)
        input.getNumber(17), -- Z
        merges,
        tick
    }
end

function pruneMergeMarkers()
    for i = #mergeMarkers, 1, -1 do
        if tick - mergeMarkers[i][5] > mergeMarkerTimeout then
            table.remove(mergeMarkers, i)
        end
    end
end

-- pingerShotは1 tickのパルスとして扱う。
-- 発射後に自機が移動しても発射位置から広がるよう、
-- パルス受信時の世界座標と捜索距離を保存する。
function receivePingerShot()
    if not pingerShot or activeSonarRange <= 0 then
        return
    end

    sonarRipples[#sonarRipples + 1] = {
        ownX,
        ownZ,
        tick,
        activeSonarRange
    }

    if #sonarRipples > sonarRippleLimit then
        table.remove(sonarRipples, 1)
    end
end

function pruneSonarRipples()
    for i = #sonarRipples, 1, -1 do
        local ripple = sonarRipples[i]
        local elapsed = (tick - ripple[3]) / 60
        local radius = elapsed * sonarWaveSpeed

        if radius > ripple[4] then
            table.remove(sonarRipples, i)
        end
    end
end

-- 現在のアクティブソーナー捜索範囲。
-- 発射波紋と異なり、常に現在の自機位置を中心とする。
function drawActiveSonarRange(w, h)
    if activeSonarRange <= 0 then
        return
    end

    local centerX, centerY =
        map.mapToScreen(
            ownX,
            ownZ,
            mapZoom,
            w,
            h,
            ownX,
            ownZ
        )

    local edgeX =
        map.mapToScreen(
            ownX,
            ownZ,
            mapZoom,
            w,
            h,
            ownX + activeSonarRange,
            ownZ
        )

    local pixelRadius = math.abs(edgeX - centerX)

    screen.setColor(
        0,
        120,
        255,
        math.floor(alpha * 0.35 + 0.5)
    )

    screen.drawCircle(
        centerX,
        centerY,
        pixelRadius
    )
end

function drawSonarRipples(w, h)
    screen.setColor(
        0,
        180,
        255,
        math.floor(alpha * 0.75 + 0.5)
    )

    for _, ripple in ipairs(sonarRipples) do
        local elapsed = (tick - ripple[3]) / 60
        local radius = math.min(
            elapsed * sonarWaveSpeed,
            ripple[4]
        )

        local centerX, centerY =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                ripple[1],
                ripple[2]
            )

        local edgeX =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                ripple[1] + radius,
                ripple[2]
            )

        local pixelRadius = math.abs(edgeX - centerX)

        -- 円周が画面と交差する場合だけ描画する。
        if centerX + pixelRadius >= 0 and
            centerX - pixelRadius < w and
            centerY + pixelRadius >= 0 and
            centerY - pixelRadius < h then
            screen.drawCircle(
                centerX,
                centerY,
                pixelRadius
            )
        end
    end
end

function updateZoom()
    local zoomIn = input.getBool(9)
    local zoomOut = input.getBool(10)

    -- 押した瞬間だけ1段階変更
    if zoomIn and not zoomInPrev then
        zoomIndex =
            math.max(
                1,
                zoomIndex - 1
            )
    end

    if zoomOut and not zoomOutPrev then
        zoomIndex =
            math.min(
                #zoomLevels,
                zoomIndex + 1
            )
    end

    zoomInPrev = zoomIn
    zoomOutPrev = zoomOut

    mapZoom =
        zoomLevels[zoomIndex]
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

-- Track Managerからpacketを受け取ってから
-- 現在まで経過した分だけ等速補間。
function predictTrack(t)
    local dt =
        (tick - t[12]) / 60

    if dt < 0 then
        dt = 0
    end

    return
        t[1] + t[4] * dt,
        t[2] + t[5] * dt,
        t[3] + t[6] * dt
end

--====================================================================
-- Own velocity estimation
--====================================================================

function updateOwnPosition()
    local x = input.getNumber(27)
    local y = input.getNumber(28)
    local z = input.getNumber(29)

    if ownPositionValid then
        local vx =
            (x - ownX) * 60

        local vz =
            (y - ownZ) * 60

        -- テレポート/スポーン等で巨大速度が出るのを防ぐ。
        local speed2 =
            vx * vx +
            vz * vz

        if speed2 < 1000000 then
            ownVx =
                ownVx +
                (vx - ownVx) *
                ownVelSmooth

            ownVz =
                ownVz +
                (vz - ownVz) *
                ownVelSmooth
        else
            ownVx = 0
            ownVz = 0
        end
    else
        ownPositionValid = true
    end

    ownX = x
    ownZ = y
    ownZ = z
end

--====================================================================
-- Vector drawing
--====================================================================

function drawVector(
    x1, y1,
    x2, y2,
    maximumLength
)
    local dx = x2 - x1
    local dy = y2 - y1

    local length =
        math.sqrt(
            dx * dx +
            dy * dy
        )

    if length < 1 then
        return
    end

    -- 非常に高速な目標でも画面を完全に横切らないよう制限。
    if length > maximumLength then
        local scale =
            maximumLength / length

        dx = dx * scale
        dy = dy * scale

        x2 = x1 + dx
        y2 = y1 + dy

        length = maximumLength
    end

    screen.drawLine(
        x1, y1,
        x2, y2
    )

    if length < 3 then
        return
    end

    local ux = dx / length
    local uy = dy / length

    local px = -uy
    local py = ux

    local arrowLength = 4
    local arrowWidth = 2.5

    local bx =
        x2 - ux * arrowLength

    local by =
        y2 - uy * arrowLength

    screen.drawLine(
        x2,
        y2,
        bx + px * arrowWidth,
        by + py * arrowWidth
    )

    screen.drawLine(
        x2,
        y2,
        bx - px * arrowWidth,
        by - py * arrowWidth
    )
end

--====================================================================
-- Markers
--====================================================================

function drawTargetMarker(
    x,
    y,
    association,
    displaySource
)
    if association == RADAR then
        -- Radar only
        screen.drawCircle(
            x,
            y,
            3
        )
    elseif association == SONAR then
        -- Sonar only
        screen.drawLine(
            x, y - 4,
            x + 4, y
        )

        screen.drawLine(
            x + 4, y,
            x, y + 4
        )

        screen.drawLine(
            x, y + 4,
            x - 4, y
        )

        screen.drawLine(
            x - 4, y,
            x, y - 4
        )
    else
        -- Radar + Sonar
        screen.drawRect(
            x - 4,
            y - 4,
            8,
            8
        )

        -- Radar position
        if displaySource == RADAR or
            displaySource == FUSION then
            screen.drawLine(
                x - 2, y,
                x + 2, y
            )
        end

        -- Sonar position
        if displaySource == SONAR or
            displaySource == FUSION then
            screen.drawLine(
                x, y - 2,
                x, y + 2
            )
        end
    end
end

function drawLockBracket(x, y)
    local r = 8
    local l = 3

    screen.drawLine(
        x - r, y - r,
        x - r + l, y - r
    )

    screen.drawLine(
        x - r, y - r,
        x - r, y - r + l
    )

    screen.drawLine(
        x + r, y - r,
        x + r - l, y - r
    )

    screen.drawLine(
        x + r, y - r,
        x + r, y - r + l
    )

    screen.drawLine(
        x - r, y + r,
        x - r + l, y + r
    )

    screen.drawLine(
        x - r, y + r,
        x - r, y + r - l
    )

    screen.drawLine(
        x + r, y + r,
        x + r - l, y + r
    )

    screen.drawLine(
        x + r, y + r,
        x + r, y + r - l
    )
end

function mapCursor()
    if mapCursorTurns <= 0 then
        return 0, 0
    end

    local x = viewX / mapCursorTurns
    local y = -viewY / mapCursorTurns

    x = math.max(-1, math.min(1, x))
    y = math.max(-1, math.min(1, y))

    return x, y
end

function findMapCursorCandidate(
    w,
    h,
    cursorX,
    cursorY,
    gatePixels
)
    if lockedSystemID > 0 then
        return 0
    end

    local candidateID = 0
    local bestDistance2 = gatePixels * gatePixels

    for id, t in pairs(tracks) do
        local tx, ty, tz = predictTrack(t)
        local sx, sy =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                tx,
                tz
            )

        if sx >= 0 and sx < w and
            sy >= 0 and sy < h then
            local dx = sx - cursorX
            local dy = sy - cursorY
            local distance2 = dx * dx + dy * dy

            if distance2 < bestDistance2 then
                bestDistance2 = distance2
                candidateID = id
            end
        end
    end

    return candidateID
end

-- Track ManagerのNumber 1はRadar IDをn*10、Sonar IDをn*10+1で
-- 出力する。System IDの到着を待たず、cache内のSensor IDから
-- Lock対象Systemを特定する。
function findEncodedLockSystem(encodedID)
    if encodedID < 1 then return 0 end

    local sensorID =
        math.floor(encodedID / 10)
    local isSonar =
        encodedID - sensorID * 10 == 1

    for id, t in pairs(tracks) do
        if (isSonar and t[11] == sensorID) or
            (not isSonar and t[10] == sensorID) then
            return id
        end
    end

    return 0
end

function drawMapCursor(cursorX, cursorY, gatePixels)
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
end

function drawMergeMarker(x, y, merges)
    if merges == 1 then
        screen.setColor(
            0,
            0,
            255,
            math.floor(alpha * .4 + .5)
        )
    elseif merges == 2 then
        screen.setColor(
            0,
            255,
            80,
            alpha
        )
    else
        screen.setColor(
            255,
            40,
            40,
            alpha
        )
    end

    screen.drawLine(
        x - 6,
        y - 6,
        x + 6,
        y + 6
    )
    screen.drawLine(
        x + 6,
        y - 6,
        x - 6,
        y + 6
    )
end

--====================================================================
-- onTick
--====================================================================

function onTick()
    tick = tick + 1
    pingerShot = input.getBool(2)
    activeSonarRange = input.getNumber(19)
    mapLockRadius = HMD_MAP_LOCK_RADIUS
    viewX = input.getNumber(25)
    viewY = input.getNumber(26)
    receiveTrackPacket()

    -- ch12を正式なLock IDとして使用する。
    -- 新しいIDのTrack packetが未着なら、直前のカーソル候補を
    -- 仮表示し、packet到着後に正式IDへ切り替える。
    local receivedLockID =
        math.floor(
            input.getNumber(12) + 0.5
        )
    local encodedLockID =
        math.floor(
            input.getNumber(1) + 0.5
        )
    local matchedLockID =
        findEncodedLockSystem(encodedLockID)

    if receivedLockID > 0 and
        tracks[receivedLockID] then
        lockedSystemID = receivedLockID
    elseif matchedLockID > 0 then
        lockedSystemID = matchedLockID
    elseif receivedLockID == 0 and
        encodedLockID == 0 then
        lockedSystemID = 0
    elseif lockedSystemID == 0 and
        cursorCandidateID > 0 then
        lockedSystemID = cursorCandidateID
    end

    receiveMergeMarker()
    pruneTracks()
    pruneMergeMarkers()

    updateOwnPosition()
    receivePingerShot()
    pruneSonarRipples()
    updateZoom()
end

--====================================================================
-- onDraw
--====================================================================

function onDraw()
    local w =
        screen.getWidth()

    local h =
        screen.getHeight()


    ------------------------------------------------------------------
    -- Base map
    ------------------------------------------------------------------

    screen.drawMap(
        ownX,
        ownZ,
        mapZoom
    )

    -- 現在のアクティブソーナー捜索範囲。
    drawActiveSonarRange(w, h)

    -- pinger発射位置から捜索距離まで広がる波紋。
    -- Mapの上、目標マーカーの下に描画する。
    drawSonarRipples(w, h)


    ------------------------------------------------------------------
    -- Cardinal directions
    ------------------------------------------------------------------

    screen.setColor(
        255,
        255,
        255,
        alpha
    )

    screen.drawText(
        w / 2 - 2,
        2,
        "N"
    )

    screen.drawText(
        w - 5,
        h / 2 - 2,
        "E"
    )

    screen.drawText(
        w / 2 - 2,
        h - 6,
        "S"
    )

    screen.drawText(
        1,
        h / 2 - 2,
        "W"
    )


    ------------------------------------------------------------------
    -- Own velocity vector
    ------------------------------------------------------------------

    local ox, oz =
        map.mapToScreen(
            ownX,
            ownZ,
            mapZoom,
            w,
            h,
            ownX,
            ownZ
        )

    local ownFutureX =
        ownX +
        ownVx * vectorTime

    local ownFutureZ =
        ownZ +
        ownVz * vectorTime

    local ovx, ovz =
        map.mapToScreen(
            ownX,
            ownZ,
            mapZoom,
            w,
            h,
            ownFutureX,
            ownFutureZ
        )

    screen.setColor(
        0,
        255,
        80,
        alpha
    )

    drawVector(
        ox,
        oz,
        ovx,
        ovz,
        vectorMaxPixels
    )


    ------------------------------------------------------------------
    -- Own marker
    ------------------------------------------------------------------

    screen.drawCircle(
        ox,
        oz,
        4
    )

    screen.drawLine(
        ox - 2,
        oz,
        ox + 2,
        oz
    )

    screen.drawLine(
        ox,
        oz - 2,
        ox,
        oz + 2
    )


    ------------------------------------------------------------------
    -- Merge detections
    -- Track markerより先に描き、色付き背景として見えるようにする。
    ------------------------------------------------------------------

    for _, m in ipairs(mergeMarkers) do
        local mx, mz =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                m[1], -- X
                m[3]  -- Z
            )

        drawMergeMarker(
            mx,
            mz,
            m[4]
        )
    end


    ------------------------------------------------------------------
    -- HMD view controlled map cursor / Lock candidate
    ------------------------------------------------------------------

    local cursorNX, cursorNY = mapCursor()
    local cursorX = w / 2 + cursorNX * w / 2
    local cursorY = h / 2 + cursorNY * h / 2
    local cursorGatePixels = mapLockRadius * h / 2
    cursorCandidateID =
        findMapCursorCandidate(
            w,
            h,
            cursorX,
            cursorY,
            cursorGatePixels
        )


    ------------------------------------------------------------------
    -- Targets
    ------------------------------------------------------------------

    for id, t in pairs(tracks) do
        local tx, ty, tz =
            predictTrack(t)

        local sx, sy =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                tx,
                tz
            )


        --------------------------------------------------------------
        -- Target future position
        --------------------------------------------------------------

        local futureX =
            tx +
            t[4] * vectorTime -- Vx

        local futureZ =
            tz +
            t[6] * vectorTime -- Vz

        local vx, vy =
            map.mapToScreen(
                ownX,
                ownZ,
                mapZoom,
                w,
                h,
                futureX,
                futureZ
            )


        --------------------------------------------------------------
        -- Vector
        --------------------------------------------------------------

        screen.setColor(
            255,
            40,
            40,
            alpha
        )

        drawVector(
            sx,
            sy,
            vx,
            vy,
            vectorMaxPixels
        )


        --------------------------------------------------------------
        -- Target marker
        --------------------------------------------------------------

        if id == lockedSystemID then
            screen.setColor(
                255,
                220,
                0,
                alpha
            )
        elseif id == cursorCandidateID then
            screen.setColor(
                0,
                220,
                255,
                alpha
            )
        else
            screen.setColor(
                255,
                40,
                40,
                alpha
            )
        end

        drawTargetMarker(
            sx,
            sy,
            t[8],
            t[9]
        )


        --------------------------------------------------------------
        -- Lock bracket
        --------------------------------------------------------------

        if id == lockedSystemID then
            drawLockBracket(
                sx,
                sy
            )
        elseif id == cursorCandidateID then
            drawLockBracket(
                sx,
                sy
            )
        end


        --------------------------------------------------------------
        -- System ID
        --------------------------------------------------------------

        screen.drawText(
            sx + 6,
            sy - 3,
            id
        )
    end


    ------------------------------------------------------------------
    -- Map cursor
    ------------------------------------------------------------------

    drawMapCursor(
        cursorX,
        cursorY,
        cursorGatePixels
    )


    ------------------------------------------------------------------
    -- Scale / vector information
    ------------------------------------------------------------------

    screen.setColor(
        255,
        255,
        255,
        alpha
    )

    local zoomText =
        mapZoom .. "km"

    screen.drawText(
        2,
        h - 6,
        zoomText
    )

    local vectorText =
        vectorTime .. "s"

    screen.drawText(
        w - #vectorText * 4 - 2,
        h - 6,
        vectorText
    )
end
