input.getNumber = input.getNumber
input.getBool = input.getBool
output.setNumber = output.setNumber
output.setBool = output.setBool
screen.drawText = screen.drawText
string.format = string.format
rangeScale = 5
math = math
MAP_SIZE = 100

PI = math.pi
PI2 = PI * 2

function clamp(value, min, max)
    return math.max(min, math.min(max, value))
end

function onTick()
    azimuth = input.getNumber(17) * PI2
    isRangeChange = input.getBool(1)
    seatViewX = input.getNumber(25)
    seatViewY = input.getNumber(26)
    isSeatViewEnable = seatViewX ~= 0 or seatViewY ~= 0

    waypoint_1 = { x = input.getNumber(18), y = input.getNumber(19) }
    waypoint_2 = { x = input.getNumber(20), y = input.getNumber(21) }

    if isRangeChange then
        if rangeScale < 20 then
            rangeScale = rangeScale + 5
        elseif rangeScale >= 20 and rangeScale < 150 then
            rangeScale = rangeScale + 20
        end
        if rangeScale >= 150 then
            rangeScale = 5
        end
    end
    zoom = rangeScale / 4
    gpsPosition = { x = input.getNumber(1), y = input.getNumber(3) }
    centerPosition = gpsPosition
end

function onDraw()
    if isSeatViewEnable then
        screen.setColor(0, 255, 0)
        local width, height, cw, ch
        local gpsPixel = {}
        local waypoint_A, waypoint_B = { x = 0, y = 0 }, { x = 0, y = 0 } -- 描画に使う座標情報
        width = screen.getWidth()
        height = screen.getHeight()
        cw = width / 2
        ch = height / 2

        -- 1. プレイヤーがミニマップの中央に来るよう、描画すべきマップの中心座標を計算
        -- ミニマップの中心が画面上のどこに来るかを設定 (今回は左下)
        local minimap_center_x = MAP_SIZE / 2
        local minimap_center_y = height - MAP_SIZE / 2

        -- 描画オフセットを逆算
        local dx, dy = map.screenToMap(0, 0, zoom, width, height, minimap_center_x, minimap_center_y)
        local map_center_x = gpsPosition.x - dx
        local map_center_y = gpsPosition.y - dy

        local minimap_x_max = minimap_center_x + MAP_SIZE / 2 - 3
        local minimap_x_min = minimap_center_x - MAP_SIZE / 2 + 1
        local minimap_y_max = minimap_center_y + MAP_SIZE / 2 - 3
        local minimap_y_min = minimap_center_y - MAP_SIZE / 2

        -- 2. 計算した中心座標で、画面"全体"にマップを描画
        screen.drawMap(map_center_x, map_center_y, zoom)
        gpsPixel.x, gpsPixel.y = map.mapToScreen(gpsPosition.x, gpsPosition.y, zoom, width, height, gpsPosition.x,
            gpsPosition.y)
        -- 3. ミニマップ以外の領域を黒い四角で"塗りつぶして隠す"
        screen.setColor(0, 0, 0)
        -- 上半分を隠す
        screen.drawRectF(0, 0, width, height - MAP_SIZE)
        -- 右下部分を隠す
        screen.drawRectF(MAP_SIZE, height - MAP_SIZE, width - MAP_SIZE, MAP_SIZE)

        -- 4. 仕上げにミニマップの枠線を描画
        screen.setColor(0, 255, 0)
        screen.drawRect(0, height - MAP_SIZE, MAP_SIZE - 1, MAP_SIZE - 1)

        -- 自機の描画
        azimuthX = math.cos(PI / 2 - azimuth) * 5
        azimuthY = math.sin(PI / 2 - azimuth) * 5
        screen.drawText(minimap_center_x - MAP_SIZE / 2 + 2, minimap_center_y - MAP_SIZE / 2 + 2, rangeScale)
        screen.drawLine(minimap_center_x, minimap_center_y, minimap_center_x - azimuthX, minimap_center_y - azimuthY)
        screen.drawCircle(minimap_center_x, minimap_center_y, 2)

        -- Waypointを描画
        if waypoint_1.x ~= 0 or waypoint_1.y ~= 0 then
            waypoint_A.x, waypoint_A.y = map.mapToScreen(map_center_x, map_center_y, zoom, width, height,
                waypoint_1.x, waypoint_1.y)
            waypoint_A.x = clamp(waypoint_A.x, minimap_x_min, minimap_x_max)
            waypoint_A.y = clamp(waypoint_A.y, minimap_y_min, minimap_y_max)
            screen.setColor(230, 0, 0)
            screen.drawText(waypoint_A.x, waypoint_A.y, "1")
        end
        if waypoint_2.x ~= 0 or waypoint_2.y ~= 0 then
            waypoint_B.x, waypoint_B.y = map.mapToScreen(map_center_x, map_center_y, zoom, width, height,
                waypoint_2.x, waypoint_2.y)
            waypoint_B.x = clamp(waypoint_B.x, minimap_x_min, minimap_x_max)
            waypoint_B.y = clamp(waypoint_B.y, minimap_y_min, minimap_y_max)
            screen.setColor(0, 0, 230)
            screen.drawText(waypoint_B.x, waypoint_B.y, "2")
        end
    end
end
