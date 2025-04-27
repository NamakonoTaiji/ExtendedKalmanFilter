-- Tick function that will be executed every logic tick
local iN = input.getNumber
local PI2 = math.pi * 2

function onTick()
    radar1dist       = iN(1)
    radar1azi        = iN(2)
    radar1ele        = iN(3)
    currentDistance  = iN(4)
    currentAzimuth   = iN(5)
    currentElevation = iN(6)
    memoryTargetFlag = input.getBool(1)

    --[[ ここから座標変換処理 ]]

    -- 角度をラジアンに変換
    local azimuthRadPrime = currentAzimuth * PI2
    local elevationRadPrime = currentElevation * PI2

    -- 90度上向きレーダーのローカル直交座標 (xL', yL', zL') を計算
    -- x': 右方向, y': 上方向(レーダーの物理的な上), z': 前方(レーダーの物理的な前方)
    local xL_prime = currentDistance * math.cos(elevationRadPrime) * math.sin(azimuthRadPrime)
    local yL_prime = currentDistance * math.sin(elevationRadPrime)
    local zL_prime = currentDistance * math.cos(elevationRadPrime) * math.cos(azimuthRadPrime)

    -- 水平レーダー基準のローカル直交座標 (xL, yL, zL) に変換
    -- 水平レーダーを基準とすると、90度上向きレーダーはX軸周りに+90度回転した状態とみなせる
    -- 水平X = 上向きX'
    -- 水平Y = 上向きZ'
    -- 水平Z = -上向きY'
    local xL = xL_prime
    local yL = zL_prime - 0.25 -- heightOffset
    local zL = -yL_prime + 1   -- frontOffset

    -- 水平レーダー基準のローカル極座標 (方位角 alpha, 仰角 beta) に変換
    local correctedAzimuthRad, correctedElevationRad
    if currentDistance > 0 then -- ゼロ除算を避ける
        -- Stormworksの math.atan は atan2(y, x) として機能する [cite: 20]
        correctedAzimuthRad = math.atan(xL, zL)
        -- asin の引数が [-1, 1] の範囲に収まるように clamp する (念のため)
        local elevationArg = math.max(-1.0, math.min(1.0, yL / currentDistance))
        correctedElevationRad = math.asin(elevationArg)
    else
        correctedAzimuthRad = 0
        correctedElevationRad = 0
    end

    -- 角度を回転単位に戻す
    local correctedAzimuth = correctedAzimuthRad / PI2
    local correctedElevation = correctedElevationRad / PI2

    -- 必要であれば、角度を [-0.5, 0.5) の範囲に正規化 (atan/asinの結果は通常この範囲内だが念のため)
    -- correctedAzimuth = (correctedAzimuth + 0.5) % 1.0 - 0.5
    -- correctedElevation = (correctedElevation + 0.5) % 1.0 - 0.5

    --[[ 座標変換処理 ここまで ]]

    if memoryTargetFlag then
        debug.log("---------------------------------------")
        debug.log(" horizon Azimuth: " .. radar1azi .. " horizon Elev: " .. radar1ele)
        debug.log(" vertical Azimuth: " .. correctedAzimuth .. " horizon Elev: " .. correctedElevation)
    end
end
