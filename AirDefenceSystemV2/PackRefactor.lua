local RADAR_ID = 1

local function getSignCode(value)
    if value < 0 then return "1" else return "2" end
end
-- packTargetData 関数 (距離エンコード変更版)
local function packTargetData(distance, azimuth, elevation)
    if distance == 0 then return 0, 0 end

    local pack1_val, pack2_val
    local e_str, f_str
    local aziSignCode, eleSignCode

    -- 1. 距離をゼロ埋め4桁の文字列に変換 (最大9999mまで対応)
    -- round distance to integer before formatting
    local intDistance = math.floor(distance + 0.5)    -- 四捨五入して整数化
    if intDistance > 9999 then intDistance = 9999 end -- 上限キャップ
    if intDistance < 0 then intDistance = 0 end       -- 下限キャップ
    local distStr4Digits = string.format("%04d", intDistance)

    -- 2. 角度の符号と小数部4桁文字列 (変更なし)
    aziSignCode = getSignCode(azimuth)
    local absAzimuth = math.abs(azimuth) + 0.00005
    eleSignCode = getSignCode(elevation)
    local absElevation = math.abs(elevation) + 0.00005
    local aziFormatted = string.format("%f", absAzimuth)
    local eleFormatted = string.format("%f", absElevation)
    local aziDotPos = string.find(aziFormatted, "%.")
    local eleDotPos = string.find(eleFormatted, "%.")
    if aziDotPos then e_str = string.sub(aziFormatted, aziDotPos + 1, aziDotPos + 4) else e_str = "0000" end
    if eleDotPos then f_str = string.sub(eleFormatted, eleDotPos + 1, eleDotPos + 4) else f_str = "0000" end
    e_str = string.format("%-4s", e_str):gsub(" ", "0")
    f_str = string.format("%-4s", f_str):gsub(" ", "0")

    -- 3. パーツ分割 (距離は4桁文字列から2桁ずつ)
    local distPart1 = string.sub(distStr4Digits, 1, 2) -- 前半2桁
    local distPart2 = string.sub(distStr4Digits, 3, 4) -- 後半2桁

    -- 4. 組み立てと数値化 (常に7桁)
    pack1_val = tonumber(aziSignCode .. e_str .. distPart1)
    pack2_val = tonumber(eleSignCode .. f_str .. distPart2)
    if pack1_val == nil then pack1_val = 0 end
    if pack2_val == nil then pack2_val = 0 end

    -- 5. 符号付け (変更なし)
    local sign1, sign2
    if RADAR_ID == 0 then
        sign1 = -1; sign2 = -1
    elseif RADAR_ID == 1 then
        sign1 = -1; sign2 = 1
    elseif RADAR_ID == 2 then
        sign1 = 1; sign2 = -1
    elseif RADAR_ID == 3 then
        sign1 = 1; sign2 = 1
    else
        sign1 = -1; sign2 = -1
    end

    return pack1_val * sign1, pack2_val * sign2
end


local PI = math.pi
local PI2 = PI * 2
function unpackTargetData(pack1, pack2)
    if pack1 == 0 and pack2 == 0 then
        return 0, 0, 0, -1
    end

    local distance, azimuthRad, elevationRad, radarId
    local signList = { [1] = -1, [2] = 1 } -- Index 1 -> 符号 -1, Index 2 -> 符号 +1

    -- 1. レーダーIDのデコード (変更なし)
    local i = (pack1 > 0) and 2 or 1
    local j = (pack2 > 0) and 2 or 1
    local radarIdMap = { { 1, 2 }, { 3, 4 } }
    local s = radarIdMap[i][j]
    radarId = s - 1

    -- 2. 絶対値を取得
    local absPack1 = math.abs(pack1)
    local absPack2 = math.abs(pack2)

    -- 3. ★修正: 絶対値を直接文字列に変換し、その後で文字列としてゼロ埋め
    local pack1Str = string.format("%.0f", absPack1) -- まず整数文字列に
    local pack2Str = string.format("%.0f", absPack2)

    -- 文字列として右寄せゼロパディングで7桁を保証する
    -- string.format("%07d", tonumber(pack1Str)) よりも安全
    pack1Str = string.format("%7s", pack1Str):gsub(" ", "0")
    pack2Str = string.format("%7s", pack2Str):gsub(" ", "0")

    -- 4. 各パーツを抽出 (ここからは変更なし)
    local aziSignCodeRaw = tonumber(string.sub(pack1Str, 1, 1)) -- 1桁目: 符号コード
    local e_str = string.sub(pack1Str, 2, 5)                    -- 2-5桁目: 方位角小数部4桁
    local distPart1 = string.sub(pack1Str, 6, 7)                -- 6-7桁目: 距離前半2桁

    local eleSignCodeRaw = tonumber(string.sub(pack2Str, 1, 1)) -- 1桁目: 符号コード
    local f_str = string.sub(pack2Str, 2, 5)                    -- 2-5桁目: 仰角小数部4桁
    local distPart2 = string.sub(pack2Str, 6, 7)                -- 6-7桁目: 距離後半/中央2桁
    -- 5. 符号インデックスの決定 (不正値対応)
    local aziSignIndex = (aziSignCodeRaw == 1 or aziSignCodeRaw == 2) and aziSignCodeRaw or 2
    local eleSignIndex = (eleSignCodeRaw == 1 or eleSignCodeRaw == 2) and eleSignCodeRaw or 2

    -- 6. 距離を復元
    distance = tonumber(distPart1 .. distPart2)
    if distance == nil then distance = 0 end

    -- 7. 角度を復元 (ラジアン単位)
    local aziFraction = tonumber("0." .. e_str)
    local eleFraction = tonumber("0." .. f_str)
    if aziFraction == nil then aziFraction = 0 end
    if eleFraction == nil then eleFraction = 0 end

    local aziSignValue = signList[aziSignIndex]
    local eleSignValue = signList[eleSignIndex]

    azimuthRad = aziFraction * aziSignValue * PI2
    elevationRad = eleFraction * eleSignValue * PI2

    -- 8. 角度を [-PI, PI) の範囲に正規化
    azimuthRad = (azimuthRad + PI) % PI2 - PI
    elevationRad = (elevationRad + PI) % PI2 - PI

    return distance, azimuthRad, elevationRad, radarId
end

p1, p2 = packTargetData(1222, 0.14231, -0.015312)
print("pack1:" .. p1, "pack2:" .. p2)

dist, azi, elev, id = unpackTargetData(p1, p2)
print("dist:" .. dist, "azi:" .. azi / PI2, "elev:" .. elev / PI2, "id:" .. id)
