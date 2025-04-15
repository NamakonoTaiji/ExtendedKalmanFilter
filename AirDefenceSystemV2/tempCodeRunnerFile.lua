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