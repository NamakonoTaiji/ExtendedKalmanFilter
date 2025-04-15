local RADAR_ID = 0

M = math

local function getSignCode(value)
    if value < 0 then return "1" else return "2" end
end
local function packTargetData(distance, azimuth, elevation)
    -- 入力値が0の場合は圧縮せずに 0, 0 を返す
    if distance == 0 then
        return 0, 0
    end

    local pack1_val, pack2_val
    local q           -- 距離の桁数フラグ (0: 1000m以上, 1: 1000m未満)
    local d_str_part  -- 距離情報から抽出した部分文字列 (4桁 or 5桁)
    local e_str       -- 方位角の小数部4桁文字列
    local f_str       -- 仰角の小数部4桁文字列
    local aziSignCode -- 方位角の符号 ('1' or '2')
    local eleSignCode -- 仰角の符号 ('1' or '2')

    -- 1. 距離に応じた q の設定と丸め処理
    local distLog = M.log(distance, 10)
    local roundedDistance
    if distLog >= 3 then -- 1000m以上
        q = 0
        roundedDistance = distance + 5 * (10 ^ (M.floor(distLog) - 4))
    else -- 1000m未満
        q = 1
        roundedDistance = distance + 5 * (10 ^ (M.floor(distLog) - 4))
    end

    -- 2 & 3. 角度の符号取得と丸め処理
    aziSignCode = getSignCode(azimuth)
    local absAzimuth = M.abs(azimuth) + 0.00005     -- 丸め
    eleSignCode = getSignCode(elevation)
    local absElevation = M.abs(elevation) + 0.00005 -- 丸め

    -- 4. 角度の小数部4桁を文字列として抽出 (オリジナル方式)
    local aziFormatted = string.format("%f", absAzimuth) -- デフォルト小数点以下6桁
    local eleFormatted = string.format("%f", absElevation)
    local aziDotPos = string.find(aziFormatted, "%.")
    local eleDotPos = string.find(eleFormatted, "%.")
    if aziDotPos then
        -- 小数点以下1文字目から4文字目までを取得
        e_str = string.sub(aziFormatted, aziDotPos + 1, aziDotPos + 4)
    else
        e_str = "0000"
    end
    if eleDotPos then
        f_str = string.sub(eleFormatted, eleDotPos + 1, eleDotPos + 4)
    else
        f_str = "0000"
    end
    -- 念のため4桁保証 (string.sub の結果が短い場合など)
    e_str = string.format("%-4s", e_str):gsub(" ", "0") -- 左寄せ4桁、空白を0で置換
    f_str = string.format("%-4s", f_str):gsub(" ", "0")

    -- 5. 距離情報から必要な部分文字列を抽出 (オリジナル方式)
    local distFormatted = string.format("%f", roundedDistance) -- デフォルト小数点以下6桁
    -- 先頭から (4 + q) 文字を取得
    d_str_part = string.sub(distFormatted, 1, 4 + q)

    -- 6. 距離部分文字列を分割 (常に前半2桁、後半/中央2桁)
    local distPart1 = string.sub(d_str_part, 1, 2)
    local distPart2 = string.sub(d_str_part, 3, 4)

    -- 7. pack1_val, pack2_val を組み立てて数値化 (常に7桁になるはず)
    pack1_val = tonumber(aziSignCode .. e_str .. distPart1)
    pack2_val = tonumber(eleSignCode .. f_str .. distPart2)

    -- tonumber が nil を返した場合のエラーハンドリング
    if pack1_val == nil then
        print("Error: Failed to convert pack1. String: '" .. aziSignCode .. e_str .. distPart1 .. "'")
        pack1_val = 0
    end
    if pack2_val == nil then
        print("Error: Failed to convert pack2. String: '" .. eleSignCode .. f_str .. distPart2 .. "'")
        pack2_val = 0
    end

    -- 8. グローバル定数 RADAR_ID に基づいて最終的な符号を決定
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
        print("Error: Invalid RADAR_ID: " .. tostring(RADAR_ID))
        sign1 = -1; sign2 = -1
    end

    -- 計算結果に符号を適用して返す
    return pack1_val * sign1, pack2_val * sign2
end
p1, p2 = packTargetData(999, 0.0, 0.1)
print(p1, p2)
