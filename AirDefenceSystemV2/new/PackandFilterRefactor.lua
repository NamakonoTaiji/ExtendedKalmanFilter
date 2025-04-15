-- Lua標準ライブラリ
local M = math
local string = string
local table = table
local tonumber = tonumber
local ipairs = ipairs

-- Stormworks API ショートカット
local inputNumber = input.getNumber
local inputBool = input.getBool
local outputNumber = output.setNumber
local propertyNumber = property.getNumber

-- 定数
local PI = M.pi
local PI2 = PI * 2
local MAX_TARGETS_PER_RADAR = 6                       -- レーダーが同時に扱える最大目標数 (仕様 [source: 5])
local MAX_OUTPUT_CHANNELS = MAX_TARGETS_PER_RADAR * 2 -- 最大出力チャンネル数 (pack1, pack2)

-- ★★★ 設定項目: このマイクロコントローラーが担当するレーダーID ★★★
-- 設置する場所に合わせて 0, 1, 2, 3 のいずれかの数値を設定してください。
local RADAR_ID = 0 -- 0:Front, 1:Right, 2:Back, 3:Left
-- ★★★ 設定ここまで ★★★

-- グローバル変数 (マイクロコントローラー内部の状態を保持)
local targetMaxData = {} -- 各目標の最大値を記録するテーブル { distance, azimuth, elevation }
local targetMinData = {} -- 各目標の最小値を記録するテーブル { distance, azimuth, elevation }
-- filterTickCounter は不要 (レーダー出力の経過時間tick(ch4)で判断するため)

-- 設定値 (プロパティから取得)
local filterDuration = propertyNumber("n") -- 範囲中間値フィルターの期間 (tick)

-- 初期化処理: targetMaxData と targetMinData テーブルを初期化
for i = 1, MAX_TARGETS_PER_RADAR do
    targetMaxData[i] = { distance = 0, azimuth = -1, elevation = -1 }
    targetMinData[i] = { distance = 9999999, azimuth = 1, elevation = 1 }
end

--[[
--------------------------------------------------------------------------------
ヘルパー関数: getSignCode
--------------------------------------------------------------------------------
入力された数値の符号に応じて、'1' (負) または '2' (正またはゼロ) の文字列を返す。
Pack関数内で角度の符号をエンコードするために使用。
(元のコードの sign 関数 [source: 72] とほぼ同じ)
--------------------------------------------------------------------------------
]]
--[[
--------------------------------------------------------------------------------
メイン関数: packTargetData
--------------------------------------------------------------------------------
入力された距離、方位角、仰角を、元の Pack 関数のロジックに基づいて
2つの数値 (pack1, pack2) に圧縮する。
圧縮された値には、角度の符号情報、距離情報、およびレーダーID情報が含まれる。
レーダーIDはグローバル定数 RADAR_ID を参照して出力値の符号を決定する。

圧縮仕様 (元のコード [source: 70-72] からの解析):
1. 距離 `distance` の桁数を判定。
   - 1000m以上 (>=3桁) の場合: `q=0`, 距離を四捨五入(?)し、整数部5桁の文字列 `d` を生成。
   - 1000m未満 (<3桁) の場合: `q=1`, 距離を四捨五入(?)し、整数部4桁(0埋め)の文字列 `d` を生成。
2. 方位角 `azimuth` の符号 `bsgn` (1 or 2) と絶対値 `babs` を取得。
3. 仰角 `elevation` の符号 `csgn` (1 or 2) と絶対値 `cabs` を取得。
4. `babs` と `cabs` を小数点以下4桁で丸め(?)、その4桁の数字列 `e`, `f` を取得。
5. `pack1_val = bsgn .. e .. dの前半2桁` を数値化。
6. `pack2_val = csgn .. f .. dの後半2桁(q=1)または3桁(q=0)` を数値化。
7. `RADAR_ID` に基づいて `pack1_val`, `pack2_val` の符号を決定して返す。
--------------------------------------------------------------------------------
]]
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

--[[
--------------------------------------------------------------------------------
メインループ: onTick
--------------------------------------------------------------------------------
毎tick呼び出され、以下の処理を行う。
1. レーダー出力の経過時間(tick) を確認し、探知更新タイミング (tick == 0) で
   フィルター用の最大/最小値テーブルをリセットする。
2. フィルター期間 (n tick) 中、検出中の目標の距離・方位角・仰角の
   最大値と最小値を更新し続ける。
3. フィルター期間が終了するtick (経過時間 == n-1 の次、つまりレーダーの次の探知更新タイミング) で、
   記録した最大/最小値から範囲中間値を計算し、packTargetData 関数で圧縮して出力する。
--------------------------------------------------------------------------------
]]
function onTick()
    -- 出力チャンネルをクリア (毎tickクリアする方が安全)
    for i = 1, MAX_OUTPUT_CHANNELS do
        outputNumber(i, 0)
    end

    -- 目標1の経過時間tickを取得 (全目標で同期している [source: 6])
    -- この値は、レーダー内部で「真の値」が更新されてからの経過tick数 [source: 7]
    local currentRadarTick = inputNumber(4)

    -- 1. 探知更新タイミング (経過tickが0になった瞬間) の処理
    if currentRadarTick == 0 then
        -- 前のフィルター期間の結果を出力するタイミング
        local outputCount = 0
        for i = 1, MAX_TARGETS_PER_RADAR do
            -- 前の期間 (tick 0 から n-1) に記録された最大/最小値を使用
            if targetMaxData[i].distance > 0 then -- 前の期間に一度でも検出された目標か？
                -- 範囲中間値を計算
                local midDistance = (targetMaxData[i].distance + targetMinData[i].distance) / 2
                local midAzimuth = (targetMaxData[i].azimuth + targetMinData[i].azimuth) / 2
                local midElevation = (targetMaxData[i].elevation + targetMinData[i].elevation) / 2

                -- 出力チャンネル数上限をチェックしつつ、パックして出力
                if outputCount < MAX_TARGETS_PER_RADAR then
                    local pack1, pack2 = packTargetData(midDistance, midAzimuth, midElevation)
                    outputNumber(outputCount * 2 + 1, pack1)
                    outputNumber(outputCount * 2 + 2, pack2)
                    outputCount = outputCount + 1
                else
                    -- 出力上限に達したらループを抜ける (Safety break)
                    break
                end
            end
        end

        -- 新しいフィルター期間のために、最大/最小値テーブルをリセット
        for i = 1, MAX_TARGETS_PER_RADAR do
            targetMaxData[i] = { distance = 0, azimuth = -1, elevation = -1 }
            targetMinData[i] = { distance = 9999999, azimuth = 1, elevation = 1 }
        end
    end

    -- 2. フィルター期間中の処理 (経過tick 0 から n-1 まで)
    -- ※ フィルター期間 `filterDuration` とレーダーの探知間隔が一致している前提
    --    もし異なる場合、この期間判定ロジックの見直しが必要
    -- if currentRadarTick < filterDuration then -- この条件は必ずしも正しくない
    -- レーダーからの各目標の検出状態を確認し、検出中なら最大/最小値を更新
    for i = 1, MAX_TARGETS_PER_RADAR do
        if inputBool(i) then -- 目標i を検出中か？
            local currentDistance = inputNumber(i * 4 - 3)
            local currentAzimuth = inputNumber(i * 4 - 2)
            local currentElevation = inputNumber(i * 4 - 1)

            -- 最大値更新
            targetMaxData[i].distance = M.max(currentDistance, targetMaxData[i].distance)
            targetMaxData[i].azimuth = M.max(currentAzimuth, targetMaxData[i].azimuth)
            targetMaxData[i].elevation = M.max(currentElevation, targetMaxData[i].elevation)

            -- 最小値更新
            targetMinData[i].distance = M.min(currentDistance, targetMinData[i].distance)
            targetMinData[i].azimuth = M.min(currentAzimuth, targetMinData[i].azimuth)
            targetMinData[i].elevation = M.min(currentElevation, targetMinData[i].elevation)
            -- else
            -- 目標が検出されなくなった場合、その目標の最大/最小値レコードを
            -- リセットするかどうかは仕様による。現状はリセットしない。
        end
    end
    -- フィルター期間のカウント (filterTickCounter) は、レーダーの経過時間(ch4)を使うので不要。
end
