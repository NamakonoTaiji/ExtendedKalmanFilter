-- Stormworks API ショートカット
local inputNumber = input.getNumber
local inputBool = input.getBool
local outputNumber = output.setNumber

-- 定数
local PI = math.pi
local PI2 = PI * 2
local MAX_TARGETS_PER_RADAR = 8                       -- このシステムが同時に扱える最大目標数 (仕様 [source: 5])
local MAX_OUTPUT_CHANNELS = MAX_TARGETS_PER_RADAR * 2 -- 最大出力チャンネル数 (pack1, pack2)
local MIN_DETECT_DISTANCE = property.getNumber("Min Dist")
local FRONT_OFFSET = 1
local HEIGHT_OFFSET = -0.25
-- ★★★ 設定項目: このマイクロコントローラーが担当するレーダーID ★★★
-- 設置する場所に合わせて 0, 1, 2, 3 のいずれかの数値を設定してください。
local RADAR_ID = 0 -- 0:Front, 1:Right, 2:Back, 3:Left
-- ★★★ 設定ここまで ★★★

-- グローバル変数 (マイクロコントローラー内部の状態を保持)
local targetMaxData = {}    -- 各目標の最大値を記録するテーブル { distance, azimuth, elevation }
local targetMinData = {}    -- 各目標の最小値を記録するテーブル { distance, azimuth, elevation }
local outputTargetData = {} -- 出力用のデータを記憶する配列

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
高仰角レーダー用のコード
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
        local outputTargetData = {}
        -- 前のフィルター期間の結果を出力するタイミング
        local outputCount = 0
        local midDistance, midAzimuth, midElevation
        for i = 1, MAX_TARGETS_PER_RADAR do
            -- 前の期間 (tick 0 から n-1) に記録された最大/最小値を使用
            if targetMaxData[i].distance > MIN_DETECT_DISTANCE then -- 前の期間に一度でも検出された目標か？
                -- 範囲中間値を計算
                midDistance = (targetMaxData[i].distance + targetMinData[i].distance) / 2
                midAzimuth = (targetMaxData[i].azimuth + targetMinData[i].azimuth) / 2
                midElevation = (targetMaxData[i].elevation + targetMinData[i].elevation) / 2
            end
        end

        local pack1, pack2 = packTargetData(midDistance, midAzimuth, midElevation)
        table.insert(outputTargetData,
            { pack1 = pack1, pack2 = pack2 })

        -- 新しいフィルター期間のために、最大/最小値テーブルをリセット
        for i = 1, MAX_TARGETS_PER_RADAR do
            targetMaxData[i] = { distance = 0, azimuth = -1, elevation = -1 }
            targetMinData[i] = { distance = 9999999, azimuth = 1, elevation = 1 }
        end
    end

    -- フィルターしたデータの出力(探知間隔更新時は1~4目標)
    if currentRadarTick == 0 then
        local outputCount = 0
        for i = 1, math.min(#outputTargetData, 4) do
            outputNumber(outputCount * 2 + 1, outputTargetData[i].pack1)
            outputNumber(outputCount * 2 + 2, outputTargetData[i].pack2)
            outputCount = outputCount + 1
        end
    elseif currentRadarTick == 1 then
        -- 探知間隔1tick目は5~8目標を出力
        local outputCount = 0
        for i = 5, math.min(#outputTargetData, 8) do
            outputNumber(outputCount * 2 + 1, outputTargetData[i].pack1)
            outputNumber(outputCount * 2 + 2, outputTargetData[i].pack2)
            outputCount = outputCount + 1
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
            local yL = zL_prime + HEIGHT_OFFSET -- heightOffset
            local zL = -yL_prime + FRONT_OFFSET -- frontOffset

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

            -- 最大値更新
            targetMaxData[i].distance = math.max(currentDistance, targetMaxData[i].distance)
            targetMaxData[i].azimuth = math.max(correctedAzimuth, targetMaxData[i].azimuth)
            targetMaxData[i].elevation = math.max(correctedElevation, targetMaxData[i].elevation)

            -- 最小値更新
            targetMinData[i].distance = math.min(currentDistance, targetMinData[i].distance)
            targetMinData[i].azimuth = math.min(correctedAzimuth, targetMinData[i].azimuth)
            targetMinData[i].elevation = math.min(correctedElevation, targetMinData[i].elevation)
            -- else
            -- 目標が検出されなくなった場合、その目標の最大/最小値レコードを
            -- リセットするかどうかは仕様による。現状はリセットしない。
        end
    end
    -- フィルター期間のカウント (filterTickCounter) は、レーダーの経過時間(ch4)を使うので不要。
end
