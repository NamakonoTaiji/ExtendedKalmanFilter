--[[
コンポジット信号ビューア スクリプト (小数点桁数・デバッグログ 設定可能版 + 色分け表示 改)

機能:
- コンポジット入力の数値チャンネル 1-32 の値を読み取る
- コンポジット入力のオンオフチャンネル 1-32 の値を読み取る
- マイクロコントローラーのプロパティから小数点以下の表示桁数、デバッグログ設定を読み取る (スポーン時に1回)
- 読み取った値をモニターに4列構成で見やすく表示する
  - 数値が 0 以外の場合は緑色で表示
  - オンオフ信号が On (true) の場合は緑色で表示
  - オンオフ信号が Off (false) の場合は赤色で表示
  - それ以外は白色で表示
- 有効な数字やオン信号をdebug.logに出力するオプション
]]

-- --- プロパティ読み取り (スクリプトロード時に1回だけ実行) ---
local propVal = property.getNumber("DecimalPlaces")
local propValOffset = propVal * 2
local NUMERICAL_DEBUG_LOG_ENABLED = property.getBool("NUM_DEBUGLOG") == true -- nilチェックとtrue比較
local BOOLEAN_DEBUG_LOG_ENABLED = property.getBool("BOOL_DEBUGLOG") == true  -- nilチェックとtrue比較
local DEBUG_STRING_PREFIX_prop = property.getText("DEBUG_STRING_PREFIX")
local BOOLEAN_COL_OFFSET = 10
if type(DEBUG_STRING_PREFIX_prop) == "string" then
    DEBUG_STRING_PREFIX = DEBUG_STRING_PREFIX_prop
end

-- グローバル変数
g_numberValues = {} -- 数値チャンネルの値を格納
g_boolValues = {}   -- オンオフチャンネルの値を格納
g_decimalPlaces = 2 -- 小数点以下の表示桁数 (デフォルト値)
local onTickCount = 0

-- --- 小数点桁数の検証と適用 (スクリプトロード時に1回だけ実行) ---
if type(propVal) == "number" then
    g_decimalPlaces = math.max(0, math.min(8, math.floor(propVal)))
else
    g_decimalPlaces = 2 -- デフォルト値
end
-- -------------------------------------------------------

-- 初期化
for i = 1, 32 do
    g_numberValues[i] = 0
    g_boolValues[i] = false
end

--[[
onTick() 関数: (変更なし)
--]]
function onTick()
    onTickCount = onTickCount + 1

    -- --- デバッグログ出力準備 ---
    local shouldPrintDebugHeader = false
    if NUMERICAL_DEBUG_LOG_ENABLED or BOOLEAN_DEBUG_LOG_ENABLED then
        for i = 1, 32 do
            if (NUMERICAL_DEBUG_LOG_ENABLED and input.getNumber(i) ~= 0) or (BOOLEAN_DEBUG_LOG_ENABLED and input.getBool(i)) then
                shouldPrintDebugHeader = true
                break
            end
        end
    end

    if shouldPrintDebugHeader then
        debug.log(string.format("-----------------------%s START-----------------------", DEBUG_STRING_PREFIX))
        debug.log(onTickCount .. " tick")
    end
    -- ---------------------------

    -- --- コンポジット入力読み取りと個別ログ出力 ---
    for i = 1, 32 do
        g_numberValues[i] = input.getNumber(i)
        g_boolValues[i] = input.getBool(i)

        if NUMERICAL_DEBUG_LOG_ENABLED and g_numberValues[i] ~= 0 then
            debug.log("NUMch: " .. i .. " >> " .. g_numberValues[i])
        end
        if BOOLEAN_DEBUG_LOG_ENABLED and g_boolValues[i] then
            -- On (true) の場合のログ（必要なら緑に対応する形で変更しても良いですが、そのままでも可）
            debug.log("BOOLch: " .. i .. " >> " .. "true (ON)")
        elseif BOOLEAN_DEBUG_LOG_ENABLED and not g_boolValues[i] then
            -- Off (false) の場合のログ（必要なら赤に対応する形で追記しても良い）
            -- debug.log("ch: " .. i .. " >> " .. "false (OFF)")
        end
    end
end

--[[
onDraw() 関数: モニターの描画が必要な時に呼び出される (オンオフの色設定を反転)
--]]
function onDraw()
    local width = screen.getWidth()
    local height = screen.getHeight()
    -- 色の定義 (RGB値)
    local bgColorR, bgColorG, bgColorB = 0, 0, 0                        -- 背景色 (黒)
    local defaultColorR, defaultColorG, defaultColorB = 255, 255, 255   -- デフォルト文字色 (白)
    local numActiveColorR, numActiveColorG, numActiveColorB = 0, 255, 0 -- 数値アクティブ色 & ブールON色 (緑)
    local boolFalseColorR, boolFalseColorG, boolFalseColorB = 255, 0, 0 -- ブールOFF色 (赤)

    local colWidth = width / 4
    local rowHeight = 6
    local maxRows = math.floor(height / rowHeight)
    local col1X = 5
    local col2X = colWidth * 1 + 5 + propValOffset
    local col3X = colWidth * 2 + 5 + propValOffset
    local col4X = colWidth * 3 + 5 - BOOLEAN_COL_OFFSET + propValOffset

    local numFormat = string.format("%%.%df", g_decimalPlaces)

    -- 背景クリア
    screen.setColor(bgColorR, bgColorG, bgColorB)
    screen.drawClear()

    -- チャンネル情報を描画するループ
    for i = 1, 16 do
        if i <= maxRows then
            local currentY = (i - 1) * rowHeight + 2

            -- 1列目: 数値チャンネル 1-16 (変更なし)
            if g_numberValues[i] ~= 0 then
                screen.setColor(numActiveColorR, numActiveColorG, numActiveColorB) -- 緑
            else
                screen.setColor(defaultColorR, defaultColorG, defaultColorB)       -- 白
            end
            local numText1 = string.format("N %02d: " .. numFormat, i, g_numberValues[i])
            screen.drawText(col1X, currentY, numText1)

            -- 2列目: 数値チャンネル 17-32 (変更なし)
            if g_numberValues[i + 16] ~= 0 then
                screen.setColor(numActiveColorR, numActiveColorG, numActiveColorB) -- 緑
            else
                screen.setColor(defaultColorR, defaultColorG, defaultColorB)       -- 白
            end
            local numText2 = string.format("N %02d: " .. numFormat, i + 16, g_numberValues[i + 16])
            screen.drawText(col2X, currentY, numText2)

            -- 3列目: オンオフチャンネル 1-16 (色設定を反転)
            -- 値(g_boolValues[i]) が true (On) かどうかチェック
            if g_boolValues[i] then
                -- true (On) なら緑色に設定
                screen.setColor(numActiveColorR, numActiveColorG, numActiveColorB) -- 緑
            else
                -- false (Off) なら赤色に設定
                screen.setColor(boolFalseColorR, boolFalseColorG, boolFalseColorB) -- 赤
            end
            local boolText1 = string.format("B %02d: %s", i, g_boolValues[i] and "T" or "F")
            screen.drawText(col3X, currentY, boolText1)

            -- 4列目: オンオフチャンネル 17-32 (色設定を反転)
            if g_boolValues[i + 16] then
                -- true (On) なら緑色に設定
                screen.setColor(numActiveColorR, numActiveColorG, numActiveColorB) -- 緑
            else
                -- false (Off) なら赤色に設定
                screen.setColor(boolFalseColorR, boolFalseColorG, boolFalseColorB) -- 赤
            end
            local boolText2 = string.format("B %02d: %s", i + 16, g_boolValues[i + 16] and "T" or "F")
            screen.drawText(col4X, currentY, boolText2)
        end
    end

    -- ループの後、念のためデフォルトの色に戻しておく
    screen.setColor(defaultColorR, defaultColorG, defaultColorB)
end
