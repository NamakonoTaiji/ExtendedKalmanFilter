--[[
コンポジット信号ビューア スクリプト (小数点桁数 設定可能版)

機能:
- コンポジット入力の数値チャンネル 1-32 の値を読み取る
- コンポジット入力のオンオフチャンネル 1-32 の値を読み取る
- マイクロコントローラーのプロパティ "DecimalPlaces" から小数点以下の表示桁数を読み取る
- 読み取った値をモニターに4列構成で見やすく表示する
- 有効な数字やオン信号をdebug.logに出力するオプション(プロパティから設定可能)
]]

local propVal = property.getNumber("DecimalPlaces")
local NUMERICAL_DEBUG_LOG_ENABLED = property.getBool("NUM_DEBUGLOG")
local BOOLEAN_DEBUG_LOG_ENABLED = property.getBool("BOOL_DEBUGLOG")
local DEBUG_STRING_PREFIX = property.getText("DEBUG_STRING_PREFIX")
-- グローバル変数
g_numberValues = {} -- 数値チャンネルの値を格納
g_boolValues = {}   -- オンオフチャンネルの値を格納
g_decimalPlaces = 2 -- 小数点以下の表示桁数 (デフォルト値: 2)
local onTickCount = 0

-- 初期化
for i = 1, 32 do
    g_numberValues[i] = 0
    g_boolValues[i] = false
end

--[[
onTick() 関数: ゲームの論理ティック(1/60秒)ごとに呼び出される
入力チャンネルの値とプロパティ値を読み取り、グローバル変数に保存する
--]]
function onTick()
    onTickCount = onTickCount + 1

    -- --- プロパティから小数点以下の桁数を読み取る ---
    -- property.getNumber("ラベル名") で数値プロパティの値を取得 [cite: 12]
    -- "DecimalPlaces" というラベルのプロパティから値を取得試行


    -- プロパティ値の検証と適用
    if type(propVal) == "number" then
        -- 読み取った値が数値の場合
        -- math.floor() で小数点以下を切り捨てて整数にする
        -- math.max(0, ...) で 0 未満にならないようにする
        -- math.min(..., 8) で 8 より大きくならないようにする (上限を8桁に設定)
        g_decimalPlaces = math.max(0, math.min(8, math.floor(propVal)))
    else
        -- プロパティが存在しないか、数値でない場合はデフォルト値(2)を使う
        g_decimalPlaces = 2
    end
    -- ---------------------------------------------
    for i = 1, 32 do
        if input.getNumber(i) ~= 0 and (NUMERICAL_DEBUG_LOG_ENABLED or BOOLEAN_DEBUG_LOG_ENABLED) then
            debug.log(string.format("-----------------------%s START-----------------------", DEBUG_STRING_PREFIX))
            debug.log(onTickCount .. "tick")
            break
        end
    end
    -- コンポジット入力チャンネルの値を読み取る (前回と同じ)
    for i = 1, 32 do
        g_numberValues[i] = input.getNumber(i)
        g_boolValues[i] = input.getBool(i)
        if g_numberValues[i] ~= 0 and NUMERICAL_DEBUG_LOG_ENABLED then
            debug.log("ch: " .. i .. " >> " .. g_numberValues[i])
        end
        if g_boolValues[i] and BOOLEAN_DEBUG_LOG_ENABLED then
            debug.log("ch: " .. i .. " >> " .. "true")
        end
    end
end

--[[
onDraw() 関数: モニターの描画が必要な時に呼び出される
保存された値を整形してモニターに描画する
--]]
function onDraw()
    local width = screen.getWidth()
    local height = screen.getHeight()
    local bgColorR, bgColorG, bgColorB = 0, 0, 0
    local textColorR, textColorG, textColorB = 255, 255, 255
    local colWidth = width / 4
    local rowHeight = 6
    local maxRows = math.floor(height / rowHeight)
    local col1X = 5
    local col2X = colWidth * 1 + 5
    local col3X = colWidth * 2 + 5
    local col4X = colWidth * 3 + 5

    -- --- 動的な数値書式指定文字列を生成 ---
    -- g_decimalPlaces の値に基づいて、"%.2f" や "%.0f" のような文字列を作る
    -- string.format の中で % を文字として使うには %% と書く必要がある
    local numFormat = string.format("%%.%df", g_decimalPlaces)
    -- 例: g_decimalPlaces が 3 なら numFormat は "%.3f" になる
    -- --------------------------------------

    screen.setColor(bgColorR, bgColorG, bgColorB)
    screen.drawClear()
    screen.setColor(textColorR, textColorG, textColorB)

    for i = 1, 16 do
        if i <= maxRows then
            local currentY = (i - 1) * rowHeight + 2

            -- 1列目: 数値チャンネル 1-16 (動的書式を使用)
            -- string.format("Num %02d: " .. numFormat, i, g_numberValues[i]) という書き方で、
            -- numFormat 変数の中身 ("%.2f" など) を使って数値をフォーマットする
            local numText1 = string.format("Num %02d: " .. numFormat, i, g_numberValues[i])
            screen.drawText(col1X, currentY, numText1)

            -- 2列目: 数値チャンネル 17-32 (動的書式を使用)
            local numText2 = string.format("Num %02d: " .. numFormat, i + 16, g_numberValues[i + 16])
            screen.drawText(col2X, currentY, numText2)

            -- 3列目: オンオフチャンネル 1-16 (前回と同じ)
            local boolText1 = string.format("Bool %02d: %s", i, g_boolValues[i] and "T" or "F")
            screen.drawText(col3X, currentY, boolText1)

            -- 4列目: オンオフチャンネル 17-32 (前回と同じ)
            local boolText2 = string.format("Bool %02d: %s", i + 16, g_boolValues[i + 16] and "T" or "F")
            screen.drawText(col4X, currentY, boolText2)
        end
    end
end
