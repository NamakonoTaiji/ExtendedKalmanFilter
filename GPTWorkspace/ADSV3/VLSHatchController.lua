--[[
ADSThreatOutputのハッチ開放ビットマスクを、セル別のハッチ開放Boolへ変換する。
射出パルスを受け取ったセルは、ミサイル通過のため一定時間開放を維持する。

入力:
- num 16: ハッチ開放ビットマスク（bit 0 = セル1、bit 1 = セル2 ...）
- bool 1..32: ADSThreatOutputのセル別射出パルス

出力:
- bool 1..32: 各VLSセルのハッチ開放指令

プロパティ:
- HATCH_HOLD_TICKS: 射出後の開放維持時間（未設定時60tick）
]]

HATCH_HOLD_TICKS = property.getNumber("HATCH_HOLD_TICKS")
if HATCH_HOLD_TICKS <= 0 then HATCH_HOLD_TICKS = 60 end

currentTick = 0
hatchHoldEndTicks = {}

function onTick()
    currentTick = currentTick + 1
    local hatchMask = math.floor(input.getNumber(16) + 0.5)

    for cell = 1, 32 do
        if input.getBool(cell) then
            hatchHoldEndTicks[cell] = currentTick + HATCH_HOLD_TICKS
        end

        local cellBit = 2 ^ (cell - 1)
        local isRequested = math.floor(hatchMask / cellBit) % 2 == 1
        local isHeldOpen = currentTick < (hatchHoldEndTicks[cell] or 0)
        output.setBool(cell, isRequested or isHeldOpen)
    end
end
