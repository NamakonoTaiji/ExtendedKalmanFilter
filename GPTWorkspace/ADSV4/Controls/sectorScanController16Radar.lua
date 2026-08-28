-- 16基フェーズドアレイ・セクター走査コントローラー
--
-- 入力
--   Bool 1: システム起動
--
-- 必要な数値プロパティ
--   Elevation   : 奇数番レーダーのManual仰角turn（推奨値: 0.125）
--   AzimuthTrim : 全レーダー共通の取付方位補正turn（推奨値: 0）
--
-- 数値出力（レーダーi = 1～16）
--   ch (i * 2 - 1): Manual方位角
--   ch (i * 2)    : Manual仰角
--
-- Bool出力
--   ch 1～16 : 各レーダーの起動信号
--   ch 17～32: 各レーダーの4tick照射開始時に出す1tickパルス
--
-- レーダー順序
--   1～4   : 前方セクター、位相0～3
--   5～8   : 右方セクター、位相0～3
--   9～12  : 後方セクター、位相0～3
--   13～16 : 左方セクター、位相0～3

local RADARS_PER_SECTOR = 4
local SECTOR_COUNT = 4
local DETECTION_INTERVAL = 4
local SCAN_POINT_COUNT = 17
local SCAN_PHASE_STEP = 3 -- (走査点数 - 1) / セクター当たりレーダー数
local SCAN_START = -42 / 360
local SCAN_STEP = 5.25 / 360

local SECTOR_CENTER = {
    0,
    0.25,
    -0.5,
    -0.25
}

local ODD_RADAR_ELEVATION = math.max(-0.125, math.min(0.125, property.getNumber("Elevation")))
local EVEN_RADAR_ELEVATION = 0.115
local AZIMUTH_TRIM = property.getNumber("AzimuthTrim")

local scanTick = 0

local function wrapTurn(turn)
    return (turn + 0.5) % 1 - 0.5
end

function onTick()
    local enabled = input.getBool(1)

    if not enabled then
        scanTick = 0
    end

    for sector = 0, SECTOR_COUNT - 1 do
        for phase = 0, RADARS_PER_SECTOR - 1 do
            local radarIndex = sector * RADARS_PER_SECTOR + phase + 1
            local localTick = scanTick - phase
            local radarEnabled = enabled and localTick >= 0
            local dwell = 0

            if radarEnabled then
                dwell = math.floor(localTick / DETECTION_INTERVAL)
            end

            -- M=13では位相0,10,7,4により、全走査点を正確に13tick間隔で再訪する
            local scanIndex = (dwell - phase * SCAN_PHASE_STEP) % SCAN_POINT_COUNT
            local scanOffset = SCAN_START + scanIndex * SCAN_STEP
            local azimuth = wrapTurn(SECTOR_CENTER[sector + 1] + AZIMUTH_TRIM + scanOffset)
            -- 偶数番は水平線より少し下まで覆い、奇数番は天頂まで覆う
            local elevation = radarIndex % 2 == 0 and EVEN_RADAR_ELEVATION or ODD_RADAR_ELEVATION

            output.setNumber(radarIndex * 2 - 1, azimuth)
            output.setNumber(radarIndex * 2, elevation)
            output.setBool(radarIndex, radarEnabled)
            output.setBool(
                16 + radarIndex,
                radarEnabled and localTick % DETECTION_INTERVAL == 0
            )
        end
    end

    if enabled then
        scanTick = scanTick + 1
    end
end
