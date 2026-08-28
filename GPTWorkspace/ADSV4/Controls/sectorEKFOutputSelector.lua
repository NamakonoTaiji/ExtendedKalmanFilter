-- 4基のセクターEKF出力を1tickずつ選択する2bitセレクター
--
-- Bool出力
--   ch1: 下位選択bit
--   ch2: 上位選択bit
--
-- 数値出力
--   ch1: 現在選択しているセクター番号（1～4、配線確認用）
--
-- Composite Switchboxを3基使用する場合
--   第1段A: セクター1 / セクター2、選択信号=Bool1
--   第1段B: セクター3 / セクター4、選択信号=Bool1
--   第2段 : 第1段A / 第1段B、選択信号=Bool2

local sectorIndex = 0

function onTick()
    output.setBool(1, sectorIndex % 2 == 1)
    output.setBool(2, sectorIndex >= 2)
    output.setNumber(1, sectorIndex + 1)

    sectorIndex = (sectorIndex + 1) % 4
end
