-- ミサイルが火器管制マイコン(ADSThreatOutput.lua)と防空システムのどちらと通信をさせるのかを判断するためのスクリプト

local isFireControlInitialized = false
local isLaunched = false
function onTick()
    if input.getNumber(32) ~= 0 then
        isFireControlInitialized = true -- ch32が入力された == 火器管制と通信中
    end
    
    if input.getNumber(32) == 0 and isFireControlInitialized then
        isLaunched = true -- 火器管置と通信済みかつ火器管制との通信が途絶えた == すでに射出済みで防空システムとの通信に切り替えるべき
    end

    output.setBool(1, isLaunched) -- trueの時は防空システム、falseの時は火器管制と通信
end