--[[
閾値以内の敵目標に対し、チャフを放出するプログラム
入力:
 - 数値1 ~ 27(防空システムからのコンポジット): 敵目標座標
 - 数値28 ~ 30(Physics Sensor): 自機座標
出力:
 - オンオフ1 ~ n: チャフ放出フラグ
--]]
local MAX_TARGET = 9                                                             -- 最大目標数
local CHAFF_DEPLOY_TARGET_DISTANCE = property.getNumber("ChaffDeployDistance")   -- チャフ放出距離の閾値
local CHAFF_DEPLOY_INTERVAL_TIME = property.getNumber("ChaffDeployIntervalTime") -- チャフ放出の時間間隔

local chaffDeployCount = 0                                                       -- チャフ放出回数を記憶
local chaffIntervalTimer = 0                                                     -- チャフを放出してから経過した時間を記憶

function onTick()
    --debug.log("-------------start-------------")
    local isChaffDeploy = false
    for i = 1, MAX_TARGET do
        output.setBool(i, false)
    end
    local ownP = {
        x = input.getNumber(28),
        y = input.getNumber(29),
        z = input.getNumber(30)
    }
    for i = 1, MAX_TARGET do
        local baseChannel = (i - 1) * 3
        local x = input.getNumber(baseChannel + 1)
        local y = input.getNumber(baseChannel + 2)
        local z = input.getNumber(baseChannel + 3)

        -- 有効な目標があれば距離を計算
        if (x ~= 0 or y ~= 0 or z ~= 0) then
            local targetP = {
                x = x,
                y = y,
                z = z
            }
            local distanceSquared = (targetP.x - ownP.x) ^ 2 + (targetP.y - ownP.y) ^ 2 + (targetP.z - ownP.z) ^ 2
            --debug.log("detectedEnemy..distance = " .. tostring(math.sqrt(distanceSquared)))
            -- 距離が閾値以内ならチャフフラグを立てる
            if (distanceSquared < CHAFF_DEPLOY_TARGET_DISTANCE ^ 2) then
                --debug.log("isChaffDeploy")
                isChaffDeploy = true
            end
        end
    end
    if (isChaffDeploy or (chaffIntervalTimer ~= 0)) and (chaffIntervalTimer < CHAFF_DEPLOY_INTERVAL_TIME) then
        chaffIntervalTimer = chaffIntervalTimer + 1
        --debug.log("chaffIntervalTimer = " .. chaffIntervalTimer)
    else
        chaffIntervalTimer = 0
    end
    if chaffIntervalTimer == 1 then
        chaffDeployCount = chaffDeployCount + 1
        output.setBool(chaffDeployCount, true)
    end
end
