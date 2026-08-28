iN = input.getNumber
iB = input.getBool
oB = output.setBool
LogicDelay = property.getNumber("LogicDelay")
if LogicDelay == 0 then LogicDelay = 4 end

approach_Velocity = 0
oldDistance = 0
isInit = true
vtStart = false
VTfuse = false
MAX_CLOSING_VEL_PER_TICK = -13.33333
function onTick()
    local distance = iN(1)
    local isDetected = iB(1)

    if isDetected then
        vtStart = true
    end

    if vtStart then
        -- 初期化処理（最初の1tick目の距離を記憶）
        if isInit then
            oldDistance = distance
            isInit = false
        end

        -- ロスト時は直前の近接速度で距離を外挿補正
        if not isDetected then
            distance = oldDistance + approach_Velocity
        end

        -- 1tick分のデルタから近接速度を更新
        approach_Velocity = math.max(distance - oldDistance,MAX_CLOSING_VEL_PER_TICK)
        oldDistance = distance

        -- ロジック遅延を考慮した起爆判定
        if distance + approach_Velocity * LogicDelay < 0 then
            VTfuse = true
        end
    end

    oB(1, VTfuse)
end