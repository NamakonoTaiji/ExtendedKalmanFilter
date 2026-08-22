--[[
Multi Sonar Ping Scheduler

入力 Number:
- ch 1: ピンガーを届かせたい距離 [m]

プロパティ:
- SONAR_COUNT: 使用するソーナー数 (1～32)

出力 On/Off:
- ch 1～SONAR_COUNT:
  対応するソーナーのPing発射タイミングだけ1 tick On

動作:
- 水中音速 1480 m/s、60 tick/s とする。
- 1台のPing周期 = 指定距離までの往復時間
    cycleTicks = distance * 2 / 1480 * 60
- 複数ソーナーはこの周期をSONAR_COUNT等分して順番に発射する。
- 例: 1480 m, 2台
    cycle = 120 tick
    sonar 1 = 0 tick
    sonar 2 = 60 tick
    sonar 1 = 120 tick ...
- 距離入力の変更は次の周期開始時に反映する。
- 距離 <= 0 の間は発射しない。
--]]

MAX_DISTANCE = property.getNumber("MAX_DISTANCE")
MIN_DISTANCE = property.getNumber("MIN_DISTANCE")
DISTANCE_STEP = (MAX_DISTANCE - MIN_DISTANCE) / property.getNumber("DIST_STEP")
SOUND_SPEED = 1480
TICKS_PER_SECOND = 60

SONAR_COUNT = math.floor(property.getNumber("SONAR_COUNT") + 0.5)
if SONAR_COUNT < 1 then
    SONAR_COUNT = 1
end
if SONAR_COUNT > 32 then
    SONAR_COUNT = 32
end

phaseTick = 0
cycleTicks = 0
running = false
distance = 8000
function startCycle(dist)
    cycleTicks = math.floor(dist * 2 * TICKS_PER_SECOND / SOUND_SPEED + 0.5)

    -- 全ソーナーを別tickに割り当てられる最低周期を保証
    if cycleTicks < SONAR_COUNT then
        cycleTicks = SONAR_COUNT
    end

    phaseTick = 0
    running = true
end

function vectorMagnitude(v)
    local x, y, z
    x = v[1]
    y = v[2]
    z = v[3]
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

function vectorSub(v1, v2)
    local x1, x2, y1, y2, z1, z2
    x1 = v1[1]
    y1 = v1[2]
    z1 = v1[3]
    x2 = v2[1]
    y2 = v2[2]
    z2 = v2[3]
    return { x1 - x2, y1 - y2, z1 - z2 }
end

function onTick()
    -- Ping出力は必ず1 tickパルス
    for i = 1, 32 do
        output.setBool(i, false)
    end

    local targetCoords  = { input.getNumber(1), input.getNumber(2), input.getNumber(3) }
    local ownCoords     = { input.getNumber(4), input.getNumber(5), input.getNumber(6) }
    local isDetecting   = input.getBool(1)
    local increaseRange = input.getBool(2)
    local decreaseRange = input.getBool(3)
    if increaseRange then
        distance = math.min(distance + DISTANCE_STEP, MAX_DISTANCE)
    elseif decreaseRange then
        distance = math.max(distance - DISTANCE_STEP, MIN_DISTANCE)
    end

    if isDetecting then
        distance = vectorMagnitude(vectorSub(targetCoords, ownCoords)) * 1.1 + 500
    end

    -- 0以下なら停止し、次に正値が入ったtickを新しい周期の先頭にする
    if distance <= 0 then
        running = false
        phaseTick = 0
        return
    end

    if not running then
        startCycle(distance)
    end

    -- 周期内を等間隔に分割
    -- 端数が出る場合は最も近い整数tickへ丸める
    for i = 1, SONAR_COUNT do
        local fireTick = math.floor((i - 1) * cycleTicks / SONAR_COUNT + 0.5)

        if phaseTick == fireTick then
            output.setBool(i, true)
            break
        end
    end

    phaseTick = phaseTick + 1

    -- 次tickから新周期。捜索距離が変更された場合は新周期
    if phaseTick >= cycleTicks or increaseRange or decreaseRange then
        running = false
    end
    output.setNumber(1, distance)
end
