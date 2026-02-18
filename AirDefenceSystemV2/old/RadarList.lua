inputNumber = input.getNumber
inputBool = input.getBool
output.setNumber = output.setNumber
output.setBool = output.setBool
math = math
maxLimitOver = 0
currentSetedNumber = 0
PackA = {}
PackC = {}
maxRange = property.getNumber("Range")
function onTick()
    filterDist = inputNumber(32) * maxRange -- 味方艦が存在するときはmaxRangeを狭めることでシステムを動作させないようにする
    maxOutNumber = 12
    currentSetedNumber = 0
    for i = 1, 12 do
        output.setNumber(i, 0)
    end
    -- 二つあるうちの一つ目のレーダーの情報をPackAに格納
    for i = 1, 6 do
        local a, b, dist
        a, b = inputNumber(i * 2 - 1), inputNumber(i * 2)
        if a * b == 0 then
            dist = 0
        else
            -- 距離を抽出
            a = string.format("%7f", math["math.abs"](a))
            b = string.format("%7f", math["math.abs"](b))
            dist = tonumber(a:sub(6, 7) .. b:sub(6, -1))
        end
        -- 距離が0でない、かつフィルター距離より小さい場合にPackAに格納
        if inputNumber(i * 2 - 1) ~= 0 and dist < filterDist then
            table.insert(PackA, { p1 = inputNumber(i * 2 - 1), p2 = inputNumber(i * 2) })
        end
    end
    -- 二つあるうちの二つ目のレーダーの情報をPackCに格納
    for i = 7, 12 do
        local a, b, dist
        a, b = inputNumber(i * 2 - 1), inputNumber(i * 2)
        if a * b == 0 then
            dist = 0
        else
            a = string.format("%7f", math["math.abs"](a))
            b = string.format("%7f", math["math.abs"](b))
            dist = tonumber(a:sub(6, 7) .. b:sub(6, -1))
        end
        if inputNumber(i * 2 - 1) ~= 0 and dist < filterDist then
            table.insert(PackC, { p1 = inputNumber(i * 2 - 1), p2 = inputNumber(i * 2) })
        end
    end

    output.setNumber(32, maxLimitOver) -- 前回最大出力上限を超えてしまい、今回その余りを出力する場合に1を出力する
    if #PackA + #PackC > 12 then
        maxLimitOver = 1
    else
        maxLimitOver = 0
    end
    -- PackAとPackCの長さを比較して、長い方を優先して出力する
    if #PackA >= #PackC and #PackA > 0 then
        for i = 1, #PackA do
            -- PackAの情報を出力
            output.setNumber(i * 2 - 1, PackA[1].p1)
            output.setNumber(i * 2, PackA[1].p2)
            table.remove(PackA, 1)                       -- 出力したPackAを削除
            currentSetedNumber = i * 2                   -- 現在出力する予定の数を記録
        end
        maxOutNumber = maxOutNumber - currentSetedNumber -- 最大出力可能数から出力する数を引く
        if #PackC > 0 and maxOutNumber > 0 then          -- まだ出力できるch数が残っている場合
            -- PackCの情報を出力
            for i = 1, math.min(#PackC, maxOutNumber / 2) do
                output.setNumber((i * 2 - 1) + currentSetedNumber, PackC[1].p1)
                output.setNumber((i * 2) + currentSetedNumber, PackC[1].p2)
                table.remove(PackC, 1)
            end
        end
    elseif #PackC > #PackA and #PackC > 0 then
        for i = 1, #PackC do
            output.setNumber(i * 2 - 1, PackC[1].p1)

            output.setNumber(i * 2, PackC[1].p2)
            table.remove(PackC, 1)
            currentSetedNumber = i * 2
        end
        maxOutNumber = maxOutNumber - currentSetedNumber
        if #PackA > 0 and maxOutNumber > 0 then
            for i = 1, math.min(#PackA, maxOutNumber / 2) do
                output.setNumber((i * 2 - 1) + currentSetedNumber, PackA[1].p1)
                output.setNumber((i * 2) + currentSetedNumber, PackA[1].p2)
                table.remove(PackA, 1)
            end
        end
    end
end
