-- 時分割多重でデータを仕分けるためのプリプロセッサ
-- 防空システムからの出力を扱い常に32chにtrueを出力する

local blinker = false
function onTick()
    for i = 1, 32 do
        output.setNumber(i, input.getNumber(i))
    end
    blinker = not blinker
    output.setBool(1, blinker)
    output.setBool(32, true)
end
