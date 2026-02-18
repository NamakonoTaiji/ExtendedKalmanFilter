function onTick()
    if input.getBool(1) then
        for i = 1, 3 do
            output.setNumber(i, input.getNumber(i + 3))
        end
        for i = 4, 9 do
            output.setNumber(i, 0)
        end
    else
        for i = 1, 9 do
            output.setNumber(i, 0)
        end
    end
    output.setBool(1, input.getBool(1))
end
