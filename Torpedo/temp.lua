local coordBuffer = {x=0,y=0,z=0}

function onTick()
    local x=input.getNumber(1)
    local y=input.getNumber(2)
    local z=input.getNumber(3)

    if x~=0 or y~=0 or z~=0 then
        coordBuffer.x=x
        coordBuffer.y=y
        coordBuffer.z=z
    end

    output.setNumber(1,coordBuffer.x)
    output.setNumber(2,coordBuffer.y)
    output.setNumber(3,coordBuffer.z)
end