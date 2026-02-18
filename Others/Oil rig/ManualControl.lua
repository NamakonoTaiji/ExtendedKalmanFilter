function onTick()
    rodLinear = 0
    drillLiner = input.getNumber(1)
    if drillLiner == 0 then
        drillLiner = 1
    end
    isOutputLinerConnectRod = false
    isOutputDrillConnectRod = true
    output.setNumber(1, rodLinear)
    output.setNumber(2, drillLiner)
    output.setBool(1, isOutputLinerConnectRod)
    output.setBool(2, isOutputDrillConnectRod)
end
