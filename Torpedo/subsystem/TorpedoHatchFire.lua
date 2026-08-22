--[[
ASROCハッチ・射出制御

入力:
- Bool 1..4: ハッチ1..4の開放確認信号
- Bool 5: 射撃要請パルス
- Bool 6: 目標探知中

出力:
- Bool 1: ハッチ1開放指令
- Bool 2: ASROC 1射出パルス
- Bool 3: ハッチ2開放指令
- Bool 4: ASROC 2射出パルス
- Bool 5: ハッチ3開放指令
- Bool 6: ASROC 3射出パルス
- Bool 7: ハッチ4開放指令
- Bool 8: ASROC 4射出パルス

射撃要請は1番から順に未使用セルへ割り当てる。
開放確認信号が30tick連続した後、目標探知中なら1tickだけ射出信号を出す。
射出後は300tickハッチを開放し、その後閉鎖する。
]]

local CELL_COUNT = 4
local HATCH_CONFIRM_TICKS = 30
local HATCH_HOLD_AFTER_LAUNCH_TICKS = 300

local STATE_UNUSED = 0
local STATE_OPENING = 1
local STATE_FIRED = 2
local STATE_SPENT = 3

local cells = {}
for cellID = 1, CELL_COUNT do
    cells[cellID] = {
        state = STATE_UNUSED,
        openSignalStartTick = nil,
        launchTick = nil
    }
end

local currentTick = 0
local nextCellID = 1
local previousLaunchRequest = false

local function acceptLaunchRequest()
    while nextCellID <= CELL_COUNT and cells[nextCellID].state ~= STATE_UNUSED do
        nextCellID = nextCellID + 1
    end

    if nextCellID > CELL_COUNT then
        return
    end

    cells[nextCellID].state = STATE_OPENING
    cells[nextCellID].openSignalStartTick = nil
    nextCellID = nextCellID + 1
end

local function updateOpeningCell(cellID, cell, targetDetected)
    local firePulse = false

    if input.getBool(cellID) then
        if cell.openSignalStartTick == nil then
            cell.openSignalStartTick = currentTick
        end

        local hatchConfirmed = currentTick - cell.openSignalStartTick >= HATCH_CONFIRM_TICKS
        if hatchConfirmed and targetDetected then
            cell.state = STATE_FIRED
            cell.launchTick = currentTick
            firePulse = true
        end
    else
        -- 開放確認が途切れた場合は30tick確認を最初からやり直す。
        cell.openSignalStartTick = nil
    end

    return true, firePulse
end

local function updateFiredCell(cell)
    local keepHatchOpen = currentTick - cell.launchTick < HATCH_HOLD_AFTER_LAUNCH_TICKS
    if not keepHatchOpen then
        cell.state = STATE_SPENT
    end
    return keepHatchOpen, false
end

function onTick()
    currentTick = currentTick + 1

    local launchRequest = input.getBool(5)
    if launchRequest and not previousLaunchRequest then
        acceptLaunchRequest()
    end
    previousLaunchRequest = launchRequest

    local targetDetected = input.getBool(6)

    for cellID = 1, CELL_COUNT do
        local cell = cells[cellID]
        local hatchOpen = false
        local firePulse = false

        if cell.state == STATE_OPENING then
            hatchOpen, firePulse = updateOpeningCell(cellID, cell, targetDetected)
        elseif cell.state == STATE_FIRED then
            hatchOpen, firePulse = updateFiredCell(cell)
        end

        local hatchOutputChannel = cellID * 2 - 1
        local fireOutputChannel = cellID * 2
        output.setBool(hatchOutputChannel, hatchOpen)
        output.setBool(fireOutputChannel, firePulse)
    end
end
