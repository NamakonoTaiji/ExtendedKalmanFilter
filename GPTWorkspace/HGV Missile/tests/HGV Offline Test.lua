local function expect(condition, message)
    if not condition then error(message, 2) end
end

local function close(actual, expected, tolerance)
    return math.abs(actual - expected) <= tolerance
end

local function loadController(path, properties)
    local numbers, bools, outNumbers, outBools, logs = {}, {}, {}, {}, {}
    property = {
        getNumber = function(name) return properties[name] or 0 end,
        getBool = function(name) return properties[name] == true end
    }
    input = {
        getNumber = function(channel) return numbers[channel] or 0 end,
        getBool = function(channel) return bools[channel] or false end
    }
    output = {
        setNumber = function(channel, value) outNumbers[channel] = value end,
        setBool = function(channel, value) outBools[channel] = value end
    }
    debug = { log = function(message) logs[#logs + 1] = message end }
    assert(loadfile(path))()
    return numbers, bools, outNumbers, outBools, logs
end

local function setOwn(numbers, x, y, z)
    numbers[25], numbers[26], numbers[27] = x, y, z
    numbers[28], numbers[29], numbers[30] = 0, 0, 0
end

local function setDataLink(numbers, id, x, y, z)
    numbers[22], numbers[23], numbers[24] = x, y, z
    numbers[19], numbers[20], numbers[21] = 0, 0, 0
    numbers[32] = id
end

local function launchRadar(numbers, shipX, shipY, shipZ)
    setOwn(numbers, 0, 2000, 0)
    setDataLink(numbers, 190001, shipX, shipY, shipZ)
    onTick()
    setDataLink(numbers, 1, shipX, shipY, shipZ)
    onTick()
end

local radarProperties = {
    TRACK_GATE_M = 150,
    TERMINAL_HORIZONTAL_RANGE = 1500,
    SEA_SKIM_POPUP_RANGE = 1000,
    DL_GATE = 200,
    TRACK_DELETE_TICKS = 60,
    THREAT_LOST_TICKS = 3,
    CANDIDATE_MIN_HITS = 12,
    CANDIDATE_MAX_TICKS = 45
}

do
    local n, _, o = loadController("HGV Radar.lua", radarProperties)
    launchRadar(n, 0, 0, 5000)
    local first = o[10]
    local searchElevation = o[11]
    onTick()
    local second = o[10]
    onTick()
    local third = o[10]
    expect(close(first, -20 / 360, 1e-6), "SEARCH first beam must be ship azimuth -20 deg")
    expect(close(second, 0, 1e-6), "SEARCH second beam must be ship azimuth")
    expect(close(third, 20 / 360, 1e-6), "SEARCH third beam must be ship azimuth +20 deg")
    local shipElevation = math.atan(-2000, 5000) / (math.pi * 2)
    expect(close(searchElevation, shipElevation * .5, 1e-6),
        "SEARCH elevation must bisect ship elevation and the horizon")
    local verticalHalfWidth = 37.8 / 360
    expect(shipElevation >= searchElevation - verticalHalfWidth and
        shipElevation <= searchElevation + verticalHalfWidth, "vertical beam must include ship elevation")
    expect(0 >= searchElevation - verticalHalfWidth and 0 <= searchElevation + verticalHalfWidth,
        "vertical beam must include the horizon")
end

do
    local n, _, o, b = loadController("HGV Radar.lua", radarProperties)
    launchRadar(n, 1500, 0, 0)
    expect(b[4] == true, "TERMINAL must latch at horizontal distance 1500 m")
    expect(close(o[11], -.125, 1e-6), "TERMINAL must point directly at the ship elevation")
end
do
    local n, mode, _, b = loadController("HGV Radar.lua", radarProperties)
    mode[2] = true
    launchRadar(n, 1001, 0, 0)
    expect(b[5] == true, "Radar bool 2 must pass through to Guidance bool 5")
    expect(b[4] == false, "sea-skimming TERMINAL must not start outside 1000 m")
    setDataLink(n, 1, 1000, 0, 0)
    onTick()
    expect(b[4] == true, "sea-skimming TERMINAL must latch at horizontal distance 1000 m")
end


do
    local n, _, _, _, logs = loadController("HGV Radar.lua", radarProperties)
    launchRadar(n, 0, 0, 5000)
    n[1], n[4], n[7], n[10], n[13], n[16] = 2001, 2002, 2003, 2004, 2005, 2006
    for _ = 3, 60 do onTick() end
    local status = logs[#logs] or ""
    expect(status:find("radar_az_deg=", 1, true), "STATUS must log radar azimuth")
    expect(status:find("radar_el_deg=", 1, true), "STATUS must log radar elevation")
    expect(status:find("observations=0", 1, true), "STATUS must log observation count")
    expect(status:find("stored_tracks=0", 1, true), "STATUS must log stored track count")
    expect(status:find("r1_m=2001", 1, true) and status:find("r6_m=2006", 1, true), "STATUS must log six ranges")
end

do
    local n, _, _, b = loadController("HGV Radar.lua", radarProperties)
    launchRadar(n, 0, 0, 5000)
    local targetZ = 1000
    for _ = 1, 60 do
        targetZ = targetZ - 5
        n[1], n[2], n[3] = targetZ, 0, 0
        onTick()
        if b[1] then break end
    end
    expect(b[1] == true, "300 m/s inbound radar target must become an active threat")
    n[1], n[2], n[3] = 0, 0, 0
    onTick()
    onTick()
    onTick()
    expect(b[1] == false, "active threat must clear after three missed ticks")
end

do
    local n, _, _, b, logs = loadController("HGV Radar.lua", radarProperties)
    launchRadar(n, 0, 0, 5000)
    local targetZ = 300
    for _ = 1, 80 do
        targetZ = targetZ - 5
        n[1] = math.abs(targetZ)
        n[2] = targetZ >= 0 and 0 or .5
        n[3] = 0
        onTick()
    end
    local passed = false
    for _, message in ipairs(logs) do
        if message:find("THREAT_PASSED", 1, true) then passed = true end
    end
    expect(passed, "active threat must end after closest-point passage")
    expect(b[1] == false, "passed threat must not remain active")
end

local guidanceProperties = {
    PN_FIN_STRENGTH = 1,
    TERMINAL_HORIZONTAL_RANGE = 1500,
    PPN_FIN_STRENGTH = 1,
    SEA_SKIM_POPUP_RANGE = 1000,
    SEA_SKIM_POPUP_ALTITUDE = 400,
    VERTICAL_MANEUVER_AMPLITUDE_M = 100,
    LOGIC_DELAY = 0,
    GRAVITY_COMP_ENABLE = false,
    MAX_FIN_COMMAND = 0
}

local function setGuidanceCommon(numbers, bools, shipX, shipY, shipZ)
    bools[3] = true
    numbers[1], numbers[2], numbers[3] = shipX, shipY, shipZ
    numbers[13], numbers[14], numbers[15] = 0, CRUISE_ALTITUDE_M, 0
    numbers[16], numbers[17], numbers[18], numbers[19] = 1, 0, 0, 0
    numbers[23] = 1
end

do
    local n, b, o = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, 5000)
    onTick()
    expect(o[2] > 0, "SEARCH must initially command the upper altitude target")
    for _ = 2, 90 do onTick() end
    expect(o[2] > 0, "upper command must last 90 ticks")
    onTick()
    expect(o[2] < 0, "vertical command must reverse after 90 ticks")

    n[23] = 2
    onTick()
    expect(close(o[2], 0, 1e-9), "CANDIDATE must stop vertical weaving at 2000 m")
end
do
    local n, b, o = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, 5000)
    b[5] = true
    n[14] = SEA_SKIM_ALTITUDE_M
    onTick()
    expect(close(o[2], 0, 1e-9), "sea-skimming cruise must hold its configured altitude without weaving")
end

do
    local n, b, o = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, SEA_SKIM_POPUP_RANGE_M)
    b[5] = true
    n[14] = SEA_SKIM_ALTITUDE_M
    onTick()
    expect(terminalLatched == true, "sea-skimming popup must start at its configured horizontal distance")
    expect(seaSkimDiveLatched == false, "popup must continue below its altitude and outside its dive range")
    expect(o[2] > 0, "popup must command a climb toward its configured altitude")
    n[14] = SEA_SKIM_POPUP_ALTITUDE_M
    onTick()
    expect(seaSkimDiveLatched == true, "reaching the popup altitude must start terminal dive")
end

do
    local n, b = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, SEA_SKIM_DIVE_RANGE_M - 1)
    b[5] = true
    n[14] = SEA_SKIM_ALTITUDE_M
    onTick()
    expect(seaSkimDiveLatched == true, "target distance below the dive range must start terminal dive")
end


do
    local n, b, o = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, 5000)
    b[1] = true
    n[7], n[8], n[9], n[12] = 0, CRUISE_ALTITUDE_M, 1000, 7
    n[20], n[21], n[22] = -300, 0, 0
    onTick()
    expect(close(o[1], 0, 1e-6), "EVADE must choose the SAM-velocity perpendicular that advances toward ship")
    local first = getBeamDirection({ 0, CRUISE_ALTITUDE_M, 0 }, { 0, CRUISE_ALTITUDE_M, 1000 }, { -300, 0, 0 }, beamSide)
    expect(close(first[1] * -300, 0, 1e-6), "beam command must be perpendicular to SAM velocity")
    local selectedSide = beamSide
    n[20], n[22] = 0, -300
    onTick()
    local second = getBeamDirection({ 0, CRUISE_ALTITUDE_M, 0 }, { 0, CRUISE_ALTITUDE_M, 1000 }, { 0, 0, -300 }, beamSide)
    expect(beamSide == selectedSide, "beam side must stay fixed for the same SAM")
    expect(close(second[2] * -300, 0, 1e-6), "updated beam command must remain perpendicular to turning SAM")
    expect(o[1] < -1, "EVADE command must follow the updated SAM course")
end

do
    local n, b, o = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, 5000)
    b[1] = true
    n[7], n[8], n[9], n[12] = 0, CRUISE_ALTITUDE_M, 1000, 8
    n[20], n[21], n[22] = -300, 0, 0
    onTick()
    expect(o[2] > 0, "HGV EVADE must initially command the upper vertical maneuver target")
    for _ = 2, 90 do onTick() end
    expect(o[2] > 0, "HGV EVADE upper command must last 90 ticks")
    onTick()
    expect(o[2] < 0, "HGV EVADE vertical command must reverse after 90 ticks")
end

do
    local n, b, o = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, 5000)
    b[1], b[5] = true, true
    n[14] = SEA_SKIM_ALTITUDE_M
    n[7], n[8], n[9], n[12] = 0, SEA_SKIM_ALTITUDE_M, 1000, 9
    n[20], n[21], n[22] = -300, 0, 0
    onTick()
    expect(close(o[2], 0, 1e-9), "sea-skimming EVADE must hold its altitude without vertical weaving")
end

do
    local n, b, _, _, logs = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, 0, 0, 5000)
    b[1] = true
    n[7], n[8], n[9], n[12] = 3, CRUISE_ALTITUDE_M, 400, 10
    n[20], n[21], n[22] = 0, 0, -300
    n[13] = 0
    onTick()
    n[13] = 1
    onTick()
    n[13] = 3
    onTick()
    local kinematics = logs[#logs] or ""
    expect(kinematics:find("[HGV_DBG] KINEMATICS", 1, true),
        "near-threat guidance must log actual HGV kinematics every tick")
    expect(kinematics:find("velocity_x_mps=120", 1, true),
        "kinematics must derive world velocity from Physics Sensor position differences")
    expect(kinematics:find("accel_x_mps2=3600", 1, true),
        "kinematics must derive world acceleration from successive velocities")
    expect(kinematics:find("actual_lateral_accel_mps2=3600", 1, true),
        "kinematics must remove the SAM LOS-parallel acceleration component")
end

do
    local n, b = loadController("HGV Guidance.lua", guidanceProperties)
    setGuidanceCommon(n, b, TERMINAL_HORIZONTAL_RANGE_M, 0, 0)
    onTick()
    expect(terminalLatched == true, "guidance TERMINAL must latch at its horizontal threshold")
end

print("HGV offline tests passed")
