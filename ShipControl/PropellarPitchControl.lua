--================================
-- CPP Ship Controller
--================================

MAX_FORWARD = property.getNumber("MAX_FORWARD")
MAX_REVERSE = property.getNumber("MAX_REVERSE")

FORWARD_GEARS = 6
REVERSE_GEARS = 2


--================================
-- Steering settings
--================================

-- Maximum target yaw rate at low speed [rps]
MAX_YAW_LOW = property.getNumber("MAX_YAW_LOW")

-- Maximum target yaw rate at high speed [rps]
MAX_YAW_HIGH = property.getNumber("MAX_YAW_HIGH")

-- Speed where high-speed limit is fully applied [m/s]
YAW_REDUCE_SPEED = 20

-- Yaw controller gain
YAW_KP = property.getNumber("YAW_KP")

-- Feed-forward pitch difference
STEER_FF = 0.06

-- Maximum steering pitch correction
MAX_STEER = property.getNumber("MAX_STEER")


--================================
-- Pitch movement
--================================

-- Maximum pitch movement per tick
PITCH_SLEW = 0.001


--================================
-- RPS protection
--================================

-- Set these to suit the engine
RPS_HIGH = 60

-- Pitch added if RPS exceeds RPS_HIGH
RPS_GAIN = 0.01

--================================
-- Direction corrections
--================================

-- If steering is backwards, change to -1
STEER_SIGN = 1

-- If yaw sensor direction is backwards, change to -1
YAW_SIGN = 1


--================================
-- Internal state
--================================

gear = 0
lastShift = 0

leftPitch = 0
rightPitch = 0


function clamp(x, mn, mx)
    return math.max(mn, math.min(mx, x))
end


function moveTowards(current, target, step)

    if current < target then
        return math.min(current + step, target)

    elseif current > target then
        return math.max(current - step, target)
    end

    return current
end


function onTick()

    --============================
    -- Inputs
    --============================

    shiftInput = input.getNumber(1)

    steer =
        clamp(
            input.getNumber(2),
            -1,
            1
        )
        * STEER_SIGN

    speed =
        math.abs(
            input.getNumber(3)
        )

    rps =
        math.abs(
            input.getNumber(4)
        )

    yawRate =
        input.getNumber(5)
        * YAW_SIGN


    --============================
    -- Gear change
    --============================

    shift = 0

    if shiftInput > 0.5 then
        shift = 1

    elseif shiftInput < -0.5 then
        shift = -1
    end


    -- Rising edge only
    if shift ~= 0 and lastShift == 0 then

        gear =
            clamp(
                gear + shift,
                -REVERSE_GEARS,
                FORWARD_GEARS
            )
    end

    lastShift = shift


    --============================
    -- Gear -> pitch
    --============================

    basePitch = 0

    if gear > 0 then

        basePitch =
            MAX_FORWARD
            * gear
            / FORWARD_GEARS

    elseif gear < 0 then

        basePitch =
            (-MAX_REVERSE)
            * gear
            / REVERSE_GEARS
    end


--============================
-- RPS protection
--============================

rpsCorrection = 0

if rps > RPS_HIGH and basePitch > 0 then

    -- 1段分のPitch幅
    gearPitchStep =
        MAX_FORWARD
        / FORWARD_GEARS

    -- RPSによる補正
    rpsCorrection =
        clamp(
            (rps - RPS_HIGH)
            * RPS_GAIN,
            0,
            gearPitchStep * 0.5
        )

    -- 現在のギアPitchに対して
    -- 最大半段分だけ増ピッチ
    basePitch =
        math.min(
            basePitch + rpsCorrection,
            MAX_FORWARD
        )
end


    --============================
    -- Target yaw rate
    --============================

    speedFactor =
        clamp(
            speed / YAW_REDUCE_SPEED,
            0,
            1
        )

    maxYaw =
        MAX_YAW_LOW
        + (MAX_YAW_HIGH - MAX_YAW_LOW)
        * speedFactor


    targetYaw =
        steer * maxYaw


    --============================
    -- Yaw feedback controller
    --============================

    yawError =
        targetYaw - yawRate


    steerCorrection =
        steer * STEER_FF
        + yawError * YAW_KP


    steerCorrection =
        clamp(
            steerCorrection,
            -MAX_STEER,
            MAX_STEER
        )


    -- No steering input:
    -- only use feedback gently to stop residual rotation
    if math.abs(steer) < 0.02 then

        steerCorrection =
            clamp(
                -yawRate * YAW_KP,
                -MAX_STEER,
                MAX_STEER
            )
    end


    --============================
    -- Differential CPP control
    --
    -- First increase outer prop.
    -- Only decrease inner prop if
    -- outer prop reaches pitch limit.
    --============================

    targetLeft = basePitch
    targetRight = basePitch

    d = math.abs(steerCorrection)


    if basePitch > 0 then

        --========================
        -- Forward
        --========================

        if steerCorrection > 0 then

            -- Right turn
            -- Left = outer prop

            available =
                MAX_FORWARD
                - basePitch

            increase =
                math.min(d, available)

            remaining =
                d - increase

            targetLeft =
                basePitch + increase

            targetRight =
                basePitch - remaining


        elseif steerCorrection < 0 then

            -- Left turn
            -- Right = outer prop

            available =
                MAX_FORWARD
                - basePitch

            increase =
                math.min(d, available)

            remaining =
                d - increase

            targetRight =
                basePitch + increase

            targetLeft =
                basePitch - remaining
        end


    elseif basePitch < 0 then

        --========================
        -- Reverse
        --
        -- Reverse steering direction
        -- naturally changes because
        -- thrust direction is reversed.
        --========================

        if steerCorrection > 0 then

            available =
                math.abs(
                    MAX_REVERSE - basePitch
                )

            increase =
                math.min(
                    d,
                    available
                )

            remaining =
                d - increase

            targetRight =
                basePitch - increase

            targetLeft =
                basePitch + remaining


        elseif steerCorrection < 0 then

            available =
                math.abs(
                    MAX_REVERSE - basePitch
                )

            increase =
                math.min(
                    d,
                    available
                )

            remaining =
                d - increase

            targetLeft =
                basePitch - increase

            targetRight =
                basePitch + remaining
        end
    end


    --============================
    -- Pitch limits
    --============================

    targetLeft =
        clamp(
            targetLeft,
            MAX_REVERSE,
            MAX_FORWARD
        )

    targetRight =
        clamp(
            targetRight,
            MAX_REVERSE,
            MAX_FORWARD
        )


    --============================
    -- Slew limiter
    --============================

    leftPitch =
        moveTowards(
            leftPitch,
            targetLeft,
            PITCH_SLEW
        )

    rightPitch =
        moveTowards(
            rightPitch,
            targetRight,
            PITCH_SLEW
        )


    --============================
    -- Outputs
    --============================

    output.setNumber(1, leftPitch)
    output.setNumber(2, rightPitch)

    output.setNumber(3, gear)
    output.setNumber(4, basePitch)

    output.setNumber(
        5,
        steerCorrection
    )

    output.setNumber(
        6,
        targetYaw
    )

    output.setNumber(
        7,
        yawRate
    )
end