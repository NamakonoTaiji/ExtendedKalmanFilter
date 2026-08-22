--================================
-- CPP Ship Controller
--================================

MAX_FORWARD = property.getNumber("MAX_FORWARD")
MAX_REVERSE = property.getNumber("MAX_REVERSE")

FORWARD_GEARS = 5
REVERSE_GEARS = 2


--================================
-- Speed hold settings
--================================

-- Target speed added by each gear [m/s]
-- Defaults make forward gears 3, 6, 9, 12, 15, 18 m/s.
FORWARD_SPEED_STEP = property.getNumber("FORWARD_SPEED_STEP")
REVERSE_SPEED_STEP = property.getNumber("REVERSE_SPEED_STEP")

-- PI gains. Values are pitch command per speed error.
SPEED_KP = property.getNumber("SPEED_KP")
SPEED_KI = property.getNumber("SPEED_KI")

-- Stormworks returns 0 for a property which does not exist.
if FORWARD_SPEED_STEP <= 0 then FORWARD_SPEED_STEP = 3 end
if REVERSE_SPEED_STEP <= 0 then REVERSE_SPEED_STEP = 3 end
if SPEED_KP <= 0 then SPEED_KP = 0.002 end
if SPEED_KI <= 0 then SPEED_KI = 0.00002 end


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

-- Maximum reverse steering difference relative to base reverse pitch.
-- Keeping this below 1 prevents the inner propeller from reaching
-- zero or changing to forward pitch during a reverse turn.
REVERSE_STEER_RATIO = property.getNumber("REVERSE_STEER_RATIO")

if REVERSE_STEER_RATIO <= 0 then REVERSE_STEER_RATIO = 0.75 end
REVERSE_STEER_RATIO = math.min(REVERSE_STEER_RATIO, 0.95)


--================================
-- Pitch movement
--================================

-- Maximum pitch movement per tick
PITCH_SLEW = 0.001


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

speedIntegral = 0
controllerGear = 0


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

    -- Number input 4 is intentionally unused (engine RPS removed).
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
    -- Gear -> target speed
    --============================

    targetSpeed = 0
    driveDirection = 0

    if gear > 0 then

        targetSpeed =
            gear
            * FORWARD_SPEED_STEP

        driveDirection = 1

    elseif gear < 0 then

        targetSpeed =
            (-gear)
            * REVERSE_SPEED_STEP

        driveDirection = -1
    end


    --============================
    -- Speed PI controller
    --
    -- The old linear gear-to-pitch mapping made first gear too
    -- powerful because hull speed is very nonlinear versus pitch.
    -- This controller instead finds the pitch required to hold the
    -- speed assigned to the selected gear.
    --============================

    basePitch = 0
    speedError = targetSpeed - speed

    -- Do not carry high-gear integral into a downshift or reversal.
    if gear == 0
        or gear * controllerGear < 0
        or math.abs(gear) < math.abs(controllerGear) then

        speedIntegral = 0
    end

    controllerGear = gear

    if driveDirection ~= 0 then

        pitchLimit =
            driveDirection > 0
            and MAX_FORWARD
            or math.abs(MAX_REVERSE)

        proportional =
            speedError
            * SPEED_KP

        requestedPitch =
            speedIntegral
            + proportional

        -- Conditional integration prevents wind-up at both limits.
        if not (
            requestedPitch >= pitchLimit and speedError > 0
        ) and not (
            requestedPitch <= 0 and speedError < 0
        ) then

            speedIntegral =
                clamp(
                    speedIntegral
                    + speedError * SPEED_KI,
                    0,
                    pitchLimit
                )
        end

        pitchMagnitude =
            clamp(
                speedIntegral
                + proportional,
                0,
                pitchLimit
            )

        basePitch =
            pitchMagnitude
            * driveDirection
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

    -- At low reverse pitch an absolute steering correction can be
    -- much larger than the propulsion pitch. Limit the left/right
    -- difference so both propellers continue producing reverse thrust.
    if basePitch < 0 then

        d =
            math.min(
                d,
                math.abs(basePitch)
                * REVERSE_STEER_RATIO
            )
    end


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

    output.setNumber(8, targetSpeed)
    output.setNumber(9, speedError)
end
