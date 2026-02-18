--[[
================================================================================
  PID Controller Library
  - Stormworks環境で確実に動作する、シンプルなテーブルベースの構造。
  - PID.update(pid_instance, ...) のように、操作対象のインスタンスを
    第1引数として渡して使用します。
================================================================================
]]

PID = {}
DT = 1 / 60

pitch = 0
roll = 0

PITCH_SEISI = property.getNumber("PITCH_SEISI") * 0.001
ROLL_SEISI = property.getNumber("ROLL_SEISI") * 0.001

-- MAX_PITCH_ANGULAR_VELOCITY = property.getNumber("MAX_PITCH_ANGULAR_VELOCITY") -- ピッチ回転の最大角速度(回転単位/s)
-- MAX_ROLL_ANGULAR_VELOCITY = property.getNumber("MAX_ROLL_ANGULAR_VELOCITY")   -- ロール回転の最大角速度(回転単位/s)
MAX_YAW_ANGULAR_VELOCITY = property.getNumber("MAX_YAW_ANGULAR_VELOCITY") -- ヨー回転の最大角速度(回転単位/s)
MAX_YAW_OUTPUT = property.getNumber("MAX_YAW_OUTPUT")                     -- ヨーの最大出力
MAX_CLIMB_SPEED = 20                                                      -- 最大上昇/降下速度(m/s)
MIN_COLLECTIVE = property.getNumber("MIN_COLLECTIVE")
NEUTRAL_COLLECTIVE = property.getNumber("NEUTRAL_COLLECTIVE")
MAX_COLLECTIVE = property.getNumber("MAX_COLLECTIVE")

PITCH_TILT_LIMIT = property.getNumber("PITCH_TILT_LIMIT(deg)") / 360
ROLL_TILT_LIMIT = property.getNumber("ROLL_TILT_LIMIT(deg)") / 360

IS_THROTTLE_ANALOG_INPUT = property.getBool("ANALOG_THROTTLE_INPUT")

ROLL_CONTROL_P = property.getNumber("ROLL_CONTROL_P")
ROLL_CONTROL_I = property.getNumber("ROLL_CONTROL_I")
ROLL_CONTROL_D = property.getNumber("ROLL_CONTROL_D")
ROLL_AUTO_CONTROL_P = property.getNumber("ROLL_AUTO_CONTROL_P")
ROLL_AUTO_CONTROL_I = property.getNumber("ROLL_AUTO_CONTROL_I")
ROLL_AUTO_CONTROL_D = property.getNumber("ROLL_AUTO_CONTROL_D")

PITCH_CONTROL_P = property.getNumber("PITCH_CONTROL_P")
PITCH_CONTROL_I = property.getNumber("PITCH_CONTROL_I")
PITCH_CONTROL_D = property.getNumber("PITCH_CONTROL_D")
PITCH_AUTO_CONTROL_P = property.getNumber("PITCH_AUTO_CONTROL_P")
PITCH_AUTO_CONTROL_I = property.getNumber("PITCH_AUTO_CONTROL_I")
PITCH_AUTO_CONTROL_D = property.getNumber("PITCH_AUTO_CONTROL_D")

YAW_CONTROL_P = property.getNumber("YAW_CONTROL_P")
YAW_CONTROL_I = property.getNumber("YAW_CONTROL_I")
YAW_CONTROL_D = property.getNumber("YAW_CONTROL_D")
YAW_AUTO_CONTROL_P = property.getNumber("YAW_AUTO_CONTROL_P")
YAW_AUTO_CONTROL_I = property.getNumber("YAW_AUTO_CONTROL_I")
YAW_AUTO_CONTROL_D = property.getNumber("YAW_AUTO_CONTROL_D")

COLLECTIVE_CONTROL_P = property.getNumber("COLLECTIVE_CONTROL_P")
COLLECTIVE_CONTROL_I = property.getNumber("COLLECTIVE_CONTROL_I")
COLLECTIVE_CONTROL_D = property.getNumber("COLLECTIVE_CONTROL_D")
COLLECTIVE_AUTO_CONTROL_P = property.getNumber("COLLECTIVE_AUTO_CONTROL_P")
COLLECTIVE_AUTO_CONTROL_I = property.getNumber("COLLECTIVE_AUTO_CONTROL_I")
COLLECTIVE_AUTO_CONTROL_D = property.getNumber("COLLECTIVE_AUTO_CONTROL_D")

--- PIDコントローラーのインスタンスを生成します
function PID.new(Kp, Ki, Kd)
    return {
        Kp = Kp,
        Ki = Ki,
        Kd = Kd,
        prev_error = 0,
        integral = 0
    }
end

--- PIDの内部状態をリセットします
function PID.reset(self)
    self.prev_error = 0
    self.integral = 0
end

--- PIDの計算を更新し、操作量を出力します
function PID.update(self, setpoint, measurement, dt, outputLimit)
    if outputLimit == nil then
        outputLimit = math.huge
    end

    local error = setpoint - measurement
    self.integral = self.integral + error * dt

    local derivative = (error - self.prev_error) / dt
    local output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    if output > outputLimit then
        self.integral = outputLimit - (self.Kp * error + self.Kd * derivative)
        output = outputLimit
    elseif output < -outputLimit then
        self.integral = -outputLimit - (self.Kp * error + self.Kd * derivative)
        output = -outputLimit
    end
    self.prev_error = error
    return output
end

--[[
================================================================================
  Configuration
  - ここでPIDコントローラーの設定を設定します。
================================================================================
]]

rollControlPID = {
    pid = PID.new(ROLL_CONTROL_P, ROLL_CONTROL_I, ROLL_CONTROL_D),

}

pitchControlPID = {
    pid = PID.new(PITCH_CONTROL_P, PITCH_CONTROL_I, PITCH_CONTROL_D),
    --pid_name = "Altitude_Control", -- (デバッグ用) 名前などを付けても良い
}

yawControlPID = {
    pid = PID.new(YAW_CONTROL_P, YAW_CONTROL_I, YAW_CONTROL_D),
}

rollAutoControlPID = {
    pid = PID.new(ROLL_AUTO_CONTROL_P, ROLL_AUTO_CONTROL_I, ROLL_AUTO_CONTROL_D),
}

pitchAutoControlPID = {
    pid = PID.new(PITCH_AUTO_CONTROL_P, PITCH_AUTO_CONTROL_I, PITCH_AUTO_CONTROL_D),
}

yawAutoControlPID = {
    pid = PID.new(YAW_AUTO_CONTROL_P, YAW_AUTO_CONTROL_I, YAW_AUTO_CONTROL_D),
}

collectiveControlPID = {
    pid = PID.new(COLLECTIVE_CONTROL_P, COLLECTIVE_CONTROL_I, COLLECTIVE_CONTROL_D),
}

collectiveAutoControlPID = {
    pid = PID.new(COLLECTIVE_AUTO_CONTROL_P, COLLECTIVE_AUTO_CONTROL_I, COLLECTIVE_AUTO_CONTROL_D),
}

function onTick()
    -- パイロットシートからの入力
    rollInput = input.getNumber(1)
    pitchInput = input.getNumber(2)
    yaw = input.getNumber(3)

    isSeatControlEnable = input.getNumber(9) ~= 0 and input.getNumber(10) ~= 0

    if isSeatControlEnable then
        throttleInput = input.getNumber(4)
        pitch = math.max(math.min(pitch + pitchInput * PITCH_SEISI, 1), -1)
        roll = math.max(math.min(roll + rollInput * ROLL_SEISI, 1), -1)
    else
        throttleInput = 0
    end

    system = input.getBool(1)
    isHoverMode = input.getBool(3)

    --傾斜計
    pitchTilt = -input.getNumber(11)
    rollTilt = input.getNumber(12)

    -- 角速度
    yawAngularVelocity = input.getNumber(13)
    pitchAngularVelocity = -input.getNumber(14) -- 上方向回転を正としたいのでマイナス
    rollAngularVelocity = -input.getNumber(15)

    -- 速度
    forwardSpeed = input.getNumber(16)
    sideSpeed = input.getNumber(17)
    upSpeed = input.getNumber(18)

    altitudeDelta = input.getNumber(19) -- 高度のデルタ

    corrective = 0.5 + throttleInput
    isRollControl = math.abs(roll) > 0.001
    isPitchControl = math.abs(pitch) > 0.001
    isYawControl = math.abs(yaw) > 0.001
    isCollectiveControl = math.abs(throttleInput) > 0.001

    if system then
        if not isHoverMode then
            isPreviousTickHoverMode = false
            -- ピッチはpitchTiltを基準に制御。(pitchが最大値の1の場合はpitch * PITCH_TILT_LIMITを目標値としてPIDに渡す。計測値はpitchTilt)
            pitch_PID_result = PID.update(pitchControlPID.pid, PITCH_TILT_LIMIT * pitch, pitchTilt, DT)
            PID.reset(pitchAutoControlPID.pid)

            -- ロールはtiltSensor基準で制御
            roll_PID_result = PID.update(rollControlPID.pid, ROLL_TILT_LIMIT * roll, rollTilt, DT)
            PID.reset(rollAutoControlPID.pid)

            -- ヨー回転は常に角速度でPID制御を行う
            yaw_PID_result = PID.update(yawControlPID.pid, MAX_YAW_ANGULAR_VELOCITY * yaw, yawAngularVelocity,
                DT)
            -- collective
            PID.reset(collectiveAutoControlPID.pid)
            if IS_THROTTLE_ANALOG_INPUT then
                if throttleInput == 0 then
                    collective = NEUTRAL_COLLECTIVE
                elseif throttleInput < 0 then
                    collective = MIN_COLLECTIVE
                elseif throttleInput > 0 then
                    collective = MAX_COLLECTIVE
                end
            else
                collective = ((throttleInput + 1) / 2) * (1 - MIN_COLLECTIVE) + MIN_COLLECTIVE
            end
        else
            pitch = 0
            roll = 0

            -- hoverModeの最初の1tickはコントロールモードのPIDを初期化
            if not isPreviousTickHoverMode then
                PID.reset(pitchControlPID.pid)
                PID.reset(rollControlPID.pid)
                isPreviousTickHoverMode = true
            end

            -- pitch
            if pitchInput > 0 then
                hoverForwardSpeed = 2
            elseif pitchInput < 0 then
                hoverForwardSpeed = -2
            else
                hoverForwardSpeed = 0
            end
            hoverPitchAngle = PID.update(pitchAutoControlPID.pid, hoverForwardSpeed, forwardSpeed, DT,
                PITCH_TILT_LIMIT) -- ホバリング時の機体のピッチ角を求める(上向きの時は+、下向きの時を-としたいので符号を反転させている)
            pitch_PID_result = PID.update(pitchControlPID.pid, hoverPitchAngle, pitchTilt, DT)

            -- roll
            if rollInput > 0 then
                hoverSideSpeed = 2
            elseif rollInput < 0 then
                hoverSideSpeed = -2
            else
                hoverSideSpeed = 0
            end
            hoverRollAngle = PID.update(rollAutoControlPID.pid, hoverSideSpeed, sideSpeed, DT,
                ROLL_TILT_LIMIT)
            roll_PID_result = PID.update(rollControlPID.pid, hoverRollAngle, rollTilt, DT)

            -- yaw
            if yaw > 0 then
                hoverYawSpeed = 0.02
            elseif yaw < 0 then
                hoverYawSpeed = -0.02
            else
                hoverYawSpeed = 0
            end
            yaw_PID_result = PID.update(yawControlPID.pid, hoverYawSpeed, yawAngularVelocity, DT
            )

            -- collective(コレクティブが小さすぎると姿勢制御が全く効かなくなるので下限を設ける)
            if throttleInput > 0 then
                hoverAltitudeSetPoint = 0.05
            elseif throttleInput < 0 then
                hoverAltitudeSetPoint = -0.05
            else
                hoverAltitudeSetPoint = 0
            end
            collective = math.max(
                PID.update(collectiveAutoControlPID.pid, hoverAltitudeSetPoint, altitudeDelta, DT) +
                NEUTRAL_COLLECTIVE,
                0.3)
        end

        yaw_PID_result = math.max(-collective, math.min(collective, yaw_PID_result))
        clockwiseRotorCollective = collective + -yaw_PID_result
        antiClockwiseRotorCollective = collective + yaw_PID_result

        clockwiseRoll = roll_PID_result
        antiClockwiseRoll = -roll_PID_result

        output.setNumber(2, pitch_PID_result)
        output.setNumber(3, clockwiseRotorCollective)
        output.setNumber(4, antiClockwiseRotorCollective)
        output.setNumber(5, clockwiseRoll)
        output.setNumber(6, antiClockwiseRoll)
        output.setNumber(7, yaw_PID_result) -- debug
        output.setBool(1, isCollectiveControl)
    end
end
