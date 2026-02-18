RPS_set = property.getNumber("RPS_set")
TRANSPORT_RPS = property.getNumber("TRANSPORT_RPS")
P = property.getNumber("RPS_P")
I = property.getNumber("RPS_I")
D = property.getNumber("RPS_D")

outputPID_Channel = 19 -- PID制御出力の出力チャンネル - 1

PID = {}
function PID.new(Kp, Ki, Kd)
    return {
        Kp = Kp,
        Ki = Ki,
        Kd = Kd,
        prev_error = 0,
        integral = 0
    }
end

pid1 = PID.new(P, I, D)

function PID.update(self, setPoint, measurement, dt)
    local error = setPoint - measurement
    self.integral = self.integral + error * dt
    local derivative = (error - self.prev_error) / dt
    local output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    self.prev_error = error
    return output
end

function controlMultipleValues(PIDs, setPoints, measurements, dt)
    local outputs = {}
    for i = 1, #PIDs do
        local output = PID.update(PIDs[i], setPoints[i], measurements[i], dt)
        table.insert(outputs, output)
    end
    return outputs
end

function PID.reset(self)
    self.prev_error = 0
    self.integral = 0
end

function onTick()
    currentRPS = input.getNumber(1)
    isEngineStart = input.getBool(1)
    isIncreaseRPS = input.getBool(2)

    output.setNumber(outputPID_Channel + 1, 0)
    if isEngineStart then
        if currentRPS < 1 then
            motor = 1
        else
            motor = 0
        end
        local PIDs = { pid1 }
        local setPoints
        if isIncreaseRPS then
            setPoints = { TRANSPORT_RPS }
        else
            setPoints = { RPS_set }
        end

        local measurements = { currentRPS }
        local dt = 1 / 60
        local outputs = controlMultipleValues(PIDs, setPoints, measurements, dt)

        for i, pid_output in ipairs(outputs) do
            output.setNumber(i + outputPID_Channel, pid_output)
        end
    else
        PID.reset(pid1)
        motor = 0
    end
    engineStart = motor ~= 0
    output.setNumber(1, motor)
    output.setBool(1, engineStart)
end
