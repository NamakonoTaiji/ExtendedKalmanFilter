--[[
================================================================================
  PID Controller Library
  - Stormworks環境で確実に動作する、シンプルなテーブルベースの構造。
  - PID.update(pid_instance, ...) のように、操作対象のインスタンスを
    第1引数として渡して使用します。
================================================================================
]]
PID = {}

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
function PID.update(self, setpoint, measurement, dt)
    local error = setpoint - measurement
    self.integral = self.integral + error * dt

    -- 積分クランプ (積分値が発散するのを防ぐ)
    local limit = 1
    if self.integral > limit then
        self.integral = limit
    elseif self.integral < -limit then
        self.integral = -limit
    end

    local derivative = (error - self.prev_error) / dt
    local output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    self.prev_error = error
    return output
end

--[[
================================================================================
  Configuration
  - ここでPIDコントローラーの設定をすべて管理します。
================================================================================
]]
CONTROLLERS = {
    {
        pid = PID.new(0.1, 0.01, 0.05), -- PIDインスタンス
        measurement_ch = 2,             -- 計測値の入力チャンネル
        setpoint_ch = 1,                -- 目標値の入力チャンネル
        output_ch = 1,                  -- 制御量の出力チャンネル
        enabled_ch = 10                 -- このPIDを有効化するON/OFF入力チャンネル
    },
    {
        pid = PID.new(0.5, 0, 0.1),
        pid_name = "Altitude_Control", -- (デバッグ用) 名前などを付けても良い
        measurement_ch = 3,
        setpoint_ch = 1,
        output_ch = 2,
        enabled_ch = 11
    }
}


--[[
================================================================================
  Main Loop
================================================================================
]]
function onTick()
    local dt = 1 / 60

    -- 設定されたすべてのコントローラーをループ処理
    for i, controller in ipairs(CONTROLLERS) do
        local is_enabled = input.getBool(controller.enabled_ch)

        if is_enabled then
            local setpoint = input.getNumber(controller.setpoint_ch)
            local measurement = input.getNumber(controller.measurement_ch)

            -- PIDを更新して出力を計算 (selfとしてインスタンスを渡す)
            local output_value = PID.update(controller.pid, setpoint, measurement, dt)

            output.setNumber(controller.output_ch, output_value)
        else
            -- 無効化されている場合は、PIDの状態をリセットし、出力を0にする
            PID.reset(controller.pid)
            output.setNumber(controller.output_ch, 0)
        end
    end
end
