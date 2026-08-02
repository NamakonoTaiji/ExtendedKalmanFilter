--[[
    弾道計算器スクリプト (高度による重力・空気抵抗変化対応版)
]]

-- Cannon Parameters
Param = {}
--Muzzle velocity, Drag, Lifespan, WindFactor
Param[1] = { 800, 0.025, 120, 0.0225 } -- Machinegun
Param[2] = { 1000, 0.02, 150, 0.25 }   -- LAC
Param[3] = { 1000, 0.01, 300, 0.325 }  -- RAC
Param[4] = { 900, 0.005, 600, 0.5 }    -- HAC
Param[5] = { 800, 0.002, 1500, 1 }     -- Battle Cannon
Param[6] = { 700, 0.001, 2400, 1.8 }   -- Artillery Cannon
Param[7] = { 600, 0.0005, 2400, 3.5 }  -- Bertha Cannon

INDEX = property.getNumber("Ammotype")

MAX_LIFESPAN = property.getNumber("FireRange")
LOGIC_DELAY = property.getNumber("LogicDelay")
YAW_ANGLE_LIMIT = math.rad(property.getNumber("YawAngleLimit"))
PITCH_ANGLE_LIMIT = math.rad(property.getNumber("PitchAngleLimit"))
PITCH_MIN_ANGLE_LIMIT = math.rad(property.getNumber("PitchMinAngleLimit"))
IS_PITCH_ROBOTIC = property.getBool("IsPitchRobotic")

PITCH_CONTROL_P = property.getNumber("PitchControlP")
PITCH_CONTROL_I = property.getNumber("PitchControlI")
PITCH_CONTROL_D = property.getNumber("PitchControlD")
YAW_CONTROL_P = property.getNumber("YawControlP")
YAW_CONTROL_I = property.getNumber("YawControlI")
YAW_CONTROL_D = property.getNumber("YawControlD")
PIVOT_ANGULAR_VELOCITY_CORRECTION = property.getNumber("PivotCorrection")
PITCH_PIVOT_MAX_SPEED = property.getNumber("PitchPivotMaxSpeed")
YAW_PIVOT_MAX_SPEED = property.getNumber("YawPivotMaxSpeed")
SHOOTABLE_ERROR_THRESHOLD = property.getNumber("ShootableErrorThreshold")

OFFSET_PHYSICS_FROM_CANNON_X = property.getNumber("OffsetPhysicsFromCannonX(Right+)")
OFFSET_PHYSICS_FROM_CANNON_Y = property.getNumber("OffsetPhysicsFromCannonY(Up+)")
OFFSET_PHYSICS_FROM_CANNON_Z = property.getNumber("OffsetPhysicsFromCannonZ(Forward+)")

-- Cannon Parameters 付近に追加
IS_HIGH_ARC = property.getBool("IsHighArc") -- 曲射オプション

PI = math.pi
PI2 = PI * 2
DT = 1 / 60
V0 = Param[INDEX][1] / 60          -- (m/tick)
K = -math.log(1 - Param[INDEX][2]) -- ★対数変換を追加
LIFESPAN = Param[INDEX][3]
WIND_FACTOR = Param[INDEX][4]

local envTable = {}
local TABLE_STEP = 500
local MAX_ALTITUDE = 45000 -- 計算上限高度 (m)

-- 砲弾の基準重力 30 m/s^2 を m/tick^2 に変換 (1秒 = 60 tick)
local BASE_GRAVITY_TICK = 30 / 3600
-- 空気密度の分母
local ATMOS_DENOM = 1013

-- 1. 飛行時間を算出する関数 (秒単位)
function getT(v, d, h_d)
    -- v: 初速 (m/s)
    -- d: 抵抗係数
    -- h_d: 水平距離 (m)
    local val = 1 - ((d * h_d) / v)
    if val <= 0 then return 0 end
    return (-math.log(val)) / d
end

-- 2. 指定時間後の弾丸の位置を算出する関数 (秒単位の解析解)
function getP(v, a, d, t)
    -- v: 初速ベクトル {x, y, z} (m/s)
    local out = {}
    for i = 1, 3 do
        -- v[i]*(1-exp(-d*t))/d + (a[i]*(d*t+exp(-d*t)-1))/d^2
        out[i] = v[i] * (1 - math.exp(-d * t)) / d + (a[i] * (d * t + math.exp(-d * t) - 1)) / (d * d)
    end
    return out
end

for h = 0, MAX_ALTITUDE, TABLE_STEP do
    -- 1. 重力加速度計算: 減衰スケールハイトは 60000m
    local g_acc = BASE_GRAVITY_TICK * math.exp(-h / 60000)

    -- 2. 空気密度計算: 難読化コードに基づく定数を使用
    local rho = 0
    if h < 44200 then
        -- (44.20 - h(km)) / 11.89
        local base = math.max(0, (44.20 - h / 1000) / 11.89)
        rho = (base ^ 5.256) / ATMOS_DENOM
    end

    -- テーブルに格納
    envTable[#envTable + 1] = { g = g_acc, r = rho }
end

--------------------------------------------------------------------------------
-- クォータニオン・ベクトル演算関数
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1 = q_a[1], q_a[2], q_a[3], q_a[4]
    local w2, x2, y2, z2 = q_b[1], q_b[2], q_b[3], q_b[4]
    return {
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    local hr, hy, hp = roll * 0.5, yaw * 0.5, pitch * 0.5
    local cr, sr = math.cos(hr), math.sin(hr)
    local cy, sy = math.cos(hy), math.sin(hy)
    local cp, sp = math.cos(hp), math.sin(hp)
    return {
        cr * cy * cp + sr * sy * sp,
        cr * cy * sp - sr * sy * cp,
        cr * sy * cp + sr * cy * sp,
        sr * cy * cp - cr * sy * sp
    }
end

function rotateVectorByQuaternion(v, q)
    local px, py, pz = v[1] or v.x or 0, v[2] or v.y or 0, v[3] or v.z or 0
    local p = { 0, px, py, pz }
    local q_conj = { q[1], -q[2], -q[3], -q[4] }
    local temp = multiplyQuaternions(q, p)
    local p_prime = multiplyQuaternions(temp, q_conj)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

function rotateVectorByInverseQuaternion(v, q)
    local px, py, pz = v[1] or 0, v[2] or 0, v[3] or 0
    local p = { 0, px, py, pz }
    local q_conj = { q[1], -q[2], -q[3], -q[4] }
    local temp = multiplyQuaternions(q_conj, p)
    local p_prime = multiplyQuaternions(temp, q)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

function localToGlobal(localPos, objPos, objQuat)
    local lVec = { localPos.x or localPos[1] or 0, localPos.y or localPos[2] or 0, localPos.z or localPos[3] or 0 }
    local rVec = rotateVectorByQuaternion(lVec, objQuat)
    return { x = rVec[1] + objPos.x, y = rVec[2] + objPos.y, z = rVec[3] + objPos.z }
end

--------------------------------------------------------------------------------
-- 環境計算関数 (高度依存の重力・空気密度)
--------------------------------------------------------------------------------
function getAtmosphere(y)
    -- 高度計算を無効化し、常に地表（高度0）の重力と空気密度を返すようにする
    return envTable[1].g, envTable[1].r
end

--------------------------------------------------------------------------------
-- 数値計算 (RK4) コアロジック
--------------------------------------------------------------------------------

-- 加速度計算関数
-- pos: {x,y,z} (relative to initial), vel: {x,y,z}
-- env: {y0 (absolute start altitude), base_drag, windVec, windFactor}
function getAcceleration(pos, vel, env)
    -- 現在の絶対高度
    local current_alt = env.y0 + pos[2]

    -- 環境パラメータ取得 (重力と、風計算用の空気密度)
    local g_curr, rho_curr = getAtmosphere(current_alt)

    -- ★修正1: 弾丸そのものの空気抵抗は高度で減衰させず、固定値を使用する
    local drag_curr = env.base_drag

    -- ★修正2: 風が弾丸に与える影響力は高度(空気密度)で減衰する
    local wind_force_x = env.windVec.x * env.windFactor * rho_curr
    local wind_force_y = env.windVec.y * env.windFactor * rho_curr
    local wind_force_z = env.windVec.z * env.windFactor * rho_curr

    -- 加速度 = 風の力 - 重力(下向き) - (速度 × 空気抵抗)
    local ax = wind_force_x - drag_curr * vel[1]
    local ay = wind_force_y - g_curr - drag_curr * vel[2]
    local az = wind_force_z - drag_curr * vel[3]

    return { ax, ay, az }
end

function calculateFlightTime(dist, v0, k)
    if k < 1e-6 then
        return dist / v0 -- 抵抗がほぼゼロの場合は真空と同じ計算
    else
        local drag_factor = (dist * k) / v0
        -- 目標が物理的な最大射程（弾が完全に失速する距離）を超えている場合の保護
        if drag_factor >= 0.999 then
            drag_factor = 0.999
        end
        return -math.log(1 - drag_factor) / k
    end
end
--------------------------------------------------------------------------------
-- 弾道シミュレーション (毎ティック演算・交差判定内包版)
--------------------------------------------------------------------------------
function evaluateTrajectory(pitch, yaw, v0_speed, env, Pr, VT, AT, max_ticks)
    local cos_p = math.cos(pitch)
    local pos = {0, 0, 0}
    local vel = {
        math.sin(yaw) * cos_p * v0_speed + env.VG.x,
        math.sin(pitch) * v0_speed + env.VG.y,
        math.cos(yaw) * cos_p * v0_speed + env.VG.z
    }
    
    for t = 1, max_ticks do
        -- 加速度・位置・速度の更新
        local ax = env.wind_force_x - env.base_drag * vel[1]
        local ay = env.wind_force_y - env.g_curr - env.base_drag * vel[2]
        local az = env.wind_force_z - env.base_drag * vel[3]
        
        pos[1] = pos[1] + vel[1]
        pos[2] = pos[2] + vel[2]
        pos[3] = pos[3] + vel[3]
        
        vel[1] = vel[1] + ax
        vel[2] = vel[2] + ay
        vel[3] = vel[3] + az
        
        -- 目標の未来位置
        local tx = Pr.x + VT.x * t + 0.5 * AT.x * t^2
        local ty = Pr.y + VT.y * t + 0.5 * AT.y * t^2
        local tz = Pr.z + VT.z * t + 0.5 * AT.z * t^2
        
        -- 目標の水平方向への単位ベクトルを計算
        local R_fut = math.sqrt(tx^2 + tz^2)
        if R_fut < 0.001 then R_fut = 0.001 end
        local dir_x = tx / R_fut
        local dir_z = tz / R_fut
        
        -- 弾と目標の水平位置の差分
        local dx = pos[1] - tx
        local dz = pos[3] - tz
        
        -- 目標への水平距離を弾が超えた瞬間に高度誤差を返す
        if (dx * dir_x + dz * dir_z > 0) and t > 2 then
            return pos[2] - ty, t
        end
        
        -- 目標の高度を大きく下回り、かつ下降中の場合は「飛距離不足」
        if pos[2] < ty - 50 and vel[2] < 0 then
            return -9999, t
        end
    end
    
    -- 寿命到達までに交差しなかった場合も「飛距離不足」
    return -9999, max_ticks
end

--------------------------------------------------------------------------------
-- 二分法(Bisection Method)を用いた弾道ソルバー
--------------------------------------------------------------------------------
function solveBallisticEuler(Pr, VT, AT, VG, y0, k, v0_speed, v_wind, windC, is_high_arc, lifespan)
    local g_curr, rho_curr = getAtmosphere(y0)
    local env = {
        base_drag = k,
        wind_force_x = v_wind.x * windC * rho_curr,
        wind_force_y = v_wind.y * windC * rho_curr,
        wind_force_z = v_wind.z * windC * rho_curr,
        g_curr = g_curr,
        VG = VG
    }
    
    local R0 = math.sqrt(Pr.x^2 + Pr.z^2)
    if R0 < 0.001 then R0 = 0.001 end
    local current_yaw = math.atan(Pr.x, Pr.z)
    
    local p_min, p_max
    if is_high_arc then
        p_min = 0.70
        p_max = 1.55
    else
        p_min = math.atan(Pr.y, R0) - 0.1
        p_max = 0.785
    end
    
    local max_ticks = math.min(lifespan, 2400)
    
    local final_t = 0
    local success = false
    local p_mid = (p_min + p_max) / 2
    local final_yaw = current_yaw
    
    for i = 1, 8 do
        p_mid = (p_min + p_max) / 2
        local e_mid, t_mid = evaluateTrajectory(p_mid, current_yaw, v0_speed, env, Pr, VT, AT, max_ticks)
        final_t = t_mid
        final_yaw = current_yaw
        
        if math.abs(e_mid) < 0.5 then
            success = true
            break
        end
        
        if is_high_arc then
            if e_mid > 0 then
                p_min = p_mid
            else
                p_max = p_mid
            end
        else
            if e_mid > 0 then
                p_max = p_mid
            else
                p_min = p_mid
            end
        end
        
        -- ★修正箇所: 飛距離不足(-9999)の時は異常な時間で未来位置が計算されるため、Yaw角の更新を行わない
        if e_mid ~= -9999 then
            local tx = Pr.x + VT.x * final_t + 0.5 * AT.x * final_t^2
            local tz = Pr.z + VT.z * final_t + 0.5 * AT.z * final_t^2
            current_yaw = math.atan(tx, tz)
        end
    end
    
    -- ループ内で0.5m以内に収束しなかった場合でも、最終誤差が2m以内なら許可
    if not success then
        local e_final, _ = evaluateTrajectory(p_mid, final_yaw, v0_speed, env, Pr, VT, AT, max_ticks)
        if math.abs(e_final) < 2.0 then success = true end
    end
    
    return p_mid, final_yaw, final_t, success
end
--------------------------------------------------------------------------------
-- 二分法(Bisection Method)を用いた弾道ソルバー
--------------------------------------------------------------------------------
function solveBallisticEuler(Pr, VT, AT, VG, y0, k, v0_speed, v_wind, windC, is_high_arc, lifespan)
    -- 環境パラメータのセットアップ
    local g_curr, rho_curr = getAtmosphere(y0)
    local env = {
        base_drag = k,
        wind_force_x = v_wind.x * windC * rho_curr,
        wind_force_y = v_wind.y * windC * rho_curr,
        wind_force_z = v_wind.z * windC * rho_curr,
        g_curr = g_curr,
        VG = VG
    }
    
    local R0 = math.sqrt(Pr.x^2 + Pr.z^2)
    if R0 < 0.001 then R0 = 0.001 end
    local current_yaw = math.atan(Pr.x, Pr.z)
    
    -- 探索範囲の初期化 (二分法のための上限と下限を設定)
    local p_min, p_max
    if is_high_arc then
        -- 曲射: 約40度(0.7 rad) から 約88度(1.55 rad) の範囲で探索
        p_min = 0.70
        p_max = 1.55
    else
        -- 直射: 目標への直線仰角から 約45度(0.785 rad) の範囲で探索
        p_min = math.atan(Pr.y, R0) - 0.1 -- 撃ち下ろしも考慮して少し下を最小値に
        p_max = 0.785
    end
    
    -- 砲弾の寿命による計算上限を設定 (最大2400 tick)
    local max_ticks = math.min(lifespan, 2400)
    
    local final_t = 0
    local success = false
    local p_mid = (p_min + p_max) / 2
    
    -- 二分法による反復計算 (8回で約0.1度の精度に収束)
    for i = 1, 8 do
        p_mid = (p_min + p_max) / 2
        local e_mid, t_mid = evaluateTrajectory(p_mid, current_yaw, v0_speed, env, Pr, VT, AT, max_ticks)
        final_t = t_mid
        
        -- 垂直誤差が0.5m以内になれば命中と判定してループを抜ける
        if math.abs(e_mid) < 0.5 then
            success = true
            break
        end
        
        -- 誤差の符号に基づいて探索範囲を半分に狭める
        if is_high_arc then
            -- 曲射の特性: 角度を上げる(真上に近づく)ほど弾は手前に落ちる(高度誤差マイナス)
            if e_mid > 0 then
                -- 弾が目標より上を通過した -> 角度を上げて手前に落とす
                p_min = p_mid
            else
                -- 弾が目標より下を通過した -> 角度を下げて奥へ飛ばす
                p_max = p_mid
            end
        else
            -- 直射の特性: 角度を上げる(45度に近づく)ほど弾は奥へ飛ぶ(高度誤差プラス)
            if e_mid > 0 then
                -- 弾が目標より上を通過した -> 角度を下げる
                p_max = p_mid
            else
                -- 弾が目標より下を通過した -> 角度を上げる
                p_min = p_mid
            end
        end
        
        -- 新しい飛行時間から目標の未来位置を再計算し、Yaw角を補正
        local tx = Pr.x + VT.x * final_t + 0.5 * AT.x * final_t^2
        local tz = Pr.z + VT.z * final_t + 0.5 * AT.z * final_t^2
        current_yaw = math.atan(tx, tz)
    end
    
    -- 最終的な誤差が2.0m以内であれば射撃許可を出す
    local e_final, _ = evaluateTrajectory(p_mid, current_yaw, v0_speed, env, Pr, VT, AT, max_ticks)
    if math.abs(e_final) < 2.0 then success = true end
    
    return p_mid, current_yaw, final_t, success
end

--------------------------------------------------------------------------------
-- PID Class
--------------------------------------------------------------------------------
PID = {}
function PID.new(Kp, Ki, Kd)
    return { Kp = Kp, Ki = Ki, Kd = Kd, prev_error = 0, integral = 0 }
end

function PID.reset(self)
    self.prev_error = 0
    self.integral = 0
end

function PID.update(self, setpoint, measurement, dt, outputLimit)
    local isShootable = false
    outputLimit = outputLimit or math.huge
    local error = setpoint - measurement
    if math.abs(error) < SHOOTABLE_ERROR_THRESHOLD then isShootable = true end
    self.integral = self.integral + error * dt
    local i_term = self.Ki * self.integral
    if i_term > outputLimit then
        self.integral = outputLimit / self.Ki
    elseif i_term < -outputLimit then
        self.integral = -outputLimit / self.Ki
    end

    local derivative = (error - self.prev_error) / dt
    local output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    output = math.max(-outputLimit, math.min(outputLimit, output))
    self.prev_error = error
    return output, isShootable
end

pitchControlPID = { pid = PID.new(PITCH_CONTROL_P, PITCH_CONTROL_I, PITCH_CONTROL_D) }
yawControlPID = { pid = PID.new(YAW_CONTROL_P, YAW_CONTROL_I, YAW_CONTROL_D) }

function clamp(value, min, max) return math.max(min, math.min(value, max)) end

local turretYaw, turretPitch = 0, 0
--------------------------------------------------------------------------------
-- Main Loop
--------------------------------------------------------------------------------
function onTick()
    local fuseTime = 0
    local lifespan = LIFESPAN * MAX_LIFESPAN
    local isDetecting = input.getBool(1)


    local yawShootable, pitchShootable = false, false
    local isDetect = false
    local isError = false

    if isDetecting then
        local targetX = input.getNumber(1)
        local targetY = input.getNumber(2)
        local targetZ = input.getNumber(3)
        local targetVectorX = input.getNumber(4)
        local targetVectorY = input.getNumber(5)
        local targetVectorZ = input.getNumber(6)
        local targetAccX = input.getNumber(7)
        local targetAccY = input.getNumber(8)
        local targetAccZ = input.getNumber(9)

        local ownPitch = input.getNumber(13)
        local ownYaw = input.getNumber(14)
        local ownRoll = input.getNumber(15)
        local selfLocalVectorX = input.getNumber(16)
        local selfLocalVectorY = input.getNumber(17)
        local selfLocalVectorZ = input.getNumber(18)
        local local_wind_azi_turn = input.getNumber(22)
        local wind_speed_mps = input.getNumber(23)

        local yawPivot_Vel = input.getNumber(24) * PI2 * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
        local pitchPivot_Vel = input.getNumber(25) * PI2 * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION

        local currentCannonYaw = input.getNumber(19) * PI2
        local currentCannonPitch = (IS_PITCH_ROBOTIC and input.getNumber(21) or input.getNumber(20)) * PI2

        local cosPitch = math.cos(ownPitch)
        if math.abs(cosPitch) < 0.001 then
            isError = true
            PID.reset(pitchControlPID.pid)
            PID.reset(yawControlPID.pid)
        else
            local q_ship = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)
            local v_self_world_array = rotateVectorByQuaternion({ selfLocalVectorX, selfLocalVectorY, selfLocalVectorZ },
                q_ship)

            local ownWorldX = input.getNumber(10) + v_self_world_array[1] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
            local ownWorldY = input.getNumber(11) + v_self_world_array[2] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
            local ownWorldZ = input.getNumber(12) + v_self_world_array[3] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION

            local offsetCoords = { OFFSET_PHYSICS_FROM_CANNON_X, OFFSET_PHYSICS_FROM_CANNON_Y,
                OFFSET_PHYSICS_FROM_CANNON_Z }
            local correctedOwn = localToGlobal(offsetCoords, { x = ownWorldX, y = ownWorldY, z = ownWorldZ }, q_ship)
            ownWorldX, ownWorldY, ownWorldZ = correctedOwn.x, correctedOwn.y, correctedOwn.z

            local c1 = rotateVectorByInverseQuaternion({ 1, 0, 0 }, q_ship)
            local c3 = rotateVectorByInverseQuaternion({ 0, 0, 1 }, q_ship)
            local r11, r13 = c1[1], c3[1]
            local r31, r33 = c1[3], c3[3]
            local local_wind_azi_rad = local_wind_azi_turn * PI2
            local V_sensor_x = -wind_speed_mps * math.sin(local_wind_azi_rad)
            local V_sensor_z = -wind_speed_mps * math.cos(local_wind_azi_rad)

            local Kx = selfLocalVectorX - V_sensor_x
            local Kz = selfLocalVectorZ - V_sensor_z
            local D = r11 * r33 - r13 * r31
            local Wind_X_mps, Wind_Z_mps = 0, 0
            if math.abs(D) > 1e-6 then
                Wind_X_mps = (r33 * Kx - r13 * Kz) / D
                Wind_Z_mps = (-r31 * Kx + r11 * Kz) / D
            end
            local V_wind_vec = { x = Wind_X_mps * DT, y = 0, z = Wind_Z_mps * DT }

            local t_delay = LOGIC_DELAY
            local targetV = { x = targetVectorX * DT, y = targetVectorY * DT, z = targetVectorZ * DT }
            local targetA = { x = targetAccX * DT * DT, y = targetAccY * DT * DT, z = targetAccZ * DT * DT }

            local P_T = {
                x = targetX + targetV.x * t_delay + targetA.x * 0.5 * t_delay ^ 2,
                y = targetY + targetV.y * t_delay + targetA.y * 0.5 * t_delay ^ 2,
                z = targetZ + targetV.z * t_delay + targetA.z * 0.5 * t_delay ^ 2
            }
            local V_T_delayed = {
                x = targetV.x + targetA.x * t_delay,
                y = targetV.y + targetA.y * t_delay,
                z = targetV.z + targetA.z * t_delay
            }

            local P_G = { x = ownWorldX, y = ownWorldY, z = ownWorldZ }
            local V_G = { x = v_self_world_array[1] * DT, y = v_self_world_array[2] * DT, z = v_self_world_array[3] * DT }

            local P_rel = { x = P_T.x - P_G.x, y = P_T.y - P_G.y, z = P_T.z - P_G.z }
            local distance = math.sqrt(P_rel.x ^ 2 + P_rel.y ^ 2 + P_rel.z ^ 2)

            -- ★追加: 目標までの距離が最大射程を超えている場合は計算をスキップ
            local max_effective_range = lifespan * V0
            local isWithinRange = distance < max_effective_range

            -- ★追加: 目標座標が完全に(0,0,0)の場合は未検知(無効データ)として弾く
            local isNotOrigin = not (math.abs(targetX) < 0.01 and math.abs(targetY) < 0.01 and math.abs(targetZ) < 0.01)

            -- 判定を厳格化 (距離が近すぎないか、射程内か、原点ではないか)
            isDetect = (distance > 0.1) and isWithinRange and isNotOrigin

            if isDetect then
                -- ★★★ 高度(P_G.y)を渡すように変更 ★★★
                local global_el, global_az, flight_time, sol_success = solveBallisticEuler(
                    P_rel, V_T_delayed, targetA, V_G,
                    P_G.y, -- 絶対初期高度
                    K, V0, V_wind_vec, WIND_FACTOR
                    ,IS_HIGH_ARC, lifespan
                )

                if flight_time > lifespan then
                    isError = true
                end

                local cosE, sinE = math.cos(global_el), math.sin(global_el)
                local cosA, sinA = math.cos(global_az), math.sin(global_az)
                local v_global_aim = { cosE * sinA, sinE, cosE * cosA }
                local v_local_aim = rotateVectorByInverseQuaternion(v_global_aim, q_ship)

                local local_azimuth = math.atan(v_local_aim[1], v_local_aim[3])
                local local_elevation = math.atan(v_local_aim[2], math.sqrt(v_local_aim[1] ^ 2 + v_local_aim[3] ^ 2))

                local azi_limit = YAW_ANGLE_LIMIT

                if not isError and sol_success then
                    turretYaw, yawShootable = PID.update(yawControlPID.pid,
                        clamp(local_azimuth, -azi_limit, azi_limit),
                        currentCannonYaw, DT, YAW_PIVOT_MAX_SPEED)
                    turretYaw = turretYaw - yawPivot_Vel

                    if IS_PITCH_ROBOTIC then
                        turretPitch = clamp(local_elevation, PITCH_MIN_ANGLE_LIMIT, PITCH_ANGLE_LIMIT) - pitchPivot_Vel
                        pitchShootable = math.abs(local_elevation - currentCannonPitch / PI2 * 4) <
                            SHOOTABLE_ERROR_THRESHOLD
                        turretPitch = turretPitch / PI2 * 4
                    else
                        turretPitch, pitchShootable = PID.update(pitchControlPID.pid,
                            clamp(local_elevation, PITCH_MIN_ANGLE_LIMIT, PITCH_ANGLE_LIMIT),
                            currentCannonPitch, DT, PITCH_PIVOT_MAX_SPEED)
                        turretPitch = turretPitch - pitchPivot_Vel
                    end
                end
            end
        end
    else
        PID.reset(pitchControlPID.pid)
        PID.reset(yawControlPID.pid)
    end

    output.setNumber(1, turretYaw)
    output.setNumber(2, turretPitch)
    output.setNumber(3, fuseTime)
    output.setBool(1, (not isError) and isDetect and yawShootable and pitchShootable)
end
