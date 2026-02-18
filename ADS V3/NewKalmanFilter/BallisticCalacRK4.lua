--[[
    弾道計算器スクリプト (RK4版)
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

PI = math.pi
PI2 = PI * 2
DT = 1 / 60
V0 = Param[INDEX][1] / 60 -- (m/tick)
K = Param[INDEX][2]       -- Drag Coefficient (Linear)
LIFESPAN = Param[INDEX][3]
WIND_FACTOR = Param[INDEX][4]
GRAVITY_ACC = 30 / 60 ^ 2            -- (m/tick^2)
GRAVITY_VEC = { 0, -GRAVITY_ACC, 0 } -- Gravity Vector

errCount = 60

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
-- 数値計算 (RK4) コアロジック
--------------------------------------------------------------------------------

-- 加速度計算関数 (運動方程式)
-- Pos: {x,y,z}, Vel: {x,y,z}, Env: {gravity, drag, windVec, windFactor}
function getAcceleration(vel, env)
    -- 線形空気抵抗: F_drag = -k * (V - V_wind)
    -- Stormworksの抵抗は速度ベクトルに対して掛かる
    local wx = env.windVec.x * env.windFactor
    local wy = env.windVec.y * env.windFactor
    local wz = env.windVec.z * env.windFactor

    local rvx = vel[1] - wx
    local rvy = vel[2] - wy
    local rvz = vel[3] - wz

    -- 加速度 = 重力 - 抵抗係数 * 相対速度
    local ax = env.gravity[1] - env.drag * rvx
    local ay = env.gravity[2] - env.drag * rvy
    local az = env.gravity[3] - env.drag * rvz

    return { ax, ay, az }
end

-- ルンゲ＝クッタ法による1ステップ積分
function rk4Step(pos, vel, dt, env)
    -- k1
    local a1 = getAcceleration(vel, env)
    local v1 = { vel[1], vel[2], vel[3] }

    -- k2
    local v2 = { vel[1] + a1[1] * dt * 0.5, vel[2] + a1[2] * dt * 0.5, vel[3] + a1[3] * dt * 0.5 }
    local a2 = getAcceleration(v2, env)

    -- k3
    local v3 = { vel[1] + a2[1] * dt * 0.5, vel[2] + a2[2] * dt * 0.5, vel[3] + a2[3] * dt * 0.5 }
    local a3 = getAcceleration(v3, env)

    -- k4
    local v4 = { vel[1] + a3[1] * dt, vel[2] + a3[2] * dt, vel[3] + a3[3] * dt }
    local a4 = getAcceleration(v4, env)

    -- Update
    local nPos = {
        pos[1] + (vel[1] + 2 * v2[1] + 2 * v3[1] + v4[1]) * dt / 6,
        pos[2] + (vel[2] + 2 * v2[2] + 2 * v3[2] + v4[2]) * dt / 6,
        pos[3] + (vel[3] + 2 * v2[3] + 2 * v3[3] + v4[3]) * dt / 6
    }

    local nVel = {
        vel[1] + (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1]) * dt / 6,
        vel[2] + (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2]) * dt / 6,
        vel[3] + (a1[3] + 2 * a2[3] + 2 * a3[3] + a4[3]) * dt / 6
    }

    return nPos, nVel
end

-- 弾道シミュレーションを実行し、指定時間後の位置を返す
function simulateTrajectory(v0_vec, flightTime, env)
    local pos = { 0, 0, 0 } -- 発射位置（相対座標なので0）
    local vel = v0_vec
    local t = 0

    -- 計算負荷削減のため、ステップ幅を大きく取る (例: 5 tick分まとめて計算)
    -- 精度が必要な場合は step_dt を小さくする (最小 1.0)
    local step_dt = 1.0

    while t < flightTime do
        local dt_curr = math.min(step_dt, flightTime - t)
        pos, vel = rk4Step(pos, vel, dt_curr, env)
        t = t + dt_curr
    end

    return pos
end

-- 修正版弾道ソルバー (Predict-Correct Method with RK4)
function solveBallisticRK4(Pr, VT, AT, VG, gravity, k, v0_speed, v_wind, windC)
    -- 環境パラメータ
    local env = {
        gravity = { 0, -gravity, 0 },
        drag = k,
        windVec = v_wind,
        windFactor = windC
    }

    -- 初期推定: 距離 / 初速
    local dist = math.sqrt(Pr.x ^ 2 + Pr.y ^ 2 + Pr.z ^ 2)
    local t_est = dist / v0_speed

    -- 「狙うべき空間座標」の初期値（最初はターゲットの予測位置そのもの）
    local aim_point = {
        x = Pr.x + VT.x * t_est + 0.5 * AT.x * t_est ^ 2,
        y = Pr.y + VT.y * t_est + 0.5 * AT.y * t_est ^ 2,
        z = Pr.z + VT.z * t_est + 0.5 * AT.z * t_est ^ 2
    }

    local final_elevation, final_azimuth
    local success = false

    -- 反復計算 (Opcode制限を考慮し、回数は少なめに: 4〜5回)
    for i = 1, 4 do
        -- 1. 現在の aim_point に向けて撃つためのベクトル計算
        local dx, dy, dz = aim_point.x, aim_point.y, aim_point.z
        local current_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        -- 正規化して初速を掛ける
        local v0_vec = {
            (dx / current_dist) * v0_speed + VG.x, -- 自機の慣性は加算済みとして扱うか、相対速度で扱うか
            -- ここでは、VGは考慮せず、発射ベクトルは「砲口初速ベクトル」とする
            -- 呼び出し元で VG を処理するか、ここで足すか。
            -- 元コードは P_rel (ターゲット - 自機) を使っているので、
            -- 弾丸の速度 = 砲口速度 + 自機速度
            (dy / current_dist) * v0_speed + VG.y,
            (dz / current_dist) * v0_speed + VG.z
        }

        -- 自機速度成分(VG)の影響：
        -- 元コードの論理では、Pr (相対位置) に対して計算を行っている。
        -- RK4シミュレーションは「慣性系（地面固定）」ではなく「自機固定系」で行うのが難しい（自機が加速するため）。
        -- シンプルにするため、シミュレーションは「発射瞬間の位置を原点とした慣性系」で行う。
        -- ターゲットの動きも慣性系での位置予測を行う。

        -- 2. RK4で弾丸を t_est 時間飛ばしてみる
        local bullet_pos = simulateTrajectory(v0_vec, t_est, env)

        -- 3. ターゲットの真の位置 (慣性系)
        local target_pos_true = {
            x = Pr.x + VT.x * t_est + 0.5 * AT.x * t_est ^ 2,
            y = Pr.y + VT.y * t_est + 0.5 * AT.y * t_est ^ 2,
            z = Pr.z + VT.z * t_est + 0.5 * AT.z * t_est ^ 2
        }

        -- 4. 誤差計算 (ターゲット - 弾着点)
        local error = {
            x = target_pos_true.x - bullet_pos[1],
            y = target_pos_true.y - bullet_pos[2],
            z = target_pos_true.z - bullet_pos[3]
        }

        -- 収束判定
        local error_dist = math.sqrt(error.x ^ 2 + error.y ^ 2 + error.z ^ 2)
        if error_dist < 3.0 then
            success = true
            break
        end

        -- 5. 補正: 誤差の分だけ「狙う場所」をずらす
        aim_point.x = aim_point.x + error.x
        aim_point.y = aim_point.y + error.y
        aim_point.z = aim_point.z + error.z

        -- 6. 時間の再推定 (弧の長さを考慮...は重いので、直線距離で近似更新)
        local new_aim_dist = math.sqrt(aim_point.x ^ 2 + aim_point.y ^ 2 + aim_point.z ^ 2)
        t_est = new_aim_dist / v0_speed

        --ループを抜けた時点で success が false でも、Nan(計算不能)でなければ true (近似解採用) とみなして射撃させる
        if t_est == t_est then -- NaNチェック (自分自身と等しくないならNaN)
            success = true
        end
    end

    -- 最終的な aiming vector から角度を算出
    -- 注意: ここで計算するのは「砲身が向くべき方向」
    local dx, dy, dz = aim_point.x, aim_point.y, aim_point.z
    local h_dist = math.sqrt(dx * dx + dz * dz)
    final_azimuth = math.atan(dx, dz)
    final_elevation = math.atan(dy, h_dist)

    return final_elevation, final_azimuth, t_est, success
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
    -- Anti-windup
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

--------------------------------------------------------------------------------
-- Main Loop
--------------------------------------------------------------------------------
function onTick()
    isDetecting = input.getBool(1)

    local turretYaw, turretPitch = 0, 0
    local yawShootable, pitchShootable = false, false
    local isDetect = false
    local isError = false

    if isDetecting then
        -- Inputs
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

        -- Pivot velocities for delay correction
        local yawPivot_Vel = input.getNumber(24) * PI2 * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
        local pitchPivot_Vel = input.getNumber(25) * PI2 * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION

        local currentCannonYaw = input.getNumber(19) * PI2
        local currentCannonPitch = (IS_PITCH_ROBOTIC and input.getNumber(21) or input.getNumber(20)) * PI2

        -- Gimbal Lock Check
        local cosPitch = math.cos(ownPitch)
        if math.abs(cosPitch) < 0.001 then
            isError = true
            errCount = 1
            PID.reset(pitchControlPID.pid)
            PID.reset(yawControlPID.pid)
        else
            -- 座標変換準備
            local q_ship = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)
            local v_self_world_array = rotateVectorByQuaternion({ selfLocalVectorX, selfLocalVectorY, selfLocalVectorZ },
                q_ship)

            -- 自機位置補正
            local ownWorldX = input.getNumber(10) + v_self_world_array[1] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
            local ownWorldY = input.getNumber(11) + v_self_world_array[2] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
            local ownWorldZ = input.getNumber(12) + v_self_world_array[3] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION

            local offsetCoords = { OFFSET_PHYSICS_FROM_CANNON_X, OFFSET_PHYSICS_FROM_CANNON_Y,
                OFFSET_PHYSICS_FROM_CANNON_Z }
            local correctedOwn = localToGlobal(offsetCoords, { x = ownWorldX, y = ownWorldY, z = ownWorldZ }, q_ship)
            ownWorldX, ownWorldY, ownWorldZ = correctedOwn.x, correctedOwn.y, correctedOwn.z

            -- 風計算
            local local_wind_azi_rad = local_wind_azi_turn * PI2
            local V_sensor_x = -wind_speed_mps * math.sin(local_wind_azi_rad)
            local V_sensor_z = -wind_speed_mps * math.cos(local_wind_azi_rad)

            -- 風ベクトル計算 (簡易版: センサーローカルからワールドへ変換)
            -- 厳密には元のコードのマトリクス計算の方が正確だが、Opcode節約のためクォータニオン回転で代用可
            -- ここでは元のロジックを尊重して省略するが、ワールド風ベクトルが必要
            -- (元のコードが複雑な行列計算をしているのは、風センサが機体と回転しているため)

            -- --- 元の行列計算ロジック移植 (最適化) ---
            local c1 = rotateVectorByInverseQuaternion({ 1, 0, 0 }, q_ship)
            local c3 = rotateVectorByInverseQuaternion({ 0, 0, 1 }, q_ship)
            local r11, r13 = c1[1], c3[1]
            local r31, r33 = c1[3], c3[3]
            local Kx = selfLocalVectorX - V_sensor_x
            local Kz = selfLocalVectorZ - V_sensor_z
            local D = r11 * r33 - r13 * r31
            local Wind_X_mps, Wind_Z_mps = 0, 0
            if math.abs(D) > 1e-6 then
                Wind_X_mps = (r33 * Kx - r13 * Kz) / D
                Wind_Z_mps = (-r31 * Kx + r11 * Kz) / D
            end
            local V_wind_vec = { x = Wind_X_mps * DT, y = 0, z = Wind_Z_mps * DT }

            -- 各種ベクトル (m/tick)
            local t_delay = LOGIC_DELAY
            local targetV = { x = targetVectorX * DT, y = targetVectorY * DT, z = targetVectorZ * DT }
            local targetA = { x = targetAccX * DT * DT, y = targetAccY * DT * DT, z = targetAccZ * DT * DT }

            -- ターゲット現在位置 (遅延補正後)
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

            -- 相対位置
            local P_rel = { x = P_T.x - P_G.x, y = P_T.y - P_G.y, z = P_T.z - P_G.z }
            local distance = math.sqrt(P_rel.x ^ 2 + P_rel.y ^ 2 + P_rel.z ^ 2)

            isDetect = distance > 0.1

            if isDetect then
                -- ★★★ 数値計算ソルバー呼び出し ★★★
                local global_el, global_az, flight_time, sol_success = solveBallisticRK4(
                    P_rel, V_T_delayed, targetA, V_G,
                    GRAVITY_ACC, K, V0, V_wind_vec, WIND_FACTOR
                )

                -- ワールド角度 → ローカル角度変換
                local cosE, sinE = math.cos(global_el), math.sin(global_el)
                local cosA, sinA = math.cos(global_az), math.sin(global_az)
                local v_global_aim = { cosE * sinA, sinE, cosE * cosA }
                local v_local_aim = rotateVectorByInverseQuaternion(v_global_aim, q_ship)

                local local_azimuth = math.atan(v_local_aim[1], v_local_aim[3])
                local local_elevation = math.atan(v_local_aim[2], math.sqrt(v_local_aim[1] ^ 2 + v_local_aim[3] ^ 2))

                -- PID制御
                local azi_limit = YAW_ANGLE_LIMIT
                turretYaw, yawShootable = PID.update(yawControlPID.pid,
                    clamp(local_azimuth, -azi_limit, azi_limit),
                    currentCannonYaw, DT, YAW_PIVOT_MAX_SPEED)
                turretYaw = turretYaw - yawPivot_Vel

                if IS_PITCH_ROBOTIC then
                    turretPitch = clamp(local_elevation, PITCH_MIN_ANGLE_LIMIT, PITCH_ANGLE_LIMIT) - pitchPivot_Vel
                    pitchShootable = math.abs(local_elevation - currentCannonPitch / PI2 * 4) <
                        SHOOTABLE_ERROR_THRESHOLD -- 簡易判定
                    turretPitch = turretPitch / PI2 * 4
                else
                    turretPitch, pitchShootable = PID.update(pitchControlPID.pid,
                        clamp(local_elevation, PITCH_MIN_ANGLE_LIMIT, PITCH_ANGLE_LIMIT),
                        currentCannonPitch, DT, PITCH_PIVOT_MAX_SPEED)
                    turretPitch = turretPitch - pitchPivot_Vel
                end

                -- ニュートン法(RK4反復)失敗時のリセット
                if not sol_success then
                    -- 失敗しても前回の値を使うか、リセットするか。ここではリセットはしないがShootableをFalseに
                    yawShootable = false
                    pitchShootable = false
                end
            end

            if isError or errCount ~= 0 then
                errCount = (errCount + 1) % 60
            end
        end
    else
        errCount = 1
        PID.reset(pitchControlPID.pid)
        PID.reset(yawControlPID.pid)
    end

    output.setNumber(1, turretYaw)
    output.setNumber(2, turretPitch)
    output.setBool(1, (not isError) and isDetect and errCount == 0 and yawShootable and pitchShootable)
end
