--[[
    弾道計算器スクリプト
]]

-- Cannon Parameters (変更なし)
Param = {}
--Cannon Parameters(Muzzle velocity, Drag, Lifespan, WindFactor)
Param = {}
--Machinegun
Param[1] = { 800, 0.025, 120, 0.0225 }
--LAC
Param[2] = { 1000, 0.02, 150, 0.25 }
--RAC
Param[3] = { 1000, 0.01, 300, 0.325 }
--HAC
Param[4] = { 900, 0.005, 600, 0.5 }
--Battle Cannon
Param[5] = { 800, 0.002, 1500, 1 }
--Artillery Cannon
Param[6] = { 700, 0.001, 2400, 1.8 }
--Bertha Cannon
Param[7] = { 600, 0.0005, 2400, 3.5 }

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
EPSILON = 10e-3
V0 = Param[INDEX][1] / 60 -- (m/tick)
K = Param[INDEX][2]
LIFESPAN = Param[INDEX][3]
WIND_FACTOR = Param[INDEX][4]
GRAVITY_ACC = 30 / 60 ^ 2 -- (m/tick^2)
EXP = math.exp(1)
time = 0
errCount = 60

--------------------------------------------------------------------------------
-- クォータニオン演算関数 (変更なし)
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2, w_result, x_result, y_result, z_result
    w1 = q_a[1]
    x1 = q_a[2]
    y1 = q_a[3]
    z1 = q_a[4]
    w2 = q_b[1]
    x2 = q_b[2]
    y2 = q_b[3]
    z2 = q_b[4]
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z
    half_roll = roll * 0.5
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    w = cr * cy * cp + sr * sy * sp
    x = cr * cy * sp - sr * sy * cp
    y = cr * sy * cp + sr * cy * sp
    z = sr * cy * cp - cr * sy * sp
    return { w, x, y, z }
end

function rotateVectorByQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp = multiplyQuaternions(q, p)
    p_prime = multiplyQuaternions(temp, q_conj)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

function rotateVectorByInverseQuaternion(vector, quaternion)
    local px, py, pz, p, q, q_conj, temp, p_prime
    px = vector[1] or 0
    py = vector[2] or 0
    pz = vector[3] or 0
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    temp = multiplyQuaternions(q_conj, p)
    p_prime = multiplyQuaternions(temp, q)
    return { p_prime[2], p_prime[3], p_prime[4] }
end

--- 乗り物などのローカル座標を、ワールド座標系のグローバル座標に変換します。
---@description 基準となるオブジェクトのグローバル位置と姿勢（クォータニオン）を用いて、
--              オブジェクト上の相対的な位置（ローカル座標）を絶対的な位置（グローバル座標）に変換します。
---@param localPosition Vector3 {x: number, y: number, z: number} 変換したいオブジェクト上のローカル座標。
---@param objectGlobalPos Vector3 {x: number, y: number, z: number} 基準オブジェクト自体のグローバル座標。
---@param objectOrientationQuat Quaternion {w: number, x: number, y: number, z: number} 基準オブジェクトの姿勢を表すクォータニオン。
---@return Vector3 {x: number, y: number, z: number} 変換後のグローバル座標。
function localToGlobal(localPosition, objectGlobalPos, objectOrientationQuat)
    -- 1. 入力テーブルからローカル座標の各成分を取得
    --    {x,y,z} 形式と {1,2,3} 形式の両方に対応します。
    local lx = localPosition.x or localPosition[1] or 0
    local ly = localPosition.y or localPosition[2] or 0
    local lz = localPosition.z or localPosition[3] or 0
    local localVec = { lx, ly, lz }

    -- 2. クォータニオンでローカル座標ベクトルを回転させ、グローバル座標系での相対ベクトルを計算
    local relativeGlobalVec = rotateVectorByQuaternion(localVec, objectOrientationQuat)

    -- 3. オブジェクトのグローバル座標に相対ベクトルを加算し、最終的なグローバル座標を算出
    local gx = relativeGlobalVec[1] + objectGlobalPos.x
    local gy = relativeGlobalVec[2] + objectGlobalPos.y
    local gz = relativeGlobalVec[3] + objectGlobalPos.z

    return { x = gx, y = gy, z = gz }
end

-- f 関数
function f(t, Prx, Pry, Prz, VTx, VTy, VTz, ATx, ATy, ATz, VGx, VGy, VGz, g, k, V0, Vw, C)
    local E = EXP ^ (-k * t)
    local t2_half = 0.5 * t * t
    local one_minus_E = 1 - E
    local one_minus_E_over_k = one_minus_E / k

    -- P_T_world(t) relative to P_G(0)
    local P_T_rel_G0_x = Prx + VTx * t + ATx * t2_half
    local P_T_rel_G0_y = Pry + VTy * t + ATy * t2_half
    local P_T_rel_G0_z = Prz + VTz * t + ATz * t2_half

    -- v_inf = C*Vw - g/k
    local v_inf_x = C * Vw.x
    local v_inf_y = -g / k + C * Vw.y -- Vw.y=0
    local v_inf_z = C * Vw.z

    -- 弾丸のv(0)のうち、V_aim (V0*u) 以外の成分 (V_G(0)) が作る軌道
    -- projectile_non_aim_part = (V_G(0) - v_inf) * ( (1-E)/k ) + v_inf * t
    local VG_minus_v_inf_x = VGx - v_inf_x
    local VG_minus_v_inf_y = VGy - v_inf_y
    local VG_minus_v_inf_z = VGz - v_inf_z

    local term_vG_part_x = VG_minus_v_inf_x * one_minus_E_over_k
    local term_vG_part_y = VG_minus_v_inf_y * one_minus_E_over_k
    local term_vG_part_z = VG_minus_v_inf_z * one_minus_E_over_k

    local term_v_inf_t_part_x = v_inf_x * t
    local term_v_inf_t_part_y = v_inf_y * t
    local term_v_inf_t_part_z = v_inf_z * t

    local projectile_non_aim_part_x = term_vG_part_x + term_v_inf_t_part_x
    local projectile_non_aim_part_y = term_vG_part_y + term_v_inf_t_part_y
    local projectile_non_aim_part_z = term_vG_part_z + term_v_inf_t_part_z

    -- LHS = P_T_rel_G0(t) - projectile_non_aim_part
    -- (これが V_aim * (1-E)/k に等しくなるべき)
    local LHS_x = P_T_rel_G0_x - projectile_non_aim_part_x
    local LHS_y = P_T_rel_G0_y - projectile_non_aim_part_y
    local LHS_z = P_T_rel_G0_z - projectile_non_aim_part_z

    -- ||LHS||^2
    local LHS_sq = LHS_x ^ 2 + LHS_y ^ 2 + LHS_z ^ 2

    -- (V0/k * (1-E))^2
    local V_aim_sq = (V0 / k * one_minus_E) ^ 2

    return LHS_sq - V_aim_sq
end

-- dfdx 関数
function dfdx(t, Prx, Pry, Prz, VTx, VTy, VTz, ATx, ATy, ATz, VGx, VGy, VGz, g, k, V0, Vw, C)
    local E = EXP ^ (-k * t)
    local kE = k * E
    local t2_half = 0.5 * t * t
    local one_minus_E = 1 - E
    local one_minus_E_over_k = one_minus_E / k

    -- v_inf = C*Vw - g/k
    local v_inf_x = C * Vw.x
    local v_inf_y = -g / k + C * Vw.y
    local v_inf_z = C * Vw.z

    -- LHS(t) の再計算
    local P_T_rel_G0_x = Prx + VTx * t + ATx * t2_half
    local P_T_rel_G0_y = Pry + VTy * t + ATy * t2_half
    local P_T_rel_G0_z = Prz + VTz * t + ATz * t2_half

    local VG_minus_v_inf_x = VGx - v_inf_x
    local VG_minus_v_inf_y = VGy - v_inf_y
    local VG_minus_v_inf_z = VGz - v_inf_z

    local term_vG_part_x = VG_minus_v_inf_x * one_minus_E_over_k
    local term_vG_part_y = VG_minus_v_inf_y * one_minus_E_over_k
    local term_vG_part_z = VG_minus_v_inf_z * one_minus_E_over_k

    local term_v_inf_t_part_x = v_inf_x * t
    local term_v_inf_t_part_y = v_inf_y * t
    local term_v_inf_t_part_z = v_inf_z * t

    local projectile_non_aim_part_x = term_vG_part_x + term_v_inf_t_part_x
    local projectile_non_aim_part_y = term_vG_part_y + term_v_inf_t_part_y
    local projectile_non_aim_part_z = term_vG_part_z + term_v_inf_t_part_z

    local LHS_x = P_T_rel_G0_x - projectile_non_aim_part_x
    local LHS_y = P_T_rel_G0_y - projectile_non_aim_part_y
    local LHS_z = P_T_rel_G0_z - projectile_non_aim_part_z

    -- d(LHS)/dt の計算
    -- d(P_T_rel_G0)/dt
    local d_PT_dt_x = VTx + ATx * t
    local d_PT_dt_y = VTy + ATy * t
    local d_PT_dt_z = VTz + ATz * t

    -- d(projectile_non_aim_part)/dt = v_non_aim(t) = (V_G(0) - v_inf) * E + v_inf
    local d_proj_non_aim_dt_x = (VGx - v_inf_x) * E + v_inf_x
    local d_proj_non_aim_dt_y = (VGy - v_inf_y) * E + v_inf_y
    local d_proj_non_aim_dt_z = (VGz - v_inf_z) * E + v_inf_z

    local d_LHS_dt_x = d_PT_dt_x - d_proj_non_aim_dt_x
    local d_LHS_dt_y = d_PT_dt_y - d_proj_non_aim_dt_y
    local d_LHS_dt_z = d_PT_dt_z - d_proj_non_aim_dt_z

    -- d(V_aim^2)/dt の計算
    local d_V_aim_sq_dt = 2 * (V0 / k * one_minus_E) * (V0 / k * kE)

    -- f'(t)
    return 2 * LHS_x * d_LHS_dt_x + 2 * LHS_y * d_LHS_dt_y + 2 * LHS_z * d_LHS_dt_z - d_V_aim_sq_dt
end

-- q 関数
function q(t, Prx, Pry, Prz, VTx, VTy, VTz, ATx, ATy, ATz, VGx, VGy, VGz, g, k, V0, Vw, C)
    local theta, phi
    local E = EXP ^ (-k * t)
    local t2_half = 0.5 * t * t
    local one_minus_E = 1 - E
    local one_minus_E_over_k = one_minus_E / k

    -- v_inf = C*Vw - g/k
    local v_inf_x = C * Vw.x
    local v_inf_y = -g / k + C * Vw.y
    local v_inf_z = C * Vw.z

    -- LHS(t) を f 関数と同様に再計算 (これが射撃方向ベクトル u に比例)
    local P_T_rel_G0_x = Prx + VTx * t + ATx * t2_half
    local P_T_rel_G0_y = Pry + VTy * t + ATy * t2_half
    local P_T_rel_G0_z = Prz + VTz * t + ATz * t2_half

    local VG_minus_v_inf_x = VGx - v_inf_x
    local VG_minus_v_inf_y = VGy - v_inf_y
    local VG_minus_v_inf_z = VGz - v_inf_z

    local term_vG_part_x = VG_minus_v_inf_x * one_minus_E_over_k
    local term_vG_part_y = VG_minus_v_inf_y * one_minus_E_over_k
    local term_vG_part_z = VG_minus_v_inf_z * one_minus_E_over_k

    local term_v_inf_t_part_x = v_inf_x * t
    local term_v_inf_t_part_y = v_inf_y * t
    local term_v_inf_t_part_z = v_inf_z * t

    local projectile_non_aim_part_x = term_vG_part_x + term_v_inf_t_part_x
    local projectile_non_aim_part_y = term_vG_part_y + term_v_inf_t_part_y
    local projectile_non_aim_part_z = term_vG_part_z + term_v_inf_t_part_z

    local aim_vec_x = P_T_rel_G0_x - projectile_non_aim_part_x
    local aim_vec_y = P_T_rel_G0_y - projectile_non_aim_part_y
    local aim_vec_z = P_T_rel_G0_z - projectile_non_aim_part_z

    phi = math.atan(aim_vec_x, aim_vec_z)
    local horizontal_dist = math.sqrt(aim_vec_x ^ 2 + aim_vec_z ^ 2)
    if horizontal_dist < 1e-9 then
        theta = (aim_vec_y > 0) and (PI * 0.5) or (-PI * 0.5)
    else
        theta = math.atan(aim_vec_y, horizontal_dist)
    end
    return theta, phi
end

-- NewtonsMethod
function NewtonsMethod(t0, Prx, Pry, Prz, VTx, VTy, VTz, ATx, ATy, ATz, VGx, VGy, VGz, g, k, V0, Vw, C)
    local t = t0
    for i = 1, 20 do
        local f_val = f(t, Prx, Pry, Prz, VTx, VTy, VTz, ATx, ATy, ATz, VGx, VGy, VGz, g, k, V0, Vw, C)
        local df_val = dfdx(t, Prx, Pry, Prz, VTx, VTy, VTz, ATx, ATy, ATz, VGx, VGy, VGz, g, k, V0, Vw, C)

        -- (以降、NewtonsMethod の中身は変更なし)
        if df_val == nil or math.abs(df_val) < 1e-9 then return 99999, true end
        local t_new = t - f_val / df_val
        if t_new < 0 then return 99999, true end
        if math.abs(f_val) < EPSILON then return t, false end
        if math.abs(t_new - t) < 1e-6 then return t_new, false end
        t = t_new
    end
    return 99999, true
end

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

--- PIDの内部状態をリセットします
function PID.reset(self)
    self.prev_error = 0
    self.integral = 0
end

--- PIDの計算を更新し、操作量を出力します
function PID.update(self, setpoint, measurement, dt, outputLimit)
    local isShootable = false
    if outputLimit == nil then
        outputLimit = math.huge
    end

    local error = setpoint - measurement

    if math.abs(error) < SHOOTABLE_ERROR_THRESHOLD then
        isShootable = true
    end

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
    return output, isShootable
end

pitchControlPID = {
    pid = PID.new(PITCH_CONTROL_P, PITCH_CONTROL_I, PITCH_CONTROL_D),
    --pid_name = "Altitude_Control", -- (デバッグ用) 名前などを付けても良い
}

yawControlPID = {
    pid = PID.new(YAW_CONTROL_P, YAW_CONTROL_I, YAW_CONTROL_D),
}

function clamp(value, min, max)
    return math.max(min, math.min(value, max))
end

function onTick()
    lifespan = LIFESPAN * MAX_LIFESPAN
    isError = true
    isDetecting = input.getBool(1)

    -- PIDの出力と状態をonTickの冒頭で初期化
    local turretYaw, turretPitch = 0, 0
    local yawShootable, pitchShootable = false, false
    local isDetect = false -- isDetecting(入力)とは別に、計算が成功したかを判定するフラグ

    if isDetecting then
        -- (1. 2. 2b. 3. の入力取得は変更なし)
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

        -- ロジック遅延分の補正をするためにピボットの角速度を計算
        local yawPivot_YawAngularVelocity = input.getNumber(24) * PI2 * DT *
            PIVOT_ANGULAR_VELOCITY_CORRECTION -- right turn +

        local pitchPivot_PitchAngularVelocity = input.getNumber(25) * PI2 * DT *
            PIVOT_ANGULAR_VELOCITY_CORRECTION -- up +

        local currentCannonYaw = input.getNumber(19) * PI2
        local currentCannonPitch
        if IS_PITCH_ROBOTIC then
            currentCannonPitch = input.getNumber(21) * PI2
        else
            currentCannonPitch = input.getNumber(20) * PI2
        end


        -- ★ 修正1: ジンバルロック検出
        local cosPitch = math.cos(ownPitch)
        if math.abs(cosPitch) < 0.001 then
            -- ジンバルロック中 (+/- 90度に近い)。
            -- このTickの計算はすべて信頼できないため、強制的にエラーとし、PIDをリセット
            isError = true
            errCount = 1 -- エラーカウントを開始
            PID.reset(pitchControlPID.pid)
            PID.reset(yawControlPID.pid)
        else
            -- ジンバルロックではない場合のみ、通常の計算を実行
            isError = false -- 計算を開始するので一旦falseに

            local q_ship = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)
            -- V_G (自機ワールド速度) を計算
            local v_self_world_array = rotateVectorByQuaternion(
                { selfLocalVectorX, selfLocalVectorY, selfLocalVectorZ },
                q_ship
            )
            -- ロジック遅延分の自機の位置を補正
            local ownWorldX = input.getNumber(10) + v_self_world_array[1] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
            local ownWorldY = input.getNumber(11) + v_self_world_array[2] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION
            local ownWorldZ = input.getNumber(12) + v_self_world_array[3] * DT * PIVOT_ANGULAR_VELOCITY_CORRECTION

            -- -- 物理センサと砲の位置のズレを補正
            local offsetCoords = { OFFSET_PHYSICS_FROM_CANNON_X, OFFSET_PHYSICS_FROM_CANNON_Y,
                OFFSET_PHYSICS_FROM_CANNON_Z } -- {right,up,forward}
            local correctedOwnWorldCoords = localToGlobal(offsetCoords, { x = ownWorldX, y = ownWorldY, z = ownWorldZ },
                q_ship)

            ownWorldX = correctedOwnWorldCoords.x
            ownWorldY = correctedOwnWorldCoords.y
            ownWorldZ = correctedOwnWorldCoords.z

            -- ★ 3b. 風影響度
            local windFactorC = WIND_FACTOR

            -- (A) 見かけ風
            local local_wind_azi_rad = local_wind_azi_turn * PI2
            local V_sensor_x = -wind_speed_mps * math.sin(local_wind_azi_rad)
            local V_sensor_z = -wind_speed_mps * math.cos(local_wind_azi_rad)

            -- (C) 回転行列
            local c1 = rotateVectorByInverseQuaternion({ 1, 0, 0 }, q_ship)
            local c3 = rotateVectorByInverseQuaternion({ 0, 0, 1 }, q_ship)
            local r11, r13 = c1[1], c3[1]
            local r31, r33 = c1[3], c3[3]

            -- (D) 風計算
            local Kx = selfLocalVectorX - V_sensor_x
            local Kz = selfLocalVectorZ - V_sensor_z
            local D = r11 * r33 - r13 * r31
            local Wind_X_mps = 0
            local Wind_Z_mps = 0
            if math.abs(D) > 1e-6 then
                Wind_X_mps = (r33 * Kx - r13 * Kz) / D
                Wind_Z_mps = (-r31 * Kx + r11 * Kz) / D
            end

            -- ★ 風ベクトル
            local V_wind_vec = {
                x = Wind_X_mps * DT,
                y = 0,
                z = Wind_Z_mps * DT
            }

            -- 4. m/tick 変換
            local targetVectorXdt = targetVectorX * DT
            local targetVectorYdt = targetVectorY * DT
            local targetVectorZdt = targetVectorZ * DT
            local targetAccXdt = targetAccX * DT * DT
            local targetAccYdt = targetAccY * DT * DT
            local targetAccZdt = targetAccZ * DT * DT

            -- 4. LogicDelay
            local t_delay = LOGIC_DELAY
            local t_delay2_half = 0.5 * t_delay * t_delay

            -- P_G
            local P_G_x = ownWorldX
            local P_G_y = ownWorldY
            local P_G_z = ownWorldZ

            local V_G_x = v_self_world_array[1] * DT
            local V_G_y = v_self_world_array[2] * DT
            local V_G_z = v_self_world_array[3] * DT

            -- (P_T, V_T, A_T)
            local P_T_x = targetX + targetVectorXdt * t_delay + targetAccXdt * t_delay2_half
            local P_T_y = targetY + targetVectorYdt * t_delay + targetAccYdt * t_delay2_half
            local P_T_z = targetZ + targetVectorZdt * t_delay + targetAccZdt * t_delay2_half
            local V_T_x = targetVectorXdt + targetAccXdt * t_delay
            local V_T_y = targetVectorYdt + targetAccYdt * t_delay
            local V_T_z = targetVectorZdt + targetAccZdt * t_delay
            local A_T_x = targetAccXdt
            local A_T_y = targetAccYdt
            local A_T_z = targetAccZdt

            -- 6. 相対状態
            local P_rel_x = P_T_x - P_G_x
            local P_rel_y = P_T_y - P_G_y
            local P_rel_z = P_T_z - P_G_z

            -- 6. 初期値
            local distance = math.sqrt(P_rel_x ^ 2 + P_rel_y ^ 2 + P_rel_z ^ 2)
            isDetect = distance > 0.1 -- 計算が成功したか
            local t0 = distance / V0


            -- 7. ニュートン法
            time, isError = NewtonsMethod(t0,
                P_rel_x, P_rel_y, P_rel_z, -- Pr
                V_T_x, V_T_y, V_T_z,       -- VT
                A_T_x, A_T_y, A_T_z,       -- AT
                V_G_x, V_G_y, V_G_z,       -- VG
                GRAVITY_ACC, K, V0, V_wind_vec, windFactorC)

            -- ★ 修正2: PID積分暴走対策
            if not isError then
                -- 8. 角度計算
                local global_elevation, global_azimuth = q(time,
                    P_rel_x, P_rel_y, P_rel_z, -- Pr
                    V_T_x, V_T_y, V_T_z,       -- VT
                    A_T_x, A_T_y, A_T_z,       -- AT
                    V_G_x, V_G_y, V_G_z,       -- VG
                    GRAVITY_ACC, K, V0, V_wind_vec, windFactorC)

                -- 9. ローカル座標変換
                local cosE = math.cos(global_elevation)
                local sinE = math.sin(global_elevation)
                local cosA = math.cos(global_azimuth)
                local sinA = math.sin(global_azimuth)
                local v_global_aim = { cosE * sinA, sinE, cosE * cosA }
                local v_local_aim = rotateVectorByInverseQuaternion(v_global_aim, q_ship)
                local v_local_x = v_local_aim[1]
                local v_local_y = v_local_aim[2]
                local v_local_z = v_local_aim[3]
                local local_azimuth = math.atan(v_local_x, v_local_z)
                local local_elevation = math.atan(v_local_y, math.sqrt(v_local_x ^ 2 + v_local_z ^ 2))

                -- 10. 射界制限
                local elev_limit_max_rad = PITCH_ANGLE_LIMIT
                local azi_limit_rad = YAW_ANGLE_LIMIT
                local elev_limit_min_rad = PITCH_MIN_ANGLE_LIMIT

                -- ★ 修正2: PIDアップデートを 'not isError' ブロックの *中* に移動
                turretYaw, yawShootable = PID.update(yawControlPID.pid,
                    clamp(local_azimuth, -azi_limit_rad, azi_limit_rad),
                    currentCannonYaw, DT, YAW_PIVOT_MAX_SPEED)
                turretYaw = turretYaw - yawPivot_YawAngularVelocity
                if IS_PITCH_ROBOTIC then
                    turretPitch = clamp(local_elevation, elev_limit_min_rad, elev_limit_max_rad) -
                        pitchPivot_PitchAngularVelocity
                    pitchShootable = math.abs(global_elevation - currentCannonPitch) < SHOOTABLE_ERROR_THRESHOLD
                    turretPitch = turretPitch / PI2 * 4
                else
                    turretPitch, pitchShootable = PID.update(pitchControlPID.pid,
                        clamp(local_elevation, elev_limit_min_rad, elev_limit_max_rad), currentCannonPitch,
                        DT,
                        PITCH_PIVOT_MAX_SPEED)
                    turretPitch = turretPitch - pitchPivot_PitchAngularVelocity
                end
            else
                -- ★ 修正2: ニュートン法が失敗した場合、PIDをリセット
                PID.reset(pitchControlPID.pid)
                PID.reset(yawControlPID.pid)
            end

            if isError or errCount ~= 0 then
                errCount = (errCount + 1) % 60
            end
        end -- ★ 修正1: ジンバルロック 'else' ブロックの終わり
    else
        -- 距離が近すぎる(isDetect=false)場合
        errCount = 1
        PID.reset(pitchControlPID.pid)
        PID.reset(yawControlPID.pid)
    end

    output.setNumber(1, turretYaw)
    output.setNumber(2, turretPitch)
    output.setBool(1, (not isError) and isDetect and errCount == 0 and yawShootable and pitchShootable)
end
