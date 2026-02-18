--[[
    弾道計算器スクリプト (改修版)
    EKFからの入力チャンネル想定:
    iN(1):  Target X (Raw)
    iN(2):  Target Y (Raw)
    iN(3):  Target Z (Raw)
    iN(4):  Target Vx (Raw, m/s)
    iN(5):  Target Vy (Raw, m/s)
    iN(6):  Target Vz (Raw, m/s)
    iN(7):  Self X
    iN(8):  Self Y
    iN(9):  Self Z
    iN(10): Self Vx (m/s)
    iN(11): Self Vy (m/s)
    iN(12): Self Vz (m/s)
    iN(19): Target Ax (Raw, m/s^2)
    iN(20): Target Ay (Raw, m/s^2)
    iN(21): Target Az (Raw, m/s^2)
    iB(1):  isTracking
]]

-- Cannon Parameters (変更なし)
Param = {}
--Cannon Parameters(Muzzle velocity, Drag, Lifespan)
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
--Rocket
Param[8] = { 500, 0.003, 2400 }

INDEX = property.getNumber("Ammotype")
YAW_ANGLE_LIMIT = property.getNumber("AzimuthLimit") / 2
MAX_LIFESPAN = property.getNumber("FireRange")
LOGIC_DELAY = property.getNumber("LogicDelay")
PITCH_ANGLE_LIMIT = property.getNumber("ElevationMax")

PI = math.pi
PI2 = PI * 2
DT = 1 / 60
EPSILON = 10e-3
V0 = Param[INDEX][1] / 60 -- (m/tick)
K = Param[INDEX][2]
LIFESPAN = Param[INDEX][3]
GRAVITY_ACC = 30 / 60 ^ 2 -- (m/tick^2)
EXP = math.exp(1)
elevation, azimuth = 0, 0
turretYaw, turretPitch = 0, 0
yaw = 0
pitch = 0
time = 0
errCount = 60
oldDis = 0
isDetectCounter = 0

--------------------------------------------------------------------------------
-- クォータニオン演算関数
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
    -- nilチェックは原則削除
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    -- nilチェックは原則削除
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    -- nilチェックは原則削除
    temp = multiplyQuaternions(q, p)
    -- nilチェックは原則削除
    p_prime = multiplyQuaternions(temp, q_conj)
    -- nilチェックは原則削除
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

-- ★ f 関数 (新しい物理モデル: 相対座標系)
-- 引数を (t, P_rel, V_rel, A_T, g, k, V0) に変更
function f(t, Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az, g, k, V0)
    local E = EXP ^ (-k * t)
    local k2 = k * k

    -- LHS (Left Hand Side)
    -- P_rel(t) - (g_terms)

    -- P_rel(t) = P_rel_0 + V_rel_0 * t + 0.5 * A_T * t^2
    local P_rel_x_t = Px + Vx * t + 0.5 * Ax * t * t
    local P_rel_y_t = Py + Vy * t + 0.5 * Ay * t * t
    local P_rel_z_t = Pz + Vz * t + 0.5 * Az * t * t

    -- g_terms (Y軸のみ)
    local g_term_y = (g / k2) * (1 - E) - (g / k) * t

    local LHS_x = P_rel_x_t
    local LHS_y = P_rel_y_t - g_term_y
    local LHS_z = P_rel_z_t

    -- RHS (Right Hand Side)
    -- (V0/k * (1-E))^2
    local RHS_sq = (V0 / k * (1 - E)) ^ 2

    return (LHS_x ^ 2 + LHS_y ^ 2 + LHS_z ^ 2) - RHS_sq
end

-- ★ dfdx 関数 (新しい物理モデル: tに関する1次微分)
function dfdx(t, Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az, g, k, V0)
    local E = EXP ^ (-k * t)
    local k2 = k * k

    -- d(LHS)/dt
    -- d(P_rel(t))/dt = V_rel_0 + A_T * t
    local d_P_rel_x_dt = Vx + Ax * t
    local d_P_rel_y_dt = Vy + Ay * t
    local d_P_rel_z_dt = Vz + Az * t

    -- d(g_terms)/dt
    local d_g_term_y_dt = (g / k2) * (k * E) - (g / k)

    local d_LHS_x_dt = d_P_rel_x_dt
    local d_LHS_y_dt = d_P_rel_y_dt - d_g_term_y_dt
    local d_LHS_z_dt = d_P_rel_z_dt

    -- d(RHS^2)/dt
    local d_RHS_sq_dt = 2 * (V0 / k * (1 - E)) * (V0 / k * (k * E))

    -- f(t) = LHS_x^2 + LHS_y^2 + LHS_z^2 - RHS_sq
    -- f'(t) = 2*LHS_x*d(LHS_x)/dt + 2*LHS_y*d(LHS_y)/dt + 2*LHS_z*d(LHS_z)/dt - d(RHS^2)/dt

    -- LHS (t) (f関数から再計算)
    local P_rel_x_t = Px + Vx * t + 0.5 * Ax * t * t
    local P_rel_y_t = Py + Vy * t + 0.5 * Ay * t * t
    local P_rel_z_t = Pz + Vz * t + 0.5 * Az * t * t
    local g_term_y = (g / k2) * (1 - E) - (g / k) * t
    local LHS_x = P_rel_x_t
    local LHS_y = P_rel_y_t - g_term_y
    local LHS_z = P_rel_z_t

    return 2 * LHS_x * d_LHS_x_dt + 2 * LHS_y * d_LHS_y_dt + 2 * LHS_z * d_LHS_z_dt - d_RHS_sq_dt
end

-- ★ q 関数 (新しい物理モデル: 射撃方向ベクトルの計算)
function q(t, Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az, g, k, V0)
    local theta, phi
    local E = EXP ^ (-k * t)
    local k2 = k * k

    -- LHS(t) を f 関数から再計算
    local P_rel_x_t = Px + Vx * t + 0.5 * Ax * t * t
    local P_rel_y_t = Py + Vy * t + 0.5 * Ay * t * t
    local P_rel_z_t = Pz + Vz * t + 0.5 * Az * t * t
    local g_term_y = (g / k2) * (1 - E) - (g / k) * t

    -- これが射撃方向ベクトル u に比例する
    local aim_vec_x = P_rel_x_t
    local aim_vec_y = P_rel_y_t - g_term_y
    local aim_vec_z = P_rel_z_t

    -- (X:東, Y:上, Z:北) 座標系での角度計算
    phi = math.atan(aim_vec_x, aim_vec_z) -- 方位角 atan(X, Z)

    local horizontal_dist = math.sqrt(aim_vec_x ^ 2 + aim_vec_z ^ 2)
    if horizontal_dist < 1e-9 then
        if aim_vec_y > 0 then
            theta = PI * 0.5  -- 真上
        else
            theta = -PI * 0.5 -- 真下
        end
    else
        theta = math.atan(aim_vec_y, horizontal_dist) -- 仰角 atan(Y, H)
    end

    return theta, phi
end

-- (NewtonsMethod は変更なし。ただし呼び出す引数が変わる)
function NewtonsMethod(t0, Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az, g, k, V0)
    local t = t0
    for i = 1, 10 do
        local f_val = f(t, Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az, g, k, V0)
        local df_val = dfdx(t, Px, Py, Pz, Vx, Vy, Vz, Ax, Ay, Az, g, k, V0)

        if df_val == nil or math.abs(df_val) < 1e-9 then
            return 99999, true
        end

        local t_new = t - f_val / df_val

        if t_new < 0 then
            return 99999, true
        end

        if math.abs(f_val) < EPSILON then
            return t, false
        end

        if math.abs(t_new - t) < 1e-6 then
            return t_new, false
        end

        t = t_new
    end
    return 99999, true
end

function onTick()
    lifespan = LIFESPAN * MAX_LIFESPAN
    isError = true

    -- 1. EKFからのグローバル状態
    local targetX = input.getNumber(1)       -- Target Global X
    local targetY = input.getNumber(2)       -- Target Global Y
    local targetZ = input.getNumber(3)       -- Target Global Z
    local targetVectorX = input.getNumber(4) -- Target Global Vx (m/s)
    local targetVectorY = input.getNumber(5) -- Target Global Vy (m/s)
    local targetVectorZ = input.getNumber(6) -- Target Global Vz (m/s)
    local targetAccX = input.getNumber(7)    -- Target Global Ax (m/s^2)
    local targetAccY = input.getNumber(8)    -- Target Global Ay (m/s^2)
    local targetAccZ = input.getNumber(9)    -- Target Global Az (m/s^2)

    -- 2. 自機のグローバル状態
    local ownWorldX = input.getNumber(10) -- Self Global X
    local ownWorldY = input.getNumber(11) -- Self Global Y
    local ownWorldZ = input.getNumber(12) -- Self Global Z
    local ownPitch = input.getNumber(13)
    local ownYaw = input.getNumber(14)
    local ownRoll = input.getNumber(15)

    -- 2b. ローカル自機速度 (m/s) を取得
    local selfLocalVectorX = input.getNumber(16) -- ローカル X (右+) 速度
    local selfLocalVectorY = input.getNumber(17) -- ローカル Y (上+) 速度
    local selfLocalVectorZ = input.getNumber(18) -- ローカル Z (前+) 速度

    -- ★ 3. ワールド風ベクトルの計算 (連立方程式)
    local local_wind_azi_turn = input.getNumber(22) -- 水平方向の風向き (-0.5~0.5)
    local wind_speed_mps = input.getNumber(23)      -- 水平方向の風の強さ (m/s)
    -- (A) センサーが読み取った「ローカルの水平見かけ風」ベクトル
    local local_wind_azi_rad = local_wind_azi_turn * PI2
    local V_sensor_x = -wind_speed_mps * math.sin(local_wind_azi_rad) -- センサーのX軸
    local V_sensor_z = -wind_speed_mps * math.cos(local_wind_azi_rad) -- センサーのZ軸

    -- (B) 自機の姿勢クォータニオン
    local q_ship = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)

    -- (C) World-to-Local 回転行列 R の必要な成分を計算
    local c1 = rotateVectorByInverseQuaternion({ 1, 0, 0 }, q_ship) -- R * {World X}
    local c2 = rotateVectorByInverseQuaternion({ 0, 1, 0 }, q_ship) -- R * {World Y} (不要)
    local c3 = rotateVectorByInverseQuaternion({ 0, 0, 1 }, q_ship) -- R * {World Z}
    local r11, r13 = c1[1], c3[1]                                   -- R の 1行目 (Local X)
    local r31, r33 = c1[3], c3[3]                                   -- R の 3行目 (Local Z)

    -- (D) 2x2 連立方程式を解く
    -- r11*Wx + r13*Wz = Kx
    -- r31*Wx + r33*Wz = Kz

    -- ★【重要】方程式の右辺 (Kx, Kz) を正しく計算
    local Kx = selfLocalVectorX - V_sensor_x
    local Kz = selfLocalVectorZ - V_sensor_z

    -- 行列式 (Determinant)
    local D = r11 * r33 - r13 * r31

    local Wind_X_mps = 0
    local Wind_Y_mps = 0 -- Y (Up) direction wind is zero (仕様)
    local Wind_Z_mps = 0
    -- kx, r13
    -- kz, r33
    if math.abs(D) > 1e-6 then
        -- ★ 解が存在する場合 (真の風 Wx, Wz を計算)
        Wind_X_mps = (r33 * Kx - r13 * Kz) / D
        Wind_Z_mps = (-r31 * Kx + r11 * Kz) / D
    else
        -- ★ 特異点 (傾きすぎ) -> 計算不能
        Wind_X_mps = 0
        Wind_Z_mps = 0
    end

    -- 4. m/tick 単位への変換
    -- (target...dt, selfVector...dt -> V_G) ...
    local targetVectorXdt = targetVectorX * DT
    local targetVectorYdt = targetVectorY * DT
    local targetVectorZdt = targetVectorZ * DT
    local targetAccXdt = targetAccX * DT * DT
    local targetAccYdt = targetAccY * DT * DT
    local targetAccZdt = targetAccZ * DT * DT

    -- V_G (自機ワールド速度) を計算 (ローカル速度を回転)
    local v_self_world_array = rotateVectorByQuaternion(
        { selfLocalVectorX, selfLocalVectorY, selfLocalVectorZ },
        q_ship
    )
    local V_G_x = v_self_world_array[1] * DT
    local V_G_y = v_self_world_array[2] * DT
    local V_G_z = v_self_world_array[3] * DT

    -- ★ 風ベクトル (m/tick) (計算した「真の風」)
    local V_wind_vec = {
        x = Wind_X_mps * DT,
        y = Wind_Y_mps * DT, -- 常に 0
        z = Wind_Z_mps * DT
    }

    globalSelfVector = rotateVectorByQuaternion({ x = selfLocalVectorX, y = selfLocalVectorY, z = selfLocalVectorZ },
        q_ship)
end
