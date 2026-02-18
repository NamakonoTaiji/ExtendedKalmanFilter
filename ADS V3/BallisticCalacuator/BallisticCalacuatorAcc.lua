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
AZIMUTH_LIMIT = property.getNumber("AzimuthLimit") / 2
MAX_LIFESPAN = property.getNumber("FireRange")
LOGIC_DELAY = property.getNumber("LogicDelay")
ELEVATION_LIMIT = property.getNumber("ElevationMax")

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
outputA, outputE = 0, 0
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
    local selfVectorX = input.getNumber(16) -- Self Global Vx (m/s)
    local selfVectorY = input.getNumber(17) -- Self Global Vy (m/s)
    local selfVectorZ = input.getNumber(18) -- Self Global Vz (m/s)

    -- 3. m/tick 単位への変換
    local targetVectorXdt = targetVectorX * DT
    local targetVectorYdt = targetVectorY * DT
    local targetVectorZdt = targetVectorZ * DT
    local targetAccXdt = targetAccX * DT * DT
    local targetAccYdt = targetAccY * DT * DT
    local targetAccZdt = targetAccZ * DT * DT

    local selfVectorXdt = selfVectorX * DT
    local selfVectorYdt = selfVectorY * DT
    local selfVectorZdt = selfVectorZ * DT

    -- 4. LogicDelay に基づく t=0 の状態を計算
    local t_delay = LOGIC_DELAY
    local t_delay2_half = 0.5 * t_delay * t_delay

    -- t=0 の目標グローバル状態
    local P_T_x = targetX + targetVectorXdt * t_delay + targetAccXdt * t_delay2_half
    local P_T_y = targetY + targetVectorYdt * t_delay + targetAccYdt * t_delay2_half
    local P_T_z = targetZ + targetVectorZdt * t_delay + targetAccZdt * t_delay2_half
    local V_T_x = targetVectorXdt + targetAccXdt * t_delay
    local V_T_y = targetVectorYdt + targetAccYdt * t_delay
    local V_T_z = targetVectorZdt + targetAccZdt * t_delay
    -- A_T (加速度) は t=0 でも一定と仮定
    local A_T_x = targetAccXdt
    local A_T_y = targetAccYdt
    local A_T_z = targetAccZdt

    -- t=0 の自機グローバル状態 (遅延を考慮しない = 現在値)
    local P_G_x = ownWorldX
    local P_G_y = ownWorldY
    local P_G_z = ownWorldZ
    local V_G_x = selfVectorXdt
    local V_G_y = selfVectorYdt
    local V_G_z = selfVectorZdt

    -- 5. ★ t=0 の相対状態を計算 (NewtonsMethod への入力)
    local P_rel_x = P_T_x - P_G_x
    local P_rel_y = P_T_y - P_G_y
    local P_rel_z = P_T_z - P_G_z
    local V_rel_x = V_T_x - V_G_x
    local V_rel_y = V_T_y - V_G_y
    local V_rel_z = V_T_z - V_G_z
    -- A_T (目標のグローバル加速度) = A_rel (自機加速度0と仮定)
    local A_rel_x = A_T_x
    local A_rel_y = A_T_y
    local A_rel_z = A_T_z

    -- 6. 初期値の計算
    local distance = math.sqrt(P_rel_x ^ 2 + P_rel_y ^ 2 + P_rel_z ^ 2)
    local disDelta = oldDis - distance
    oldDis = distance               -- ★ BUGFIX: global oldDis を更新 (local を削除)

    local isDetect = distance > 0.1 -- ゼロ判定を少し甘くする
    local isShootable = false
    local t0 = distance / V0

    outputA = 0
    outputE = 0

    if isDetect then
        -- 7. ★ NewtonsMethod を新しい引数 (相対座標) で呼び出す
        time, isError = NewtonsMethod(t0,
            P_rel_x, P_rel_y, P_rel_z,
            V_rel_x, V_rel_y, V_rel_z,
            A_rel_x, A_rel_y, A_rel_z,
            GRAVITY_ACC, K, V0)

        if not isError then
            -- 8. ★ q 関数を新しい引数 (相対座標) で呼び出す
            local global_elevation, global_azimuth = q(time,
                P_rel_x, P_rel_y, P_rel_z,
                V_rel_x, V_rel_y, V_rel_z,
                A_rel_x, A_rel_y, A_rel_z,
                GRAVITY_ACC, K, V0)

            -- (以降のローカル座標変換処理は変更なし)
            ------------------------------------------------
            -- ★ グローバル角度からローカル角度への変換
            ------------------------------------------------
            local q_ship = eulerZYX_to_quaternion(ownRoll, ownYaw, ownPitch)
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

            ------------------------------------------------
            -- ★ 射撃可否判定 (ローカル角度を使用)
            ------------------------------------------------
            local elev_limit_rad = ELEVATION_LIMIT * PI2
            local azi_limit_rad = AZIMUTH_LIMIT * PI2

            if local_elevation < elev_limit_rad and local_elevation > -0.1 * PI2 and
                local_azimuth < azi_limit_rad and local_azimuth > -azi_limit_rad then
                outputA = local_azimuth
                outputE = local_elevation

                if time < lifespan then
                    isShootable = true
                end
            end
        end

        if isError or errCount ~= 0 then
            errCount = (errCount + 1) % 60
        end
    else
        errCount = 1
        oldDis = 0
    end

    if not isShootable then
        isDetect = false
    end

    output.setNumber(1, outputA / PI2)
    output.setNumber(2, outputE / PI2 * 4)
    output.setBool(1, (not isError) and isDetect and errCount == 0)
end
