-- zeros(rows, cols)
function zeros(rows, cols)
    local m = {}; for r = 1, rows do
        m[r] = {}; for c = 1, cols do m[r][c] = 0 end
    end
    return m
end

function mul(...)
    local mats = { ... }; local A = mats[1]; local R; for i = 2, #mats do
        local B = mats[i]; if #A[1] ~= #B then
            -- debug.log("Error: Matrix mul dim mismatch")
            return nil
        end
        R = zeros(#A, #B[1]); for r = 1, #A do
            for c = 1, #B[1] do
                local sVal = 0; for k = 1, #B do sVal = sVal + A[r][k] * B[k][c] end
                R[r][c] = sVal
            end
        end
        A = R
    end
    return A
end

function rotateVectorZYX(vector, pitch, yaw, roll)
    -- 回転行列 R = Rx(pitch) * Ry(yaw) * Rz(roll)

    -- Rz
    local RZ = { { math.cos(roll), -math.sin(roll), 0 }, { math.sin(roll), math.cos(roll), 0 }, { 0, 0, 1 } }
    -- Ry
    local RY = { { math.cos(yaw), 0, math.sin(yaw) }, { 0, 1, 0 }, { -math.sin(yaw), 0, math.cos(yaw) } }
    -- Rx
    local RX = { { 1, 0, 0 }, { 0, math.cos(pitch), -math.sin(pitch) }, { 0, math.sin(pitch), math.cos(pitch) } }
    local R = mul(RZ, RY, RX)


    --[[ 展開したものを使用して計算コスト削減
    local R = { { math.cos(roll) * math.cos(yaw), math.cos(roll) * math.sin(yaw) * math.sin(pitch) - math.sin(roll) * math.cos(pitch), math.cos(roll) * math.sin(yaw) * math.cos(pitch) + math.sin(roll) * math.sin(pitch) },
        { math.sin(roll) * math.cos(yaw), math.sin(roll) * math.sin(yaw) * math.sin(pitch) + math.cos(roll) * math.cos(pitch), math.sin(roll) * math.sin(yaw) * math.cos(pitch) - math.cos(roll) * math.sin(pitch) },
        { -math.sin(yaw),                 math.cos(yaw) * math.sin(pitch),                                                     math.cos(yaw) * math.cos(pitch) } }
    --]]
    return mul(R, vector)
end

-- 二つのクォータニオン q_a = {w, x, y, z}, q_b = {w, x, y, z} の積を計算する関数
-- q_result = q_a * q_b
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1 = q_a[1], q_a[2], q_a[3], q_a[4]
    local w2, x2, y2, z2 = q_b[1], q_b[2], q_b[3], q_b[4]

    local w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    local x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    local y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    local z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return { w_result, x_result, y_result, z_result }
end

-- ZYXオイラー角 (Roll: phi, Yaw: psi, Pitch: theta) からクォータニオン q = (w, x, y, z) を計算する関数
-- 入力角度はラジアン単位
function eulerZYX_to_quaternion(roll, yaw, pitch)
    -- オイラー角の半分を計算
    local half_roll = roll * 0.5
    local half_yaw = yaw * 0.5
    local half_pitch = pitch * 0.5

    -- 角度の半分のcosとsinを事前計算
    local cr = math.cos(half_roll)
    local sr = math.sin(half_roll)
    local cy = math.cos(half_yaw)
    local sy = math.sin(half_yaw)
    local cp = math.cos(half_pitch)
    local sp = math.sin(half_pitch)

    -- クォータニオンの成分を計算
    local w = cr * cy * cp + sr * sy * sp
    local x = cr * cy * sp - sr * sy * cp -- X成分
    local y = cr * sy * cp + sr * cy * sp -- Y成分
    local z = sr * cy * cp - cr * sy * sp -- Z成分

    -- クォータニオンをテーブルとして返す (wが最初の要素)
    -- または {x=x, y=y, z=z, w=w} のような形式でも良い
    return { w, x, y, z }
end

-- ベクトル v = {x, y, z} をクォータニオン q = {w, x, y, z} で回転させる関数 (標準版: p' = q p q*)
function rotateVectorByQuaternion(vector, quaternion)
    -- ベクトルを純粋クォータニオン p = (0, vx, vy, vz) に変換
    local px = vector[1] or vector.x or 0
    local py = vector[2] or vector.y or 0
    local pz = vector[3] or vector.z or 0
    local p = { 0, px, py, pz }

    -- 回転クォータニオン q
    local q = quaternion

    -- 回転クォータニオンの共役 q* = (w, -x, -y, -z) を計算
    local q_conj = { q[1], -q[2], -q[3], -q[4] }

    -- p' = q * p * q* を計算 (標準的な順序)
    local p_prime = multiplyQuaternions(multiplyQuaternions(q, p), q_conj) -- ★ 標準の掛け算順序

    -- p' のベクトル部 {x, y, z} を回転後のベクトルとして返す
    return { p_prime[2], p_prime[3], p_prime[4] }
end

-- **** 使用例の修正 ****
-- (eulerZYX_to_quaternion と multiplyQuaternions は変更なし)

-- 1. 回転を定義するオイラー角 (ラジアン単位！)
local roll_angle = math.rad(0)
local yaw_angle = math.rad(20)
local pitch_angle = math.rad(50)

-- 2. オイラー角から回転クォータニオンを生成
local rotation_quaternion = eulerZYX_to_quaternion(roll_angle, yaw_angle, pitch_angle)

-- 3. 回転させたい元のベクトル (座標)
local original_vector = { 1, 0, 0 }

-- 4. ★標準版の関数★ を使ってベクトルを回転
local rotated_vector = rotateVectorByQuaternion(original_vector, rotation_quaternion) -- ★ 標準版関数を呼び出す
print("クォータニオン (標準版)>> Rotated Vector: x=" ..
    rotated_vector[1] .. ", y=" .. rotated_vector[2] .. ", z=" .. rotated_vector[3])

-- 比較用: 回転行列の結果 (前回 Y=0 になった方)
local original_matrix = { { original_vector[1] }, { original_vector[2] }, { original_vector[3] } }
local rotated_matrix = rotateVectorZYX(original_matrix, pitch_angle, yaw_angle, roll_angle)
print("回転行列 (比較用)>> Rotated Matrix: x=" ..
    rotated_matrix[1][1] .. ", y=" .. rotated_matrix[2][1] .. ", z=" .. rotated_matrix[3][1])
