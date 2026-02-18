--[[
アクティブソナー距離測定スクリプト (単純近似 + 角度比較ログ)

機能:
- 定期的にソナーのアクティブモードをON/OFF制御。
- isPinging=false時に targetData を初期化。
- エコー受信時に targetData に table.insert でデータを追加。
- ★目標の真のグローバル座標(ch26-28)と自機座標/姿勢から、真のローカル角度を計算。
- ★ソナーが観測したローカル角度(ch1-16)を読み取り、真の角度と比較してログ出力。
- 単純計算で距離を求め、検出順に出力チャンネル 1, 2, ... に出力。

入力チャンネル設定:
- Composite On/Off 1-8: Sonar Echo Detect Target 1-8
- Composite Number 1-16: Sonar Angles Target 1-8 (1:T1 Azi, 2:T1 Ele, ...) - 回転単位(Turns) <-- 今回読み取る
- Composite Number 17: Physics Sensor Global Pos X (East)
- Composite Number 18: Physics Sensor Global Pos Y (Up)
- Composite Number 19: Physics Sensor Global Pos Z (North)
- Composite Number 20: Physics Sensor Euler Pitch (X rot) (Radian)
- Composite Number 21: Physics Sensor Euler Yaw (Y rot) (Radian)
- Composite Number 22: Physics Sensor Euler Roll (Z rot) (Radian)
- Composite Number 26: True Target Global Pos X (East) <-- 追加！
- Composite Number 27: True Target Global Pos Y (Up)   <-- 追加！
- Composite Number 28: True Target Global Pos Z (North) <-- 追加！

出力チャンネル設定:
- Composite Number 1-8: Calculated Distance (検出順, 単純計算)
- Composite On/Off 1: Sonar Active Mode Trigger (to Sonar Input Ch2)
--]]

-- 定数
local SOUND_SPEED = 1480
local TICKS_PER_SECOND = 60
local MAX_TARGETS = 8
local MAX_OUTPUT_CHANNELS = 8
local PI = math.pi
local PI2 = PI * 2

-- プロパティ読み込み
local PING_INTERVAL_TICKS = property.getNumber("PING_INTERVAL_TICKS") or 60
local SEND_LOGIC_DELAY = property.getNumber("SEND_LOGIC_DELAY") or 0
local RECEIVE_LOGIC_DELAY = property.getNumber("RECEIVE_LOGIC_DELAY") or 0

-- グローバル変数
local pingSentTick = 0
local isPinging = false
local currentTick = 0
local targetData = {} -- 検出したターゲットデータを格納するリスト

-- === ヘルパー関数 (ベクトル, クォータニオン, 座標変換) ===
function vectorMagnitude(v)
    local x = v[1] or v.x or 0; local y = v[2] or v.y or 0; local z = v[3] or v.z or 0; return math.sqrt(x ^ 2 + y ^ 2 +
        z ^ 2)
end

function vectorSub(v1, v2)
    local x1 = v1[1] or v1.x or 0; local y1 = v1[2] or v1.y or 0; local z1 = v1[3] or v1.z or 0; local x2 = v2[1] or v2
        .x or 0; local y2 = v2[2] or v2.y or 0; local z2 = v2[3] or v2.z or 0; return { x1 - x2, y1 - y2, z1 - z2 }
end

function multiplyQuaternions(q_a, q_b)
    local w1 = q_a[1]; local x1 = q_a[2]; local y1 = q_a[3]; local z1 = q_a[4]; local w2 = q_b[1]; local x2 = q_b[2]; local y2 =
        q_b[3]; local z2 = q_b[4]; local w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2; local x = w1 * x2 + x1 * w2 + y1 * z2 -
        z1 * y2; local y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2; local z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2; return {
        w,
        x, y, z }
end

function eulerZYX_to_quaternion(r, y, p)
    local hr = r * 0.5; local hy = y * 0.5; local hp = p * 0.5; local cr = math.cos(hr); local sr = math.sin(hr); local cy =
        math.cos(hy); local sy = math.sin(hy); local cp = math.cos(hp); local sp = math.sin(hp); local w = cr * cy * cp +
        sr * sy * sp; local x = cr * cy * sp - sr * sy * cp; local y = cr * sy * cp + sr * cy * sp; local z = sr * cy *
        cp -
        cr * sy * sp; return { w, x, y, z }
end

function rotateVectorByInverseQuaternion(v, q)
    local px = v[1] or v.x or 0; local py = v[2] or v.y or 0; local pz = v[3] or v.z or 0; local p_vec = { 0, px, py, pz }; local q_conj = {
        q[1], -q[2], -q[3], -q[4] }; local temp = multiplyQuaternions(q_conj, p_vec); local p_prime = multiplyQuaternions(
        temp, q); return { p_prime[2], p_prime[3], p_prime[4] }
end

-- グローバル座標からローカル座標へ変換する関数
function globalToLocalCoords(globalTargetPos, ownGlobalPos, ownOrientationQuat)
    local relativeVectorGlobal = vectorSub(globalTargetPos, ownGlobalPos)
    local localVector = rotateVectorByInverseQuaternion(relativeVectorGlobal, ownOrientationQuat)
    return { x = localVector[1], y = localVector[2], z = localVector[3] }
end

-- === メイン処理 ===
function onTick()
    currentTick = currentTick + 1

    -- --- Ping (アクティブモード) 開始/終了処理 ---
    if not isPinging and currentTick >= pingSentTick + PING_INTERVAL_TICKS then
        pingSentTick = currentTick + SEND_LOGIC_DELAY
        isPinging = true
    elseif isPinging then
        -- タイムアウト処理
        if currentTick >= pingSentTick + PING_INTERVAL_TICKS then
            isPinging = false
            targetData = {} -- isPinging=false時に初期化
        end
    end

    -- ソナーのアクティブモード制御信号を出力
    output.setBool(1, isPinging)

    -- このTickで検出された結果を格納するローカルリスト
    local detectionsThisTick = {}

    -- --- エコー受信 & 距離計算 & 角度比較処理 ---
    if isPinging then
        -- 自機状態読み取り
        local ownGlobalPos = {
            x = input.getNumber(17),
            y = input.getNumber(18),
            z = input.getNumber(19)
        }
        local pitch = input.getNumber(20)
        local yaw = input.getNumber(21)
        local roll = input.getNumber(22)
        local ownOrientation = eulerZYX_to_quaternion(roll, yaw, pitch)

        -- ★ 目標の真のグローバル座標読み取り ★
        local trueTargetGlobalPos = {
            x = input.getNumber(26),
            y = input.getNumber(27),
            z = input.getNumber(28)
        }
        local isTrueTargetPosValid = not (trueTargetGlobalPos.x == 0 and trueTargetGlobalPos.y == 0 and trueTargetGlobalPos.z == 0)


        -- 各ソナー入力チャンネル(1-8)をチェック
        for i = 1, MAX_TARGETS do
            local echoDetected = input.getBool(i)

            if echoDetected then
                local echoReceivedTick_actual = currentTick - RECEIVE_LOGIC_DELAY
                local pingTimeTick = echoReceivedTick_actual - pingSentTick

                if pingTimeTick > 0 then
                    local deltaTimeSec = pingTimeTick / TICKS_PER_SECOND

                    -- 1. 単純な距離計算
                    local calculated_distance = (SOUND_SPEED * deltaTimeSec) / 2

                    -- 2. ソナー観測角度の読み取り (回転単位)
                    local measuredAziTurns = input.getNumber((i - 1) * 2 + 1)
                    local measuredEleTurns = input.getNumber((i - 1) * 2 + 2)

                    -- 3. 真のローカル角度の計算 (回転単位)
                    local trueAziTurns = 0
                    local trueEleTurns = 0
                    if isTrueTargetPosValid then
                        -- 真の目標位置をローカル座標に変換
                        local trueTargetLocalPos = globalToLocalCoords(trueTargetGlobalPos, ownGlobalPos, ownOrientation)
                        local localX = trueTargetLocalPos.x
                        local localY = trueTargetLocalPos.y
                        local localZ = trueTargetLocalPos.z
                        local localDist = vectorMagnitude(trueTargetLocalPos)

                        if localDist > 1e-6 then -- ゼロ除算回避
                            -- ローカル座標から真の角度を計算
                            -- 方位角 (ローカルZ軸基準、atan(X/Z))
                            trueAziTurns = math.atan(localX, localZ) / PI2
                            -- 仰俯角 (ローカルXY平面からの角度、asin(Y/Dist))
                            local asin_arg = math.max(-1.0, math.min(1.0, localY / localDist))
                            trueEleTurns = math.asin(asin_arg) / PI2
                        end
                    end

                    -- 4. ログ出力で比較
                    debug.log(string.format("Tick:%d Ch:%d Dist:%.2f Azi:%.4f(M)/%.4f(T) Ele:%.4f(M)/%.4f(T)",
                        currentTick, i, calculated_distance,
                        measuredAziTurns, trueAziTurns,
                        measuredEleTurns, trueEleTurns))

                    -- 5. 結果をリストに追加 (距離のみ)
                    table.insert(detectionsThisTick, { distance = calculated_distance })
                end -- pingTimeTick > 0 check end
            end     -- echoDetected check end
        end         -- for loop end
    end             -- isPinging check end

    -- --- 距離データの出力 ---
    -- このTickで検出された結果リスト (detectionsThisTick) を順番に出力
    for i = 1, MAX_OUTPUT_CHANNELS do
        if i <= #detectionsThisTick and detectionsThisTick[i] ~= nil then
            output.setNumber(i, detectionsThisTick[i].distance)
        else
            output.setNumber(i, 0) -- データがないチャンネルは0
        end
    end
end

-- onDraw関数は省略
