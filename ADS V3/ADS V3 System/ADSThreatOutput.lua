---KalmanFilterから時分割で受け取った目標情報から脅威を選別しVLSの発射、データリンクを行うスクリプト
---detectionTickLagが1の時かつhitsが一定値以上超えている場合は撃墜するべき脅威として判定、VLSに目標情報、ID、発射信号が送信される。
---ミサイルはこの情報を受取ったのちはカルマンフィルターと直接交信を行い目標IDの座標へ中間誘導する。
--[[
入力:
- bool 1: ミサイル対水上モード発射フラグ
- num 1-3: 全周防空システムからの目標情報 (X座標E-W,Y座標Altitude,Z座標N-S)
- num 4-6: 全周防空システムからの目標情報(X軸速度,Y軸速度,Z軸速度,m/s)
- num 7: 全周防空システムから目標に割り当てられたID
- num 8: 目標情報の推定精度(イプシロン)
- num 9: 防空システムが追跡していた目標情報を更新したときのフラグ(1->true,0->false)
- num 10: 目標が最後にレーダーで観測された時点のゲーム内tick
- num 11: 目標が最後にレーダーで観測されてから経過したTick数
- num 12: 目標を何回連続で同定できたか
- num 13: 飛行中のミサイルから返信されるミサイルID
- num 14: ミサイル側受信周波数
- num 15: 対水上目標座標X
- num 16: 対水上目標座標Y
- num 17: 対水上目標座標Z
- num 18: 対水上目標ID
- num 27: 自機X座標
- num 28: 自機Y座標
- num 29: 自機Z座標
- num 30: 自機オイラー角 Pitch (ラジアン)
- num 31: 自機オイラー角 Yaw (ラジアン)
- num 32: 自機オイラー角 Roll (ラジアン)
出力
- bool 1..N: VLS発射セル (1 -> On)
- num 1-6: 目標位置・速度 (X, Y, Z, Vx, Vy, Vz)
- num 12: 目標ID
- num 13: 発射するミサイルに割り当てる個別のミサイルID
- num 14: ミサイルに割り当てる専用周波数
- num 30: 無線機へ出力する周波数
- num 31: ミサイルが受信する無線周波数
- num 32: ミサイルへの接続が脅威度判定/火器管制スクリプトであることを識別させるために個別の信号を出力
]]
launchChannel = 0

PI = math.pi
PI2 = math.pi * 2

INTERCEPT_THREAT_SPEED = property.getNumber("INTERCEPT_THREAT_SPEED") or 150 -- 迎撃する目標の近接速度
-- 「こちらを向いている」と判定する角度の閾値 (コサイン値)
-- 0.0 = 90度以内, 0.5 = 60度以内, 0.866 = 30度以内
THREAT_FACING_THRESHOLD = property.getNumber("THREAT_FACING_THRESHOLD") or 0.5
THREAT_HITS_THRESHOLD = property.getNumber("THREAT_HITS_THRESHOLD") or 5 -- 迎撃に必要な最低ヒット数 (Hits)
MISSILE_TIMEOUT_TICKS = property.getNumber("MISSILE_TIMEOUT_TICKS") or 30
BASE_FREQUENCY = property.getNumber("BASE_FREQUENCY")
ADS_SEND_FREQ = property.getNumber("AirDefenceSystemFreq")
ANTI_SHIP_FREQ = math.floor(ADS_SEND_FREQ / 2)

-- ミサイル管理用データ構造
nextMissileID = 1            -- 次に割り当てるミサイルID
activeMissiles = {}          -- 飛行中のミサイル情報 [missileID] = { targetID, freq, lastSeenTick }
freqRotList = {}             -- 周波数巡回用のテーブル [1] = 1001, [2] = 1002 ...
currentRotIndex = 1
threatTargetsID = {}         -- 追跡中の目標IDとミサイルIDの対応 [targetID] = missileID
currentTick = 0
antiShipModeTargetQueue = {} -- 対水上モードで発射待ちのミサイルのキュー {antiShipTargetCoordX, antiShipTargetCoordY, antiShipTargetCoordY}

---@class Vector3
---@field x number
---@field y number
---@field z number

---@class Quaternion
---@field qw number
---@field qx number
---@field qy number
---@field qz number

--------------------------------------------------------------------------------
-- クォータニオン演算関数
--------------------------------------------------------------------------------
function multiplyQuaternions(q_a, q_b)
    local w1, x1, y1, z1, w2, x2, y2, z2, w_result, x_result, y_result, z_result
    -- nilチェックは原則削除
    w1 = q_a[1]
    x1 = q_a[2]
    y1 = q_a[3]
    z1 = q_a[4]
    w2 = q_b[1]
    x2 = q_b[2]
    y2 = q_b[3]
    z2 = q_b[4]
    -- nilチェックは原則削除
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return { w_result, x_result, y_result, z_result }
end

-- 物理センサーのロールピッチヨーからクォータニオンへ変換
function eulerZYX_to_quaternion(roll, yaw, pitch)
    local half_roll, half_yaw, half_pitch, cr, sr, cy, sy, cp, sp, w, x, y, z
    -- nilチェックは原則削除
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
    -- nilチェックは原則削除
    px = vector[1] or vector.x or 0
    py = vector[2] or vector.y or 0
    pz = vector[3] or vector.z or 0
    -- nilチェックは原則削除
    p = { 0, px, py, pz }
    q = quaternion
    q_conj = { q[1], -q[2], -q[3], -q[4] }
    -- nilチェックは原則削除
    temp = multiplyQuaternions(q_conj, p)
    -- nilチェックは原則削除
    p_prime = multiplyQuaternions(temp, q)
    -- nilチェックは原則削除
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

--- ワールド座標系のグローバル座標を、特定のオブジェクトを基準としたローカル座標に変換します。
---@description 基準オブジェクトからターゲットへの相対ベクトルを計算し、
--              オブジェクトの姿勢（逆クォータニオン）で回転させることでローカル座標を求めます。
---@param globalTargetPos Vector3 変換したいターゲットのグローバル座標。
---@param objectGlobalPos Vector3 基準となるオブジェクトのグローバル座標。
---@param objectOrientationQuat Quaternion 基準となるオブジェクトの姿勢を表すクォータニオン。
---@return Vector3 基準オブジェクトから見たターゲットのローカル座標。
function globalToLocal(globalTargetPos, objectGlobalPos, objectOrientationQuat)
    -- 1. 基準オブジェクトからターゲットへの相対ベクトルをグローバル座標系で計算
    local relativeVecGlobal = {
        x = (globalTargetPos.x or 0) - (objectGlobalPos.x or 0),
        y = (globalTargetPos.y or 0) - (objectGlobalPos.y or 0),
        z = (globalTargetPos.z or 0) - (objectGlobalPos.z or 0)
    }

    -- 2. 逆クォータニオンを使ってグローバルな相対ベクトルを回転させ、ローカル座標系でのベクトルに変換
    local localVec = rotateVectorByInverseQuaternion(
        { relativeVecGlobal.x, relativeVecGlobal.y, relativeVecGlobal.z },
        objectOrientationQuat
    )

    -- 3. ローカル座標ベクトルを {x, y, z} 形式のテーブルとして返す
    return { x = localVec[1], y = localVec[2], z = localVec[3] }
end

--- ローカル座標から方位角と仰角へ変換
---@param localPosVec Vector3 変換したいローカル座標(X右方向, Y上方向, Z前方向)
---@return table azimuthとelevationを返します(ラジアン)
function localCoordsToLocalAngle(localPosVec)
    local horizontalDistance, currentLocalAzimuth, currentLocalElevation
    horizontalDistance = math.sqrt(localPosVec.x ^ 2 + localPosVec.z ^ 2)
    currentLocalAzimuth = math.atan(localPosVec.x, localPosVec.z)        -- atan(左右, 前後)
    currentLocalElevation = math.atan(localPosVec.y, horizontalDistance) -- atan(上下, 水平距離)
    return { azimuth = currentLocalAzimuth, elevation = currentLocalElevation }
end

-- ローカル極座標からローカル直交座標へ
---@param dist number 距離
---@param localAziRad number 方位角(ラジアン)
---@param localEleRad number 仰角(ラジアン)
---@return Vector3 ローカル座標(x右方向, y上方向, z前方向)
function localAngleDistToLocalCoords(dist, localAziRad, localEleRad)
    local localX, localY, localZ
    localX = dist * math.cos(localEleRad) * math.sin(localAziRad)
    localY = dist * math.sin(localEleRad)
    localZ = dist * math.cos(localEleRad) * math.cos(localAziRad)
    return { x = localX, y = localY, z = localZ }
end

-- 角度差を計算 (-PI から PI の範囲)
function calculateAngleDifference(angle1, angle2)
    local diff = angle2 - angle1
    while diff <= -PI do diff = diff + PI2 end
    while diff > PI do diff = diff - PI2 end
    return diff
end

function onTick()
    currentTick = currentTick + 1
    -- -----------------------------------------------------------------
    -- 1. 受信機から届いたIDの処理（遅延があろうが届いたIDを生かす！）
    -- -----------------------------------------------------------------
    local rxMissileID = input.getNumber(13) -- 受信機から届いたミサイルID

    if rxMissileID > 0 and activeMissiles[rxMissileID] then
        -- 届いたIDの生存時刻を更新（何色の電波で拾ったかどうかも関係なし）
        activeMissiles[rxMissileID].lastSeenTick = currentTick
    end

    -- -----------------------------------------------------------------
    -- 2. 毎tick無条件で受信周波数を切り替える（高速ローテーション）
    -- -----------------------------------------------------------------
    freqRotList = {}
    for mID, mData in pairs(activeMissiles) do
        table.insert(freqRotList, mData.freq)
    end

    local listeningFreq = 0
    if #freqRotList > 0 then
        currentRotIndex = (currentRotIndex % #freqRotList) + 1
        listeningFreq = freqRotList[currentRotIndex]
    else
        currentRotIndex = 0
    end

    -- 無線受信機の周波数へ出力
    output.setNumber(30, listeningFreq)
    -- -----------------------------------------------------------------
    -- 3. タイムアウト判定（120tick間一度も通信が届かなかった場合）
    -- -----------------------------------------------------------------
    for mID, mData in pairs(activeMissiles) do
        if currentTick - mData.lastSeenTick > MISSILE_TIMEOUT_TICKS then
            -- 迎撃失敗とみなして目標のロック解除（再迎撃を許可）
            threatTargetsID[mData.targetID] = nil
            activeMissiles[mID] = nil
        end
    end

    -- -----------------------------------------------------------------
    -- 対水上モードの処理
    -- -----------------------------------------------------------------
    -- ミサイル側の受信周波数                                                                                                 -- 対水上モードで戦闘する場合の無線周波数
    local isAntiShipModeActive = input.getBool(1)                                                              -- trueになる度に対水上モードで一発発射
    local antiShipTargetCoords = { x = input.getNumber(15), y = input.getNumber(16), z = input.getNumber(17) } -- 対水上モードで発射する目標の座標
    local id = input.getNumber(18)                                                                             -- 火器管制レーダーから送られてくる目標ID
    local targetShipId = id +
        190000
    local antiShipTargetInfos = nil
    -- 脅威度判定/火器管制マイコンであることを示すために + 100000, 対水上モードであることを示すために + 90000する
    if isAntiShipModeActive and id ~= 0 then
        antiShipTargetInfos = {
            x = antiShipTargetCoords.x,
            y = antiShipTargetCoords.y,
            z = antiShipTargetCoords.z,
            id =
                targetShipId
        }
    end

    -- -----------------------------------------------------------------
    -- 対空モードの処理
    -- -----------------------------------------------------------------

    local ownWorldCoords = { x = input.getNumber(27), y = input.getNumber(28), z = input.getNumber(29) }          -- 自機座標 {x -> 東, y -> 高度, z -> 北}

    local isDetected = input.getNumber(9) == 1
    local trackingID = input.getNumber(7)

    ------ 追跡中ターゲットをローカル角度に変換
    detectedTarget = {
        x  = input.getNumber(1),
        y  = input.getNumber(2),
        z  = input.getNumber(3),
        vX = input.getNumber(4),
        vY = input.getNumber(5),
        vZ = input.getNumber(6),
    }
    if isDetected or antiShipTargetInfos then -- 防空システムが目標を検出中もしくは対水上モードでの発射キューがある場合
        detectedTarget.epsilon          = input.getNumber(8)
        detectedTarget.lastSeenTick     = input.getNumber(10)
        detectedTarget.detectionTickLag = input.getNumber(11)
        detectedTarget.hits             = input.getNumber(12)
        detectedTarget.id               = trackingID + 100000 -- 防空システムからのIDなのか発射管制装置のIDなのか区別するために10万足す

        -- 1. 目標自身の絶対速度を計算 (m/s)
        local targetSpeed               = math.sqrt(detectedTarget.vX ^ 2 + detectedTarget.vY ^ 2 + detectedTarget.vZ ^ 2)
        detectedTarget.speed            = targetSpeed -- (任意) 情報をテーブルに保存

        -- 2. 「こちらを向いているか」を判定
        local isFacingUs                = false
        if targetSpeed > 0.1 then -- 速度がゼロに近い場合は計算不可
            -- 目標から自機へ向かうベクトル (レーダーオフセット前の自機座標を使用)
            local vecToOwn = {
                x = ownWorldCoords.x - detectedTarget.x,
                y = ownWorldCoords.y - detectedTarget.y,
                z = ownWorldCoords.z - detectedTarget.z
            }
            -- 目標から自機までの距離
            local distToOwn = math.sqrt(vecToOwn.x ^ 2 + vecToOwn.y ^ 2 + vecToOwn.z ^ 2)

            if distToOwn > 0.1 then
                -- 目標の速度ベクトル (V_target) と 目標から自機へのベクトル (V_to_own) の内積を計算
                local dotProduct = detectedTarget.vX * vecToOwn.x +
                    detectedTarget.vY * vecToOwn.y +
                    detectedTarget.vZ * vecToOwn.z

                -- 正規化された内積 (コサイン類似度) を計算
                -- この値が 1 に近いほど、目標は正確に自機を向いている
                local normalizedDot = dotProduct / (targetSpeed * distToOwn)

                -- コサイン値が指定した閾値 (例: 0.5 = 60度以内) より大きければ「こちらを向いている」と判定
                if normalizedDot > THREAT_FACING_THRESHOLD then
                    isFacingUs = true
                end
            end
        end
        -- 3. 最終的な脅威判定
        local isThreat = isFacingUs and (targetSpeed > INTERCEPT_THREAT_SPEED)
        detectedTarget.isThreat = isThreat -- (任意) 情報をテーブルに保存

        local checkThreatTargetsID = (threatTargetsID[detectedTarget.id] == nil)

        -- 4. 脅威であり、かつ迎撃条件を満たす場合の処理 (VLS発射など)
        if isThreat and detectedTarget.detectionTickLag == 1 and detectedTarget.hits > THREAT_HITS_THRESHOLD and checkThreatTargetsID then
            -- このターゲットは迎撃すべき脅威と判定

            local assignedMissileID = nextMissileID
            nextMissileID = nextMissileID + 1

            -- ミサイルごとにユニークな周波数を計算 (例: 1001, 1002...)
            local assignedFreq = BASE_FREQUENCY + assignedMissileID

            -- VLS発射信号 (チャンネル1 On/Off)
            launchChannel = launchChannel + 1
            output.setBool(launchChannel, true)

            -- ミサイル管理テーブルに追加
            threatTargetsID[detectedTarget.id] = assignedMissileID
            activeMissiles[assignedMissileID] = {
                targetID = detectedTarget.id,
                freq = assignedFreq,
                lastSeenTick = currentTick
            }

            -- データリンク情報 (チャンネル 1-6 Number)
            output.setNumber(1, detectedTarget.x)   -- 目標 Global X
            output.setNumber(2, detectedTarget.y)   -- 目標 Global Y
            output.setNumber(3, detectedTarget.z)   -- 目標 Global Z
            output.setNumber(4, detectedTarget.vX)  -- 目標 Global vX
            output.setNumber(5, detectedTarget.vY)  -- 目標 Global vY
            output.setNumber(6, detectedTarget.vZ)  -- 目標 Global vZ
            output.setNumber(12, detectedTarget.id) -- 目標ID
            output.setNumber(13, assignedMissileID) -- ミサイルの個別識別用ID
            output.setNumber(14, assignedFreq)      -- ミサイルに割り当てる専用周波数
            output.setNumber(15, detectedTarget.detectionTickLag)
            -- ミサイルが受信する無線の周波数
            output.setNumber(31, ADS_SEND_FREQ)
        elseif antiShipTargetInfos then -- 対水上モードの発射シーケンス
            local assignedMissileID = nextMissileID
            nextMissileID = nextMissileID + 1

            local assignedFreq = BASE_FREQUENCY + assignedMissileID

            launchChannel = launchChannel + 1
            output.setBool(launchChannel, true)

            output.setNumber(1, antiShipTargetInfos.x)
            output.setNumber(2, antiShipTargetInfos.y)
            output.setNumber(3, antiShipTargetInfos.z)
            output.setNumber(12, antiShipTargetInfos.id)
            output.setNumber(13, assignedMissileID)
            output.setNumber(14, assignedFreq)
            -- ミサイルが受信する無線の周波数
            output.setNumber(31, ANTI_SHIP_FREQ)
            table.remove(antiShipTargetInfos, 1)
        end
    end
    output.setNumber(32, 114514) -- 火器管制マイコンからの通信が途絶えたときにわかるようにするための信号
end
