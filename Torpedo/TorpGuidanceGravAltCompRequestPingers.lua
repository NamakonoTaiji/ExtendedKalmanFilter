--[[
ミサイルの誘導部分を担うスクリプト。アクティブレーダー圏内までは防空システムと通信して中間誘導/単追尾を行う。終末誘導時はアクティブ誘導/比例航法
至近距離ではミサイル出力を使用して近接信管で目標を破壊する。

-- 入力:
- bool 1: 目標を検出中
- bool 2: 第一ピンガーリクエスト
- bool 3: 第二ピンガーリクエスト
- num 1: 推定目標座標 X
- num 2: 推定目標座標 Y
- num 3: 推定目標座標 Z
- num 4: 推定目標速度 Vx
- num 5: 推定目標速度 Vy
- num 6: 推定目標速度 Vz
- num 7: 推定目標加速度 Ax
- num 8: 推定目標加速度 Ay
- num 9: 推定目標加速度 Az
- num 10: レーダー方位角マニュアル制御
- num 11: レーダー仰角マニュアル制御
- num 12: トラック中の目標ID
- num 13-15: 自機X,Y,Z座標パススルー
- num 16-19: 自機姿勢(クォータニオンw,x,y,z)
- num 32: 最新のイプシロンε

-- 出力:
- bool 1: ミサイル出力レーダーon/off
- bool 2: 対水上用衝撃信管on/off
- num 1: Yaw軸フィン
- num 2: Pitch軸フィン
]]


DT                          = 1 / 60
PI                          = math.pi
PI2                         = PI * 2

launchedCount               = 0
currentTick                 = 0
isPingerRequested           = false
pingerRequestedTick         = 0
oldChosenViewTargetID       = 0
reorientMode                = false

REORIENT_ENTER_ANGLE        = 70 * PI / 180
REORIENT_EXIT_ANGLE         = 20 * PI / 180

SOUND_SPEED_PER_TICK        = 24.666
MIN_INITIAL_PINGS           = 4

PN_FIN_STRENGTH             = property.getNumber("PN_FIN_STRENGTH")
PPN_FIN_STRENGTH            = property.getNumber("PPN_FIN_STRENGTH")
CLOSEST_DISTANCE_THRESHOLD  = property.getNumber("CLOSEST_DISTANCE_THRESHOLD")
TARGET_LOST_THRESHOLD_TICKS = property.getNumber("T_LOST")
DISTANCE_LOOKAHEAD          = property.getNumber("DISTANCE_LOOKAHEAD")
MAX_FIN_COMMAND             = property.getNumber("MAX_FIN_COMMAND")

-- 重力補償 + 高度保持
-- pitchAngleに加算する補正量として使用
GRAVITY_COMP_ENABLE         = property.getBool("GRAVITY_COMP_ENABLE")
ALT_HOLD_ENABLE             = property.getBool("ALT_HOLD_ENABLE")
ALT_HOLD_TARGET             = property.getNumber("ALT_HOLD_TARGET")
ALT_HOLD_KP                 = property.getNumber("ALT_HOLD_KP")
ALT_HOLD_KD                 = property.getNumber("ALT_HOLD_KD")
GRAVITY_COMP_GAIN           = property.getNumber("GRAVITY_COMP_GAIN")
MAX_ALTITUDE_CORRECTION     = property.getNumber("MAX_ALTITUDE_CORRECTION")
ALT_HOLD_DISABLE_DISTANCE   = property.getNumber("ALT_HOLD_DISABLE_DISTANCE")

ORBIT_DIRECTION             = 1
ORBIT_RADIUS                = property.getNumber("ORBIT_RADIUS")
RECOVERY_RADIUS             = ORBIT_RADIUS * 1.5
ORBIT_LOOKAHEAD             = DISTANCE_LOOKAHEAD
RECOVERY_BOOST_START_ANGLE  = 120 * PI / 180
-- 前方に入るほど目標中心へ切り込む強さ
RECOVERY_ATTACK_GAIN        = 1.4
-- 前方に入ったとき接線成分をどれだけ減らすか
RECOVERY_TANGENT_REDUCTION  = 0.65
-- Recovery中の旋回ゲイン
RECOVERY_YAW_BASE_GAIN      = 0.25
RECOVERY_YAW_BOOST          = 1.75
RECOVERY_BACK_LIMIT_RATIO   = 0.25

IS_VTFUSE_ENABLED           = property.getBool("VTFUSE_ENABLE")
VT_FUSE_IMPACT_DIST         = property.getNumber("VT_FUSE_IMPACT_DIST")

targetCoords                = { 0, 0, 0 }
initialTargetCoords         = { 0, 0, 0 }
targetInfos                 = {}
chosenViewTargetID          = 0
velocityBuffer              = { x = 0, y = 0, z = 0 }
initialPingCounts           = 0
previousPingState           = false
previousLaunchState         = false
reorientOrbitDirection      = 1
targetUpdatedCounter        = math.huge

---@class Vector3
---@field x number
---@field y number
---@field z number

--- 3Dベクトル a から b を引きます (a - b)
---@param a Vector3 {x: number, y: number, z: number} または {number, number, number}
---@param b Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return Vector3 結果のベクトル {x, y, z}
function subtract(a, b)
    local ax = a.x or a[1] or 0
    local ay = a.y or a[2] or 0
    local az = a.z or a[3] or 0
    local bx = b.x or b[1] or 0
    local by = b.y or b[2] or 0
    local bz = b.z or b[3] or 0
    return { x = ax - bx, y = ay - by, z = az - bz }
end

--- 3Dベクトル v の大きさを計算します
---@param v Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return number ベクトルの大きさ
function vector_magnitude(v)
    local x = v.x or v[1] or 0
    local y = v.y or v[2] or 0
    local z = v.z or v[3] or 0
    return math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
end

--- 3Dベクトル v を正規化 (単位ベクトル化) します
---@param v Vector3 {x: number, y: number, z: number} または {number, number, number}
---@param isReturnVector? boolean
---@return Vector3 正規化されたベクトル {x, y, z}
function normalize(v, isReturnVector)
    local x = v.x or v[1] or 0
    local y = v.y or v[2] or 0
    local z = v.z or v[3] or 0
    local mag = math.sqrt(x ^ 2 + y ^ 2 + z ^ 2)
    if mag > 1e-9 then -- ゼロ除算を避ける
        if isReturnVector then
            return { x = x / mag, y = y / mag, z = z / mag }
        else
            return { x / mag, y / mag, z / mag }
        end
    else
        if isReturnVector then
            return { x = 0, y = 0, z = 1 }
        else
            return { 0, 0, 1 } -- ゼロベクトルの場合は前方 Z を返す (安全策)
        end
    end
end

--- 2つの3Dベクトルの外積 (クロス積) を計算します (a x b)
---@param a Vector3 {x: number, y: number, z: number} または {number, number, number}
---@param b Vector3 {x: number, y: number, z: number} または {number, number, number}
---@return Vector3 外積ベクトル {rx, ry, rz} (配列形式)
function cross_product(a, b)
    local ax = a.x or a[1] or 0
    local ay = a.y or a[2] or 0
    local az = a.z or a[3] or 0
    local bx = b.x or b[1] or 0
    local by = b.y or b[2] or 0
    local bz = b.z or b[3] or 0

    local rx = ay * bz - az * by
    local ry = az * bx - ax * bz
    local rz = ax * by - ay * bx

    return { rx, ry, rz }
end

--------------------------------------------------------------------------------
-- クォータニオン演算関数
--------------------------------------------------------------------------------

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

---3次元ベクトルをクォータニオンで回転させる関数（軽量化版）
---@param vector table {x, y, z} または {1, 2, 3} 形式の回転対象ベクトル
---@param quaternion table {w, x, y, z} (インデックス 1, 2, 3, 4) 形式のクォータニオン
---@param isInverse? boolean true を渡すと逆回転を実行（省略時は false/順回転）
---@return number[] @回転後の 3次元ベクトル {x, y, z}
function rotateVectorByQuaternion(vector, quaternion, isInverse)
    local w, x, y, z = quaternion[1], quaternion[2], quaternion[3], quaternion[4]
    local vx, vy, vz = vector[1] or vector.x, vector[2] or vector.y, vector[3] or vector.z

    -- 外積演算の共通部分（軽量化計算）
    local tx = 2 * (y * vz - z * vy)
    local ty = 2 * (z * vx - x * vz)
    local tz = 2 * (x * vy - y * vx)

    -- 逆回転なら w の符号を反転
    if isInverse then
        w = -w
    end

    return {
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx)
    }
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
    local localVec = rotateVectorByQuaternion(
        { relativeVecGlobal.x, relativeVecGlobal.y, relativeVecGlobal.z },
        objectOrientationQuat, true
    )

    -- 3. ローカル座標ベクトルを {x, y, z} 形式のテーブルとして返す
    return { x = localVec[1], y = localVec[2], z = localVec[3] }
end

---ローカル座標から方位角と仰角へ変換
---@param localPosVec {x:number,y:number,z:number}
---@return {azimuth:number,elevation:number}
function coordsToAngle(localPosVec)
    local horizontalDistance, currentLocalAzimuth, currentLocalElevation
    horizontalDistance = math.sqrt(localPosVec.x ^ 2 + localPosVec.z ^ 2)
    currentLocalAzimuth = math.atan(localPosVec.x, localPosVec.z)
    currentLocalElevation = math.atan(localPosVec.y, horizontalDistance)
    return {
        azimuth = currentLocalAzimuth,
        elevation = currentLocalElevation
    }
end

-- グローバル座標系での前フレームの正規化されたLOSベクトルを保存
-- {x, y, z} 形式で保存
oldLOS_vec_global_normalized = { x = 0, y = 0, z = 1 } -- 初期値 (例: 前方)

function getLocalGravity(quaternion)
    -- ワールド座標の重力方向
    -- World座標: Y下方向
    local gravityWorld = { 0, -1, 0 }

    -- World -> Local 変換
    local gravityLocal =
        rotateVectorByQuaternion(
            gravityWorld,
            quaternion, true
        )

    return {
        x = gravityLocal[1],
        y = gravityLocal[2],
        z = gravityLocal[3]
    }
end

function clamp(v, min, max)
    if v < min then
        return min
    elseif v > max then
        return max
    else
        return v
    end
end

--------------------------------------------------------------------------------
-- Local座標上の「この方向へ速度を曲げたい」というベクトルを
-- pitch/yawフィン補正へ変換
--
-- 重力補償・高度保持の双方で必ずこの関数を使用する
--------------------------------------------------------------------------------
function vectorToFinCorrection(correctionVector, velocityDir)
    --------------------------------------------------
    -- 速度方向成分を除去
    --------------------------------------------------

    local parallel =
        correctionVector.x * velocityDir.x +
        correctionVector.y * velocityDir.y +
        correctionVector.z * velocityDir.z

    local normal = {
        x = correctionVector.x -
            parallel * velocityDir.x,

        y = correctionVector.y -
            parallel * velocityDir.y,

        z = correctionVector.z -
            parallel * velocityDir.z
    }

    --------------------------------------------------
    -- 必要加速度方向 → 必要旋回軸
    --
    -- 実機テストで確認済みの外積順
    --------------------------------------------------

    local turnAxis =
        cross_product(
            normal,
            velocityDir
        )

    --------------------------------------------------
    -- 実機テストで確認済みのフィン符号
    --------------------------------------------------

    local pitchCorrection =
        turnAxis[1]

    local yawCorrection =
        -turnAxis[2]

    return pitchCorrection, yawCorrection
end

--------------------------------------------------------------------------------
-- 重力補償 + 高度保持
--
-- 戻り値:
-- pitchCorrection : 機体X軸周りの補正
-- yawCorrection   : 機体Y軸周りの補正
--
-- PN/単追尾のpitch/yawは既にロール補正済みなので、
-- この関数も機体座標系で補正量を返す。
--------------------------------------------------------------------------------

function getGravityAltitudeCorrection(
    ownCoordsVec,
    localVelocity,
    gravity,
    horizontalDistance
)
    local pitchCorrection = 0
    local yawCorrection = 0

    --------------------------------------------------
    -- Local Velocityから速度方向を作る
    -- 重力補償・高度保持の両方で使用
    --------------------------------------------------

    local speed =
        math.sqrt(
            localVelocity.x * localVelocity.x +
            localVelocity.y * localVelocity.y +
            localVelocity.z * localVelocity.z
        )

    local velocityDir = nil

    if speed > 0.1 then
        velocityDir = {
            x = localVelocity.x / speed,
            y = localVelocity.y / speed,
            z = localVelocity.z / speed
        }
    end


    --------------------------------------------------
    -- 重力補償
    --------------------------------------------------

    if GRAVITY_COMP_ENABLE and velocityDir then
        local gravityParallel =
            gravity.x * velocityDir.x +
            gravity.y * velocityDir.y +
            gravity.z * velocityDir.z

        local gravityNormal = {
            x = gravity.x - gravityParallel * velocityDir.x,
            y = gravity.y - gravityParallel * velocityDir.y,
            z = gravity.z - gravityParallel * velocityDir.z
        }
        local gravityCorrectionVector = {
            x = -gravityNormal.x * GRAVITY_COMP_GAIN,
            y = -gravityNormal.y * GRAVITY_COMP_GAIN,
            z = -gravityNormal.z * GRAVITY_COMP_GAIN
        }

        local gravityPitch, gravityYaw =
            vectorToFinCorrection(
                gravityCorrectionVector,
                velocityDir
            )

        pitchCorrection =
            pitchCorrection + gravityPitch

        yawCorrection =
            yawCorrection + gravityYaw
    end


    --------------------------------------------------
    -- 高度保持
    --------------------------------------------------

    if ALT_HOLD_ENABLE
        and velocityDir
        and horizontalDistance > ALT_HOLD_DISABLE_DISTANCE then
        local altitudeError =
            ALT_HOLD_TARGET - ownCoordsVec.y

        --------------------------------------------------
        -- World UpをLocal座標で表す
        --------------------------------------------------

        local worldUpLocal = {
            x = -gravity.x,
            y = -gravity.y,
            z = -gravity.z
        }

        --------------------------------------------------
        -- 世界座標基準の鉛直速度
        --------------------------------------------------

        local verticalRate =
            localVelocity.x * worldUpLocal.x +
            localVelocity.y * worldUpLocal.y +
            localVelocity.z * worldUpLocal.z

        --------------------------------------------------
        -- 高度PD
        --------------------------------------------------

        local altitudeCorrection =
            altitudeError * ALT_HOLD_KP -
            verticalRate * ALT_HOLD_KD

        altitudeCorrection =
            clamp(
                altitudeCorrection,
                -MAX_ALTITUDE_CORRECTION,
                MAX_ALTITUDE_CORRECTION
            )

        local altitudeVector = {
            x = worldUpLocal.x * altitudeCorrection,
            y = worldUpLocal.y * altitudeCorrection,
            z = worldUpLocal.z * altitudeCorrection
        }

        local altitudePitch, altitudeYaw =
            vectorToFinCorrection(
                altitudeVector,
                velocityDir
            )

        pitchCorrection =
            pitchCorrection + altitudePitch

        yawCorrection =
            yawCorrection + altitudeYaw
    end


    return pitchCorrection, yawCorrection
end

function onTick()
    local yawAngle, pitchAngle
    currentTick = currentTick + 1
    targetUpdatedCounter = targetUpdatedCounter + 1
    -- 目標座標
    -- X=0を通過する目標も扱えるよう、3軸のどれかが入力されていれば更新する
    local tx = input.getNumber(24)
    local ty = input.getNumber(25)
    local tz = input.getNumber(26)
    local isDetected = input.getBool(1)
    local trackingID = input.getNumber(12)
    if tx ~= 0 or ty ~= 0 or tz ~= 0 then
        initialTargetCoords = { tx, ty, tz }
        -- 中間誘導座標は終末誘導の捕捉基準として別に保持する。
        -- ソナー目標を捕捉した後は、残留したデータリンク入力で追尾座標を
        -- 上書きしない。
        if chosenViewTargetID == 0 then
            targetCoords = { tx, ty, tz }
        end
        -- debug.log("Target coordinates initialized: " .. tx .. ", " .. ty .. ", " .. tz)
    end

    local ownCoords = {
        input.getNumber(27),
        input.getNumber(28),
        input.getNumber(29)
    }
    local ownOrientation =
        eulerZYX_to_quaternion(
            input.getNumber(32),
            input.getNumber(31),
            input.getNumber(30)
        )

    -- Physics Sensor Local Velocity
    local selfLocalVelocity = {
        x = input.getNumber(19),
        y = input.getNumber(20),
        z = input.getNumber(21)
    }

    local ownCoordsVec = {
        x = ownCoords[1],
        y = ownCoords[2],
        z = ownCoords[3]
    }
    local isTargetFound = false
    if isDetected then
        local receivedTarget = {
            x = input.getNumber(1),
            y = input.getNumber(2),
            z = input.getNumber(3),

            vX = input.getNumber(4),
            vY = input.getNumber(5),
            vZ = input.getNumber(6),

            epsilon = input.getNumber(8),
            lastSeenTick = input.getNumber(10),
            detectionTickLag = input.getNumber(11),
            id = trackingID,
            receivedTick = currentTick
        }

        local selectedTargetVelocity = { x = 0, y = 0, z = 0 }

        if receivedTarget.id == chosenViewTargetID then
            isTargetFound = true -- データリンクのIDが生きている場合はフラグを立ててループを抜ける
            targetCoords = { receivedTarget.x, receivedTarget.y, receivedTarget.z }
            selectedTargetVelocity = { x = receivedTarget.vX, y = receivedTarget.vY, z = receivedTarget.vZ }
            velocityBuffer = {
                x = selectedTargetVelocity.x,
                y = selectedTargetVelocity.y,
                z = selectedTargetVelocity.z
            }
            targetUpdatedCounter = 0
            --[[             debug.log("Target updated ID: " ..
                chosenViewTargetID .. " Coordinates: " .. target.x .. ", " .. target.y .. ", " .. target.z) ]]
        end

        local found = false

        for i, target in ipairs(targetInfos) do
            if target.id == trackingID then
                targetInfos[i] = receivedTarget
                found = true
                break
            end
        end

        if not found then
            table.insert(targetInfos, receivedTarget)
        end
    end

    -- 目標が見つからなかった場合は速度で更新
    if not isTargetFound then
        targetCoords = {
            targetCoords[1] + velocityBuffer.x * DT,
            targetCoords[2] + velocityBuffer.y * DT,
            targetCoords[3] + velocityBuffer.z * DT
        }
    end

    local isLaunch = input.getNumber(23) == 1
    local isPing   = input.getNumber(22) == 1 -- ピンガーの発信信号

    if isLaunch then
        launchedCount = launchedCount + 1
    end

    -- ASROCから切り離された時点を初期走査の起点にする。
    -- 搭載中にソナー系が動作していても、そのPingやトラックは持ち込まない。
    if isLaunch and not previousLaunchState then
        initialPingCounts = 0
        targetInfos = {}
        chosenViewTargetID = 0
        velocityBuffer = { x = 0, y = 0, z = 0 }
        previousPingState = false
    end

    -- NewtonDistanceFinderのPing出力は、送信から待受終了までONのレベル信号。
    -- ONの各tickではなく立ち上がりだけを1回として数える。
    if isLaunch and isPing and not previousPingState then
        initialPingCounts = initialPingCounts + 1
    end
    local initialScanComplete =
        isLaunch and
        initialPingCounts >= MIN_INITIAL_PINGS and
        not isPing
    previousPingState = isPing
    previousLaunchState = isLaunch

    -- 距離に応じた次回Pingまでの待ち時間より短いT_LOSTが設定されても、
    -- 次の反射が返る前に選択中のトラックを消さない。
    local distanceBeforeSelection =
        vector_magnitude(subtract(targetCoords, ownCoords))
    local expectedTrackUpdateTicks
    if chosenViewTargetID == 0 then
        expectedTrackUpdateTicks =
            (distanceBeforeSelection + CLOSEST_DISTANCE_THRESHOLD) *
            2 / SOUND_SPEED_PER_TICK + 15
    else
        expectedTrackUpdateTicks =
            distanceBeforeSelection * 2.2 / SOUND_SPEED_PER_TICK + 15
    end
    local targetLostTimeoutTicks =
        math.max(TARGET_LOST_THRESHOLD_TICKS, expectedTrackUpdateTicks)

    --------------------------------------------------------------------------
    -- 消失ターゲット削除
    --------------------------------------------------------------------------
    for i = #targetInfos, 1, -1 do
        local target = targetInfos[i]
        -- KFから受け取った時点の観測遅延に、時分割出力後の経過tickを加える。
        local targetAge =
            target.detectionTickLag +
            currentTick - target.receivedTick

        if targetAge > targetLostTimeoutTicks then
            table.remove(targetInfos, i)
            -- debug.log("Target ID " .. target.id .. " removed due to loss of detection.")
        end
    end

    -- 選択された目標IDが生きているか調べる
    local selectedTargetExists = false

    if chosenViewTargetID ~= 0 then
        for _, target in ipairs(targetInfos) do
            if target.id == chosenViewTargetID then
                selectedTargetExists = true
                break
            end
        end
    end

    -- 選択中の目標を失った場合は、直前の探知物から別の探知物へ
    -- 即座に乗り換えない。中間誘導座標へ戻り、4回の新しいPingで再走査する。
    if chosenViewTargetID ~= 0 and not selectedTargetExists then
        chosenViewTargetID = 0
        initialPingCounts = 0
        targetInfos = {}
        targetCoords = {
            initialTargetCoords[1],
            initialTargetCoords[2],
            initialTargetCoords[3]
        }
        velocityBuffer = { x = 0, y = 0, z = 0 }
        reorientMode = false
        isTargetFound = false
        initialScanComplete = false
    end

    --------------------------------------------------------------------------
    -- 中間誘導されていた目標座標に最も近いターゲット選択
    --------------------------------------------------------------------------
    if initialScanComplete then
        -- まだ補足している目標がいない場合、最も近いターゲットを選択する
        local closestDistanceSq = math.huge
        local closestDistanceThresholdSq = CLOSEST_DISTANCE_THRESHOLD ^ 2
        if not isTargetFound and chosenViewTargetID == 0 then
            for _, target in ipairs(targetInfos) do
                local distanceDiffSq =
                    (target.x - initialTargetCoords[1]) ^ 2 +
                    (target.y - initialTargetCoords[2]) ^ 2 +
                    (target.z - initialTargetCoords[3]) ^ 2
                if distanceDiffSq < closestDistanceSq and
                    distanceDiffSq < closestDistanceThresholdSq then
                    closestDistanceSq = distanceDiffSq
                    chosenViewTargetID = target.id
                    isTargetFound = true
                    local selectedTargetVelocity = { x = target.vX, y = target.vY, z = target.vZ }
                    targetCoords = { target.x, target.y, target.z }
                    velocityBuffer = selectedTargetVelocity
                    targetUpdatedCounter = 0
                    --[[                         debug.log("Target found, Distance = : " ..
                            math.sqrt(closestDistanceSq) ..
                            ", ID = " ..
                            chosenViewTargetID .. " Coordinates: " .. target.x .. ", " .. target.y .. ", " ..
                            target.z) ]]
                end
            end
        end
    end

    local DISTANCE = vector_magnitude(subtract(targetCoords, ownCoords))
    local pingerIntervalTick = DISTANCE * 2.2 / SOUND_SPEED_PER_TICK + 5

    -- 捕捉前は探索半径の遠端からの反射まで待ち、周辺全体を取りこぼさない。
    if chosenViewTargetID == 0 then
        pingerIntervalTick =
            (DISTANCE + CLOSEST_DISTANCE_THRESHOLD) * 2 / SOUND_SPEED_PER_TICK
    end

    local targetCoordsVec       = {
        x = targetCoords[1],
        y = targetCoords[2],
        z = targetCoords[3]
    }
    local LOS_vec_global

    local activeTargetCoordsVec = targetCoordsVec

    local dx                    = targetCoordsVec.x - ownCoordsVec.x
    local dz                    = targetCoordsVec.z - ownCoordsVec.z
    local horizontalDist        = math.sqrt(dx * dx + dz * dz)

    -- 遠距離では低高度の中間点、300m未満では目標速度から作ったリード点を使用する
    -- 近距離でも目標座標そのものを狙わないため、追尾航法への遷移を防ぐ
    pnFinStrength               = PN_FIN_STRENGTH

    -- 中間誘導/終末誘導の切替時はLOS履歴をリセットし、切替スパイクを防ぐ
    local targetChanged         = chosenViewTargetID ~= 0 and chosenViewTargetID ~= oldChosenViewTargetID

    if targetChanged then
        oldLOS_vec_global_normalized =
            normalize(subtract(activeTargetCoordsVec, ownCoordsVec))
    end

    oldChosenViewTargetID = chosenViewTargetID
    local isPPN = false
    if DISTANCE > 1000 then
        isPPN = true
    end
    -- 3. グローバル座標系でのLOS (Line of Sight) ベクトルを計算
    LOS_vec_global = subtract(activeTargetCoordsVec, ownCoordsVec)
    if isLaunch then
        if launchedCount > 240 and not isPPN then
            if chosenViewTargetID ~= 0 then
                -- 現在の機首前方をグローバル座標へ変換
                local forwardGlobal =
                    rotateVectorByQuaternion({ 0, 0, 1 }, ownOrientation)

                -- 水平面だけで機首と目標の角度を求める
                -- 深度差は再指向判定に含めない
                local targetDX = targetCoordsVec.x - ownCoordsVec.x
                local targetDZ = targetCoordsVec.z - ownCoordsVec.z

                local forwardXZLength =
                    math.sqrt(
                        forwardGlobal[1] * forwardGlobal[1] +
                        forwardGlobal[3] * forwardGlobal[3]
                    )

                local targetXZLength =
                    math.sqrt(
                        targetDX * targetDX +
                        targetDZ * targetDZ
                    )

                local headingError = 0

                if forwardXZLength > 0.001 and targetXZLength > 0.001 then
                    local headingDot =
                        (
                            forwardGlobal[1] * targetDX +
                            forwardGlobal[3] * targetDZ
                        ) /
                        (forwardXZLength * targetXZLength)

                    headingDot = math.max(-1, math.min(1, headingDot))

                    headingError = math.acos(headingDot)
                end

                if reorientMode then
                    -- 十分に目標方向へ戻ったらPNへ復帰
                    if headingError < REORIENT_EXIT_ANGLE then
                        reorientMode = false

                        -- Recovery中のLOS差分をPNへ持ち込まない
                        oldLOS_vec_global_normalized =
                            normalize(LOS_vec_global)
                    end
                else
                    if headingError > REORIENT_ENTER_ANGLE then
                        reorientMode = true

                        ------------------------------------------------------------
                        -- Recovery開始時に旋回方向を一度だけ決定する
                        ------------------------------------------------------------

                        local rx = ownCoordsVec.x - targetCoordsVec.x
                        local rz = ownCoordsVec.z - targetCoordsVec.z

                        local radius =
                            math.sqrt(rx * rx + rz * rz)

                        if radius > 0.001 and forwardXZLength > 0.001 then
                            local radialX = rx / radius
                            local radialZ = rz / radius

                            local forwardX =
                                forwardGlobal[1] / forwardXZLength

                            local forwardZ =
                                forwardGlobal[3] / forwardXZLength

                            -- 時計回り接線
                            local tangentCWX = radialZ
                            local tangentCWZ = -radialX

                            -- 反時計回り接線
                            local tangentCCWX = -radialZ
                            local tangentCCWZ = radialX

                            -- 現在の機首方向との内積
                            local cwScore =
                                forwardX * tangentCWX +
                                forwardZ * tangentCWZ

                            local ccwScore =
                                forwardX * tangentCCWX +
                                forwardZ * tangentCCWZ

                            -- より少ない旋回で入れる方を選択
                            if cwScore >= ccwScore then
                                reorientOrbitDirection = 1
                            else
                                reorientOrbitDirection = -1
                            end
                        end

                        -- Recovery開始時点からLOS履歴を追従
                        oldLOS_vec_global_normalized =
                            normalize(LOS_vec_global)
                    end
                end

                -- オーバーシュートした際のリカバリーモード
                if reorientMode then
                    ----------------------------------------------------------------
                    -- 通過後回復モード
                    -- 目標を中心とする水平ベクトル場で切り返す
                    ----------------------------------------------------------------

                    local rx =
                        ownCoordsVec.x - targetCoordsVec.x

                    local rz =
                        ownCoordsVec.z - targetCoordsVec.z

                    local radiusNow =
                        math.sqrt(rx * rx + rz * rz)

                    ------------------------------------------------------------
                    -- ほぼ目標中心にいる場合
                    ------------------------------------------------------------
                    if radiusNow < 1 then
                        rx = forwardGlobal[1]
                        rz = forwardGlobal[3]

                        radiusNow =
                            math.sqrt(rx * rx + rz * rz)

                        if radiusNow < 0.001 then
                            rx = 0
                            rz = 1
                            radiusNow = 1
                        end
                    end

                    ------------------------------------------------------------
                    -- 目標から魚雷への半径方向
                    ------------------------------------------------------------
                    local radialX =
                        rx / radiusNow

                    local radialZ =
                        rz / radiusNow

                    ------------------------------------------------------------
                    -- Recovery開始時に選んだ方向の接線
                    ------------------------------------------------------------
                    local tangentX =
                        reorientOrbitDirection * radialZ

                    local tangentZ =
                        -reorientOrbitDirection * radialX

                    ------------------------------------------------------------
                    -- ORBIT_RADIUSへ収束させる
                    ------------------------------------------------------------
                    ------------------------------------------------------------
                    -- 目標が前方へ入るほど、周回から攻撃コースへ移行する
                    ------------------------------------------------------------

                    -- 120°以上:
                    --   ほぼ純粋な周回
                    --
                    -- 120° → 40°:
                    --   徐々に中心方向への切り込みを強くする
                    local frontFactor =
                        (RECOVERY_BOOST_START_ANGLE - headingError) /
                        (RECOVERY_BOOST_START_ANGLE - REORIENT_EXIT_ANGLE)

                    frontFactor =
                        math.max(0, math.min(1, frontFactor))

                    -- smoothstep
                    -- 境界で指令が急変しないようにする
                    frontFactor =
                        frontFactor * frontFactor *
                        (3 - 2 * frontFactor)

                    ------------------------------------------------------------
                    -- 周回半径維持
                    --
                    -- 目標が後ろにいる間は従来通りRECOVERY_RADIUSへ収束。
                    -- 前方に入るほど半径維持を弱める。
                    ------------------------------------------------------------
                    local radiusError =
                        (radiusNow - RECOVERY_RADIUS) /
                        RECOVERY_RADIUS

                    local radiusHold =
                        radiusError *
                        PPN_FIN_STRENGTH *
                        (1 - frontFactor)

                    radiusHold =
                        math.max(
                            -2,
                            math.min(2, radiusHold)
                        )

                    ------------------------------------------------------------
                    -- 前方へ入るほど目標中心への内向き成分を追加
                    ------------------------------------------------------------
                    local attackCorrection =
                        RECOVERY_ATTACK_GAIN *
                        frontFactor

                    local inwardCorrection =
                        radiusHold +
                        attackCorrection

                    ------------------------------------------------------------
                    -- 接線成分も前方ほど弱める
                    ------------------------------------------------------------
                    local tangentScale =
                        1 -
                        RECOVERY_TANGENT_REDUCTION *
                        frontFactor

                    ------------------------------------------------------------
                    -- 最終的なRecovery進行方向
                    ------------------------------------------------------------
                    local desiredX =
                        tangentX * tangentScale -
                        radialX * inwardCorrection

                    local desiredZ =
                        tangentZ * tangentScale -
                        radialZ * inwardCorrection

                    local desiredLength =
                        math.sqrt(
                            desiredX * desiredX +
                            desiredZ * desiredZ
                        )

                    if desiredLength > 0.001 then
                        desiredX =
                            desiredX / desiredLength

                        desiredZ =
                            desiredZ / desiredLength
                    else
                        desiredX = tangentX
                        desiredZ = tangentZ
                    end

                    ------------------------------------------------------------
                    -- 水平は旋回ベクトル
                    -- 垂直はALT_HOLD_TARGETを維持
                    ------------------------------------------------------------
                    local reorientTarget = {
                        x =
                            ownCoordsVec.x +
                            desiredX * ORBIT_LOOKAHEAD,

                        y = ALT_HOLD_TARGET,

                        z =
                            ownCoordsVec.z +
                            desiredZ * ORBIT_LOOKAHEAD
                    }

                    local targetLocalPosVec =
                        globalToLocal(
                            reorientTarget,
                            ownCoordsVec,
                            ownOrientation
                        )

                    local targetAngle =
                        coordsToAngle(targetLocalPosVec)

                    ------------------------------------------------------------
                    -- Radiusモードと同じ角度制御
                    ------------------------------------------------------------
                    local recoveryYawGain =
                        RECOVERY_YAW_BASE_GAIN +
                        RECOVERY_YAW_BOOST * frontFactor

                    local recoveryYawLimit =
                        MAX_FIN_COMMAND *
                        (
                            RECOVERY_BACK_LIMIT_RATIO +
                            (1 - RECOVERY_BACK_LIMIT_RATIO) * frontFactor
                        )

                    yawAngle =
                        targetAngle.azimuth *
                        recoveryYawGain

                    yawAngle =
                        math.max(
                            -recoveryYawLimit,
                            math.min(recoveryYawLimit, yawAngle)
                        )

                    pitchAngle =
                        targetAngle.elevation

                    ------------------------------------------------------------
                    -- PNへ戻った瞬間にLOS角速度が跳ねないよう追従
                    ------------------------------------------------------------
                    oldLOS_vec_global_normalized =
                        normalize(LOS_vec_global)
                    --[[                     debug.log(
                        "Recovery Orbit: " ..
                        headingError * 180 / PI
                    ) ]]
                else
                    -- debug.log("PN")
                    -- 4. グローバルLOSベクトルを正規化
                    local currentLOS_vec_global_normalized = normalize(LOS_vec_global)

                    -- 5. グローバル座標系でのLOS角速度ベクトル (omega) を計算
                    --    omega_vec = (v_old x v_current)
                    --    (v_old x v_current) の大きさは sin(theta)
                    --    thetaが小さい場合, sin(theta) ~= theta (ラジアン)
                    --    角速度 (rad/s) = theta / DT
                    local omega_LOS_global_vec             = cross_product(oldLOS_vec_global_normalized,
                        currentLOS_vec_global_normalized)

                    -- DTで割る (角速度ベクトルにする)
                    local omega_LOS_global                 = {
                        omega_LOS_global_vec[1] / DT,
                        omega_LOS_global_vec[2] / DT,
                        omega_LOS_global_vec[3] / DT
                    }

                    -- 6. グローバルなLOS角速度ベクトルを、自機のローカル座標系に変換
                    --    rotateVectorByInverseQuaternion は q_conj * p * q を行い、
                    --    グローバルベクトル p をローカル座標系に変換します。
                    --    p = omega_LOS_global (グローバルでのLOS回転)
                    --    q = ownOrientation (自機の姿勢)
                    --    結果 = omega_LOS_local (自機から見たLOS回転)
                    local omega_LOS_local                  = rotateVectorByQuaternion(omega_LOS_global,
                        ownOrientation, true)

                    -- 7. ローカルなLOS角速度の各成分を取得
                    --    自機の座標系 (X:右, Y:上, Z:前) と仮定
                    --    coordsToAngle の定義 atan(x, z) [方位], atan(y, horiz) [仰角] より:
                    --    - 方位角 (Yaw) の変化は、Y軸 (Up) 周りの回転
                    --    - 仰角 (Pitch) の変化は、X軸 (Right) 周りの回転

                    local los_rate_pitch                   = omega_LOS_local[1] -- X軸 (Right) 周りの角速度 (rad/s)
                    local los_rate_yaw                     = omega_LOS_local[2] -- Y軸 (Up)    周りの角速度 (rad/s)
                    -- local los_rate_roll  = omega_LOS_local[3] -- Z軸 (Fwd)   周りの角速度 (通常不要)

                    -- 8. 状態を更新 (次のフレームのために)
                    oldLOS_vec_global_normalized           = currentLOS_vec_global_normalized

                    -- 9. 誘導指令として使う
                    --    ご提示のコードの変数名に合わせる
                    yawAngle                               = los_rate_yaw * pnFinStrength
                    pitchAngle                             = -los_rate_pitch * pnFinStrength
                end

                -- 過大なフィン指令による発散を防ぐ
                local maxFin           = math.max(MAX_FIN_COMMAND, 0)
                local commandMagnitude =
                    math.sqrt(yawAngle * yawAngle + pitchAngle * pitchAngle)

                if commandMagnitude > maxFin and commandMagnitude > 0 then
                    local commandScale = maxFin / commandMagnitude
                    yawAngle = yawAngle * commandScale
                    pitchAngle = pitchAngle * commandScale
                end
            else
                ------------------------------------------------------------------------
                -- 目標座標を中心とする水平ベクトル場
                --
                -- 接線方向を基本とし、半径誤差に比例する半径方向成分を加える。
                -- 重要: この方向をLOS角速度へ変換せず、機体座標での方位誤差として
                --       直接フィンへ入力する。
                ------------------------------------------------------------------------
                local rx = ownCoordsVec.x - targetCoordsVec.x
                local rz = ownCoordsVec.z - targetCoordsVec.z
                local radiusNow = math.sqrt(rx * rx + rz * rz)

                if radiusNow < 1 then
                    -- 中心にほぼ重なった場合だけ、現在の機首方向から半径方向を決める
                    local forwardGlobal =
                        rotateVectorByQuaternion({ 0, 0, 1 }, ownOrientation)
                    rx = forwardGlobal[1]
                    rz = forwardGlobal[3]
                    radiusNow = math.sqrt(rx * rx + rz * rz)
                    if radiusNow < 0.001 then
                        rx, rz, radiusNow = 0, 1, 1
                    end
                end

                local radialX = rx / radiusNow
                local radialZ = rz / radiusNow

                -- +1:時計回り、-1:反時計回り
                local tangentX = ORBIT_DIRECTION * radialZ
                local tangentZ = -ORBIT_DIRECTION * radialX

                local transitionDistance = 500

                -- Orbit半径からどれだけ外側にいるか
                local outsideDistance =
                    math.max(radiusNow - ORBIT_RADIUS, 0)

                -- 0: Orbit半径上
                -- 1: 十分な遠距離
                local approachFactor =
                    math.max(0, math.min(1,
                        outsideDistance / transitionDistance
                    ))

                -- smoothstep
                approachFactor =
                    approachFactor * approachFactor *
                    (3 - 2 * approachFactor)

                -- 遠距離ほど内向き、Orbit半径付近ほど接線方向
                local tangentScale = 1 - approachFactor
                local inwardScale = approachFactor

                local desiredX =
                    tangentX * tangentScale -
                    radialX * inwardScale

                local desiredZ =
                    tangentZ * tangentScale -
                    radialZ * inwardScale

                --[[                 -- 外側では内向き、内側では外向き。
                -- 誤差を半径で正規化するため、中心位置は常に入力目標座標になる。
                local radiusError = (radiusNow - ORBIT_RADIUS) / ORBIT_RADIUS
                local radialCorrection =
                    math.max(-2, math.min(2, radiusError * PPN_FIN_STRENGTH))

                local desiredX = tangentX - radialX * radialCorrection
                local desiredZ = tangentZ - radialZ * radialCorrection
                local desiredLength = math.sqrt(desiredX * desiredX + desiredZ * desiredZ)

                if desiredLength < 0.001 then
                    desiredX, desiredZ = tangentX, tangentZ
                else
                    desiredX = desiredX / desiredLength
                    desiredZ = desiredZ / desiredLength
                end ]]

                ------------------------------------------------------------------------
                -- 水平誘導と深度誘導を分離
                --
                -- 水平: ベクトル場の進行方向へORBIT_LOOKAHEAD先
                -- 垂直: ALT_HOLD_TARGETを絶対Y座標として追従
                --
                -- 深度誤差が大きくても水平半径計算には一切混ぜない。
                ------------------------------------------------------------------------
                local activeOrbitCoordsVec = {
                    x = ownCoordsVec.x + desiredX * ORBIT_LOOKAHEAD,
                    y = ALT_HOLD_TARGET,
                    z = ownCoordsVec.z + desiredZ * ORBIT_LOOKAHEAD
                }

                local targetLocalPosVec =
                    globalToLocal(activeOrbitCoordsVec, ownCoordsVec, ownOrientation)
                local targetAngle = coordsToAngle(targetLocalPosVec)

                -- 方位・仰角の「角度誤差」を直接制御する。
                -- LOS角速度のみの比例航法は、周回軌道保持には適さない。
                yawAngle = targetAngle.azimuth
                pitchAngle = targetAngle.elevation
                --debug.log("Orbit")
            end
        else
            local targetLocalPosVec = globalToLocal(activeTargetCoordsVec, ownCoordsVec, ownOrientation)
            local targetAngle = coordsToAngle(targetLocalPosVec)
            yawAngle = targetAngle.azimuth * PPN_FIN_STRENGTH
            pitchAngle = targetAngle.elevation * PPN_FIN_STRENGTH
        end
    else
        yawAngle = 0
        pitchAngle = 0
    end

    -- 重力、高度補正適用
    local gravity =
        getLocalGravity(
            ownOrientation
        )

    local gravityPitchCorrection,
    gravityYawCorrection =
        getGravityAltitudeCorrection(
            ownCoordsVec,
            selfLocalVelocity,
            gravity,
            horizontalDist
        )

    pitchAngle =
        pitchAngle +
        gravityPitchCorrection


    yawAngle =
        yawAngle +
        gravityYawCorrection

    -- 再度フィン上限を適用
    if MAX_FIN_COMMAND > 0 then
        pitchAngle = math.max(-MAX_FIN_COMMAND, math.min(MAX_FIN_COMMAND, pitchAngle))
    end

    local fuse = false
    if launchedCount > 300 and isTargetFound and not IS_VTFUSE_ENABLED then
        fuse = true
    end

    if targetUpdatedCounter < 20 and DISTANCE < VT_FUSE_IMPACT_DIST then
        fuse = true
    end
    output.setBool(1, fuse)
    output.setNumber(1, yawAngle)
    output.setNumber(2, pitchAngle)
    output.setNumber(3, targetCoords[1])
    output.setNumber(4, targetCoords[2])
    output.setNumber(5, targetCoords[3])
    output.setNumber(6, pingerIntervalTick)

    -- booleanで直接ピンガーをリクエストすることもできる
    local isSecondPingerRequest = false
    if currentTick == pingerRequestedTick + math.floor(pingerIntervalTick / 2) then
        isSecondPingerRequest = true
    end

    local isPingerRequest = false
    if currentTick >= pingerRequestedTick + pingerIntervalTick then
        isPingerRequest = true
        pingerRequestedTick = currentTick
    end

    output.setBool(2, isPingerRequest)
    output.setBool(3, isSecondPingerRequest)
end
