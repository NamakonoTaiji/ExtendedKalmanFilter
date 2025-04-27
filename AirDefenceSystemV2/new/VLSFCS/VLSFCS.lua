-- MissileFCS_v14_PostFireConflictCheck.lua

--[[
================================================================================
変更点:
- 目標選択は前がLocked/Openingになったら開始 (v13同様)。
- 競合チェックを追加: hatch_opening状態で発射条件を確認する「前」に、
  「前のFCS発射後20tick以内」かつ「共有バス入力で自分の担当IDが他と重複」
  していないかチェックし、重複ならsearchingに戻る。
- 通常発射待機条件を追加: 通常発射は、前のFCSが発射済み(Tracking)の場合のみ許可。
- 高速発射オプションは維持（こちらも前のFCS発射済みを暗黙の前提とする）。
- 共有バスの状態フラグ(21-23)を使用。発射信号(オンオフ1)は1Tick ON。
================================================================================
入力 (単一コンポジット):
- 数値 1-30: KF目標座標
- 数値 31: ミサイル信号強度
- オンオフ 11-20: 共有バス(入力) - 前のFCSが出力した担当フラグ
- オンオフ 21: 共有バス(入力) - 前のFCS Searching フラグ
- オンオフ 22: 共有バス(入力) - 前のFCS Locked/Opening フラグ
- オンオフ 23: 共有バス(入力) - 前のFCS Tracking フラグ
- オンオフ 24: 共有バス(入力) - 前のFCS Idle フラグ
- オンオフ 30: ハッチ開放判定フラグ
- オンオフ 31: 外部発射トリガー

出力 (単一コンポジット):
- オンオフ 1: 発射信号 (Fire) - 1Tick ON
- オンオフ 2: 状態表示フラグ (Searching以外 ON)
- オンオフ 3: VLSハッチ制御信号
- オンオフ 11-20: 共有バス(出力) - 目標担当フラグ (自分含む)
- オンオフ 21: このFCS Searching フラグ
- オンオフ 22: このFCS Locked/Opening フラグ
- オンオフ 23: このFCS Tracking フラグ
- オンオフ 24: このFCS Idle フラグ
- 数値 1-3: アンテナ 目標座標
================================================================================
]]
local inputNumber = input.getNumber
local inputBool = input.getBool
local outputNumber = output.setNumber
local outputBool = output.setBool
local propertyNumber = property.getNumber
local propertyBool = property.getBool
-- (ライブラリ、API、定数、プロパティはv13と同じ)
local MAX_TARGETS = 10
local SHARED_BUS_TARGET_OFFSET = 10
local SHARED_BUS_STATE_OFFSET = 20
local MISSILE_SIGNAL_THRESHOLD = 0.1
local FAST_FIRE_DELAY = propertyNumber("FastFireDelay")
local isFirstLauncher = propertyBool("IsFirstLauncher")
local isTriggerDelayed = false

-- グローバル変数
local assignedOutputId = nil
local lockOnStatus = "searching"
local targetInfo = {}
local targetCurrentPosBuffer = { x = 0, y = 0, z = 0 }
local targetOldPosBuffer = { x = 0, y = 0, z = 0 }
local targetVelocityBuffer = { vX = 0, vY = 0, vZ = 0 }
local currentTick = 0
local prevFcsFiredTick = -1 -- 前が Tracking になった Tick を記録
local isPrevFcsFired = false

function selectNewTarget(kfTargetsExist, assignedFlagsInput)
    for outputId = 1, MAX_TARGETS do
        -- kfTargetsExistに値があるとき
        if kfTargetsExist[outputId] then
            -- 前任のFCSが担当している場合はスキップ、誰も担当がいない場合はoutputIdを返す
            local isAssignedByOther = assignedFlagsInput[outputId] or false
            --debug.log("i: " .. tostring(outputId) .. " assignedFlagsInput: " .. tostring(assignedFlagsInput[outputId]))
            if not isAssignedByOther then
                return outputId
            end
        end
    end
    return nil
end

function checkAvailableTargetsExist(kfTargetsExist, assignedFlagsInput, selfAssignedId)
    for outputId = 1, MAX_TARGETS do
        if outputId ~= selfAssignedId then
            if kfTargetsExist[outputId] then
                local isAssigned = assignedFlagsInput[outputId] or false
                if not isAssigned then return true end
            end
        end
    end
    return false
end

function onTick()
    --debug.log("--------------start-----------------")
    currentTick = currentTick + 1

    -- 0. 出力クリア
    outputBool(1, false)
    outputBool(2, false)
    outputBool(3, false)
    outputNumber(1, 0)
    outputNumber(2, 0)
    outputNumber(3, 0)
    for i = 4, 30 do outputNumber(i, 0) end
    for i = 11, 20 + 3 do outputBool(i, false) end

    -- 1. 入力読み込み
    local missileSignal = inputNumber(31) -- 要確認
    local kfTargetsExist = {}
    local assignedFlagsInput = {}
    local assignedFlagsOutput = {}
    local isPrevFCSFiredCheck = inputBool(31)
    local prevFcsIsLockedOrOpening = inputBool(SHARED_BUS_STATE_OFFSET + 2)
    local prevFcsIsTracking = isFirstLauncher or inputBool(SHARED_BUS_STATE_OFFSET + 3)
    local prevFcsIsIdle = inputBool(SHARED_BUS_STATE_OFFSET + 4)
    local isHatchReady = inputBool(30)
    for outputId = 1, MAX_TARGETS do
        local baseChannel = (outputId - 1) * 3
        local x = inputNumber(baseChannel + 1)
        local y = inputNumber(baseChannel + 2)
        local z = inputNumber(baseChannel + 3)
        --debug.log("outputId: " .. outputId .. " x: " .. x .. " y: " .. y .. " z: " .. z)
        kfTargetsExist[outputId] = (x ~= 0 or y ~= 0 or z ~= 0)                       -- 防空システムから共有されている敵情報のチャンネルをキーとしてtrueを代入
        assignedFlagsInput[outputId] = inputBool(outputId + SHARED_BUS_TARGET_OFFSET) -- 前のFCSが出力した担当フラグをキーとしてtrueを代入
        assignedFlagsOutput[outputId] = assignedFlagsInput[outputId]                  -- 出力用担当フラグ
    end
    -- 前のFCS発射(Tracking)Tickを記録/リセット
    if isPrevFCSFiredCheck then
        isPrevFcsFired = true
    end
    if isPrevFcsFired and (prevFcsFiredTick <= FAST_FIRE_DELAY) then
        prevFcsFiredTick = prevFcsFiredTick + 1
        --debug.log("prevFcsFiredTick : " .. tostring(prevFcsFiredTick))
    end
    --debug.log(prevFcsFiredTick .. " ? " .. FAST_FIRE_DELAY)
    if prevFcsFiredTick >= FAST_FIRE_DELAY then
        isTriggerDelayed = true -- 前のFCSが発射してから遅延分時間が経過したか
    end
    -- 2. ミサイルリセット判定
    if lockOnStatus == "tracking" and missileSignal < MISSILE_SIGNAL_THRESHOLD then
        lockOnStatus = "idle"
        assignedOutputId = nil
        targetInfo = {}
        --prevFcsFiredTick = -1;
    end

    -- 3. 状態遷移
    if lockOnStatus == "searching" then
        -- 前のFCSがLocked/Opening または Trackingまたはidle、または自分が最初なら選択開始
        --debug.log("--- Tick: " .. currentTick .. " ---")
        --debug.log("FCS State: searching, trying to select target.")
        local existingTargetsStr = "kfTargetsExist: "
        for id = 1, MAX_TARGETS do
            if kfTargetsExist[id] then existingTargetsStr = existingTargetsStr .. id .. "=true, " end
        end
        --debug.log(existingTargetsStr)
        local assignedFlagsStr = "assignedFlagsInput: "
        for id = 1, MAX_TARGETS do
            if assignedFlagsInput[id] then assignedFlagsStr = assignedFlagsStr .. id .. "=true, " end
        end
        --debug.log(assignedFlagsStr)
        -- ★★★ 確認ここまで ▲▲▲ ★★★

        local selectedOutputId = selectNewTarget(kfTargetsExist, assignedFlagsInput)
        --debug.log("selectNewTarget returned: " .. tostring(selectedOutputId)) -- 戻り値も確認
        local canStartSelection = isFirstLauncher or prevFcsIsLockedOrOpening or prevFcsIsTracking or prevFcsIsIdle
        if canStartSelection then
            local selectedOutputId = selectNewTarget(kfTargetsExist, assignedFlagsInput)

            if selectedOutputId then
                assignedOutputId = selectedOutputId
                lockOnStatus = "locked"
                local baseChannel = (assignedOutputId - 1) * 3
                targetInfo = {
                    x = inputNumber(baseChannel + 1),
                    y = inputNumber(baseChannel + 2),
                    z = inputNumber(baseChannel + 3)
                }
                if not (targetInfo.x ~= 0 or targetInfo.y ~= 0 or targetInfo.z ~= 0) then
                    assignedOutputId = nil
                    targetInfo = {}
                    lockOnStatus = "searching"
                end
            end
        end
    elseif lockOnStatus == "locked" then
        -- locked状態では特に競合チェックはせず、すぐにhatch_openingへ

        if assignedOutputId and kfTargetsExist[assignedOutputId] then
            outputBool(3, true)
            lockOnStatus = "hatch_opening"
        else
            lockOnStatus = "searching"
            assignedOutputId = nil
            targetInfo = {}
            --prevFcsFiredTick = -1
        end
    elseif lockOnStatus == "hatch_opening" then
        if assignedOutputId and kfTargetsExist[assignedOutputId] then
            outputBool(3, true)
            -- ハッチ開放継続
            local baseChannel = (assignedOutputId - 1) * 3
            targetInfo = {
                x = inputNumber(baseChannel + 1),
                y = inputNumber(baseChannel + 2),
                z = inputNumber(baseChannel + 3)
            };
            if targetInfo.x ~= 0 or targetInfo.y ~= 0 or targetInfo.z ~= 0 then
                outputBool(2, true) -- 状態表示ON

                -- 発射前の競合最終チェック
                local conflictDetected = false
                -- 共有バス入力で、自分の担当IDが他の誰か(特に前)に担当されているか？
                if assignedFlagsInput[assignedOutputId] then
                    conflictDetected = true
                    lockOnStatus = "searching"
                    assignedOutputId = nil
                end

                -- 競合が検出されなかった場合のみ、発射条件を評価
                if not conflictDetected then
                    -- 発射条件
                    local canFire = false
                    if (isTriggerDelayed or isFirstLauncher) and isHatchReady then
                        canFire = true
                    end
                    --debug.log("triggerDelayed : " .. tostring(isTriggerDelayed))
                    -- 発射実行 (通常 または 高速)
                    if canFire and inputBool(assignedOutputId) then
                        --debug.log("Fire!")
                        outputBool(1, true)
                        -- 発射信号は1TickだけON
                        lockOnStatus = "tracking"
                    end
                else
                    --debug.log("conflictDetected")
                end

                -- データリンク (競合でロールバックしない場合)
                if not conflictDetected then
                    outputNumber(1, targetInfo.x)
                    outputNumber(2, targetInfo.y)
                    outputNumber(3, targetInfo.z)
                end
            else -- 目標座標が0になった
                lockOnStatus = "searching"
                assignedOutputId = nil
                --prevFcsFiredTick = -1;
            end
        else -- assignedOutputIdが存在しない or kfTargetsExistがfalse
            lockOnStatus = "searching"
            assignedOutputId = nil
            --prevFcsFiredTick = -1;
        end
    elseif lockOnStatus == "tracking" then
        outputBool(3, true)
        if assignedOutputId and kfTargetsExist[assignedOutputId] then
            local baseChannel = (assignedOutputId - 1) * 3
            targetInfo = {
                x = inputNumber(baseChannel + 1),
                y = inputNumber(baseChannel + 2),
                z = inputNumber(baseChannel + 3)
            }
            -- 記憶する現在地をデータリンクの座標で更新
            targetCurrentPosBuffer = targetInfo

            -- 1tick前の座標情報を初期化
            if targetOldPosBuffer.x == 0 then
                targetOldPosBuffer = targetCurrentPosBuffer
            end
            -- 目標速度を計算
            targetVelocityBuffer = {
                vX = targetCurrentPosBuffer.x - targetOldPosBuffer.x,
                vY = targetCurrentPosBuffer.y - targetOldPosBuffer.y,
                vZ = targetCurrentPosBuffer.z - targetOldPosBuffer.z
            }
            targetOldPosBuffer = targetCurrentPosBuffer

            outputBool(2, true)
            outputNumber(1, targetInfo.x)
            outputNumber(2, targetInfo.y)
            outputNumber(3, targetInfo.z)
        else
            targetCurrentPosBuffer = {
                x = targetCurrentPosBuffer.x + targetVelocityBuffer.vX,
                y = targetCurrentPosBuffer.y + targetVelocityBuffer.vY,
                z = targetCurrentPosBuffer.z + targetVelocityBuffer.vZ
            }
            outputBool(2, true)
            outputNumber(1, targetCurrentPosBuffer.x)
            outputNumber(2, targetCurrentPosBuffer.y)
            outputNumber(3, targetCurrentPosBuffer.z)
        end
    elseif lockOnStatus == "idle" then
        targetInfo = {}
        outputBool(2, true)
        outputNumber(1, 0)
        outputNumber(2, 0)
        outputNumber(3, 0)
    end
    --debug.log(tostring(lockOnStatus))
    -- 4. 出力
    -- 4.1 共有バス出力 (担当フラグ & 状態フラグ)
    if assignedOutputId then assignedFlagsOutput[assignedOutputId] = true end
    for outputId = 1, MAX_TARGETS do
        outputBool(outputId + SHARED_BUS_TARGET_OFFSET,
            assignedFlagsOutput[outputId] or false)
    end
    -- 状態フラグ出力
    outputBool(SHARED_BUS_STATE_OFFSET + 1, lockOnStatus == "searching")
    outputBool(SHARED_BUS_STATE_OFFSET + 2, lockOnStatus == "locked" or lockOnStatus == "hatch_opening")
    outputBool(SHARED_BUS_STATE_OFFSET + 3, lockOnStatus == "tracking")
    outputBool(SHARED_BUS_STATE_OFFSET + 4, lockOnStatus == "idle")

    -- 4.2 発射信号出力 (上で1TickだけONになる)

    -- 4.3 状態表示フラグ - searching 以外で ON
    local isActiveState = (lockOnStatus ~= "searching")
    outputBool(2, isActiveState)

    -- (データリンク、ハッチは処理済み)
end
