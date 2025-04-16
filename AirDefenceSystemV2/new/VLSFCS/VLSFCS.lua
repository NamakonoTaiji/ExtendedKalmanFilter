-- Missile Fire Control System v2 (VLSFCS.lua)

--[[
================================================================================
変更点:
- KFからは敵対目標のみ入力される前提 (オンオフ1-10入力不要)。
- ミサイル信号強度(数値入力31)を監視し、途絶えたらリセット。
- デイジーチェーンの発射制御: 前のFCSがtracking中でない場合のみ発射可能。
- 状態フラグ(searching/locked/tracking)を共有バスで伝達 (オンオフ21-23)。
================================================================================
入力 (コンポジット):
- 数値 1-30: KFからの敵対目標座標 (X, Y, Z x 10)
- 数値 31: 担当ミサイル信号強度 (Missile Signal Strength)
- 数値 32: 自機 X 座標 (Own X)
- 数値 33: 自機 Y 座標 (Own Y)
- 数値 34: 自機 Z 座標 (Own Z)
- オンオフ 11-20: 共有バス(入力) - 他のFCS担当フラグ
- オンオフ 21: 共有バス(入力) - 前のFCS Searchingフラグ
- オンオフ 22: 共有バス(入力) - 前のFCS Lockedフラグ
- オンオフ 23: 共有バス(入力) - 前のFCS Trackingフラグ
- オンオフ 31: 発射トリガー (外部から)

出力 (コンポジット):
- オンオフ 1: ミサイルコネクタ 発射信号 (Fire)
- オンオフ 2: ロックオン中/追跡中フラグ (Locked/Tracking)
- オンオフ 11-20: 共有バス(出力) - 目標担当フラグ (自分含む)
- オンオフ 21: 共有バス(出力) - このFCS Searchingフラグ
- オンオフ 22: 共有バス(出力) - このFCS Lockedフラグ
- オンオフ 23: 共有バス(出力) - このFCS Trackingフラグ
- 数値 1: アンテナ 担当目標 X 座標 (Target X)
- 数値 2: アンテナ 担当目標 Y 座標 (Target Y)
- 数値 3: アンテナ 担当目標 Z 座標 (Target Z)
- 数値 31: アンテナ 送信周波数 (Frequency)
================================================================================
]]

-- Lua標準ライブラリ
local M = math
local T = table

-- Stormworks API ショートカット
local inputNumber = input.getNumber
local inputBool = input.getBool
local outputNumber = output.setNumber
local outputBool = output.setBool
local propertyNumber = property.getNumber

-- 定数
local MAX_TARGETS = 10
local SHARED_BUS_TARGET_OFFSET = 10  -- 目標担当フラグ ch 11-20
local SHARED_BUS_STATE_OFFSET = 20   -- 状態フラグ ch 21-23
local MISSILE_SIGNAL_THRESHOLD = 0.1 -- ミサイル信号喪失と判断する閾値 (要調整)

-- プロパティ
local fcsId = propertyNumber("FCS_ID") or 0
local frequency = propertyNumber("Frequency") or 0 -- 固定周波数指定を推奨

-- グローバル変数 (状態保持)
local assignedTargetIndex = nil
local lockOnStatus = "searching" -- searching, locked, tracking
local targetInfo = {}
local commsFrequency = frequency

-- 起動時処理
function onSpawn()
    if commsFrequency == 0 then
        commsFrequency = 10000 + fcsId -- 例: IDごとに異なるデフォルト周波数
        print("FCS " .. fcsId .. ": Warning: Frequency not set in properties, using default: " .. commsFrequency)
    end
    -- 状態初期化
    assignedTargetIndex = nil
    lockOnStatus = "searching"
    targetInfo = {}
end

onSpawn() -- スクリプトロード時に実行

-- 目標選択ロジック (最も近い未割り当て目標を選択)
function selectTarget(kfTargets, assignedFlagsInput, ownX, ownY, ownZ)
    local bestTargetIndex = nil
    local minDistanceSq = math.huge
    for i = 1, MAX_TARGETS do
        -- kfTargets[i] が存在するかどうかだけチェック (敵対判定はKF側で実施済み)
        if kfTargets[i] then
            local isAssignedByOther = assignedFlagsInput[i] or false
            if not isAssignedByOther then -- 他のFCSが担当していないか
                -- 自機からの距離を計算
                local dx = kfTargets[i].x - ownX
                local dy = kfTargets[i].y - ownY
                local dz = kfTargets[i].z - ownZ
                local distSq = dx * dx + dy * dy + dz * dz
                if distSq < minDistanceSq then
                    minDistanceSq = distSq
                    bestTargetIndex = i
                end
            end
        end
    end
    return bestTargetIndex -- 選択した目標Indexを返す (見つからなければ nil)
end

function onTick()
    -- 0. 出力クリア (共有バスフラグと周波数以外)
    outputBool(1, false)                                       -- Fire
    outputBool(2, false)                                       -- Locked/Tracking Flag
    outputNumber(1, 0); outputNumber(2, 0); outputNumber(3, 0) -- Target Coords
    outputNumber(31, commsFrequency)                           -- Frequency

    -- 1. 入力読み込み
    local missileSignal = inputNumber(31)
    local fireTrigger = inputBool(31)
    local ownX = inputNumber(32); local ownY = inputNumber(33); local ownZ = inputNumber(34);

    local kfTargets = {}
    local assignedFlagsInput = {}
    local assignedFlagsOutput = {} -- 出力用フラグを初期化
    -- 前のFCSの状態フラグ読み込み
    local prevFcsIsSearching = inputBool(SHARED_BUS_STATE_OFFSET + 1)
    local prevFcsIsLocked = inputBool(SHARED_BUS_STATE_OFFSET + 2)
    local prevFcsIsTracking = inputBool(SHARED_BUS_STATE_OFFSET + 3)

    -- KF目標データと共有バス担当フラグ(入力)読み込み & 担当フラグ(出力)準備
    for i = 1, MAX_TARGETS do
        local x = inputNumber(i * 3 - 2); local y = inputNumber(i * 3 - 1); local z = inputNumber(i * 3);
        -- 座標が0でない場合のみ目標が存在するとみなす
        if x ~= 0 or y ~= 0 or z ~= 0 then kfTargets[i] = { x = x, y = y, z = z } else kfTargets[i] = nil end
        -- 共有バスからの担当フラグ入力読み込み
        assignedFlagsInput[i] = inputBool(i + SHARED_BUS_TARGET_OFFSET)
        -- 出力用担当フラグに入力値をコピー (後で自分の担当を追加する)
        assignedFlagsOutput[i] = assignedFlagsInput[i]
    end

    -- 2. ミサイル状態監視とリセット判定
    -- ミサイル追跡中に信号が途絶えたらリセット
    local resetCondition = (lockOnStatus == "tracking" and missileSignal < MISSILE_SIGNAL_THRESHOLD)
    if resetCondition then
        -- debug.log("FCS "..fcsId..": Missile signal lost. Resetting to searching.")
        lockOnStatus = "searching"
        assignedTargetIndex = nil
        targetInfo = {}
        -- リセットした場合、このTickの以降の処理はスキップし、状態出力のみ行う
    else
        -- 3. 状態遷移 & 目標選択/更新 (ミサイル信号がある or 追跡中でない場合)
        if lockOnStatus == "searching" then
            local selectedIndex = selectTarget(kfTargets, assignedFlagsInput, ownX, ownY, ownZ)
            if selectedIndex then
                assignedTargetIndex = selectedIndex
                targetInfo = kfTargets[assignedTargetIndex]
                lockOnStatus = "locked"
                -- debug.log("FCS "..fcsId..": Locked Target "..assignedTargetIndex)
            end
        elseif lockOnStatus == "locked" then
            -- 担当目標がKFリストにまだ存在するか確認
            if assignedTargetIndex and kfTargets[assignedTargetIndex] then
                targetInfo = kfTargets[assignedTargetIndex] -- 最新情報に更新
                outputBool(2, true)                         -- Locked/Tracking フラグ ON

                -- 発射条件チェック: トリガーON かつ 前のFCSがTracking中でないか
                local canFire = fireTrigger and not prevFcsIsTracking
                if canFire then
                    outputBool(1, true) -- FIRE!
                    lockOnStatus = "tracking"
                    -- debug.log("FCS "..fcsId..": Firing at Target "..assignedTargetIndex)
                end
            else
                -- ロックオン中に目標を見失った場合 (KFリストから消えた)
                lockOnStatus = "searching"
                assignedTargetIndex = nil
                targetInfo = {}
                -- debug.log("FCS "..fcsId..": Locked target ".. (assignedTargetIndex or "N/A") .." lost.")
            end
        elseif lockOnStatus == "tracking" then
            -- 担当目標がKFリストにまだ存在するか確認 (データリンク更新のため)
            if assignedTargetIndex and kfTargets[assignedTargetIndex] then
                targetInfo = kfTargets[assignedTargetIndex] -- 座標情報を更新
                outputBool(2, true)                         -- Locked/Tracking フラグ ON
            else
                -- 追跡中にKFリストから目標が消えた場合
                -- ミサイルは飛び続けるので状態は tracking のまま、信号喪失を待つ
                -- ただしデータリンクは停止させるため targetInfo をクリア
                targetInfo = {}
                outputBool(2, true) -- 追跡中フラグは維持
                -- debug.log("FCS "..fcsId..": Tracking target ".. (assignedTargetIndex or "N/A") .." lost from KF list, missile continues.")
            end
        end
    end


    -- 4. 出力
    -- 4.1 共有バス出力 (担当フラグ & 状態フラグ)
    -- 担当フラグ: 自分の担当目標があればフラグを立てる
    if assignedTargetIndex then -- ミサイル喪失リセット直後はnilになっている
        assignedFlagsOutput[assignedTargetIndex] = true
    end
    -- 共有バスに担当フラグ(入力+自分)を出力
    for i = 1, MAX_TARGETS do outputBool(i + SHARED_BUS_TARGET_OFFSET, assignedFlagsOutput[i] or false) end

    -- 状態フラグ: 現在の lockOnStatus に基づいて出力
    outputBool(SHARED_BUS_STATE_OFFSET + 1, lockOnStatus == "searching")
    outputBool(SHARED_BUS_STATE_OFFSET + 2, lockOnStatus == "locked")
    outputBool(SHARED_BUS_STATE_OFFSET + 3, lockOnStatus == "tracking")

    -- 4.2 データリンク出力 (座標) & ロックオン/追跡フラグ
    -- ロックオン中または追跡中、かつ目標情報が存在する場合にデータリンク情報を送信
    if (lockOnStatus == "locked" or lockOnStatus == "tracking") and targetInfo.x then
        outputNumber(1, targetInfo.x)
        outputNumber(2, targetInfo.y)
        outputNumber(3, targetInfo.z)
        outputBool(2, true)  -- Locked/Tracking フラグ ON
    else
        outputBool(2, false) -- Locked/Tracking フラグ OFF
    end

    -- 4.3 周波数出力 (常に送信)
    outputNumber(31, commsFrequency)
end
