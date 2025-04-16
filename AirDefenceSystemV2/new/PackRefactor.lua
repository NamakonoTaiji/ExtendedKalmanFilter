-- MissileFCS_v7_FixedChannel.lua

--[[
================================================================================
前提:
- KF側が敵対目標に Output ID (1からMAX_TARGETS) を割り当て、再利用する。
- KF側が Output ID に基づく固定チャンネルに座標を出力する。
変更点:
- データアソシエーションロジックを削除。
- 担当目標は Output ID (1-9 or 10) で管理。
- 目標選択は、KF出力チャンネルを直接見て、最初の未担当Output IDを探す。
- 自機座標入力は不要。
================================================================================
入力 (コンポジット):
- 数値 1-27 (or 30): KFからの敵対目標座標 (X, Y, Z x 9 or 10、OutputIDベース固定ch)
- 数値 31: 担当ミサイル信号強度
- オンオフ 11-19 (or 20): 共有バス(入力) - 他FCS担当フラグ (OutputID 1-9 or 10)
- オンオフ 21-23: 共有バス(入力) - 前のFCS状態フラグ
- オンオフ 31: 発射トリガー

出力 (コンポジット):
- オンオフ 1: 発射信号
- オンオフ 2: Locked/Tracking フラグ
- オンオフ 11-19 (or 20): 共有バス(出力) - 目標担当フラグ (OutputID 1-9 or 10)
- オンオフ 21-23: 共有バス(出力) - このFCS状態フラグ
- 数値 1-3: アンテナ 目標座標
- 数値 31: アンテナ 送信周波数
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
local MAX_TARGETS = propertyNumber("MaxTargets") or 9 -- KF側の設定と合わせる
local SHARED_BUS_TARGET_OFFSET = 10                   -- ch 11 + OutputID
local SHARED_BUS_STATE_OFFSET = 20
local MISSILE_SIGNAL_THRESHOLD = 0.1

-- プロパティ
local fcsId = propertyNumber("FCS_ID") or 0
local frequency = propertyNumber("Frequency") or 0

-- グローバル変数
local assignedOutputId = nil     -- ★担当目標の Output ID (1-9) を保持
local lockOnStatus = "searching" -- searching, locked, tracking
local targetInfo = {}            -- 担当目標の最新座標
-- lastLockedTargetCoords と findAssociatedTarget 関数は不要になる
local commsFrequency = frequency

-- 起動時処理
function onSpawn()
    if commsFrequency == 0 then
        commsFrequency = 10000 + fcsId
        print("FCS " .. fcsId .. ": Warning: Frequency not set, using default: " .. commsFrequency)
    end
    assignedOutputId = nil
    lockOnStatus = "searching"
    targetInfo = {}
end

onSpawn()

-- ★目標選択: 最初の未担当 Output ID (1-MAX_TARGETS) を返す
-- kfTargetsExist: キーがOutputID, 値が存在(true)/非存在(false)のテーブル
-- assignedFlagsInput: キーがOutputID, 値が担当済み(true)/未担当(false)のテーブル
function selectNewTarget(kfTargetsExist, assignedFlagsInput)
    for outputId = 1, MAX_TARGETS do -- Output ID は 1 から始まる
        -- そのIDに対応する目標がKFから出力されているか(座標が0でないか)確認
        if kfTargetsExist[outputId] then
            -- 他のFCSが担当していないか確認
            local isAssignedByOther = assignedFlagsInput[outputId] or false
            if not isAssignedByOther then
                -- 未担当の最初の Output ID を返す
                return outputId
            end
        end
    end
    -- 未担当の目標が見つからなかった場合
    return nil
end

function onTick()
    -- 0. 出力クリア
    outputBool(1, false); outputBool(2, false);
    outputNumber(1, 0); outputNumber(2, 0); outputNumber(3, 0);
    outputNumber(31, commsFrequency);

    -- 1. 入力読み込み (自機座標は不要)
    local missileSignal = inputNumber(31)
    local fireTrigger = inputBool(31)

    local kfTargetsExist = {}      -- キー: OutputID (1-MAX_TARGETS), 値: boolean
    local assignedFlagsInput = {}  -- キー: OutputID (1-MAX_TARGETS)
    local assignedFlagsOutput = {} -- キー: OutputID (1-MAX_TARGETS)
    local prevFcsIsSearching = inputBool(SHARED_BUS_STATE_OFFSET + 1)
    local prevFcsIsLocked = inputBool(SHARED_BUS_STATE_OFFSET + 2)

    -- KF目標データの存在確認 & 共有バス担当フラグ読み込み
    for outputId = 1, MAX_TARGETS do
        local baseChannel = (outputId - 1) * 3
        local x = inputNumber(baseChannel + 1); local y = inputNumber(baseChannel + 2); local z = inputNumber(
            baseChannel + 3);
        -- 座標が0でなければ目標が存在すると判断
        kfTargetsExist[outputId] = (x ~= 0 or y ~= 0 or z ~= 0)
        -- 担当フラグ読み込み (キーは OutputID)
        assignedFlagsInput[outputId] = inputBool(outputId + SHARED_BUS_TARGET_OFFSET)
        assignedFlagsOutput[outputId] = assignedFlagsInput[outputId] -- 出力用にコピー
    end

    -- 2. ミサイル状態監視とリセット判定
    local resetCondition = (lockOnStatus == "tracking" and missileSignal < MISSILE_SIGNAL_THRESHOLD)
    if resetCondition then
        lockOnStatus = "searching"; assignedOutputId = nil; targetInfo = {};
    else
        -- 3. 状態遷移 & 目標選択/更新
        if lockOnStatus == "searching" then
            local selectedOutputId = selectNewTarget(kfTargetsExist, assignedFlagsInput)
            if selectedOutputId then
                assignedOutputId = selectedOutputId -- ★Output ID を保持
                -- 対応するチャンネルから座標を読み込む
                local baseChannel = (assignedOutputId - 1) * 3
                targetInfo = {
                    x = inputNumber(baseChannel + 1),
                    y = inputNumber(baseChannel + 2),
                    z = inputNumber(
                        baseChannel + 3)
                }
                -- 念のため座標が有効か確認
                if targetInfo.x ~= 0 or targetInfo.y ~= 0 or targetInfo.z ~= 0 then
                    lockOnStatus = "locked"
                else
                    assignedOutputId = nil; targetInfo = {}; -- 読み込み時に0だったらリセット
                end
            end
        elseif lockOnStatus == "locked" then
            -- 担当している Output ID の目標がまだ存在するか確認
            if assignedOutputId and kfTargetsExist[assignedOutputId] then
                -- 対応するチャンネルから最新の座標を読み込む
                local baseChannel = (assignedOutputId - 1) * 3
                targetInfo = {
                    x = inputNumber(baseChannel + 1),
                    y = inputNumber(baseChannel + 2),
                    z = inputNumber(
                        baseChannel + 3)
                }
                -- 座標が有効ならロックオン維持＆発射判定
                if targetInfo.x ~= 0 or targetInfo.y ~= 0 or targetInfo.z ~= 0 then
                    outputBool(2, true) -- Locked/Tracking フラグ ON
                    local prevFcsIsNotReady = prevFcsIsSearching or prevFcsIsLocked
                    local canFire = fireTrigger and not prevFcsIsNotReady
                    if canFire then
                        outputBool(1, true); lockOnStatus = "tracking";
                    end
                else
                    -- ロック中に座標が0になった
                    lockOnStatus = "searching"; assignedOutputId = nil; targetInfo = {};
                end
            else -- 担当していた Output ID の目標が存在しなくなった
                lockOnStatus = "searching"; assignedOutputId = nil; targetInfo = {};
            end
        elseif lockOnStatus == "tracking" then
            -- 担当している Output ID の目標がまだ存在するか確認
            if assignedOutputId and kfTargetsExist[assignedOutputId] then
                -- 対応するチャンネルから最新の座標を読み込む (データリンク用)
                local baseChannel = (assignedOutputId - 1) * 3
                targetInfo = {
                    x = inputNumber(baseChannel + 1),
                    y = inputNumber(baseChannel + 2),
                    z = inputNumber(
                        baseChannel + 3)
                }
                outputBool(2, true) -- Tracking フラグ ON
                -- 座標が0になったらデータリンク停止
                if targetInfo.x == 0 and targetInfo.y == 0 and targetInfo.z == 0 then
                    targetInfo = {}
                end
            else                                      -- 追跡中に目標ロスト (KF出力が0になった)
                targetInfo = {}; outputBool(2, true); -- ミサイル信号喪失待ち
            end
        end
    end

    -- 4. 出力
    -- 4.1 共有バス出力 (担当フラグ & 状態フラグ)
    if assignedOutputId then
        assignedFlagsOutput[assignedOutputId] = true -- キーは OutputID
    end
    for outputId = 1, MAX_TARGETS do                 -- キーは OutputID
        outputBool(outputId + SHARED_BUS_TARGET_OFFSET, assignedFlagsOutput[outputId] or false)
    end
    -- 状態フラグ出力
    outputBool(SHARED_BUS_STATE_OFFSET + 1, lockOnStatus == "searching")
    outputBool(SHARED_BUS_STATE_OFFSET + 2, lockOnStatus == "locked")
    outputBool(SHARED_BUS_STATE_OFFSET + 3, lockOnStatus == "tracking")

    -- 4.2 データリンク出力 & Locked/Tracking フラグ
    if (lockOnStatus == "locked" or lockOnStatus == "tracking") and targetInfo.x then
        outputNumber(1, targetInfo.x); outputNumber(2, targetInfo.y); outputNumber(3, targetInfo.z);
        outputBool(2, true)
    else
        outputBool(2, false)
    end

    -- 4.3 周波数出力
    outputNumber(31, commsFrequency)
end
