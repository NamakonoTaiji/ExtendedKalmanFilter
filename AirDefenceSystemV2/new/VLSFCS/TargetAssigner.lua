-- TargetAssigner.lua

--[[
================================================================================
Target Assigner マイコン
================================================================================
機能:
- KFから目標リスト(OutputIDベース固定ch)を入力。
- 全てのFCSから準備完了/アイドル状態を入力。
- 割り当て可能なFCSに、未割り当ての目標を割り当てる。
- 各FCSに、担当すべきOutputID、目標座標、ハッチ開準備指示を個別に出力。

入力 (コンポジット):
- Node 1 (From KF):
    - 数値 1-30: KFからの目標座標 (OutputID 1-10 対応)
- Node 2 (From FCS Group):
    - オンオフ 1-10: 各FCSからの準備完了/アイドル状態フラグ (FCS 1-10 対応)

出力 (コンポジット):
- Node 1 (To FCS 1):
    - 数値 1: 担当OutputID (0=なし)
    - 数値 2-4: 目標座標 (X, Y, Z)
    - オンオフ 1: ハッチ開準備指示
- Node 2 (To FCS 2):
    - (同上)
- ...
- Node 10 (To FCS 10):
    - (同上)
================================================================================
]]

-- API ショートカット
local inputNumber = input.getNumber
local inputBool = input.getBool
local outputNumber = output.setNumber
local outputBool = output.setBool
local propertyNumber = property.getNumber

-- 定数
local MAX_TARGETS = 10 -- KFおよびFCSの数と合わせる
local NUM_FCS = 10     -- 接続されているFCSの総数

-- グローバル変数 (割り当て状態を保持)
-- assignedTargetToFcs[outputId] = fcsIndex  (どの目標をどのFCSが担当しているか)
local assignedTargetToFcs = {}
-- assignedFcsToTarget[fcsIndex] = outputId (どのFCSがどの目標を担当しているか)
local assignedFcsToTarget = {}

function onTick()
    -- 1. 入力読み込み
    -- KFからの目標リスト読み込み & 存在する目標IDリスト作成
    local existingTargetIds = {} -- 現在KFが出力しているOutputIDのリスト
    local targetCoords = {}      -- 各OutputIDの座標を一時保存 (キー: OutputID)
    for outputId = 1, MAX_TARGETS do
        local baseChannel = (outputId - 1) * 3
        local x = inputNumber(baseChannel + 1, 1) -- Node 1から読み込み
        local y = inputNumber(baseChannel + 2, 1)
        local z = inputNumber(baseChannel + 3, 1)
        if x ~= 0 or y ~= 0 or z ~= 0 then
            table.insert(existingTargetIds, outputId)        -- 存在するIDをリストに追加
            targetCoords[outputId] = { x = x, y = y, z = z } -- 座標を保存
        end
    end

    -- FCSからの準備完了/アイドル状態読み込み
    local readyFcsIndices = {} -- 割り当て可能なFCSのインデックスリスト
    for fcsIndex = 1, NUM_FCS do
        -- Node 2のオンオフチャンネル 1 から NUM_FCS を読む想定
        if inputBool(fcsIndex, 2) then -- 準備完了/アイドルなら true
            table.insert(readyFcsIndices, fcsIndex)
        end
    end

    -- 2. 割り当て解除処理
    local targetsToUnassign = {} -- 解除するOutputIDのリスト
    -- 2.1 KFリストから消えた目標の割り当てを解除
    for outputId, fcsIndex in pairs(assignedTargetToFcs) do
        local exists = false
        for _, existingId in ipairs(existingTargetIds) do
            if outputId == existingId then
                exists = true
                break
            end
        end
        if not exists then
            table.insert(targetsToUnassign, outputId)
            -- debug.log("TA: Target OutputID "..outputId.." disappeared from KF. Unassigning from FCS "..fcsIndex)
        end
    end
    -- 2.2 アイドルに戻ったFCSの割り当てを解除 (ミサイル喪失など)
    for fcsIndex, outputId in pairs(assignedFcsToTarget) do
        local isReady = false
        for _, readyIndex in ipairs(readyFcsIndices) do
            if fcsIndex == readyIndex then
                isReady = true
                break
            end
        end
        if isReady and outputId ~= 0 then -- アイドル状態(ready=true)なのにまだ割り当てが残っている場合
            -- この目標がまだKFリストに存在するか確認 (消えていたら上の処理で解除されるはず)
            local targetStillExists = false
            for _, existingId in ipairs(existingTargetIds) do
                if outputId == existingId then
                    targetStillExists = true
                    break
                end
            end

            if targetStillExists then
                -- 目標は存在するがFCSがアイドルに戻った -> 解除リストに追加
                local alreadyInUnassign = false
                for _, idToUnassign in ipairs(targetsToUnassign) do
                    if idToUnassign == outputId then
                        alreadyInUnassign = true
                        break
                    end
                end
                if not alreadyInUnassign then table.insert(targetsToUnassign, outputId) end
                -- debug.log("TA: FCS "..fcsIndex.." became ready while assigned to OutputID "..outputId..". Unassigning.")
            end
            -- もし目標も消えていたら、どのみち解除される
        end
    end

    -- 2.3 割り当て解除実行
    for _, outputId in ipairs(targetsToUnassign) do
        local fcsIndex = assignedTargetToFcs[outputId]
        if fcsIndex then
            assignedFcsToTarget[fcsIndex] = nil -- FCS側の割り当て解除
        end
        assignedTargetToFcs[outputId] = nil     -- 目標側の割り当て解除
    end

    -- 3. 新規割り当て処理
    local availableTargetIds = {} -- まだ割り当てられていない目標IDのリスト
    for _, outputId in ipairs(existingTargetIds) do
        if not assignedTargetToFcs[outputId] then
            table.insert(availableTargetIds, outputId)
        end
    end
    -- OutputIDの昇順にソート (必須ではないが、若いIDを優先的に割り当てるため)
    table.sort(availableTargetIds)

    local availableFcsIndices = {} -- 割り当て可能で、まだ何も割り当てられていないFCSのリスト
    for _, fcsIndex in ipairs(readyFcsIndices) do
        if not assignedFcsToTarget[fcsIndex] then
            table.insert(availableFcsIndices, fcsIndex)
        end
    end
    -- FCS Index の昇順にソート (必須ではないが、若いFCSを優先的に使うため)
    table.sort(availableFcsIndices)

    -- マッチング実行 (シンプルな先頭からの割り当て)
    local numToAssign = math.min(#availableTargetIds, #availableFcsIndices)
    for i = 1, numToAssign do
        local targetIdToAssign = availableTargetIds[i]
        local fcsIndexToAssign = availableFcsIndices[i]
        -- 割り当て情報を記録
        assignedTargetToFcs[targetIdToAssign] = fcsIndexToAssign
        assignedFcsToTarget[fcsIndexToAssign] = targetIdToAssign
        -- debug.log("TA: Assigning Target OutputID "..targetIdToAssign.." to FCS "..fcsIndexToAssign)
    end

    -- 4. 各FCSへの出力
    for fcsIndex = 1, NUM_FCS do
        local targetOutputId = assignedFcsToTarget[fcsIndex] -- このFCSが担当する目標IDを取得
        local nodeIndex = fcsIndex                           -- 出力ノード番号とFCSインデックスを一致させる想定

        if targetOutputId then
            -- 担当目標ありの場合
            local coords = targetCoords[targetOutputId]
            outputNumber(1, targetOutputId, nodeIndex) -- 担当OutputID
            outputNumber(2, coords.x, nodeIndex)       -- Target X
            outputNumber(3, coords.y, nodeIndex)       -- Target Y
            outputNumber(4, coords.z, nodeIndex)       -- Target Z
            outputBool(1, true, nodeIndex)             -- ハッチ開準備指示 ON
        else
            -- 担当目標なしの場合
            outputNumber(1, 0, nodeIndex) -- OutputID = 0
            outputNumber(2, 0, nodeIndex) -- Coords = 0
            outputNumber(3, 0, nodeIndex)
            outputNumber(4, 0, nodeIndex)
            outputBool(1, false, nodeIndex) -- ハッチ開準備指示 OFF
        end
    end
end -- onTick End
