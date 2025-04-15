--[[
================================================================================
RadarList.lua (リファクタリング最終版 - 遅延出力対応)
================================================================================
機能:
- 2つの PackandFilter マイコンからの圧縮済み目標情報 (pack1, pack2) を入力として受け取る (それぞれ最大6目標、計12目標分)。
- 受け取った目標情報を、出力チャンネルの上限 (12ch = 6目標分) に合わせて整理して出力する。
- 目標数が出力上限を超えた場合、そのTickでは出力できる分だけを出力し、
  あふれた目標データは内部バッファに保存する。
- 次のTickでは、バッファに保存された あふれた目標データ を優先的に出力し、
  出力チャンネル 32 に 1 (遅延フラグ) をセットする。
- レーダー探知間隔が2tickである前提で、遅延出力を行うTickでは新しい入力はないと仮定する。

入力 (コンポジット信号):
- 数値 1-12: PackandFilter A (例: ID 0) からの出力 (ch 1-2: 目標1, ..., ch 11-12: 目標6)
- 数値 13-24: PackandFilter C (例: ID 2) からの出力 (ch 13-14: 目標7, ..., ch 23-24: 目標12)

出力 (コンポジット信号):
- 数値 1-12: 整理された目標情報 (pack1, pack2 のペア) を最大6目標分出力。
- 数値 32: 遅延出力フラグ (1: 今回の出力は1Tick前のあふれたデータ, 0: 通常データ)
================================================================================
]]

-- Lua標準ライブラリ
local M = math
local table = table

-- Stormworks API ショートカット
local inputNumber = input.getNumber
local outputNumber = output.setNumber

-- 定数
local MAX_INPUT_TARGETS_PER_SOURCE = 6             -- 各 PackandFilter からの最大目標数
local MAX_OUTPUT_TARGETS = 6                       -- このRadarListが出力できる最大目標数
local MAX_OUTPUT_CHANNELS = MAX_OUTPUT_TARGETS * 2 -- 出力チャンネル数 (pack1, pack2)

-- グローバル変数 (状態保持)
local overflowBuffer = {}          -- 前のTickであふれた目標データ {p1, p2} を保持するバッファ
local isOutputtingOverflow = false -- 現在、遅延データを出力中かを示すフラグ

function onTick()
    local currentOutputCount = 0 -- 今回出力した目標数
    local delayedFlag = 0        -- 今回出力する遅延フラグ (デフォルトは0)
    local targetsToOutput = {}   -- 今回出力する目標データのリスト

    -- 出力チャンネルクリア (毎Tickクリア)
    for i = 1, MAX_OUTPUT_CHANNELS do
        outputNumber(i, 0)
    end
    outputNumber(32, 0) -- 遅延フラグもクリア

    -- 1. 遅延データを出力するかどうかの判断
    if isOutputtingOverflow then
        -- 前Tickであふれたデータをバッファから取り出して出力リストへ
        targetsToOutput = overflowBuffer
        delayedFlag = 1              -- 遅延フラグを立てる
        overflowBuffer = {}          -- バッファをクリア
        isOutputtingOverflow = false -- 遅延出力フラグをリセット
        -- debug.log("RadarList: Outputting delayed data. Count:", #targetsToOutput)
    else
        -- 2. 通常のデータ処理 (遅延出力中でない場合)
        local targetsFromSourceA = {}
        local targetsFromSourceC = {}

        -- 入力ソースA (ch 1-12) からデータを読み込みリスト化
        for i = 1, MAX_INPUT_TARGETS_PER_SOURCE do
            local pack1 = inputNumber(i * 2 - 1)
            local pack2 = inputNumber(i * 2)
            if pack1 ~= 0 or pack2 ~= 0 then
                table.insert(targetsFromSourceA, { p1 = pack1, p2 = pack2 })
            end
        end

        -- 入力ソースC (ch 13-24) からデータを読み込みリスト化
        for i = 1, MAX_INPUT_TARGETS_PER_SOURCE do
            local pack1 = inputNumber((i + MAX_INPUT_TARGETS_PER_SOURCE) * 2 - 1)
            local pack2 = inputNumber((i + MAX_INPUT_TARGETS_PER_SOURCE) * 2)
            if pack1 ~= 0 or pack2 ~= 0 then
                table.insert(targetsFromSourceC, { p1 = pack1, p2 = pack2 })
            end
        end

        -- 結合して処理対象リストを作成 (元の優先順位ロジックは省略し、単純に結合)
        -- ※ 元の優先順位が必要ならここのロジックを修正
        local allTargets = {}
        for _, t in ipairs(targetsFromSourceA) do table.insert(allTargets, t) end
        for _, t in ipairs(targetsFromSourceC) do table.insert(allTargets, t) end

        -- 出力リストとオーバーフローバッファに振り分け
        for i = 1, #allTargets do
            if currentOutputCount < MAX_OUTPUT_TARGETS then
                table.insert(targetsToOutput, allTargets[i])
                currentOutputCount = currentOutputCount + 1
            else
                -- 出力上限を超えたらバッファへ
                table.insert(overflowBuffer, allTargets[i])
            end
        end

        -- もしバッファにデータが入ったら、次のTickで遅延出力フラグを立てる
        if #overflowBuffer > 0 then
            isOutputtingOverflow = true
            -- debug.log("RadarList: Data overflowed. Buffering", #overflowBuffer, "targets for next tick.")
        end
        -- 通常データなので delayedFlag は 0 のまま
    end

    -- 3. 決定したデータを出力
    outputNumber(32, delayedFlag) -- 遅延フラグを出力
    local outputChannelIndex = 1
    for i = 1, #targetsToOutput do
        if outputChannelIndex <= MAX_OUTPUT_CHANNELS then
            outputNumber(outputChannelIndex, targetsToOutput[i].p1)
            outputNumber(outputChannelIndex + 1, targetsToOutput[i].p2)
            outputChannelIndex = outputChannelIndex + 2
        end
    end
end
