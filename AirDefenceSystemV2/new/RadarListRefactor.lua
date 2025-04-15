--[[
RadarList.lua (リファクタリング版 v2)
機能:
- 2つの PackandFilter マイコンからの圧縮済み目標情報 (pack1, pack2) を入力として受け取る (それぞれ最大6目標、計12目標分)。
- 2系統 (最大12目標) の入力情報を、出力チャンネル数 (最大6目標分) に合わせて最適化し、kalmanfilter マイコンへ渡す。
- 目標数が最大出力数 (6) を超えた場合、超過フラグをチャンネル 32 に出力する。

入力 (コンポジット信号):
- 数値 1-12: PackandFilter 1 (例: レーダー1&3) からの出力 (ch 1-2: 目標1, ..., ch 11-12: 目標6)
- 数値 13-24: PackandFilter 2 (例: レーダー2&4) からの出力 (ch 13-14: 目標7, ..., ch 23-24: 目標12)
- (数値 32: 未使用)

出力 (コンポジット信号):
- 数値 1-12: 整理された目標情報 (pack1, pack2 のペア) を最大6目標分出力。
- 数値 32: 目標数超過フラグ (1: 超過, 0: 正常)

プロパティ:
- (なし) ※Rangeプロパティは不要になった
]]

-- Lua標準ライブラリ
local M = math
local table = table

-- Stormworks API ショートカット
local inputNumber = input.getNumber
local outputNumber = output.setNumber

-- 定数
local MAX_INPUT_TARGETS_PER_SOURCE = 6                       -- 各 PackandFilter からの最大目標数
local TOTAL_INPUT_TARGETS = MAX_INPUT_TARGETS_PER_SOURCE * 2 -- 全入力目標数
local MAX_OUTPUT_TARGETS = 6                                 -- kalmanfilter へ渡す最大目標数
local MAX_OUTPUT_CHANNELS = MAX_OUTPUT_TARGETS * 2           -- 出力チャンネル数 (pack1, pack2)

-- グローバル変数
local targetsFromSourceA = {} -- PackandFilter 1 からの目標リスト
local targetsFromSourceC = {} -- PackandFilter 2 からの目標リスト

--[[
-- unpackDistance 関数は不要になったため削除
]]

function onTick()
    -- 状態リセット
    targetsFromSourceA = {}
    targetsFromSourceC = {}
    local isOverLimit = 0

    -- 出力チャンネルクリア
    for i = 1, MAX_OUTPUT_CHANNELS do
        outputNumber(i, 0)
    end
    outputNumber(32, 0) -- 超過フラグクリア

    -- 入力ソースA (ch 1-12) から目標情報を読み込みリスト化
    for i = 1, MAX_INPUT_TARGETS_PER_SOURCE do
        local pack1 = inputNumber(i * 2 - 1)
        local pack2 = inputNumber(i * 2)
        if pack1 ~= 0 or pack2 ~= 0 then -- 目標情報があるか？
            -- 距離フィルターは削除されたため、そのままリストに追加
            table.insert(targetsFromSourceA, { p1 = pack1, p2 = pack2 })
        end
    end

    -- 入力ソースC (ch 13-24) から目標情報を読み込みリスト化
    for i = 1, MAX_INPUT_TARGETS_PER_SOURCE do
        local pack1 = inputNumber((i + MAX_INPUT_TARGETS_PER_SOURCE) * 2 - 1) -- ch 13, 15, ...
        local pack2 = inputNumber((i + MAX_INPUT_TARGETS_PER_SOURCE) * 2)     -- ch 14, 16, ...
        if pack1 ~= 0 or pack2 ~= 0 then
            -- 距離フィルターは削除されたため、そのままリストに追加
            table.insert(targetsFromSourceC, { p1 = pack1, p2 = pack2 })
        end
    end

    -- 合計目標数が最大出力数 (6) を超えるかチェック
    if #targetsFromSourceA + #targetsFromSourceC > MAX_OUTPUT_TARGETS then
        isOverLimit = 1
    end
    outputNumber(32, isOverLimit) -- 超過フラグを出力

    -- 目標を出力チャンネルに書き込む (元の優先順位ロジックを踏襲)
    local outputChannelIndex = 1

    local function writeTarget(target)
        if outputChannelIndex <= MAX_OUTPUT_CHANNELS then
            outputNumber(outputChannelIndex, target.p1)
            outputNumber(outputChannelIndex + 1, target.p2)
            outputChannelIndex = outputChannelIndex + 2
        end
    end

    -- Source A と Source C のどちらが多いかで優先順位を決める
    if #targetsFromSourceA >= #targetsFromSourceC then
        -- Source A を優先して書き込む
        for i = 1, #targetsFromSourceA do
            if outputChannelIndex <= MAX_OUTPUT_CHANNELS then
                writeTarget(targetsFromSourceA[i])
            else
                break
            end
        end
        -- 残りがあれば Source C を書き込む
        for i = 1, #targetsFromSourceC do
            if outputChannelIndex <= MAX_OUTPUT_CHANNELS then
                writeTarget(targetsFromSourceC[i])
            else
                break
            end
        end
    else
        -- Source C を優先して書き込む
        for i = 1, #targetsFromSourceC do
            if outputChannelIndex <= MAX_OUTPUT_CHANNELS then
                writeTarget(targetsFromSourceC[i])
            else
                break
            end
        end
        -- 残りがあれば Source A を書き込む
        for i = 1, #targetsFromSourceA do
            if outputChannelIndex <= MAX_OUTPUT_CHANNELS then
                writeTarget(targetsFromSourceA[i])
            else
                break
            end
        end
    end
end
