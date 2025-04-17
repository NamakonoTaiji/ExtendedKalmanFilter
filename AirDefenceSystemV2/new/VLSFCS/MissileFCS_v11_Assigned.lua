-- MissileFCS_v11_Assigned.lua

--[[
================================================================================
Fire Control System (Target Assigner 対応版)
================================================================================
機能:
- Target Assigner (TA) から担当目標情報 (OutputID, 座標) とハッチ準備指示を入力。
- 指示された目標に対して、ハッチ開放 -> 準備待機 -> 発射 -> 追跡 を行う。
- ミサイル信号喪失でアイドル状態に戻る。
- TAに自身の準備完了/アイドル状態を報告。

入力 (コンポジット):
- Node 1 (From Target Assigner):
    - 数値 1: 担当OutputID (0=なし)
    - 数値 2: 目標 X 座標
    - 数値 3: 目標 Y 座標
    - 数値 4: 目標 Z 座標
    - オンオフ 1: ハッチ開準備指示
- Node 2 (Other Inputs):
    - 数値 31: 担当ミサイル信号強度 (例)
    - オンオフ 31: 外部発射トリガー (例)

出力 (コンポジット):
- Node 1 (To Target Assigner):
    - オンオフ 1: ★準備完了/アイドル状態報告フラグ (true=割り当て可)
- Node 2 (To Missile Connector):
    - オンオフ 1: 発射信号 (Fire) - 1TickのみON
- Node 3 (To Antenna):
    - 数値 1: 目標 X 座標 (データリンク)
    - 数値 2: 目標 Y 座標 (データリンク)
    - 数値 3: 目標 Z 座標 (データリンク)
- Node 4 (Misc Outputs):
    - オンオフ 2: 状態表示フラグ (Idle以外でON)
    - オンオフ 3: VLSハッチ制御信号 (Open=true / Close=false)
================================================================================
]]

-- API ショートカット
local inputNumber = input.getNumber
local inputBool = input.getBool
local outputNumber = output.setNumber
local outputBool = output.setBool
local propertyNumber = property.getNumber

-- 定数
local MISSILE_SIGNAL_THRESHOLD = 0.1
local HATCH_OPEN_DELAY_TICKS = propertyNumber("HatchDelay") or 60

-- プロパティ (デバッグ用など)
local fcsId_prop = propertyNumber("FCS_ID") or 0

-- グローバル変数
local assignedOutputId = 0  -- TAから指示されたOutput ID (0は担当なし)
local lockOnStatus = "idle" -- idle, preparing, ready_to_fire, tracking
local targetInfo = {}       -- TAから指示された目標座標
local hatchOpenStartTime = 0
local currentTick = 0
local hasFired = false -- このFCSが発射したかどうか

-- 起動時処理
function onSpawn()
    assignedOutputId = 0
    lockOnStatus = "idle"
    targetInfo = {}
    hatchOpenStartTime = 0
    currentTick = 0
    hasFired = false
end

onSpawn()

function onTick()
    currentTick = currentTick + 1

    -- 0. 出力クリア (一部を除く)
    outputBool(1, false, 2)                                             -- 発射信号OFF (Node 2, Ch 1)
    outputBool(2, false, 4)                                             -- 状態表示OFF (Node 4, Ch 2)
    outputBool(3, false, 4)                                             -- ハッチ閉 (Node 4, Ch 3)
    outputNumber(1, 0, 3); outputNumber(2, 0, 3); outputNumber(3, 0, 3) -- データリンククリア (Node 3)
    -- TAへの準備完了フラグ(Node 1, Ch 1)は状態に応じて設定

    -- 1. 入力読み込み
    -- TAからの指示 (Node 1)
    local instructedOutputId = inputNumber(1, 1)
    local instructedX = inputNumber(2, 1)
    local instructedY = inputNumber(3, 1)
    local instructedZ = inputNumber(4, 1)
    local hatchPrepareSignal = inputBool(1, 1)
    -- その他入力 (Node 2)
    local missileSignal = inputNumber(31, 2)
    local fireTrigger = inputBool(31, 2)

    -- 2. ミサイル状態監視とリセット判定 -> idle 状態へ
    if lockOnStatus == "tracking" and missileSignal < MISSILE_SIGNAL_THRESHOLD then
        lockOnStatus = "idle" -- アイドル状態へ
        assignedOutputId = 0  -- 担当解除
        targetInfo = {}
        hatchOpenStartTime = 0
        hasFired = false -- ★発射フラグもリセット？ -> いや、使い捨てなら維持しない？ TAが判断する
        -- -> TAがアイドル状態を見て割り当て解除するのでFCS側はリセットでOK
        -- debug.log("FCS "..fcsId_prop..": Missile lost. Returning to idle.")
    end

    -- 3. 状態遷移ロジック

    if lockOnStatus == "idle" then
        -- TAに「準備完了」を報告
        outputBool(1, true, 1) -- Node 1, Ch 1
        -- TAから新しい指示が来たか確認
        if instructedOutputId > 0 and hatchPrepareSignal then
            assignedOutputId = instructedOutputId                              -- 担当IDを設定
            targetInfo = { x = instructedX, y = instructedY, z = instructedZ } -- 座標を設定
            lockOnStatus = "preparing"                                         -- 準備状態へ遷移
            -- debug.log("FCS "..fcsId_prop..": Received assignment OutputID: "..assignedOutputId..". Preparing.")
        end
        -- アイドル中は他の出力はOFF (クリア処理で実施済み)
    elseif lockOnStatus == "preparing" then
        -- TAに「準備中(完了でない)」を報告
        outputBool(1, false, 1)
        -- TAから指示解除されたか確認
        if instructedOutputId == 0 then
            lockOnStatus = "idle"; assignedOutputId = 0; targetInfo = {}; hatchOpenStartTime = 0;
            -- debug.log("FCS "..fcsId_prop..": Assignment cancelled. Returning to idle.")
        else
            -- 指示継続中
            outputBool(3, true, 4) -- ハッチを開ける信号を出す (Node 4, Ch 3)
            outputBool(2, true, 4) -- 状態表示ON (Node 4, Ch 2)
            -- タイマー開始 (初回のみ)
            if hatchOpenStartTime == 0 then
                hatchOpenStartTime = currentTick
            end
            -- ハッチ開放時間をチェック
            local hatchOpenElapsed = currentTick - hatchOpenStartTime
            if hatchOpenElapsed >= HATCH_OPEN_DELAY_TICKS then
                lockOnStatus = "ready_to_fire" -- 準備完了状態へ
                -- debug.log("FCS "..fcsId_prop..": Hatch open complete. Ready to fire.")
            end
            -- この状態ではまだデータリンクしない
        end
    elseif lockOnStatus == "ready_to_fire" then
        -- TAに「準備中(完了でない)」を報告
        outputBool(1, false, 1)
        -- TAから指示解除されたか確認
        if instructedOutputId == 0 then
            lockOnStatus = "idle"; assignedOutputId = 0; targetInfo = {}; hatchOpenStartTime = 0;
            -- debug.log("FCS "..fcsId_prop..": Assignment cancelled. Returning to idle.")
        else
            -- 指示継続中
            outputBool(3, true, 4) -- ハッチは開けたまま
            outputBool(2, true, 4) -- 状態表示ON
            -- 最新の座標をTAから受け取る (データリンク用、必要なら)
            -- targetInfo = {x=instructedX, y=instructedY, z=instructedZ} -- TAが送り続けている前提

            -- 発射トリガーを待つ
            if fireTrigger then
                outputBool(1, true, 2) -- 発射信号ON (Node 2, Ch 1)
                hasFired = true        -- 発射済みフラグ (内部管理用、TAには報告しない)
                lockOnStatus = "tracking"
                -- debug.log("FCS "..fcsId_prop..": Firing at OutputID: "..assignedOutputId)
            end
            -- 発射準備完了したらデータリンクを開始しても良いかも？
            if assignedOutputId > 0 and (instructedX ~= 0 or instructedY ~= 0 or instructedZ ~= 0) then
                outputNumber(1, instructedX, 3); outputNumber(2, instructedY, 3); outputNumber(3, instructedZ, 3)
            end
        end
    elseif lockOnStatus == "tracking" then
        -- TAに「準備中(完了でない)」を報告
        outputBool(1, false, 1)
        -- (リセット判定は Tick 先頭で行われている)
        -- TAから指示解除されることは通常ないはずだが、念のためチェック
        if instructedOutputId == 0 then
            lockOnStatus = "idle"; assignedOutputId = 0; targetInfo = {}; hatchOpenStartTime = 0; hasFired = false;
            -- debug.log("FCS "..fcsId_prop..": Assignment cancelled during tracking? Returning to idle.")
        else
            -- 指示継続中
            outputBool(3, true, 4) -- ハッチは開けたまま
            outputBool(2, true, 4) -- 状態表示ON
            -- 最新の座標をTAから受け取りデータリンク
            if assignedOutputId > 0 and (instructedX ~= 0 or instructedY ~= 0 or instructedZ ~= 0) then
                outputNumber(1, instructedX, 3); outputNumber(2, instructedY, 3); outputNumber(3, instructedZ, 3)
            else
                -- TAからの座標が0になったらデータリンク停止
                outputNumber(1, 0, 3); outputNumber(2, 0, 3); outputNumber(3, 0, 3)
            end
        end
        -- 発射信号は1Tickのみなのでここでは出力しない
    end
end -- onTick End
