-- MissileFCS_v10_SequentialSelect.lua

--[[
================================================================================
Missile Fire Control System (VLS逐次選択版) - コメント付き
================================================================================
機能概要:
- KFから敵対目標情報(OutputIDベース固定ch)を入力。
- 共有バスを使用し、他のFCSと目標担当状況・発射済み状況を共有。
- ★変更点: 目標選択は前のFCSが発射済みになってから開始する（逐次選択）。
- OutputIDが最小の未担当目標を選択しロックオン。
- ロックオン後、VLSハッチを開放 (オンオフch 3)。
- ハッチ開放後、指定Tick数待機 (HatchDelayプロパティ)。
- 待機完了後、かつ外部トリガーがONならミサイル発射。
- ★変更点: 発射信号(オンオフ1)は一度ONになったらリセットされるまでONを維持。
- ミサイル追跡中はデータリンク情報をアンテナに出力 (数値ch 1-3)。
- 担当ミサイルからの信号強度(数値ch 31)を監視し、途絶えたら 'idle' 状態へ。
- 'idle' 状態では共有バスの中継のみを行う。

これにより信号遅延による競合を回避しつつ、結果的に（1Tickずつ遅れて）
各FCSが発射プロセスを開始し、同時迎撃が可能になることを目指す。
================================================================================
入力 (コンポジット):
- 数値 1-最大27 (or 30): KFからの敵対目標座標 (X, Y, Z x 9 or 10、OutputIDベース固定ch)
- 数値 31: 担当ミサイル信号強度
- オンオフ 11-最大19 (or 20): 共有バス(入力) - 他FCS担当フラグ (チャンネル 10 + OutputID)
- オンオフ 21: 共有バス(入力) - ★前のFCSが発射済みフラグ (Prev Fired)
- オンオフ 31: 発射トリガー (外部から)

出力 (コンポジット):
- オンオフ 1: ★発射信号 (Fire) - 一度ONになったらリセットまで維持
- オンオフ 2: 状態表示フラグ (Searching以外でON)
- オンオフ 3: VLSハッチ制御信号 (Open=true / Close=false)
- オンオフ 11-最大19 (or 20): 共有バス(出力) - 目標担当フラグ (自分含む全FCSの情報)
- オンオフ 21: 共有バス(出力) - ★このFCSが発射済みフラグ (Self Fired)
- 数値 1: アンテナ 担当目標 X 座標 (Target X)
- 数値 2: アンテナ 担当目標 Y 座標 (Target Y)
- 数値 3: アンテナ 担当目標 Z 座標 (Target Z)
- (数値 31: アンテナ 送信周波数 - 削除)
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
local propertyBool = property.getBool

-- 定数
-- KF側の最大追跡目標数と合わせる (プロパティで設定可能)
local MAX_TARGETS = 10
-- 共有バスで担当フラグをやり取りする際のチャンネルオフセット (例: ch11 = ID1担当)
local SHARED_BUS_TARGET_OFFSET = 10
-- 共有バスで「発射済みフラグ」をやり取りするチャンネル番号
local SHARED_BUS_FIRED_FLAG_CH = 21 -- ★オンオフ21を使用
-- ミサイル信号がこれ以下になったら喪失とみなす閾値 (要調整)
local MISSILE_SIGNAL_THRESHOLD = 0.1
-- ハッチ開放待機Tick数 (プロパティで設定可能、デフォルト60tick=1秒)
local HATCH_OPEN_DELAY_TICKS = propertyNumber("HatchDelay") or 120

-- プロパティ
-- このFCSがデイジーチェーンの最初のランチャーかどうか (最初のランチャーは前の状態を気にせず目標選択/発射できる)
local isFirstLauncher = propertyBool("IsFirstLauncher") or false

-- グローバル変数 (マイクロコントローラー内部で状態を保持する変数)
local assignedOutputId = nil     -- 現在担当している目標のOutput ID (1からMAX_TARGETS)
local lockOnStatus = "searching" -- 現在のFCSの状態 (searching, locked, hatch_opening, tracking, idle)
local targetInfo = {}            -- 現在担当している目標の最新座標 {x, y, z}
local hatchOpenStartTime = 0     -- ハッチを開け始めた時刻 (Tick)
local currentTick = 0            -- このマイクロコントローラーが起動してからのTickカウント
local hasFired = false           -- ★自身が一度でも発射したかどうかのフラグ

-- 起動時処理 (マイクロコントローラーがスポーン/リロードされた時に実行される想定)
function onSpawn()
    -- 状態変数を初期化
    assignedOutputId = nil
    lockOnStatus = "searching"
    targetInfo = {}
    hatchOpenStartTime = 0
    currentTick = 0
    hasFired = false -- 発射フラグもリセット
end

onSpawn() -- スクリプト読み込み時に一度だけ実行

-- 目標選択関数: KFからの目標リストと共有バスの担当状況を元に、
--                OutputIDが最も若く、まだ誰も担当していない目標のOutputIDを返す
-- (この関数自体に変更はないが、呼び出しタイミングが変更される)
function selectNewTarget(kfTargetsExist, assignedFlagsInput)
    -- Output ID 1 から順番にチェック
    for outputId = 1, MAX_TARGETS do
        -- KFがそのIDの目標情報を出力しているか (座標が0でないか)
        if kfTargetsExist[outputId] then
            -- 共有バスをチェックし、他のFCSが担当していないか
            local isAssignedByOther = assignedFlagsInput[outputId] or false
            if not isAssignedByOther then
                -- 条件を満たす最初の目標IDが見つかったら、そのIDを返す
                return outputId
            end
        end
    end
    -- 担当可能な目標が見つからなかった場合は nil を返す
    return nil
end

-- メインループ (毎Tick実行される)
function onTick()
    currentTick = currentTick + 1 -- Tickカウンターを増やす

    -- 0. 出力チャンネルのクリア (発射信号、担当フラグ、発射済みフラグ以外)
    -- 発射信号(ch1)と担当フラグ(ch11-?)と発射済フラグ(ch21)は維持または後で設定するため、ここではクリアしない
    outputBool(2, false)                                        -- State Flag
    outputBool(3, false)                                        -- Hatch Control
    outputNumber(1, 0); outputNumber(2, 0); outputNumber(3, 0); -- Datalink Coords
    -- 周波数出力は削除

    -- 1. 入力情報の読み込み
    local missileSignal = inputNumber(31) -- 担当ミサイルからの信号強度

    local kfTargetsExist = {}             -- KFが現在出力している目標IDの存在確認用テーブル (キー: OutputID, 値: true/false)
    local assignedFlagsInput = {}         -- 共有バスから読み込んだ担当フラグ (キー: OutputID, 値: true/false)
    local assignedFlagsOutput = {}        -- 共有バスに出力するための担当フラグ (キー: OutputID, 値: true/false)
    -- ★前のFCSが発射済みかどうかを共有バス(ch21)から読み込む
    local prevFcsHasFired = inputBool(SHARED_BUS_FIRED_FLAG_CH)

    -- KF目標存在確認テーブルを作成
    -- 同時に共有バスの入力担当フラグを読み込み、出力フラグテーブルにコピー（中継のため）
    for outputId = 1, MAX_TARGETS do
        local baseChannel = (outputId - 1) * 3
        local x = inputNumber(baseChannel + 1); local y = inputNumber(baseChannel + 2); local z = inputNumber(
            baseChannel + 3);
        kfTargetsExist[outputId] = (x ~= 0 or y ~= 0 or z ~= 0)
        -- 共有バスの担当フラグ読み込み (チャンネル = 10 + OutputID)
        assignedFlagsInput[outputId] = inputBool(outputId + SHARED_BUS_TARGET_OFFSET)
        -- 出力用テーブルに、まず入力値をコピー (後で自分の担当分を上書きする)
        assignedFlagsOutput[outputId] = assignedFlagsInput[outputId]
    end

    -- 2. ミサイル状態監視とリセット判定 -> idle 状態へ
    -- ミサイル追跡中に信号が途絶えたら idle に遷移
    if lockOnStatus == "tracking" and missileSignal < MISSILE_SIGNAL_THRESHOLD then
        lockOnStatus = "idle"  -- ★ Idle 状態へ
        assignedOutputId = nil -- 担当解除
        targetInfo = {}        -- 目標情報クリア
        hatchOpenStartTime = 0 -- タイマーリセット
        -- hasFired フラグは true のまま維持する (一度発射したらリセットされるまで発射済み)
        -- debug.log("FCS "..fcsId..": Missile signal lost. Entering Idle state.")
    end

    -- 3. 状態遷移とそれに応じた処理 (リセット判定でidleになった場合はidleの処理へ)

    -- 状態: searching (目標探索中)
    if lockOnStatus == "searching" then
        -- ★ 前のFCSが発射済み、または自分が最初のランチャーの場合のみ目標選択を開始
        if isFirstLauncher or prevFcsHasFired then
            -- debug.log("FCS "..fcsId..": Previous FCS fired or I am first. Starting target selection.")
            local selectedOutputId = selectNewTarget(kfTargetsExist, assignedFlagsInput)
            -- 新しい目標が見つかった場合
            if selectedOutputId then
                assignedOutputId = selectedOutputId -- その目標IDを担当とする
                -- 対応するチャンネルから座標情報を取得
                local baseChannel = (assignedOutputId - 1) * 3
                targetInfo = {
                    x = inputNumber(baseChannel + 1),
                    y = inputNumber(baseChannel + 2),
                    z = inputNumber(
                        baseChannel + 3)
                }
                -- 取得した座標が有効なら、状態を'locked'に進める
                if targetInfo.x ~= 0 or targetInfo.y ~= 0 or targetInfo.z ~= 0 then
                    lockOnStatus = "locked"
                    -- debug.log("FCS "..fcsId..": Locked target OutputID: "..assignedOutputId)
                else
                    -- 取得した座標が無効だった場合(KF出力のタイムラグなど)、担当を解除して探索継続
                    assignedOutputId = nil
                    targetInfo = {}
                end
                -- else
                -- debug.log("FCS "..fcsId..": No assignable target found.")
            end
            -- else
            -- debug.log("FCS "..fcsId..": Waiting for previous FCS to fire...")
        end
        -- searching 状態ではハッチは閉(出力3=false)、発射信号OFF(出力1=false)

        -- 状態: locked (目標捕捉完了、ハッチ開放開始)
    elseif lockOnStatus == "locked" then
        -- 担当目標IDが存在し、KFからもまだ目標が出力されていることを確認
        if assignedOutputId and kfTargetsExist[assignedOutputId] then
            outputBool(3, true)              -- ★ハッチを開ける信号をONにする
            hatchOpenStartTime = currentTick -- ★ハッチ開放タイマーを開始
            lockOnStatus = "hatch_opening"   -- ★次の状態 'hatch_opening' へ遷移
            -- debug.log("FCS "..fcsId..": Locked. Opening hatch for OutputID: "..assignedOutputId)
        else
            -- locked状態に入った瞬間に目標を見失った場合、searchingに戻る
            lockOnStatus = "searching"; assignedOutputId = nil; targetInfo = {}; hatchOpenStartTime = 0;
            -- debug.log("FCS "..fcsId..": Target lost immediately after lock. Returning to search.")
        end
        -- locked 状態では発射信号OFF

        -- 状態: hatch_opening (ハッチ開放中 & 発射待機中)
    elseif lockOnStatus == "hatch_opening" then
        -- 担当目標IDが存在し、KFからもまだ目標が出力されていることを確認
        if assignedOutputId and kfTargetsExist[assignedOutputId] then
            outputBool(3, true) -- ★ハッチ開放信号を継続してON
            -- 最新の目標座標を取得 (データリンク用)
            local baseChannel = (assignedOutputId - 1) * 3
            targetInfo = {
                x = inputNumber(baseChannel + 1),
                y = inputNumber(baseChannel + 2),
                z = inputNumber(
                    baseChannel + 3)
            }

            -- 取得した座標が有効かチェック
            if targetInfo.x ~= 0 or targetInfo.y ~= 0 or targetInfo.z ~= 0 then
                outputBool(2, true) -- 状態表示フラグ ON

                -- ハッチ開放に必要な時間が経過したかチェック
                local hatchOpenElapsed = currentTick - hatchOpenStartTime
                local isHatchReady = hatchOpenElapsed >= HATCH_OPEN_DELAY_TICKS

                -- 発射条件をチェック (トリガーON AND ハッチ準備OK)
                -- 前のFCSの発射済みチェックは searching -> locked 遷移時に行われているので不要
                local canFire = isHatchReady

                -- 発射条件がすべて満たされた場合
                if canFire then
                    hasFired = true           -- ★自身が発射したフラグを立てる
                    lockOnStatus = "tracking" -- 状態を'tracking'へ遷移
                    -- 発射信号(出力1)は hasFired フラグで制御される
                    -- debug.log("FCS "..fcsId..": Firing at OutputID: "..assignedOutputId)
                end
                -- データリンクはこの状態から開始
                outputNumber(1, targetInfo.x); outputNumber(2, targetInfo.y); outputNumber(3, targetInfo.z);
            else
                -- ハッチ開放中に目標座標が0になった場合、searchingに戻る
                lockOnStatus = "searching"; assignedOutputId = nil; targetInfo = {}; hatchOpenStartTime = 0;
                -- debug.log("FCS "..fcsId..": Target coords became zero during hatch opening. Returning to search.")
            end
        else
            -- ハッチ開放中に担当目標が存在しなくなった場合、searchingに戻る
            lockOnStatus = "searching"; assignedOutputId = nil; targetInfo = {}; hatchOpenStartTime = 0;
            -- debug.log("FCS "..fcsId..": Assigned target disappeared during hatch opening. Returning to search.")
        end
        -- hatch_opening 状態では発射信号OFF (発射の瞬間は hasFired フラグで制御)

        -- 状態: tracking (ミサイル発射後、追跡中)
    elseif lockOnStatus == "tracking" then
        -- ミサイル追跡中もハッチは開けたままにする（閉じる場合は変更）
        outputBool(3, true)
        -- 担当目標IDが存在し、KFからもまだ目標が出力されているか確認
        if assignedOutputId and kfTargetsExist[assignedOutputId] then
            -- 最新の目標座標を取得 (データリンク用)
            local baseChannel = (assignedOutputId - 1) * 3
            targetInfo = {
                x = inputNumber(baseChannel + 1),
                y = inputNumber(baseChannel + 2),
                z = inputNumber(
                    baseChannel + 3)
            }
            outputBool(2, true) -- 状態表示フラグON
            -- 座標が0になったらデータリンク情報をクリア（ミサイルは飛び続ける）
            if targetInfo.x == 0 and targetInfo.y == 0 and targetInfo.z == 0 then
                targetInfo = {}
                outputNumber(1, 0); outputNumber(2, 0); outputNumber(3, 0); -- データリンク停止
                -- debug.log("FCS "..fcsId..": Target coords became zero during tracking. Stopping datalink for OutputID: "..assignedOutputId)
            else
                -- データリンク継続
                outputNumber(1, targetInfo.x); outputNumber(2, targetInfo.y); outputNumber(3, targetInfo.z);
            end
        else
            -- 追跡中にKFリストから目標が消えた場合
            targetInfo = {}                                             -- データリンク停止
            outputBool(2, true)                                         -- 状態表示フラグはONのまま (Tracking状態なので)
            outputNumber(1, 0); outputNumber(2, 0); outputNumber(3, 0); -- データリンク停止
            -- debug.log("FCS "..fcsId..": Target disappeared from KF list during tracking OutputID: ".. (assignedOutputId or "N/A") .. ". Missile continues.")
        end
        -- tracking 状態では発射信号ONを維持 (hasFiredフラグで制御)

        -- 状態: idle (ミサイル信号喪失後)
    elseif lockOnStatus == "idle" then
        -- この状態では能動的な処理は行わない
        -- 共有バスの中継は onTick 先頭のループで行われている
        -- 発射信号は hasFired フラグにより ON を維持
        -- 状態表示フラグ (ch2) は OFF (下の出力処理で設定)
        -- ハッチ (ch3) は OFF (onTick 先頭でクリア済み)
    end

    -- 4. 出力処理
    -- 4.1 共有バスへの出力 (担当フラグ & 発射済みフラグ)
    -- 担当フラグ: 自分が担当している目標IDに対応するフラグを立てる (idle状態では assignedOutputId が nil なので立たない)
    if assignedOutputId then
        assignedFlagsOutput[assignedOutputId] = true
    end
    -- 全FCSの担当フラグ情報を共有バスに出力
    for outputId = 1, MAX_TARGETS do
        outputBool(outputId + SHARED_BUS_TARGET_OFFSET, assignedFlagsOutput[outputId] or false)
    end

    -- 発射済みフラグ: 自身が一度でも発射したか(`hasFired`)を共有バス(ch21)に出力
    outputBool(SHARED_BUS_FIRED_FLAG_CH, hasFired)

    -- 4.2 ミサイルコネクタへの発射信号出力 (一度発射したらONを維持)
    outputBool(1, hasFired)

    -- 4.3 状態表示フラグ (オンオフ 2)
    -- searching 以外の状態 (locked, hatch_opening, tracking, idle) なら ON にする？
    -- -> idle 状態は活動終了なので OFF の方が適切かもしれない。ここでは searching のみ OFF とする。
    local isActiveState = (lockOnStatus ~= "searching")
    outputBool(2, isActiveState)

    -- 4.4 データリンク用座標出力 (既に各状態の分岐内で処理済み)

    -- 4.5 VLSハッチ制御出力 (既に各状態の分岐内で処理済み、idle/searchingではOFF)

    -- 4.6 周波数出力 (削除済み)
end -- onTick終了
