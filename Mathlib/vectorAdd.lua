--- 2つのベクトル（テーブル）を受け取り、要素ごとに加算した新しいベクトルを返す関数
-- @param vecA number[] 1つ目のベクトル (例: {1, 2, 3})
-- @param vecB number[] 2つ目のベクトル (例: {4, 5, 6})
-- @return number[] or nil 加算結果のベクトル (例: {5, 7, 9})、次元が異なる場合は nil
function vectorAdd(vecA, vecB)
    -- 入力された2つのベクトルの次元数（要素数）を取得
    local dimA = #vecA
    local dimB = #vecB

    -- 次元数が異なる場合は計算できないため、エラーメッセージを出力して nil を返す
    if dimA ~= dimB then
        print("Error: vectorAdd - Vectors must have the same dimension.")
        return nil
    end

    -- 結果を格納するための新しい空のテーブル（ベクトル）を作成
    local resultVec = {}

    -- 1番目の要素から次元数番目（dimA または dimB）の要素までループ
    for i = 1, dimA do
        -- vecA の i番目の要素と vecB の i番目の要素を足し合わせる
        local sum = vecA[i] + vecB[i]
        -- 計算結果を resultVec の i番目の要素として格納
        resultVec[i] = sum
    end

    -- 計算結果のベクトルを返す
    return resultVec
end

-- === 使用例 ===

-- 2つの3次元ベクトルを定義
local vector1 = { 1, 2, 3 }
local vector2 = { 10, 20, 30 }

-- vectorAdd 関数を使ってベクトルを加算
local sumVector = vectorAdd(vector1, vector2)

-- 結果が nil でないか確認 (次元が同じだったので nil ではないはず)
if sumVector then
    -- 結果のベクトルをコンソールに出力 (デバッグ用)
    -- Stormworksの print() はコンソールに表示されます
    print("Vector1: {" .. table.concat(vector1, ", ") .. "}")
    print("Vector2: {" .. table.concat(vector2, ", ") .. "}")
    print("Sum Vector: {" .. table.concat(sumVector, ", ") .. "}") -- 出力例: Sum Vector: {11, 22, 33}
end

-- 次元が異なるベクトルの例 (エラーになるはず)
local vector3 = { 1, 2 }
local vector4 = { 10, 20, 30 }
local errorResult = vectorAdd(vector3, vector4) -- "Error: vectorAdd - Vectors must have the same dimension." が出力される

-- 状態ベクトル (6次元) の加算例
local state1 = { 10, 20, 30, 1, 2, 3 }
local state2 = { 1, 1, 1, 0.1, 0.1, 0.1 }
local sumState = vectorAdd(state1, state2)
if sumState then
    print("Sum State: {" .. table.concat(sumState, ", ") .. "}") -- 出力例: Sum State: {11, 21, 31, 1.1, 2.1, 3.1}
end
