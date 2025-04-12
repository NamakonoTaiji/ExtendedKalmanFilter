--- 2つのベクトル（テーブル）を受け取り、要素ごとに引き算した新しいベクトルを返す関数 (vecA - vecB)
-- @param vecA number[] 1つ目のベクトル (引かれる数) (例: {10, 20, 30})
-- @param vecB number[] 2つ目のベクトル (引く数) (例: {1, 2, 3})
-- @return number[] or nil 減算結果のベクトル (例: {9, 18, 27})、次元が異なる場合は nil
function vectorSubtract(vecA, vecB)
    -- 入力された2つのベクトルの次元数（要素数）を取得
    local dimA = #vecA
    local dimB = #vecB

    -- 次元数が異なる場合は計算できないため、エラーメッセージを出力して nil を返す
    if dimA ~= dimB then
        print("Error: vectorSubtract - Vectors must have the same dimension.")
        return nil
    end

    -- 結果を格納するための新しい空のテーブル（ベクトル）を作成
    local resultVec = {}

    -- 1番目の要素から次元数番目（dimA または dimB）の要素までループ
    for i = 1, dimA do
        -- vecA の i番目の要素から vecB の i番目の要素を引き算する
        local diff = vecA[i] - vecB[i]
        -- 計算結果を resultVec の i番目の要素として格納
        resultVec[i] = diff
    end

    -- 計算結果のベクトルを返す
    return resultVec
end

-- === 使用例 ===

-- 2つの3次元ベクトルを定義
local vector1 = { 10, 20, 30 }
local vector2 = { 3, 5, 7 }

-- vectorSubtract 関数を使ってベクトルを引き算
local diffVector = vectorSubtract(vector1, vector2)

-- 結果が nil でないか確認
if diffVector then
    -- 結果のベクトルをコンソールに出力
    print("Vector1: {" .. table.concat(vector1, ", ") .. "}")
    print("Vector2: {" .. table.concat(vector2, ", ") .. "}")
    print("Difference Vector (Vector1 - Vector2): {" .. table.concat(diffVector, ", ") .. "}") -- 出力例: Difference Vector (Vector1 - Vector2): {7, 15, 23}
end

-- 状態ベクトル (6次元) の引き算例
local state1 = { 10, 20, 30, 1, 2, 3 }
local predictedState = { 9.5, 19.8, 29.9, 0.9, 1.9, 2.9 }
local innovation = vectorSubtract(state1, predictedState) -- 観測値と予測値の差 (イノベーション) 計算などに使う
if innovation then
    -- innovation ベクトルを文字列に変換して出力 (小数点が含まれる可能性があるので注意)
    local innovationStr = {}
    for i = 1, #innovation do
        table.insert(innovationStr, tostring(innovation[i]))
    end
    print("Innovation Vector: {" .. table.concat(innovationStr, ", ") .. "}")
    -- 出力例 (近似値): Innovation Vector: {0.5, 0.2, 0.1, 0.1, 0.1, 0.1}
end
