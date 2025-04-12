--- ベクトル（テーブル）とスカラー値（数値）を受け取り、
-- ベクトルの各要素をスカラー値で掛け算した新しいベクトルを返す関数
-- @param vec number[] 対象のベクトル (例: {1, 2, 3})
-- @param scalar number ベクトルに乗算するスカラー値 (例: 2)
-- @return number[] 計算結果のベクトル (例: {2, 4, 6})
function vectorScale(vec, scalar)
    -- 結果を格納するための新しい空のテーブル（ベクトル）を作成
    local resultVec = {}
    -- ベクトルの次元数（要素数）を取得
    local dim = #vec

    -- 1番目の要素から次元数番目の要素までループ
    for i = 1, dim do
        -- vec の i番目の要素に scalar を掛け算する
        local product = vec[i] * scalar
        -- 計算結果を resultVec の i番目の要素として格納
        resultVec[i] = product
    end

    -- 計算結果のベクトルを返す
    return resultVec
end

-- === 使用例 ===

-- 3次元ベクトルとスカラー値を定義
local myVector = { 10, -20, 30 }
local scaleFactor = 0.5

-- vectorScale 関数を使ってベクトルをスカラー倍
local scaledVector = vectorScale(myVector, scaleFactor)

-- 結果のベクトルをコンソールに出力
print("Original Vector: {" .. table.concat(myVector, ", ") .. "}")
print("Scale Factor: " .. tostring(scaleFactor))
print("Scaled Vector: {" .. table.concat(scaledVector, ", ") .. "}") -- 出力例: Scaled Vector: {5, -10, 15}

-- イノベーションベクトルにカルマンゲインの一部 (仮にスカラーだとする) を掛ける例
local innovation = { 0.5, 0.2, 0.1, 0.1, 0.1, 0.1 } -- 6次元ベクトル
local gainFactor = 0.8                              -- 仮のゲイン係数

local correctionTerm = vectorScale(innovation, gainFactor)

if correctionTerm then
    local correctionStr = {}
    for i = 1, #correctionTerm do
        table.insert(correctionStr, string.format("%.2f", correctionTerm[i])) -- 小数点以下2桁で表示
    end
    print("Correction Term: {" .. table.concat(correctionStr, ", ") .. "}")
    -- 出力例: Correction Term: {0.40, 0.16, 0.08, 0.08, 0.08, 0.08}
end
