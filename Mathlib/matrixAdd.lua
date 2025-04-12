--- 2つの行列（テーブルのテーブル）を受け取り、要素ごとに加算した新しい行列を返す関数
-- @param matA number[][] 1つ目の行列 (m行n列)
-- @param matB number[][] 2つ目の行列 (m行n列)
-- @return number[][] or nil 加算結果の行列 (m行n列)、次元が異なる場合は nil
function matrixAdd(matA, matB)
    -- 行列が空でないか基本的なチェック
    if #matA == 0 or #matA[1] == 0 or #matB == 0 or #matB[1] == 0 then
        print("Error: matrixAdd - Matrices cannot be empty.")
        return nil
    end

    -- 行列Aの行数(m)と列数(n)を取得
    local rowsA = #matA
    local colsA = #matA[1]
    -- 行列Bの行数(m)と列数(n)を取得
    local rowsB = #matB
    local colsB = #matB[1]

    -- 行列Aと行列Bの次元が一致するか確認 (行数と列数が両方とも同じ)
    if rowsA ~= rowsB or colsA ~= colsB then
        print("Error: matrixAdd - Matrices must have the same dimensions.")
        return nil
    end

    -- 結果を格納するための新しい行列（m行n列）を作成
    local resultMat = {}
    for i = 1, rowsA do
        resultMat[i] = {} -- 新しい行を作成
        for j = 1, colsA do
            -- 対応する要素同士を足し算する
            resultMat[i][j] = matA[i][j] + matB[i][j]
        end
    end

    -- 計算結果の行列を返す
    return resultMat
end

-- === 使用例 ===
local matrixP = { { 10, 1 }, { 1, 5 } }    -- 例: 共分散行列 P
local matrixQ = { { 0.1, 0 }, { 0, 0.1 } } -- 例: プロセスノイズ Q

local matrixP_plus_Q = matrixAdd(matrixP, matrixQ)

if matrixP_plus_Q then
    print("--- matrixAdd Example ---")
    print("Matrix P:")
    for i = 1, #matrixP do print("  {" .. table.concat(matrixP[i], ", ") .. "}") end
    print("Matrix Q:")
    for i = 1, #matrixQ do print("  {" .. table.concat(matrixQ[i], ", ") .. "}") end
    print("Result Matrix (P + Q):")
    for i = 1, #matrixP_plus_Q do print("  {" .. table.concat(matrixP_plus_Q[i], ", ") .. "}") end
    -- 出力例:
    -- Result Matrix (P + Q):
    --   {10.1, 1}
    --   {1, 5.1}
end
