--- 2つの行列（テーブルのテーブル）を受け取り、その積 (matA * matB) を計算した新しい行列を返す関数
-- @param matA number[][] 1つ目の行列 (m行n列) (例: {{1,2},{3,4}})
-- @param matB number[][] 2つ目の行列 (n行p列) (例: {{10,20,30},{40,50,60}})
-- @return number[][] or nil 計算結果の行列 (m行p列) (例: {{90,120,150},{190,260,330}})、次元が不整合の場合は nil
function matrixMultiply(matA, matB)
    -- 行列が空でないか基本的なチェック
    if #matA == 0 or #matA[1] == 0 or #matB == 0 or #matB[1] == 0 then
        print("Error: matrixMultiply - Matrices cannot be empty.")
        return nil
    end

    -- 行列Aの行数(m)と列数(n)を取得
    local rowsA = #matA
    local colsA = #matA[1]
    -- 行列Bの行数(n)と列数(p)を取得
    local rowsB = #matB
    local colsB = #matB[1]

    -- 行列Aの列数(n)と行列Bの行数(n)が一致するか確認
    if colsA ~= rowsB then
        print("Error: matrixMultiply - Matrix A columns (" .. colsA .. ") must match Matrix B rows (" .. rowsB .. ").")
        return nil
    end

    -- 結果を格納するための新しい行列（m行p列）を作成し、0で初期化
    local resultMat = {}
    for i = 1, rowsA do
        resultMat[i] = {} -- 新しい行を作成
        for k = 1, colsB do
            resultMat[i][k] = 0
        end
    end

    -- 結果行列の各要素を計算 (3重ループ)
    -- i: 結果行列の行 (1 から m)
    for i = 1, rowsA do
        -- k: 結果行列の列 (1 から p)
        for k = 1, colsB do
            -- j: 内積計算のためのインデックス (1 から n)
            local sum = 0       -- 内積の合計を初期化
            for j = 1, colsA do -- or rowsB, どちらでも同じ
                sum = sum + matA[i][j] * matB[j][k]
            end
            resultMat[i][k] = sum -- 計算結果を格納
        end
    end

    -- 計算結果の行列を返す
    return resultMat
end

-- === 使用例 ===

-- 2x2 行列 A と 2x3 行列 B を定義
local matrixA = {
    { 1, 2 },
    { 3, 4 }
}
local matrixB = {
    { 10, 11, 12 },
    { 20, 21, 22 }
}

-- matrixMultiply 関数を使って 行列A * 行列B を計算 (結果は 2x3 行列になるはず)
local matrixC = matrixMultiply(matrixA, matrixB)

-- 結果が nil でないか確認
if matrixC then
    print("Matrix A:")
    for i = 1, #matrixA do print("  {" .. table.concat(matrixA[i], ", ") .. "}") end
    print("Matrix B:")
    for i = 1, #matrixB do print("  {" .. table.concat(matrixB[i], ", ") .. "}") end
    print("Result Matrix C (A * B):")
    for i = 1, #matrixC do print("  {" .. table.concat(matrixC[i], ", ") .. "}") end
    -- 出力例:
    -- Result Matrix C (A * B):
    --   {50, 53, 56}  (1*10+2*20=50, 1*11+2*21=53, 1*12+2*22=56)
    --   {110, 117, 124} (3*10+4*20=110, 3*11+4*21=117, 3*12+4*22=124)
end

-- 次元が不整合な例 (エラーになるはず)
local matrixD = { { 1, 2 }, { 3, 4 } }               -- 2x2
local matrixE = { { 1 }, { 2 }, { 3 } }              -- 3x1
local errorResult = matrixMultiply(matrixD, matrixE) -- "Error: matrixMultiply - Matrix A columns (2) must match Matrix B rows (3)." が出力される
