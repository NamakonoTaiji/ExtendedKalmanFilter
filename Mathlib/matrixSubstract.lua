--- 2つの行列（テーブルのテーブル）を受け取り、要素ごとに引き算した新しい行列を返す関数 (matA - matB)
-- @param matA number[][] 1つ目の行列 (引かれる数) (m行n列)
-- @param matB number[][] 2つ目の行列 (引く数) (m行n列)
-- @return number[][] or nil 減算結果の行列 (m行n列)、次元が異なる場合は nil
function matrixSubtract(matA, matB)
    -- 行列が空でないか基本的なチェック
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        print("Error: matrixSubtract - Matrices cannot be empty or invalid.")
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
        print("Error: matrixSubtract - Matrices must have the same dimensions. A(" ..
            rowsA .. "x" .. colsA .. "), B(" .. rowsB .. "x" .. colsB .. ")")
        return nil
    end

    -- 結果を格納するための新しい行列（m行n列）を作成
    local resultMat = {}
    for i = 1, rowsA do
        resultMat[i] = {} -- 新しい行を作成
        for j = 1, colsA do
            -- 対応する要素が存在するか確認
            if matA[i] == nil or matA[i][j] == nil or matB[i] == nil or matB[i][j] == nil then
                print("Error: matrixSubtract - Invalid matrix element during subtraction at [" .. i .. "][" .. j .. "]")
                return nil
            end
            -- 対応する要素同士を引き算する
            resultMat[i][j] = matA[i][j] - matB[i][j]
        end
    end

    -- 計算結果の行列を返す
    return resultMat
end

-- === 使用例 (matrixSubtract) ===
local matrixP_pred = { { 10.1, 1 }, { 1, 5.1 } }  -- 例: 予測共分散 P_pred
local matrixKHKT = { { 1.0, 0.5 }, { 0.5, 1.1 } } -- 例: K*S*K^T の計算結果 (仮)

local matrixP_updated = matrixSubtract(matrixP_pred, matrixKHKT)

if matrixP_updated then
    print("--- matrixSubtract Example ---")
    print("Matrix P_pred:")
    for i = 1, #matrixP_pred do print("  {" .. table.concat(matrixP_pred[i], ", ") .. "}") end
    print("Matrix KSKT:")
    for i = 1, #matrixKHKT do print("  {" .. table.concat(matrixKHKT[i], ", ") .. "}") end
    print("Result Matrix P_updated (P_pred - KSKT):")
    for i = 1, #matrixP_updated do print("  {" .. table.concat(matrixP_updated[i], ", ") .. "}") end
    -- 出力例:
    -- Result Matrix P_updated (P_pred - KSKT):
    --   {9.1, 0.5}
    --   {0.5, 4.0}
end
