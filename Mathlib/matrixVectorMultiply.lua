--- 行列（テーブルのテーブル）とベクトル（テーブル）を受け取り、
-- その積 (行列 * ベクトル) を計算した新しいベクトルを返す関数
-- @param mat number[][] 対象の行列 (m行n列) (例: {{1,2},{3,4}})
-- @param vec number[] 対象のベクトル (n次元) (例: {10, 20})
-- @return number[] or nil 計算結果のベクトル (m次元) (例: {50, 110})、次元が不整合の場合は nil
function matrixVectorMultiply(mat, vec)
    -- 行列が空でないか、ベクトルが空でないか基本的なチェック
    if #mat == 0 or #mat[1] == 0 or #vec == 0 then
        print("Error: matrixVectorMultiply - Matrix or vector is empty.")
        return nil
    end

    -- 行列の行数 (m) と列数 (n) を取得
    local numRows = #mat
    local numCols = #mat[1] -- 1行目の要素数を列数とする

    -- ベクトルの次元数を取得
    local vecDim = #vec

    -- 行列の列数 (n) とベクトルの次元数 (n) が一致するか確認
    if numCols ~= vecDim then
        print("Error: matrixVectorMultiply - Matrix columns (" ..
            numCols .. ") must match vector dimension (" .. vecDim .. ").")
        return nil
    end

    -- 結果を格納するための新しいベクトル（m次元）を作成し、0で初期化
    local resultVec = {}
    for i = 1, numRows do
        resultVec[i] = 0
    end

    -- 行列の各行についてループ (i = 1 から m まで)
    for i = 1, numRows do
        -- 行列の各列についてループ (j = 1 から n まで)
        for j = 1, numCols do
            -- resultVec[i] に mat[i][j] * vec[j] を足し込む (内積の計算)
            resultVec[i] = resultVec[i] + mat[i][j] * vec[j]
        end
    end

    -- 計算結果のベクトルを返す
    return resultVec
end

-- === 使用例 ===

-- 2x3 行列と 3次元ベクトルを定義
local matrixA = {
    { 1, 2, 3 },
    { 4, 5, 6 }
}
local vectorX = { 10, 20, 30 }

-- matrixVectorMultiply 関数を使って 行列 * ベクトル を計算
local vectorY = matrixVectorMultiply(matrixA, vectorX)

-- 結果が nil でないか確認
if vectorY then
    print("Matrix A:")
    for i = 1, #matrixA do print("  {" .. table.concat(matrixA[i], ", ") .. "}") end
    print("Vector X: {" .. table.concat(vectorX, ", ") .. "}")
    print("Result Vector Y (A * X): {" .. table.concat(vectorY, ", ") .. "}") -- 出力例: Result Vector Y (A*X): {140, 320} (1*10+2*20+3*30=140, 4*10+5*20+6*30=320)
end

-- 状態遷移行列 F (6x6) と状態ベクトル x (6次元) の予測計算例
local F = { -- 仮の等速直線運動モデル (dt=1/60)
    { 1, 0, 0, 1 / 60, 0,      0 },
    { 0, 1, 0, 0,      1 / 60, 0 },
    { 0, 0, 1, 0,      0,      1 / 60 },
    { 0, 0, 0, 1,      0,      0 },
    { 0, 0, 0, 0,      1,      0 },
    { 0, 0, 0, 0,      0,      1 }
}
local currentState = { 100, 200, 50, 10, 5, 1 } -- 現在の状態 [X, Y, Z, VX, VY, VZ]

local predictedState = matrixVectorMultiply(F, currentState)

if predictedState then
    local stateStr = {}
    for i = 1, #predictedState do table.insert(stateStr, string.format("%.2f", predictedState[i])) end
    print("Predicted State (F * current): {" .. table.concat(stateStr, ", ") .. "}")
    -- 出力例 (近似値): Predicted State (F * current): {100.17, 200.08, 50.02, 10.00, 5.00, 1.00}
end

-- 次元が不整合な例 (エラーになるはず)
local matrixB = { { 1, 2 }, { 3, 4 } }                     -- 2x2 行列
local vectorZ = { 10, 20, 30 }                             -- 3次元ベクトル
local errorResult = matrixVectorMultiply(matrixB, vectorZ) -- "Error: matrixVectorMultiply - Matrix columns (2) must match vector dimension (3)." が出力される
