--- 行列（テーブルのテーブル）を受け取り、その転置行列を計算した新しい行列を返す関数
-- @param mat number[][] 対象の行列 (m行n列) (例: {{1,2,3},{4,5,6}})
-- @return number[][] or nil 計算結果の転置行列 (n行m列) (例: {{1,4},{2,5},{3,6}})、入力が不正な場合は nil
function matrixTranspose(mat)
    -- 行列が空でないか基本的なチェック
    if #mat == 0 or #mat[1] == 0 then
        print("Error: matrixTranspose - Matrix cannot be empty.")
        return nil
    end

    -- 元の行列の行数(m)と列数(n)を取得
    local rows = #mat
    local cols = #mat[1]

    -- 結果を格納するための新しい行列（n行m列）を作成
    local resultMat = {}
    -- 新しい行列の行を初期化 (転置後の行数 = 元の列数 n)
    for j = 1, cols do
        resultMat[j] = {} -- 新しい行 (j行目) を作成
    end

    -- 元の行列の各要素を転置後の位置にコピー (2重ループ)
    -- i: 元の行列の行 (1 から m)
    for i = 1, rows do
        -- j: 元の行列の列 (1 から n)
        for j = 1, cols do
            -- 元の mat[i][j] を、結果 resultMat[j][i] に代入する
            resultMat[j][i] = mat[i][j]
        end
    end

    -- 計算結果の転置行列を返す
    return resultMat
end

-- === 使用例 ===

-- 2x3 行列を定義
local matrixA = {
    { 1, 2, 3 },
    { 4, 5, 6 }
}

-- matrixTranspose 関数を使って転置行列を計算 (結果は 3x2 行列になるはず)
local matrixAT = matrixTranspose(matrixA)

-- 結果が nil でないか確認
if matrixAT then
    print("Original Matrix A (2x3):")
    for i = 1, #matrixA do print("  {" .. table.concat(matrixA[i], ", ") .. "}") end
    print("Transposed Matrix A^T (3x2):")
    for i = 1, #matrixAT do print("  {" .. table.concat(matrixAT[i], ", ") .. "}") end
    -- 出力例:
    -- Transposed Matrix A^T (3x2):
    --   {1, 4}
    --   {2, 5}
    --   {3, 6}
end

-- 状態遷移行列 F (6x6) の転置例
local F = { -- 前の例と同じ
    { 1, 0, 0, 1 / 60, 0,      0 },
    { 0, 1, 0, 0,      1 / 60, 0 },
    { 0, 0, 1, 0,      0,      1 / 60 },
    { 0, 0, 0, 1,      0,      0 },
    { 0, 0, 0, 0,      1,      0 },
    { 0, 0, 0, 0,      0,      1 }
}
local FT = matrixTranspose(F)

if FT then
    print("Transposed Matrix F^T (6x6):")
    for i = 1, #FT do
        local rowStr = {}
        for j = 1, #FT[i] do table.insert(rowStr, string.format("%.4f", FT[i][j])) end -- 見やすいようにフォーマット
        print("  {" .. table.concat(rowStr, ", ") .. "}")
    end
    -- 出力例 (一部):
    -- Transposed Matrix F^T (6x6):
    --   {1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000}
    --   {0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000}
    --   {0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000}
    --   {0.0167, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000}
    --   {0.0000, 0.0167, 0.0000, 0.0000, 1.0000, 0.0000}
    --   {0.0000, 0.0000, 0.0167, 0.0000, 0.0000, 1.0000}
end
