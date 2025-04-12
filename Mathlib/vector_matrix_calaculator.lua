--[[
  Stormworks UKF実装のための基本的な行列・ベクトル演算関数群
  (ログ出力は debug.log("CODE") 形式)
]]

--[[ Log/Error Code Mappings (for Helper Functions):
  E:va01 - vectorAdd: Dimension mismatch.
  E:vs01 - vectorSubtract: Dimension mismatch.
  E:mv01 - matrixVectorMultiply: Empty/invalid matrix or vector.
  E:mv02 - matrixVectorMultiply: Dimension mismatch (mat cols vs vec dim).
  E:mv03 - matrixVectorMultiply: Invalid matrix element access.
  E:mv04 - matrixVectorMultiply: Invalid vector element access.
  E:mm01 - matrixMultiply: Empty/invalid matrices.
  E:mm02 - matrixMultiply: Dimension mismatch (matA cols vs matB rows).
  E:mm03 - matrixMultiply: Invalid matrix element access during multiplication.
  E:mt01 - matrixTranspose: Empty/invalid matrix.
  W:mt01 - matrixTranspose: Inconsistent column count (Warning).
  E:ma01 - matrixAdd: Empty/invalid matrices.
  E:ma02 - matrixAdd: Dimension mismatch.
  E:ma03 - matrixAdd: Invalid matrix element access during addition.
  E:ms01 - matrixSubtract: Empty/invalid matrices.
  E:ms02 - matrixSubtract: Dimension mismatch.
  E:ms03 - matrixSubtract: Invalid matrix element access during subtraction.
]]

--- vectorAdd: ベクトル同士の加算
function vectorAdd(vecA, vecB)
    local dimA = #vecA; local dimB = #vecB
    if dimA ~= dimB then
        debug.log("E:va01"); return nil
    end
    local r = {}; for i = 1, dimA do r[i] = vecA[i] + vecB[i] end; return r
end

--- vectorSubtract: ベクトル同士の減算 (vecA - vecB)
function vectorSubtract(vecA, vecB)
    local dimA = #vecA; local dimB = #vecB
    if dimA ~= dimB then
        debug.log("E:vs01"); return nil
    end
    local r = {}; for i = 1, dimA do r[i] = vecA[i] - vecB[i] end; return r
end

--- vectorScale: ベクトルのスカラー倍
function vectorScale(vec, scalar)
    local r = {}; local dim = #vec; for i = 1, dim do r[i] = vec[i] * scalar end; return r
end

--- matrixVectorMultiply: 行列とベクトルの乗算 (mat * vec)
function matrixVectorMultiply(mat, vec)
    if not mat or #mat == 0 or not mat[1] or #mat[1] == 0 or not vec or #vec == 0 then
        debug.log("E:mv01"); return nil
    end
    local nr = #mat; local nc = #mat[1]; local vd = #vec
    if nc ~= vd then
        debug.log("E:mv02"); return nil
    end
    local rV = {}; for i = 1, nr do
        rV[i] = 0; for j = 1, nc do
            if not mat[i] or mat[i][j] == nil then
                debug.log("E:mv03"); return nil
            end; if vec[j] == nil then
                debug.log("E:mv04"); return nil
            end; rV[i] = rV[i] + mat[i][j] * vec[j]
        end
    end; return rV
end

--- matrixMultiply: 行列同士の乗算 (matA * matB)
function matrixMultiply(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:mm01"); return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if cA ~= rB then
        debug.log("E:mm02"); return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for k = 1, cB do
            local s = 0; for j = 1, cA do
                if not matA[i] or matA[i][j] == nil or not matB[j] or matB[j][k] == nil then
                    debug.log("E:mm03"); return nil
                end; s = s + matA[i][j] * matB[j][k]
            end; rM[i][k] = s
        end
    end; return rM
end

--- matrixTranspose: 行列の転置
function matrixTranspose(mat)
    if not mat or #mat == 0 or (mat[1] and #mat[1] == 0) then
        debug.log("E:mt01"); return nil
    end
    local r = #mat; local c = #mat[1] or 0; local rM = {}; for j = 1, c do rM[j] = {} end
    for i = 1, r do
        local cCL = #mat[i] or 0; if cCL ~= c and i > 1 then debug.log("W:mt01") end
        for j = 1, c do
            if mat[i] and mat[i][j] ~= nil then
                if not rM[j] then rM[j] = {} end; rM[j][i] = mat[i][j]
            else
                if not rM[j] then rM[j] = {} end; rM[j][i] = nil
            end
        end
    end; return rM
end

--- matrixAdd: 行列同士の加算
function matrixAdd(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:ma01"); return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if rA ~= rB or cA ~= cB then
        debug.log("E:ma02"); return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for j = 1, cA do
            if matA[i] == nil or matA[i][j] == nil or matB[i] == nil or matB[i][j] == nil then
                debug.log("E:ma03"); return nil
            end; rM[i][j] = matA[i][j] + matB[i][j]
        end
    end; return rM
end

--- matrixSubtract: 行列同士の減算 (matA - matB)
function matrixSubtract(matA, matB)
    if not matA or #matA == 0 or not matA[1] or #matA[1] == 0 or not matB or #matB == 0 or not matB[1] or #matB[1] == 0 then
        debug.log("E:ms01"); return nil
    end
    local rA = #matA; local cA = #matA[1]; local rB = #matB; local cB = #matB[1]
    if rA ~= rB or cA ~= cB then
        debug.log("E:ms02"); return nil
    end
    local rM = {}; for i = 1, rA do
        rM[i] = {}; for j = 1, cA do
            if matA[i] == nil or matA[i][j] == nil or matB[i] == nil or matB[i][j] == nil then
                debug.log("E:ms03"); return nil
            end; rM[i][j] = matA[i][j] - matB[i][j]
        end
    end; return rM
end
