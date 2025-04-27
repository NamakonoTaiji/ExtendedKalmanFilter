iN=input.getNumber
iB=input.getBool
oN=output.setNumber
oB=output.setBool
DT=screen.drawText
SF=string.format
M=math
pi=M.pi
pi2=pi*2
dt=1/60
sin,cos=M.sin,M.cos
azimuthG,elevG=0,0
R={{0,0,0},{0,0,0},{0,0,0}}
function scalar(a, Mat)
	local r={}
	for i=1, #Mat do
		r[i]={}
		for j=1, #Mat[1] do
			r[i][j]=Mat[i][j] * a
		end
	end
	return r
end
function ZYXRotate(the,phi,psi)
    RX={{1,0,0},{0,cos(the),-sin(the)},{0,sin(the),cos(the)}}
    RY={{cos(phi),0,sin(phi)},{0,1,0},{-sin(phi),0,cos(phi)}}
    RZ={{cos(psi),-sin(psi),0},{sin(psi),cos(psi),0},{0,0,1}}
    matrix=multiplier((RX),multiplier((RY),(RZ)))
    return matrix
end
function onTick()
	azimuth=iN(18)*pi2
    elev=iN(19)*pi2
    eulerX=iN(4)
    eulerY=iN(5)
    eulerZ=iN(6)
    eulerMatR=ZYXRotate(-eulerX,-eulerY,-eulerZ)
    Rthetaphi={{cos(azimuth),-sin(elev)*sin(azimuth),cos(elev)*sin(azimuth)},
    {0,cos(elev),sin(elev)},
    {-sin(azimuth),-sin(elev)*cos(azimuth),cos(elev)*cos(azimuth)}}
    R=multiplier(transpose(eulerMatR),Rthetaphi)
    elevG=M.asin(R[2][3])
    azimuthG=M.atan(R[1][3],R[3][3])
    oN(1,iN(20))
    oN(2,azimuthG)
    oN(3,elevG)
	oN(10,iN(24))
	oN(11,iN(1))
	oN(12,iN(2))
	oN(13,iN(3))
	oB(2,iB(1))
end
function inv(M)
	local n=#M
	local r={}
	for i=1, n do
		r[i]={}
		for j=1, n do r[i][j]=(i == j) and 1 or 0 end
	end
	for i=1, n do
		local pv=M[i][i]
		for j=1, n do
			M[i][j]=M[i][j] / pv
			r[i][j]=r[i][j] / pv
		end
		for k=1, n do
			if k ~= i then
				local f=M[k][i]
				for j=1, n do
					M[k][j]=M[k][j] - f * M[i][j]
					r[k][j]=r[k][j] - f * r[i][j]
				end
			end
		end
	end
	return r
end
function multiplier(A, B)
	if #A[1] ~= #B then return nil end
	local r={}
	for i=1, #A do r[i]={} end
	for i=1, #A do
		for j=1, #B[1] do
			local s=0
			for k=1, #B do s=s + A[i][k] * B[k][j] end
			r[i][j]=s
		end
	end
	return r
end
function transpose(M)
	local r={}
	for i=1, #M[1] do
		r[i]={}
		for j=1, #M do r[i][j]=M[j][i] end
	end
	return r
end