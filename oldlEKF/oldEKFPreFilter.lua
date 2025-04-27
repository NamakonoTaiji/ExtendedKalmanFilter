iN=input.getNumber
iB=input.getBool
oN=output.setNumber
oB=output.setBool
DT=screen.drawText
SF=string.format
pN=property.getNumber
M=math
pi=M.pi
pi2=pi*2
sin,cos=M.sin,M.cos
maxSamp=-1
maxSampIO=0
minData=M.huge
maxData=0
maxAzimuth=0
minAzimuth=9
maxElev=0
minElev=9
outputR=0
outputA=0
outputE=0
phyX,phyY,phyZ,eulerX,eulerY,eulerZ=0,0,0,0,0,0
function onTick()
    range=iN(1)
    azimuth=iN(2)
    elev=iN(3)
    
    isDetect=iB(1)
    if not isDetect then
        maxData=0
        minData=M.huge
        maxAzimuth=-9
        minAzimuth=9
        maxElev=-9
        minElev=9
        maxSampIO=0
        maxSamp=-1
    end

    if isDetect then
    	if maxSamp>=iN(4) then
        	maxSampIO=1
    	end
        maxSamp=M.max(maxSamp,iN(4))
        if iN(4)==0 then
            phyX=iN(8)
            phyY=iN(12)
            phyZ=iN(16)
            eulerX=iN(20)
            eulerY=iN(24)
            eulerZ=iN(28)
            maxData=0
            minData=M.huge
            maxAzimuth=-9
            minAzimuth=9
            maxElev=-9
            minElev=9
        end
        
        maxData=M.max(range,maxData)
        minData=M.min(range,minData)
        maxAzimuth=M.max(azimuth,maxAzimuth)
        minAzimuth=M.min(azimuth,minAzimuth)
        maxElev=M.max(elev,maxElev)
        minElev=M.min(elev,minElev)
    
        outputR=(maxData+minData)/2
        outputA=(maxAzimuth+minAzimuth)/2
        outputE=(maxElev+minElev)/2
    end
    oN(1,phyX)
    oN(2,phyY)
    oN(3,phyZ)
    oN(4,eulerX)
    oN(5,eulerY)
    oN(6,eulerZ)
    oN(20,outputR)
    oN(18,outputA)
    oN(19,outputE)
    oN(24,(maxSamp+1)*maxSampIO)
	oB(1,iB(1))
end

function clamp(v, min, max)
    return M.min(M.max(v, min), max)
end

function STRF(v)
    return SF("%1.3f",v)
end