cS="-"
cR=" "
cQ="S"
cP="m"
cO="R"

n=255
aF=true
bh=ipairs
bF=tostring
bM=pairs
ay=false
bD=input
k=math
v=screen
h=v.drawLine
bo=v.drawCircle
F=v.drawText
q=v.setColor
af=k.sqrt
aC=k.atan
aK=k.tan
aN=k.min
u=k.max
U=k.abs
c=bD.getNumber
l=k.floor
D=bD.getBool
aE=k.sin
aW=k.cos
aj=property.getNumber
aM=k.pi
C=aM*2
Y=0
M=1
an=2
function ar(cm,cr)local I=aj(cm)if I==0 then return cr end
return I
end
function onDraw()ck()end
cf=aj("offsetX")cH=aj("offsetY")cp=aj("offsetZ")o=ar("RGBAlpha",n)aL=k.rad(ar("HMD_FOV_HEIGHT",58))cg=ar("HMD_RENDER_TRACK_TIMEOUT",180)ci=ar("HMD_LOCKED_TRACK_TIMEOUT",300)ab={}N=0
aH=0
bi=0
aP=0
aB=0
aV=0
bg=0
aQ=0
aT=0
bt=ay
bp=0
bz=0
bG=0
r=0
bU=ay
be=Y
bT=ay
bu=ay
bA=0
function bI(ao,ah,ax)ao=ao/2
ah=ah/2
ax=ax/2
local Q,K,O,X,T,P
Q,K=aW(ao),aE(ao)O,X=aW(ah),aE(ah)T,P=aW(ax),aE(ax)return{Q*O*T+K*X*P,Q*O*P-K*X*T,Q*X*T+K*O*P,K*O*T-Q*X*P}end
function bV(_,a,g,al,cA)local j,d,e,m
local aS,bj,aD
local aU,ba,aG
local b_,aZ,bd
j,d,e,m=al[1],al[2],al[3],al[4]aS=1-2*(e*e+m*m)bj=2*(d*e-m*j)aD=2*(d*m+e*j)aU=2*(d*e+m*j)ba=1-2*(d*d+m*m)aG=2*(e*m-d*j)b_=2*(d*m-e*j)aZ=2*(e*m+d*j)bd=1-2*(d*d+e*e)if cA then
return
aS*_+aU*a+b_*g,bj*_+ba*a+aZ*g,aD*_+aG*a+bd*g
end
return
aS*_+bj*a+aD*g,aU*_+ba*a+aG*g,b_*_+aZ*a+bd*g
end
function cE(cv,cI)local z=cI-cv
while z<=-aM do z=z+C end
while z>aM do z=z-C end
return z
end
function cD()if not D(1)then return end
local i=l(c(2)+.5)if i<1 then return end
ab[i]={c(3),c(4),c(5),c(6),c(7),c(8),c(9),l(c(10)+.5),l(c(11)+.5),l(c(13)+.5),l(c(14)+.5),N}end
function cB()for i,f in bM(ab)do
local cM=i==r and
ci or
cg
if N-f[12]>cM then
ab[i]=nil
end
end
end
function cG(f)local V=(N-f[12])/60
if V<0 then V=0 end
return
f[1]+f[4]*V,f[2]+f[5]*V,f[3]+f[6]*V
end
function bZ(p)if p>=10000 then
local cJ=l(p/100)/10
return bF(cJ.."km")end
return l(p+.5)..cP
end
function ch(bw)local cj=l(U(bw)+.5)local cc=bw>=0 and "+" or cS
return cc..cj.."m/s"
end
function bL(I,bk,bR)if I<bk then return bk end
if I>bR then return bR end
return I
end
function cs(d,e)local bm=u(d[1],e[1])local bB=aN(d[1]+d[3],e[1]+e[3])local bv=u(d[2],e[2])local bH=aN(d[2]+d[4],e[2]+e[4])if bB<=bm or bH<=bv then
return 0
end
return(bB-bm)*(bH-bv)end
function bY(am,ai,J,E,cl,c_,w)local cK={{am+7,ai-E-2},{am+7,ai+3},{am-J-7,ai-E-2},{am-J-7,ai+3}}local ae
local bs=k.huge
for bl,by in bh(cK)do
local bN={bL(by[1],0,u(0,cl-J)),bL(by[2],0,u(0,c_-E)),J,E}local L=0
for bl,cu in bh(w)do
L=L+cs(bN,cu)end
if L<bs then
ae=bN
bs=L
if L==0 then break end
end
end
w[#w+1]=ae
return ae[1],ae[2]end
function onTick()N=N+1
cD()cB()aH=c(25)bi=c(26)local bJ=c(27)local bq=c(28)local bx=c(29)if bt then
local R=(bJ-aP)*60
local S=(bq-aB)*60
local y=(bx-aV)*60
local ce=R*R+S*S+y*y
if ce<1000000 then
bg=R
aQ=S
aT=y
else
bg=0
aQ=0
aT=0
end
else
bt=aF
end
aP=bJ
aB=bq
aV=bx
bp=c(30)bz=c(31)bG=c(32)r=l(c(12)+.5)bU=D(31)bA=c(24)be=D(3)and an or(D(2)and M or Y)bT=D(4)bu=D(5)end
function ck()local j=v.getWidth()local aq=v.getHeight()local ag=j/2
local av=aq/2
local cy=av/aK(aL/2)local br=2*aC(aK(aL/2)*j/aq)local cx=ag/aK(br/2)local cC=bI(bG,bz,bp)local ct=bI(0,aH*C,-bi*C)local ac={}local W
local bb
local as
for i,f in bM(ab)do
local cF,cd,cq=cG(f)local ap=cF-aP
local az=cd-aB
local at=cq-aV
local aR=af(ap*ap+az*az+at*at)if i==r then
bb=aR
if aR>.001 then
local cL=f[4]-bg
local cw=f[5]-aQ
local cz=f[6]-aT
as=-(ap*cL+az*cw+at*cz)/aR
else
as=0
end
end
local _,a,g=bV(ap,az,at,cC,aF)_=_+cf
a=a+cH
g=g+cp
local p=af(_*_+a*a+g*g)local aJ=aC(_,g)local aY=aC(a,af(_*_+g*g))local R,S,y=bV(_,a,g,ct,aF)if y>.1 then
local G=ag+cx*R/y
local H=av-cy*S/y
if G>=0 and G<j and
H>=0 and H<aq then
ac[#ac+1]={i,G,H,p,f[7],f[8],f[9],i==r,aJ,aY}elseif i==r then
W={aJ,aY}end
elseif i==r then
W={aJ,aY}end
end
table.sort(ac,function(d,e)if d[8]~=e[8]then
return d[8]end
return d[4]<e[4]end)local aa
if be==an then
aa="F>"..(bT and cQ or cO)elseif be==M then
aa=cQ
else
aa=cO
end
local cb=r>0 and "*" or(bU and "L" or cS)local bQ=cb..cR..aa..(bu and " A" or "")q(0,n,0,o)F(2,2,bQ)local cN=l(bA+.5)local bP="Active: "..bF(cN)..cP
F(2,8,bP)local aA
if r>0 then
local bE=bb and("RNG "..bZ(bb))or
"RNG ---"
local bC=as and("VC "..ch(as))or
"VC ---"
local bn=u(#bE,#bC)*4
local aI=u(0,j-bn-2)q(0,n,0,o)F(aI,2,bE)F(aI,8,bC)aA={aI,0,bn+2,15}end
local w={{0,0,u(#bQ,#bP)*4+4,14}}if aA then
w[#w+1]=aA
end
for bl,s in bh(ac)do
local i=s[1]local _=s[2]local a=s[3]local p=s[4]local bS=s[5]local au=s[6]local Z=s[7]local ca=s[8]q(n,0,0,o)if au==Y then
bo(_,a,4)elseif au==M then
h(_,a-4,_+4,a)h(_+4,a,_,a+4)h(_,a+4,_-4,a)h(_-4,a,_,a-4)else
v.drawRect(_-4,a-4,8,8)if Z==Y or
Z==an then
h(_-2,a,_+2,a)end
if Z==M or
Z==an then
h(_,a-2,_,a+2)end
end
if bS>0 then
q(n,n,0,o/2)bo(_,a,bS*10)end
if ca then
local b=10
local t=3
q(n,0,0,o)h(_-b,a-b,_-b+t,a-b)h(_-b,a-b,_-b,a-b+t)h(_+b,a-b,_+b-t,a-b)h(_+b,a-b,_+b,a-b+t)h(_-b,a+b,_-b+t,a+b)h(_-b,a+b,_-b,a+b-t)h(_+b,a+b,_+b-t,a+b)h(_+b,a+b,_+b,a+b-t)end
local ad
if au==Y then
ad=cO
elseif au==M then
ad=cQ
else
ad="RS"
end
local bK="ID "..i..cR..ad
local B
if p>=10000 then
B=l(p/100)/10
B=B.."k"
else
B=l(p)..cP
end
local J=u(#bK,#B)*4
local E=11
local bc,aX=bY(_,a,J,E,j,aq,w)q(0,180,100,o/2)h(_,a,bc,aX+5)q(0,120,n,o)F(bc,aX,bK)q(0,n,0,o)F(bc,aX+6,B)end
if W then
local co=aH*C
local cn=-bi*C
local A=cE(co,W[1])/(br/2)local x=(W[2]-cn)/(aL/2)if U(A)<.001 then A=.001 end
if U(x)<.001 then x=.001 end
local bW=6
local bX=aN((ag-bW)/U(A),(av-bW)/U(x))local aO=ag+A*bX
local bf=av+x*bX
local bO=af(A*A+x*x)local ak=A/bO
local aw=x/bO
local G=-aw
local H=ak
q(n,0,0,o)v.drawTriangleF(aO+ak*4,bf+aw*4,aO-ak*3+G*3,bf-aw*3+H*3,aO-ak*3-G*3,bf-aw*3-H*3)end
end
