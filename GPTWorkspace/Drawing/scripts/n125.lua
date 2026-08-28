de=":"

aX=true
by=ipairs
n=pairs
l=nil
R=false
bG=output
bh=input
k=bh.getNumber
o=bh.getBool
e=bG.setNumber
K=bG.setBool
aJ=property.getNumber
d=math
aT=d.pi
aI=aT*2
function N(cW,cH)local bD=aJ(cW)if bD==0 then return cH end
return bD
end
cd=aJ("offsetX")ch=aJ("offsetY")ca=aJ("offsetZ")bo=N("TARGET_LOST_THRESHOLD_TICKS",120)cT=N("ASSOC_POS_GATE",250)cB=N("ASSOC_VEL_GATE",60)bm=2
bb=3
bA=d.max(1,d.floor(N("ASSOC_CONFIRM",2)+.5))bs=3
bV=bs*2
cb=bA
co=120
cv=bo*2
aH=R
J=0
ao=1
S=0
U=1
as=2
t={}v={}h={}aW={}m=0
aQ=1
i=0
V=l
aS=0
aL=0
aO=0
T=S
bf=R
aB=R
bI=0
b_=R
aN={1,2,5,10,20,30,50,90}Y=d.max(1,d.min(#aN,d.floor(N("MAP_ZOOM_INDEX",1)+.5)))br=aN[Y]bv=R
bx=R
cM=N("HMD_MAP_ASPECT",4/3)aU=N("HMD_MAP_CURSOR_ANGLE",45)/360
bB=0.01;function cE(f,g,j,bO,bt,bl)local ad,Z,ak,O,ab,ai
ad,Z=d.cos(bO),d.sin(bO)ak,O=d.cos(bt),d.sin(bt)ab,ai=d.cos(bl),d.sin(bl)return
ad*ak*f+Z*ak*g-O*j,(ad*O*ai-Z*ab)*f+(Z*O*ai+ad*ab)*g+ak*ai*j,(ad*O*ab+Z*ai)*f+(Z*O*ab-ad*ai)*g+ak*ab*j
end
function r(u)local aD=(m-u[9])/60
if aD<0 then aD=0 end
return
u[1]+u[4]*aD,u[2]+u[5]*aD,u[3]+u[6]*aD,u[4],u[5],u[6],u[7]end
function aF(aA,ax,X)local cr,cL,cj,bU,cm,cA=r(aA)local bX,bT,cf,cZ,cK,cn=r(ax)local ay,bu,ap=cr-bX,cL-bT,cj-cf
local bq,bJ,bM=bU-cZ,cm-cK,cA-cn
local bP=ay*ay+bu*bu+ap*ap
local bj=bq*bq+bJ*bJ+bM*bM
local an=cT*X
local aw=cB*X
an=an*an
aw=aw*aw
if bP>an or bj>aw then return l end
return bP/an+bj/aw
end
function cl(Q,c)for G,_ in n(h)do
if(Q==J and _[2]==c)or(Q==ao and _[3]==c)then
return _
end
end
end
function bc(Q,c)if not c or c<1 then return l end
local _={aQ,0,0,0,-1,-1,m,m,0,0,0,0,0,0,0,Q}aQ=aQ+1
if Q==J then
_[2]=c
else
_[3]=c
end
h[_[1]]=_
return _
end
function bp(Q,c)if c and not cl(Q,c)then
bc(Q,c)end
end
function bz(w,aP)if k(w+9)~=1 then return l end
local c=d.floor(k(w+7)+.5)if c<1 then return l end
aP[c]={k(w+1),k(w+2),k(w+3),k(w+4),k(w+5),k(w+6),k(w+8),k(w+10),m}return c
end
function bK(aP)for c,c_ in n(aP)do
if m-c_[9]>bo then
aP[c]=l
end
end
end
function bn(P,ag,ah,cx)local s=aW[P]if not s or m-s[4]>co then
s={0,-1,-1,m}aW[P]=s
end
if s[2]~=ag and s[3]~=ah then
s[1]=s[1]+1
s[2]=ag
s[3]=ah
s[4]=m
end
return s[1]>=cx
end
function cQ(_)local a,b,f,g,j,z,B,A,D,E
a=_[2]>0 and t[_[2]]or l
b=_[3]>0 and v[_[3]]or l
if a or b then _[8]=m end
if _[2]>0 and _[3]>0 then
if T==as then
if a and b then
local cU,bY,da,cR,ct,cS=r(a)local cP,O,cN,d_,ck,cy=r(b)f=(cU+cP)/2
g=(bY+O)/2
j=(da+cN)/2
z=(cR+d_)/2
B=(ct+ck)/2
A=(cS+cy)/2
D=0
E=as
elseif a then
f,g,j,z,B,A,D=r(a)E=S
elseif b then
f,g,j,z,B,A,D=r(b)E=U
end
elseif T==S then
if a then
f,g,j,z,B,A,D=r(a)E=S
elseif aB and b then
f,g,j,z,B,A,D=r(b)E=U
end
else
if b then
f,g,j,z,B,A,D=r(b)E=U
elseif aB and a then
f,g,j,z,B,A,D=r(a)E=S
end
end
elseif a then
f,g,j,z,B,A,D=r(a)E=S
elseif b then
f,g,j,z,B,A,D=r(b)E=U
end
if f then
_[9],_[10],_[11]=f,g,j
_[12],_[13],_[14]=z,B,A
_[15]=D or 0
_[16]=E
elseif _[9]~=0 then
_[9]=_[9]+_[12]/60
_[10]=_[10]+_[13]/60
_[11]=_[11]+_[14]/60
end
end
function bF()for G,_ in n(h)do
cQ(_)end
for c,_ in n(h)do
local a,b
a=_[2]>0 and t[_[2]]b=_[3]>0 and v[_[3]]if not a and not b and
m-_[8]>cv then
h[c]=l
if i==c then
i=0
end
end
end
end
function bk(_)return{_[9],_[10],_[11],_[12],_[13],_[14],0,0,m}end
function bg()local _=h[i]if _ and _[9]~=0 then
V=bk(_)end
end
function cu()if i~=0 or
not V then
return
end
local ae=d.huge
local aY=0
for c,_ in n(h)do
if _[9]~=0 then
local y=aF(V,bk(_),bb)if y and y<ae then
ae=y
aY=c
end
end
end
if aY>0 then
i=aY
end
end
function cO()local aV={}for G,_ in n(h)do
if _[2]>0 and _[3]>0 then
local a,b
a=t[_[2]]b=v[_[3]]if a and b and
a[9]~=_[5]and
b[9]~=_[6]then
local X=i==_[1]and bb or bm
if aF(a,b,X)then
_[4]=0
else
_[4]=_[4]+1
end
_[5]=a[9]_[6]=b[9]local cg=i==_[1]and bV or bs
if _[4]>=cg then
aV[#aV+1]=_
end
end
end
end
for G,_ in by(aV)do
local aZ,aj,aC
aZ=R
if i==_[1]then
aZ=T==U or(T==as and bf)end
if aZ then
aj=_[2]_[2]=0
bc(J,aj)else
aC=_[3]_[3]=0
bc(ao,aC)end
_[4]=0
_[5]=-1
_[6]=-1
end
end
function cq(ac,af)local H,remove
if i==af[1]then
H,remove=af,ac
elseif i==ac[1]or ac[7]<=af[7]then
H,remove=ac,af
else
H,remove=af,ac
end
H[2]=ac[2]H[3]=af[3]H[4]=0
H[5]=-1
H[6]=-1
H[8]=m
h[remove[1]]=l
return H
end
function cc()local aG={}for G,p in n(h)do
if p[2]>0 and p[3]==0 and
t[p[2]]then
for G,q in n(h)do
if q[3]>0 and q[2]==0 and
v[q[3]]then
local y=aF(t[p[2]],v[q[3]],1)if y then
aG[#aG+1]={y,p,q}end
end
end
end
end
table.sort(aG,function(aA,ax)return aA[1]<ax[1]end)local ba,bd,p,q,a,b,P
ba={}bd={}for G,bH in by(aG)do
p=bH[2]q=bH[3]if h[p[1]]and
h[q[1]]and
not ba[p[1]]and
not bd[q[1]]then
ba[p[1]]=aX
bd[q[1]]=aX
a=t[p[2]]b=v[q[3]]P="a"..p[2]..de..q[3]if bn(P,a[9],b[9],bA)then
cq(p,q)end
end
end
end
function bS()local be,aE,a,b,X,ae,aM,I,F,aa
be={}for bN=1,2 do
for G,_ in n(h)do
aE=i==_[1]F=l
aa=l
aM=l
I=l
if _[2]>0 and _[3]>0 and((bN==1 and aE)or(bN==2 and not aE))then
a=t[_[2]]b=v[_[3]]if not a and b then
F=J
aa=b
elseif a and not b then
F=ao
aa=a
end
if F~=l then
X=aE and bb or bm
ae=d.huge
for G,C in n(h)do
if C[1]~=_[1]and
C[1]~=i and
not be[C[1]]then
local at
if F==J and
C[2]>0 and
C[3]==0 then
at=t[C[2]]elseif F==ao and
C[3]>0 and
C[2]==0 then
at=v[C[3]]end
if at then
local y=aF(aa,at,X)if y and y<ae then
ae=y
aM=at
I=C
end
end
end
end
if I then
be[I[1]]=aX
local ag,ah,bi,P
if F==J then
ag=aM[9]ah=aa[9]else
ag=aa[9]ah=aM[9]end
bi=F==J
and I[2]or I[3]P="q".._[1]..de..F..de..bi
if bn(P,ag,ah,cb)then
if F==J then
_[2]=I[2]else
_[3]=I[3]end
h[I[1]]=l
_[4]=0
_[5]=-1
_[6]=-1
end
end
end
end
end
end
end
function cD(_)if not _ then return 0 end
local a,b
a=_[2]>0 and t[_[2]]b=_[3]>0 and v[_[3]]if _[2]>0 and _[3]==0 then
return a and _[2]*10 or 0
end
if _[3]>0 and _[2]==0 then
return b and _[3]*10+1 or 0
end
if T==U or T==as and bf then
if b then return _[3]*10+1 end
if aB and a then return _[2]*10 end
else
if a then return _[2]*10 end
if aB and b then return _[3]*10+1 end
end
return 0
end
function dc(aA,ax)local W=ax-aA
while W<=-aT do W=W+aI end
while W>aT do W=W-aI end
return W
end
function dd()local bC=o(9)local bR=o(10)if bC and not bv then
Y=d.max(1,Y-1)end
if bR and not bx then
Y=d.min(#aN,Y+1)end
bv=bC
bx=bR
br=aN[Y]end
function cX()if aU<=0 then
return 0,0
end
local f=aL/aU
local g=-aO/aU
f=d.max(-1,d.min(1,f))g=d.max(-1,d.min(1,g))return f,g
end
function bW(L,M)if i~=0 or V then return end
local cY=br*1000
local aR=cY/2
local aK=aR/cM
local cJ,ce=cX()local cV=L+cJ*aR
local cI=M-ce*aK
local ar=bB*bB
for G,_ in n(h)do
if _[9]~=0 then
local cs=(_[9]-L)/aR
local cw=-(_[11]-M)/aK
if d.abs(cs)<=1 and
d.abs(cw)<=1 then
local ay=(_[9]-cV)/aK
local ap=(_[11]-cI)/aK
local az=ay*ay+ap*ap
if az<ar then
ar=az
i=_[1]end
end
end
end
end
function cp(L,al,M,au,av,am)if i~=0 or V then return end
local ar=.2741556778
local cF=aL*aI
local cG=-aO*aI
for G,_ in n(h)do
if _[9]~=0 then
local f,g,j=cE(_[9]-L,_[10]-al,_[11]-M,au,av,am)f=f+cd
g=g+ch
j=j+ca
local db=d.atan(f,j)local ci=d.atan(g,d.sqrt(f*f+j*j))local bw=dc(cF,db)local bL=ci-cG
local az=bw*bw+bL*bL
if az<ar then
ar=az
i=_[1]end
end
end
end
function cC()local x
for c,_ in n(h)do
if _[9]~=0 and
c>aS and(not x or c<x)then
x=c
end
end
if not x then
for c,_ in n(h)do
if _[9]~=0 and(not x or c<x)then
x=c
end
end
end
if x then
aS=x
return h[x]end
aS=0
end
function bZ(_)local bE=_~=l
K(1,bE)if not bE then
for cz=2,14 do
e(cz,0)end
e(12,i)return
end
local aq
if _[2]>0 and _[3]>0 then
aq=2
elseif _[3]>0 then
aq=1
else
aq=0
end
e(2,_[1])e(3,_[9])e(4,_[10])e(5,_[11])e(6,_[12])e(7,_[13])e(8,_[14])e(9,_[15])e(10,aq)e(11,_[16]or aq)e(12,i)e(13,_[2])e(14,_[3])end
function onTick()m=m+1
local L,al,M,am,av,au
local aC,aj
local bQ=0
bg()if m%3600==0 then
aW={}end
aH=o(31)b_=o(6)T=o(2)and as or(o(1)and U or S)bf=o(3)aB=o(4)dd()aL=k(25)aO=k(26)bI=k(21)bB=k(22)L=k(27)al=k(28)M=k(29)am=k(30)av=k(31)au=k(32)aC=bz(0,v)aj=bz(10,t)bp(ao,aC)bp(J,aj)bK(t)bK(v)bF()cO()bS()cc()bF()if not aH then
i=0
V=l
else
if V then
cu()else
if b_ then
bW(L,M)else
cp(L,al,M,au,av,am)end
end
bg()end
if aH and i>0 then
bQ=cD(h[i])end
e(1,bQ)bZ(cC())e(25,aL)e(26,aO)e(27,L)e(28,al)e(29,M)e(30,am)e(31,av)e(32,au)e(24,bI)e(22,bB)K(2,o(1))K(3,o(2))K(4,o(3))K(5,o(4))K(9,o(9))K(10,o(10))K(30,b_)K(31,aH)end
