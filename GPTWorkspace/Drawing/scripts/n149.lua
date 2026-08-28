
i=255
R=.5
ao=pairs
aI=ipairs
ai=false
aL=input
T=math
C=screen
E=C.drawText
d=C.drawLine
O=C.drawCircle
o=C.setColor
aW=T.abs
s=map.mapToScreen
ar=table.remove
f=aL.getNumber
ak=aL.getBool
q=T.floor
S=T.min
V=T.max
aH=0
ba=1
aO=2
function r(bx,bl)local be=property.getNumber(bx)if be==0 then
return bl
end
return be
end
ah={1,2,5,10,20,30,50,90}N=V(1,S(#ah,q(r("MAP_ZOOM_INDEX",1)+R)))p=ah[N]aq=r("HMD_MAP_CURSOR_ANGLE",45)/360
bq=0.18;ab=r("VECTOR_TIME",10)aR=r("VECTOR_MAX_PIXELS",35)aE=r("OWN_VEL_SMOOTH",.15)bt=r("RENDER_TRACK_TIMEOUT",90)bK=r("LOCKED_RENDER_TRACK_TIMEOUT",600)bp=r("MERGE_MARKER_TIMEOUT",60)bb=1480
bB=V(1,q(r("SONAR_RIPPLE_LIMIT",32)+R))n=r("RGBAlpha",i)M={}K={}A={}t=0
h=0
g=0
g=0
U=0
W=0
aP=ai
z=0
ac=0
aD=ai
Q=0
aS=ai
aN=ai
aT=0
b_=0
function by()if not ak(1)then
return
end
local m=q(f(2)+.5)if m<1 then
return
end
M[m]={f(3),f(4),f(5),f(6),f(7),f(8),f(9),q(f(10)+.5),q(f(11)+.5),q(f(13)+.5),q(f(14)+.5),t}end
function bC()local Y=q(f(18)+.5)if Y<1 then
return
end
K[#K+1]={f(15),f(16),f(17),Y,t}end
function bF()for X=#K,1,-1 do
if t-K[X][5]>bp then
ar(K,X)end
end
end
function bR()if not aD or Q<=0 then
return
end
A[#A+1]={h,g,t,Q}if#A>bB then
ar(A,1)end
end
function bS()for X=#A,1,-1 do
local w=A[X]local aw=(t-w[3])/60
local at=aw*bb
if at>w[4]then
ar(A,X)end
end
end
function bg(e,c)if Q<=0 then
return
end
local G,P=s(h,g,p,e,c,h,g)local au=s(h,g,p,e,c,h+Q,g)local F=aW(au-G)o(0,120,i,q(n*.35+R))O(G,P,F)end
function bO(e,c)o(0,180,i,q(n*.75+R))for bA,w in aI(A)do
local aw=(t-w[3])/60
local at=S(aw*bb,w[4])local G,P=s(h,g,p,e,c,w[1],w[2])local au=s(h,g,p,e,c,w[1]+at,w[2])local F=aW(au-G)if G+F>=0 and
G-F<e and
P+F>=0 and
P-F<c then
O(G,P,F)end
end
end
function bv()local aQ=ak(9)local aU=ak(10)if aQ and not aS then
N=V(1,N-1)end
if aU and not aN then
N=S(#ah,N+1)end
aS=aQ
aN=aU
p=ah[N]end
function bu()for m,j in ao(M)do
local bn=m==z and
bK or
bt
if t-j[12]>bn then
M[m]=nil
end
end
end
function aC(j)local Z=(t-j[12])/60
if Z<0 then
Z=0
end
return
j[1]+j[4]*Z,j[2]+j[5]*Z,j[3]+j[6]*Z
end
function br()local _=f(27)local a=f(28)local bz=f(29)if aP then
local aa=(_-h)*60
local an=(a-g)*60
local bo=aa*aa+an*an
if bo<1000000 then
U=U+(aa-U)*aE
W=W+(an-W)*aE
else
U=0
W=0
end
else
aP=true
end
h=_
g=a
g=bz
end
function aK(ap,am,L,I,as)local y=L-ap
local u=I-am
local D=T.sqrt(y*y+u*u)if D<1 then
return
end
if D>as then
local aZ=as/D
y=y*aZ
u=u*aZ
L=ap+y
I=am+u
D=as
end
d(ap,am,L,I)if D<3 then
return
end
local aF=y/D
local aB=u/D
local aJ=-aB
local aY=aF
local bd=4
local al=2.5
local aM=L-aF*bd
local aX=I-aB*bd
d(L,I,aM+aJ*al,aX+aY*al)d(L,I,aM-aJ*al,aX-aY*al)end
function bL(_,a,az,af)if az==aH then
O(_,a,3)elseif az==ba then
d(_,a-4,_+4,a)d(_+4,a,_,a+4)d(_,a+4,_-4,a)d(_-4,a,_,a-4)else
C.drawRect(_-4,a-4,8,8)if af==aH or
af==aO then
d(_-2,a,_+2,a)end
if af==ba or
af==aO then
d(_,a-2,_,a+2)end
end
end
function ay(_,a)local b=8
local B=3
d(_-b,a-b,_-b+B,a-b)d(_-b,a-b,_-b,a-b+B)d(_+b,a-b,_+b-B,a-b)d(_+b,a-b,_+b,a-b+B)d(_-b,a+b,_-b+B,a+b)d(_-b,a+b,_-b,a+b-B)d(_+b,a+b,_+b-B,a+b)d(_+b,a+b,_+b,a+b-B)end
function bk()if aq<=0 then
return 0,0
end
local _=aT/aq
local a=-b_/aq
_=V(-1,S(1,_))a=V(-1,S(1,a))return _,a
end
function bj(e,c,k,l,aj)if z>0 then
return 0
end
local bc=0
local ax=aj*aj
for m,j in ao(M)do
local ad,bi,ae=aC(j)local v,x=s(h,g,p,e,c,ad,ae)if v>=0 and v<e and
x>=0 and x<c then
local y=v-k
local u=x-l
local aG=y*y+u*u
if aG<ax then
ax=aG
bc=m
end
end
end
return bc
end
function findEncodedLock(v)if v<1 then return 0 end
local c=q(v/10)local so=v-c*10==1
for m,j in ao(M)do
if(so and j[11]==c)or(not so and j[10]==c)then return m end
end
return 0
end
function bJ(k,l,aj)o(0,i,0,n)O(k,l,aj)d(k-7,l,k-2,l)d(k+2,l,k+7,l)d(k,l-7,k,l-2)d(k,l+2,k,l+7)end
function bh(_,a,Y)if Y==1 then
o(0,0,i,q(n*.4+.5))elseif Y==2 then
o(0,i,80,n)else
o(i,40,40,n)end
d(_-6,a-6,_+6,a+6)d(_+6,a-6,_-6,a+6)end
function onTick()t=t+1
aD=ak(2)Q=f(19)bq=f(22)aT=f(25)b_=f(26)by()local ag=q(f(12)+R)local en=q(f(1)+R)local em=findEncodedLock(en)
if ag>0 and M[ag]then
z=ag
elseif em>0 then
z=em
elseif ag==0 and en==0 then
z=0
elseif z==0 and
ac>0 then
z=ac
end
bC()bu()bF()br()bR()bS()bv()end
function onDraw()local e=C.getWidth()local c=C.getHeight()C.drawMap(h,g,p)bg(e,c)bO(e,c)o(i,i,i,n)E(e/2-2,2,"N")E(e-5,c/2-2,"E")E(e/2-2,c-6,"S")E(1,c/2-2,"W")local H,J=s(h,g,p,e,c,h,g)local bP=h+U*ab
local bG=g+W*ab
local bm,bI=s(h,g,p,e,c,bP,bG)o(0,i,80,n)aK(H,J,bm,bI,aR)O(H,J,4)d(H-2,J,H+2,J)d(H,J-2,H,J+2)for bA,av in aI(K)do
local bM,bD=s(h,g,p,e,c,av[1],av[3])bh(bM,bD,av[4])end
local bw,bE=bk()local k=e/2+bw*e/2
local l=c/2+bE*c/2
local aA=bq*c/2
ac=bj(e,c,k,l,aA)for m,j in ao(M)do
local ad,bi,ae=aC(j)local v,x=s(h,g,p,e,c,ad,ae)local bs=ad+j[4]*ab
local bN=ae+j[6]*ab
local aa,bQ=s(h,g,p,e,c,bs,bN)o(i,40,40,n)aK(v,x,aa,bQ,aR)if m==z then
o(i,220,0,n)elseif m==ac then
o(0,220,i,n)else
o(i,40,40,n)end
bL(v,x,j[8],j[9])if m==z then
ay(v,x)elseif m==ac then
ay(v,x)end
E(v+6,x-3,m)end
bJ(k,l,aA)o(i,i,i,n)local bf=p.."km"
E(2,c-6,bf)local aV=ab.."s"
E(e-#aV*4-2,c-6,aV)end
