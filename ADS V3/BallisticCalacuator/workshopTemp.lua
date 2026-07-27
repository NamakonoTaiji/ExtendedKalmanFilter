fn="y (m)"
fm="z (m)"
fl="x (m)"

v=360
aY=.1
bg=1000
bt=600
ap=false
df=property
ef=output
dM=input
B=math
cz=B.acos
bx=B.exp
cP=B.abs
r=B.atan
bQ=table.unpack
k=B.cos
n=B.sin
aK=B.sqrt
c=dM.getNumber
bR=dM.getBool
af=ef.setNumber
el=ef.setBool
g=df.getNumber
ei=df.getBool
F=B.pi*2
x=30/3600
dC=bt/3600
fj=60
i=0
fa=0
dp=7.5
dB=9.5
dA=8
dO=0
ff=1
eB=.05
eC=1
cn=2000
cH=60*(cn/bg)dj=aK(240*cn)eD=B.floor(3600/cH)dF=8
ek=aY
d_=aY
bK=10000
ea=2
bD={{bt,.0005,2400,.105},{700,.001,2400,.11},{800,.002,2400,.12},{900,.005,bt,.125},{bg,.01,300,.13},{bg,.02,150,.135},{800,.025,120,.15},{50,.003,3600,.125}}do
function dl(dW,dD,cW)return aL({0,n(-cW/2),0,k(-cW/2)},aL({0,0,n(-dD/2),k(-dD/2)},{n(-dW/2),0,0,k(-dW/2)}))end
function aL(f,S)local q,a,j,o=bQ(f)local _,b,e,bG=bQ(S)return{o*_-j*b+a*e+q*bG,j*_+o*b-q*e+a*bG,-a*_+q*b+o*e+j*bG,-q*_-a*b-j*e+o*bG}end
function aF(ad,ag,N,aH,aW,aC,f)_,b,e=bQ(aL(f,aL({ad,ag,N,0},{-f[1],-f[2],-f[3],f[4]})))return _+aH,b+aC,e+aW
end
function W(X,Y,au,aH,aW,aC,f)return bQ(aL({-f[1],-f[2],-f[3],f[4]},aL({X-aH,Y-aC,au-aW,0},f)))end
function aR(eq,a,l,q,w,db,cl)local bv,p,j,ah,Z,aj,o,T,aV,S,f,ca,cQ
bv=w>0 and w or-w
j,ah,Z=q,w,bv
aj=a-q
o=aj
cQ=ap
for ew=1,db do
p=l>0 and l or-l
if l*w<0 then cQ=true end
if p<cl then
return a,p,ew
end
if w*l>0 then
q,w,bv=j,ah,Z
o=a-j
aj=o
end
if bv<p then
q,a,j=a,j,a
w,l,ah=l,ah,l
bv,p,Z=p,Z,p
end
if cQ then
T=(q-a)/2
aV=l/ah
if Z<p then
o,aj=T,T
else
if q==j then
S=2*T*aV
f=1-aV
else
f=ah/w
ca=l/w
S=aV*(2*T*f*(f-ca)-(a-j)*(ca-1))f=(f-1)*(ca-1)*(aV-1)end
aV,aj=aj,o
ct=3*T*f
if 2*(S>0 and S or-S)<(ct>0 and ct or-ct)then
o=-S/f
else
o,aj=T,T
end
end
j,ah,Z=a,l,p
a=a+o
else
bX=l-w
if p>Z*1.1 then
cR=(j+a)/2
elseif((bX>0 and bX or-bX)<cl*aY or a==j)and p>cl then
cR=a+.001
else
cR=a-l*(a-q)/bX
end
ah,j,Z=l,a,p
a=cR
end
l=eq(a)end
return a,p,db
end
function eJ(bI,bW,cr,cy,ak)local dr,cX,_,b,e,dd,du,cT,aX,as
dr=bI*n(bW*F)-cr
cX=bI*k(bW*F)-cy
_,b,e=aF(dr,cX,0,0,0,0,ak)dd,du,cT=aF(0,0,1,0,0,0,ak)bH=_-(dd*e)/cT
bL=b-(du*e)/cT
aX=r(bH,bL)as=bF(bH,bL)return as,aX
end
function bF(_,b)return aK(_*_+b*b)end
function ax(_,min,max)if _>=max then
_=max
elseif _<=min then
_=min
end
return _
end
function cC(h,_,b,e,an,aA,s,ay,aB,aa)return ay*h*h/2+an*h+_,aB*h*h/2+aA*h+b,aa*h*h/2+s*h+e,ay*h+an,aB*h+aA,aa*h+s
end
dm,dz=0,0
dL,dh=0,0
dy,eb=0,0
ej,dc=0,0
function cf(aE,aN,aI,eN,eZ,cp,eL,min,max)local ar,bf,ck,bb
ar=eN-eZ
bf=cP(ar)<5/v and cp+ar or cp
ck=ar-eL
bb=aE*ar+aN*bf+aI*ck
if bb>max or bb<min then
bf=cp
bb=aE*ar+aN*bf+aI*ck
end
return ax(bb,min,max),bf,ar
end
function Q(_)return(_+.5)%1-.5
end
function c_(aH,aW,aC,ak,cr,er,cy,eA,fc,eW,M,G,H,J,I,K,bq,br,bs,dx)local bh,bw,b_,dE,cZ,ds,dY,di,dw,at,az,av,co,cS,cs,bP,aQ,cos,sin,bz,dg,dv,dU,bu,ba,bn
bh,bw,b_=W(M,G,H,aH,aW,aC,ak)dE,cZ,ds=W(J,I,K,0,0,0,ak)dY,di,dw=W(eA,eW,fc,0,0,0,ak)bu,ba,bn=W(bq,br,bs,aH,aW,aC,ak)co,cS,cs=dE-cr,cZ-cy,ds-er
bP=bh*bh+bw*bw+b_*b_
at=(bw*cs-b_*cS)/bP-(-dY)az=(b_*co-bh*cs)/bP-(-di)av=(bh*cS-bw*co)/bP-(-dw)aQ=aK(at*at+az*az+av*av)cos=k(aQ*dx)sin=n(aQ*dx)/aQ
bz=(at*bu+az*ba+av*bn)*(1-cos)/aQ/aQ
dg=cos*bu+sin*(az*bn-av*ba)+bz*at
dv=cos*ba+sin*(av*bu-at*bn)+bz*az
dU=cos*bn+sin*(at*ba-az*bu)+bz*av
return bB(dg,dv,dU,ap)end
function bB(_,b,e,ep)local ch,cL
ch=r(e,aK(_*_+b*b))cL=r(_,b)if ep then
return ch,cL
else
return ch/F,cL/F
end
end
function bY(X,Y,au,t)local de,dS=bF(X,Y),r(X,Y)-t
return de*n(dS),de*k(dS),au
end
function bT(h)local exp,cM,cI,aT,bA
aT=i+h
bA=aT*aT/2
dX=eS*bA+ex*aT+ev
bZ=eR*bA+eX*aT+cU
ci=eP*bA+ey*aT+eg
exp=bx(-d*h)cM=(bj-bo/d)*(d*h-1+exp)/d/d/h+bo*h/2/d+by+m
cI=fe*(((44.20-cM/bg)/11.89)^5.256)/60780
x=bx(-cM/60000)/120
ay=-bH*cI
aB=cx-bL*cI
aa=bV-x
_,an=((cd-ay/d)*(1-exp)+ay*h)/d+cw,(cd-ay/d)*exp+ay/d
b,aA=((ce-aB/d)*(1-exp)+aB*h)/d+cE,(ce-aB/d)*exp+aB/d
e,s=((bj-aa/d)*(1-exp)+aa*h)/d+by,(bj-aa/d)*exp+aa/d
ac=cn/(s<0 and-s or s)ac=ac>dj and dj or ac
ac=ac<cH and cH or ac
U=aG and(ci-e)or(b-bZ)aJ=U>0 and(not aG or s<0)return U,aJ
end
function cq(aU)ev,cU,eg=bY(aw,ao,ai,am)ex,eX,ey=bY(J,I,K,am)eS,eR,eP=bY(bU,bJ,bN,am)bH=as*n(aX-am)bL=as*k(aX-am)x=bx(-m/60000)/120
_,b,e=es,dt*k(dK+aU),dt*n(dK+aU)an,aA,s=dN*n(dT-am),bS*k(aU)+dN*k(dT-am),bS*n(aU)+eo
local y=60
i=0
cw,cE,by,cd,ce,bj=_,b,e,an,aA,s
bo=-x
cx,bV=0,0
cK=-bK
aJ=ap
if cg then
cx=dC*k(aU)bV=dC*n(aU)bo=-x+bV
U,aJ=bT(y)if aJ then
i,em,bE=aR(bT,y,U,0,(aG and eg or cU),10,.01)return aG and(bZ-b)or(ci-e)end
cK=U
i=i+y
y=ac
cx,bV=0,0
cw,cE,by,cd,ce,bj=_,b,e,an,aA,s
bo=-x
end
bE=0
for fh=1,eD do
if i>dk then
break
end
U,aJ=bT(y)if aJ then
y,em,bE=aR(bT,y,U,0,cK,10,.01)i=i+y
break
end
cK=U
i=i+y
y=ac
cw,cE,by,cd,ce,bj=_,b,e,an,aA,s
bo=aa
end
return aG and(bZ-b)or(ci-e)end
function cm(eE)am=eE
eu=cq(u)cY=u+.001
eT=cq(cY)u,eK,bE=aR(cq,cY,eT,u,eu,dF,ek)return dX-_
end
end
function onTick()do
M,G,H=c(1),c(2),c(3)J,I,K=c(4),c(5),c(6)bU,bJ,bN=c(7),c(8),c(9)z=dl(c(10),c(11),c(12))aS,bp,aP=c(13)/60,c(14)/60,c(15)/60
bO,cb,cc=c(16)*F/60,c(17)*F/60,c(18)*F/60
A,m,D=c(19),c(20),c(21)cJ=dl(c(22),c(23),c(24))bI=c(25)/60
bW=c(26)aO=" (degree)"
al=g("Weapon Type")cB=g("Stabilizer")ez=cB>=0
bm=g("standby pitch position"..aO)/v
cv=g("standby yaw position"..aO)/v
bk=g("min pitch"..aO)/v
bc=g("max pitch"..aO)/v
cD=ei("Pitch Swivel Mode")bl=g("min yaw"..aO)/v
bi=g("max yaw"..aO)/v
cA=ei("Yaw Swivel Mode")ae=g("Pivot rotation speed gain")da=g("Pitch gear ratio (1 : ?)")/g("Types of Pitch PIVOT")ee=g("Yaw gear ratio (1 : ?)")/g("Types of Yaw PIVOT")f_=g("manual P")fd=g("manual I")eF=g("manual D")aq="Turret phy. offset "
fb=-g(aq..fl)eY=-g(aq..fn)eQ=-g(aq..fm)aq="Muzzle offset "
es=g(aq..fl)eh=g(aq..fn)cV=g(aq..fm)dt=bF(eh,cV)dK=r(cV,eh)cj=bR(1)cO=bR(2)aG=bR(4)dQ=bR(5)end
do
bd=ap
i=0
ab,R=bm,cv
be,bM=bm,cv
aM,aZ=bm,cv
t,u=0,0
end
do
X,Y,au=aF(0,1,0,0,0,0,cJ)ad,ag,N=W(X,Y,au,0,0,0,z)P,O=bB(ad,ag,N,ap)X,Y,au=aF(0,0,1,0,0,0,cJ)ad,ag,N=W(X,Y,au,0,0,0,z)if N<0 then
P=Q(.5-P)O=Q(O+.5)end
end
dJ,ed,eo=aF(aS,aP,bp,0,0,0,z)if cj and cO and al~=9 then
bS,d,dk,fe=bD[al][1]/60,bD[al][2],bD[al][3],bD[al][4]cg=al==8
A,D,m=aF(fb,eY,eQ,A,m,D,cJ)aw,ao,ai=M-A,G-D,H-m
aw,ao,ai,J,I,K=cC(fa,aw,ao,ai,J,I,K,bU,bJ,bN)dN=bF(dJ,ed)dT=r(dJ,ed)as,aX=eJ(bI,bW,aS,aP,z)as=as/((((44.33-m/bg)/11.89)^5.256)/1013)x=bx(-m/60000)/120
do
i=aK(aw*aw+ao*ao+ai*ai)/(bS+(cg and bt or 0))eI,eG,fi=cC(i,aw,ao,ai,J,I,K,bU,bJ,bN)t=r(eI,eG)fk,V,cN=bY(aw,ao,ai,t)L=bS+(cg and bt/60 or 0)cG=-V*x/L
bC=cz(d*V/aK(cG*cG+L*L))+r(cG,L)function aD(a)return V*(L*n(a)+x/d)/L/k(a)+x*B.log(1-d*V/L/k(a))/(d*d)-cN
end
if not aG then
u=aR(aD,r(cN,V),aD(r(cN,V)),bC,aD(bC),10,(aY/v)*F)else
u=aR(aD,cz(d*V/L)-1e-6,aD(cz(d*V/L)-1e-6),bC,aD(bC),10,(aY/v)*F)end
end
do
en=cm(t)eV=u>F/4 and-2 or 2
dI=t+(r(dX,bZ)-r(_,b))*eV
t,fg,bE=aR(cm,dI,cm(dI),t,en,dF,d_)end
bd=i<dk and fg<d_ and eK<ek
bq,br,bs=A+bK*k(u)*n(t),D+bK*k(u)*k(t),m+bK*n(u)ad,ag,N=W(bq,br,bs,A,m,D,z)ab,R=bB(ad,ag,N,ap)dG,dn,dH,dP,dV,dq=cC(i,M,G,H,J,I,K,bU,bJ,bN)end
if not bd and cj and cO then
be,bM=c_(A,m,D,z,aS,bp,aP,bO,cb,cc,M,G,H,J,I,K,M,G,H,dp)aM,aZ=c_(A,m,D,z,aS,bp,aP,bO,cb,cc,M,G,H,J,I,K,M,G,H,dB)ad,ag,N=W(M,G,H,A,m,D,z)ab,R=bB(ad,ag,N,ap)end
do
et=Q(O)>bl and Q(O)<bi and P>bk and P<bc
eM=ab>bk and ab<bc
eU=R>bl and R<bi
dZ=cP(Q(ab-P))*v
e_=cP(Q(R-O))*v
eO=dZ<ea and e_<ea
eH=((al==9 and cO and cj)or bd)and eO and et and eM and eU and not dQ
end
do
if ez then
if bd then
be,bM=c_(A,m,D,z,aS,bp,aP,bO,cb,cc,dG,dn,dH,dP,dV,dq,bq,br,bs,dp)aM,aZ=c_(A,m,D,z,aS,bp,aP,bO,cb,cc,dG,dn,dH,dP,dV,dq,bq,br,bs,dB)end
if dQ then
be=bm
aM=bm
end
cF=cA and(ax(bM,bl,bi)-O)or Q(bM-O)cu=cD and(ax(be,bk,bc)-P)or be-P
dR,dz,dm=cf(dA,dO,cB,0,-cu,dz,dm,-ae,ae)ec,dh,dL=cf(dA,dO,cB,0,-cF,dh,dL,-ae,ae)aE,aN,aI=ff,eB,eC
else
aE,aN,aI=f_,fd,eF
dR,ec=0,0
aM,aZ=ab,R
end
cF=cA and(ax(R,bl,bi)-O)or Q(R-O)cu=cD and(ax(ab,bk,bc)-P)or ab-P
E,dy,eb=cf(aE,aN,aI,0,-cu,dy,eb,-ae,ae)C,ej,dc=cf(aE,aN,aI,0,-cF,ej,dc,-ae,ae)E=dR+E
C=ec+C
E=E*da
C=C*ee
if da<0 then
E=cD and ax(aM,bk,bc)*4 or aM*4
end
if ee<0 then
C=cA and ax(aZ,bl,bi)*4 or aZ*4
end
E=(E~=E)and 0 or E
C=(C~=C)and 0 or C
end
af(1,E)af(2,C)el(1,eH)el(2,bd)af(3,dZ)af(4,e_)af(5,i/60)af(30,i)af(31,u)af(32,t)end
