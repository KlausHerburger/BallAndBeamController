%1.) State space

syms mR mW mB kr MR MW rR lW g
syms z zd zdd x xd xdd phi phid phidd 

% Equations of motion
fun1 = (mR + mW + mB)*xdd - mB*z*sin(phi)*phidd + mB*cos(phi)*zdd - 2*mB*zd*phid*sin(phi) - mB*xd*phid*z*cos(phi) == -kr*xd + MR/rR;
fun2 = -mB*z*sin(phi)*xdd + (1/12*mW*lW^2 + mB*z^2)*phidd + 2*mB*phid*zd*z + mB*g*z*cos(phi) == MW;
fun3 = mB*cos(phi)*xdd + 3/2*mB*zdd - mB*phid^2*z + mB*g*sin(phi) == 0;

eqns = [fun1, fun2, fun3];
S = solve(eqns,[zdd, xdd, phidd]);

% Linearization
syms MR MW
syms z zd zdd x xd xdd phi phid phidd
mR = 0.8;
mW = 0.01;
mB = 0.05;
lW = 0.6;
rR = 0.041;
kr = 7.2*10^-3;
g = 9.81;

fun1 = (mR + mW + mB)*xdd - mB*z*sin(phi)*phidd + mB*cos(phi)*zdd - 2*mB*zd*phid*sin(phi) - mB*xd*phid*z*cos(phi) == -kr*xd + MR/rR;
fun2 = -mB*z*sin(phi)*xdd + (1/12*mW*lW^2 + mB*z^2)*phidd + 2*mB*phid*zd*z + mB*g*z*cos(phi) == MW;
fun3 = mB*cos(phi)*xdd + 3/2*mB*zdd - mB*phid^2*z + mB*g*sin(phi) == 0;
eqns = [fun1, fun2, fun3];
S = solve(eqns,[zdd, xdd, phidd]);

xz = [x,phi,z,xd,phid,zd];
u = [MR, MW];
g_ = [x;phi;z];
f = [xd, phid, zd, S.xdd, S.phidd, S.zdd];
A_ = jacobian(f,xz);
B_ = jacobian(f,u);
C_ = jacobian(g_,xz);
D_ = jacobian(g_,u);

A = subs(A_,{phi,z,xd,phid,zd},{0,0,0,0,0});
B = subs(B_,{phi,z,xd,phid,zd},{0,0,0,0,0});
C = subs(C_,{phi,z,xd,phid,zd},{0,0,0,0,0});
D = subs(D_,{phi,z,xd,phid,zd},{0,0,0,0,0});

% Stability, controllability, observability

eig(A)
% One eigenvalue has positive real part -> system unstable

S0 = [B,A*B,A*A*B,A*A*A*B,A*A*A*A*B,A*A*A*A*A*B];
rank(S0)
% The controllability matrix has full rank -> the system is controllable

P0 = [C;C*A;C*A*A;C*A*A*A;C*A*A*A*A;C*A*A*A*A*A];
rank(P0)
% The observability matrix has full rank -> the system is observable

% Simplified model

g_ = z;
u = MW;
f = [phid, zd, (MW - 2*mB*phid*zd*z-mB*g*z*cos(phi))/(1/12*mW*lW^2+mB*z^2),2/3*(phid^2*z-g*sin(phi))];
xz = [phi,z,phid,zd];
A_phiz = jacobian(f,xz);
B_phiz = jacobian(f,u);
C_phiz = jacobian(g_,xz);
D_phiz = jacobian(g_,u);

Aphiz = subs(A_phiz,{phi,z,phid,zd},{0,0,0,0});
Bphiz = subs(B_phiz,{phi,z,phid,zd},{0,0,0,0});
Cphiz = subs(C_phiz,{phi,z,phid,zd},{0,0,0,0});
Dphiz = subs(D_phiz,{phi,z,phid,zd},{0,0,0,0});

eig(Aphiz)
% Ooe eigenvalue has positive real part -> system unstable

S0phiz = [Bphiz,Aphiz*Bphiz,Aphiz*Aphiz*Bphiz,Aphiz*Aphiz*Aphiz*Bphiz];
rank(S0phiz)
% The controllability matrix has full rank -> the system is controllable

P0phiz = [Cphiz;Cphiz*Aphiz;Cphiz*Aphiz*Aphiz;Cphiz*Aphiz*Aphiz*Aphiz];
rank(P0phiz)
% The observability matrix has full rank -> the system is observable

% Transfer functions

syms s
G_v2y3 = Cphiz*inv(s*eye(4)-Aphiz)*Bphiz
% G_v2y3 has poles with positive real part -> unstable

G_v1y1 = 1/(rR*s*(kr+s*(mR+mB+mW)))
% G_v1y1 has one pole on the imaginary axis -> boundary stable