%% Initialization of the controller

% Transfer function
s = tf('s');
G_v1y1 = tf([28.36],[1 0.008372 0]);

% State space model
A = [0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,981/2480,0,-27/3100,0,0;0,0,-1635,0,0,0;0,-42183/6200,0,9/1550,0,0];
B = [0,0;0,0;0,0;37500/1271,0;0,10000/3;-25000/1271,0];
b1 = B(:,1);
b2 = B(:,2);
C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];
D = zeros(3,2);

% Trajectory planner vectors
P = [b1, A*b1, A*A*b1, A*A*A*b1, A*A*A*A*b1, A*A*A*A*A*b1];
P_inv = inv(P);
kappa = 1/P_inv(6,1);
lambda = ([0 0 0 0 0 kappa]*P_inv)';
T = [lambda';lambda'*A;lambda'*A*A; lambda'*A*A*A; lambda'*A*A*A*A; lambda'*A*A*A*A*A];
a = -lambda'*A*A*A*A*A*A*inv(T);

% Trajectory planning
w0 = 0.2;
wT = 0.3;
T_end = 1;
syms w0 wT T_end
syms t
zquer = w0 + (wT - w0) * (1716*(t/T_end)^7 +  -9009*(t/T_end)^8 +    20020*(t/T_end)^9 + -24024*(t/T_end)^10 +   16380*(t/T_end)^11 +   -6006*(t/T_end)^12 +  924*(t/T_end)^13);
dzquer = diff(zquer,t);
ddzquer = diff(dzquer,t);
dddzquer = diff(ddzquer,t);
ddddzquer = diff(dddzquer,t);
dddddzquer = diff(ddddzquer,t);
ddddddzquer = diff(dddddzquer,t);
zquervec = [zquer;dzquer;ddzquer;dddzquer;ddddzquer;dddddzquer];
xquer = inv(T)* zquervec;
uquer = 1/kappa * (ddddddzquer + a*zquervec);
order = [13,12,11,10,9,8,7,6,5,4,3,2,1,24,23,22,21,20,19,18,17,16,15,14,35,34,33,32,31,30,29,28,27,26,25,46,45,44,43,42,41,40,39,38,37,36,57,56,55,54,53,52,51,50,49,48,47,68,67,66,65,64,63,62,61,60,59,58];
x1 = [coeffs(xquer(1),t),coeffs(xquer(2),t),coeffs(xquer(3),t),coeffs(xquer(4),t),coeffs(xquer(5),t),coeffs(xquer(6),t)];
%x = double(x1(order))
order = [12,11,10,9,8,7,6,5,4,3,2,1];
u1 = coeffs(uquer,t);
%u = double(u1(order));

% Simplified state space model

A = [0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,981/2480,0,-27/3100,0,0;0,0,-1635,0,0,0;0,-42183/6200,0,9/1550,0,0];
B = [0,0;0,0;0,0;37500/1271,0;0,10000/3;-25000/1271,0];
C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];
%C = eye(6);
D = zeros(3,2);

% Controller pole Placement
Poles = [-2,-5,-2.1,-0.7,-1,-0.5]*7;
K = place(A,B,Poles);

% Observer pole placement
ObserverPoles = Poles*3;
L = place(A',C',ObserverPoles);
L = L';

% State controller LQR:
Q = [1, 0, 0, 0, 0, 0; 0, 0.01, 0, 0, 0, 0; 0, 0, 1.8, 0, 0, 0; 0, 0, 0, 0.05, 0, 0; 0, 0, 0, 0, 0.01, 0; 0, 0, 0, 0, 0, 0.1];
S = [m_R, 0; 0, (m_W + m_B)];
ricatti_controller = lqr(A,B,Q,S);


