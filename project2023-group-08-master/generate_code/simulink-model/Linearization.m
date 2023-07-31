clc 
clear
params();
syms  phi theta psi dphi dtheta dpsi  real
u=sym('u',[4 1],'real');  % inpt of the thrust
m = 0.027;       %Mass
g = 9.81;     %Gravity
d = 0.046;        %Arm length
lift = 1.9796e-9; %Lift constant
drag = 2.5383e-11;%Drag constant
F = 0.25;      %Air friction coefficient
    
   % Moment of inertia matrix
    Jx = 1.1463e-5;
    Jy = 1.6993e-5;
    Jz = 2.9944e-5;
    J = [Jx,0,0;0,Jy,0;0,0,Jz];
    T_angleder2w =[ 1,        0,            sin(theta)
                      0, cos(phi),  -cos(theta)*sin(phi)
                      0, sin(phi),   cos(phi)*cos(theta)];
X=[phi;theta;dphi;dtheta;dpsi];

 w=T_angleder2w*X(3:5);%wb=T_angleder2w*[dphi;dtheta;dpsi]; 
 tx=d*(u(3)+u(4)-u(1)-u(2))/sqrt(2);
    ty=d*(u(2)+u(3)-u(1)-u(4))/sqrt(2);
    tz=drag/lift*(u(2)+u(4)-u(1)-u(3));

w_d=J\(cross(-w,J*w)+[tx;ty;tz]);
f=simplify([w(1:2);w_d]);
X0=[0;0;0;0;0];
% X0=[-pi/4;-pi/3;0.0001;0.0001;-0.0001];
f_0=subs(f,X,X0);
U=solve(f_0 == 0,u);
c10=U.u1;
c20=U.u2;
c30=U.u3;
c40=U.u4;
U_0=[c10;c20;c30;c40];


A=jacobian(f,X);
Ac=subs(A,X,X0);
A_eig=vpa(real(eig(Ac)));
A_eig_sign=sign(real(eig(Ac)));
B=vpa(jacobian(f,u),8);
%B_0=simplify(subs(B,u,U_0))
Bc=subs(B,u,U_0);


% Calculation of LQR controller with integral action 
clc
Ts = 0.001;

% Weighting matrices 

% slow respons
%Q = diag([10 10 0.01 0.01 1]);

%medium
%Q = diag([100 100 1 1 1]); % used in the real crazyflie 

% fast respons
Q = diag([1000 1000 1 1 10]); % used in simulink in report and in c-simulation 

% fastest
%Q = diag([1000 1000 0.1 0.1 10]);

% zero based thrust
%Q = diag([1000 1000 1 1 10]);

% more fast
%Q = diag([10000 10000 0.1 0.1 10]);

K_u = 0.0001;     % used in the report for simulink
%K_u = 0.0000001; % used in the real crazyflie
R = K_u*eye(4);

sysc = ss(double(Ac),double(Bc),eye(5),zeros(5,4));
sysd = c2d(sysc,Ts,'zoh');
Bd = (sysd.B).*1.2865e-04;%2.2454e-06; 
Ad = sysd.A;

[K,S,E] = dlqr(Ad,Bd,Q,R);

K
load('FlightData.mat');

