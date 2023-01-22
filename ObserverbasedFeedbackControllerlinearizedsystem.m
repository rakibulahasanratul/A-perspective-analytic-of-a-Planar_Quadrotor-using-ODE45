% Observer based Control
clc;
clear all
%Parameters
g = 9.8;              %Accelaration
m = 0.18;             %mass
J = 2.5e-4;            %Inertia

A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/J];
C = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
D = 0;

sys = ss(A,B,C,D);

% Percent Overshoot
OS = 15;
% Settling Time
Ts = 5;

% Damping coefficient
k = -log(OS/100)/sqrt((pi^2 + (log(OS/100))^2));

% Frequency
w = 4/(k*Ts);
%Characteristic Polynomial
poly = [1 2*k*w w^2];
R = roots(poly);
P1 = [R(1) R(2) 10*real(R(1)) 10*real(R(1))-1 10*real(R(1))-2 10*real(R(1))-3]
P2 = 10*P1;

K = place(A,B,P1);
L = place(A',C',P2)'

%State tracking
Ace = [(A-B*K) (B*K); zeros(size(A)) (A-L*C)];
Bce = [B; zeros(size(B))];
Cce = [C zeros(size(C))];
Dce = 0;
%Initialization
x0 = [10; 10; 10; 0; 0; 0; 0; 0; 0; 0; 0; 0];
%System Control
sys_cl = ss(Ace,Bce,Cce,Dce);

t = 0:0.1:10;
r(1,:) =1*zeros(size(t));
r(2,:) =1*zeros(size(t));
[y,t,x]=lsim(sys_cl,r,t,x0);
plot(t,y(:,1),'r--',t,y(:,2),'g',t,y(:,3),'b')
xlabel("time"); ylabel("output")
legend('y','x','phi');
title(['Step Response with Observer based Feedback Controller ','Overshoot=15 ','Settling Time=5 '])