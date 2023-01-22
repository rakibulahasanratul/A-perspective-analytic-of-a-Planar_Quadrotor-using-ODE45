%% Feedback Controler
clc;
clear all;

%%Physical parameter of a planar quadrotor
g = 9.8;              %Accelaration
m = 0.18;             %mass
J = 2.5e-4;            %Inertia

%%State Equation
A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/J];
C = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
D = 0;

%%System Representation
sys = ss(A,B,C,D);

%%Percent Overshoot
OvS = 10;
%%Settling Time
Ts = 6;
% Damping coeff
k = -log(OvS/100)/sqrt((pi^2 + (log(OvS/100))^2));

% Frequency
ws = 4/(k*Ts);

%%Characteristic Polynomial
poly = [1 2*k*ws ws^2];
R = roots(poly);
%Poles placement
P = [R(1) R(2) 10*real(R(1)) 10*real(R(1))-1 10*real(R(1))-2 10*real(R(1))-3];

K = place(A,B,P)                  

%state tracking
AC = (A-B*K);
BC = B;
CC = C;
DC = D;

x0 = [10; 10; 10; 0; 0; 0];

%x0 = [1; 1; 0; 0; 0; 0];

sys_cl = ss(AC,BC,CC,DC);
 
t = 0:0.1:10;
r(1,:) =1*zeros(size(t));
r(2,:) =1*zeros(size(t));
[y,t,x]=lsim(sys_cl,r,t,x0);
plot(t,y(:,1),'r--',t,y(:,2),'g',t,y(:,3),'b')
ylabel("Output")
xlabel("time")
legend('y','x','phi');
title(['Step Response with Closed Loop Feedback Control ','Overshoot=10 ','Settling Time=6 ', 'Initial State: x0 = [10; 10; 0; 0; 0; 0]'])
