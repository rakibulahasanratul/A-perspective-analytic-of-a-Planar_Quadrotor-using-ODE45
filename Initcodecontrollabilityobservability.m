%% Degree of controllability
clear; clc;

clc;
clear all;

% symbolic variables (assumed to be real)
%syms c t real

syms g m j t real

% g=9.8;
% m=0.18;
% j=2.5*10^-4;


% state-space matrices

A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B=  [0 0 0; 0 0 0; 0 0 0; 0 0 0; -1/m 0 -1; 0 1/j 0];
C = [1 0 0 0 0 0; 0 1 0 0 0 0];
D = [0 0 0;0 0 0];

% state dimension
n = size(A,1);
%controllability matrix
P= [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];
disp('Controllability Matrix')
disp(P)
%print('Controllability Matrix:',P);
% test if the system is controllable
controllable = ( rank(P) == n );
disp('Controllable Rank')
disp(controllable)
% basis for the controllable subspace (requires symbolic P)
ctrb_space = colspace(sym(P));
disp('Controllability Subspace')
disp(ctrb_space)
%observablity matrix
Q= [C ;C*A;(C.^2)*A;(C.^3)*A;(C.^4)*A;(C.^5)*A];
disp('Observablity Matrix')
disp(Q)


m=0.18;
j=2.5*10^(-4);
g=9.8;

A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B=  [0 0 0; 0 0 0; 0 0 0; 0 0 0; -1/m 0 -1; 0 1/j 0];
C = [1 0 0 0 0 0; 0 1 0 0 0 0];
D = [0 0 0;0 0 0];

sys = ss(A,B,C,D);
figure(1)
step(sys)




