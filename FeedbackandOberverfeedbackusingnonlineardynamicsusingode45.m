%[t,s] = ode45(@closedloop,[0 20],[10; 10; 0; 0; 0; 0; 10; 10; 0; 0; 0; 0]);
[t,s] = ode45(@closedloop,[0 20],[1; 1; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0]);

%dynamics_quad(stilde,utilde)

x = s(:,1);
figure
subplot(1,3,1)
plot(t,x)
title('State response of X with respect to time')

y = s(:,2);
subplot(1,3,2)
plot(t,y)
title('State response of Y with respect to time')

phi = s(:,3);
subplot(1,3,3)
plot(t,phi)
title('State response of phi with respect to time')

% [t,s] = ode45(@dynamics_quad,[0 1],[10 10 0 0 0 0]);
% subplot(1,3,3)
% plot(x,y)
% title('Entire State Response of non linear dynamics x,y')

% g = 9.8;              %Accelaration
% m = 0.18;             %mass
% J = 2.5e-4;  
% A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
% B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/J];
% K=[0.5932,11.1698,-4.1168,1.2623,2.8543,-0.3273;  -0.0023,-0.0016,0.0235,-0.0031,-0.0002,0.0045];
% 
%PP=eig(A-B*K)

function dstates=closedloop(t,states)

g = 9.8;
m = 0.18; 

s=states(1:6);
shat=states(7:12);
%utilde=[0;0];
utilde=[m*g;0];
%stilde=[0; 0; 0; 0; 0; 0];
stilde=[t; 0; 0; 0; 0; 0];
K=[0.5932,11.1698,-4.1168,1.2623,2.8543,-0.3273;  -0.0023,-0.0016,0.0235,-0.0031,-0.0002,0.0045];
u = utilde - K*(s - stilde);
C = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
L=1000*[0.1203,-0.0286,0,0,0,0;0.0233,0.0857,0,0,0,0;0,0,0.1900,0,0,0;2.7366,-2.6109,-0.0098,0,0,0;2.3259,-1.4314,0,0,0,0;0,0,8.8000,0,0,0];
y=C*s;
yhat=C*shat;
dshat=dynamics_quad(shat,u)+L*(y-yhat);
ds = dynamics_quad(s,u);
dstates=[ds;dshat];

end

function ds = dynamics_quad(s,u)
% parameters
m = 0.18;
g = 9.8;
i = 2.5e-4;
% u1=15; u2=0.215;

x = s(1);
y = s(2);
phi = s(3);
xdot = s(4);
ydot = s(5);
phidot = s(6);

xdotdot = -u(1)*sin(phi)/m;
ydotdot = -g+(u(1)*cos(phi))/m;
phidotdot = u(2)/i;

ds = [xdot,ydot,phidot,xdotdot,ydotdot,phidotdot]';
end
