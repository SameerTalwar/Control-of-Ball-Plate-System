%% 
% suppressing all warnings
clc
clear all
id='MATLAB:plot:IgnoreImaginaryXYPart';
warning('off',id)

%%
timestep=.1;
simspan1=0:.1:2;
simspan2=2.1:.1:20;
simspan=[simspan1 simspan2];% Simulation Span
init=[0;0;0;0];% Initial Condition
amplitude=.04;% Radius of the circle

% Designing desired trajectory
desiredx1=zeros(1,size(simspan1,2));
desiredx2=amplitude*sin(simspan2-2.1);
desiredx=[desiredx1 desiredx2];

desiredy1=(amplitude/5)*simspan1;
desiredy2=amplitude*cos(simspan2-2.1);
desiredy=[desiredy1 desiredy2];
desiredvx=gradient(desiredx)/timestep;
desiredvy=gradient(desiredy)/timestep;

desiredalpha = desiredvx/(9.8*amplitude*amplitude);
desiredalphadot = gradient(desiredalpha)/timestep;
desiredbeta = desiredvy/(9.8*amplitude*amplitude);
desiredbetadot = gradient(desiredbeta)/timestep;

desiredax=gradient(desiredvx)/timestep;
desireday=gradient(desiredvy)/timestep;
desiredalphadotdot = gradient(desiredalphadot)/timestep;
desiredbetadotdot = gradient(desiredbetadot)/timestep;

desired=[desiredx;desiredvx;desiredalpha;desiredalphadot;desiredy;desiredvy;desiredbeta;desiredbetadot;desiredax;desireday;desiredalphadotdot;desiredbetadotdot];

m = 0.05;
r = 0.02;
g = 9.8;
M = 0.5;
a = 0.2;
Ib = 2*m*r*r/5;
Ip = M*a*a/12;
c1=1+(Ib/(m*r*r));
c2 = m/(Ib+Ip);

C1=g/c1;
C2=g*c1;

A=zeros(8,8,201);
for n=1:201
    A(:,:,n) = JacobianEvaluatorBPS(desired(:,n));
end

B = [0 0;
    0 0; 
    0 0;
    c2/m 0;
    0 0;
    0 0;
    0 0;
    0 c2/m]; 

C = [1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0];

D = [0 0 ; 0 0];

Q = [100000 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 100 0 0 0 0 0;
    0 0 0 1 0 0 0 0
    0 0 0 0 100000 0 0 0
    0 0 0 0 0 1 0 0
    0 0 0 0 0 0 100 0
    0 0 0 0 0 0 0 1];

R = [0.001 0; 0 0.001];

K=zeros(2,8,201);
for n=1:201
    K(:,:,n) = lqr(A(:,:,n),B,Q,R);
end


%%
%Starting Simulation

X0 = zeros(201,8);
sys = ss((A(:,:,21)-(B*K(:,:,21))),B,C,D);
t = [0: 0.01: 1];
[Y, t, X] = initial(sys, X0(1,:), t)
plot(Y(:,1),Y(:,2))
for n=1:201
    sys = ss((A(:,:,n)-(B*K(:,:,n))),B,C,D);
    t = [0: 0.01: 1];
    [Y, t, X] = initial(sys, X0(n,:), t);
    %X0(:,n+1) = X(end,:)'
    %figure(2)
    %hold on
    %plot(X(end,1),X(end,5))
end