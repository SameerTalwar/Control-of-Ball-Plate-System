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
X0 = [0.09 0 0 0 0.05 0 0 0];% Initial Condition
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

desired=[desiredx;desiredvx;desiredalpha;desiredalphadot;desiredy;desiredvy;desiredbeta;desiredbetadot;desiredax;desireday;desiredalphadotdot;desiredbetadotdot]';

%%
errorX0 = desired(1,1:8)-X0;
indvtspan = [0:0.01:0.1];
X=zeros(201,8);
for n=1:200
    [t,eX] = ode45(@(t,eX) ballplateLQR(t,eX,errorX0),indvtspan,errorX0);
    X(n,:) = desired(n,1:8) - eX(11,:);
    errorX0 = desired(n+1,1:8) - X(n,:)
end


%%
%Starting Simulation

figure(1)
plot(simspan, X(:,1), simspan, X(:,5))
figure(2)
plot(X(1:60,1),X(1:60,5))