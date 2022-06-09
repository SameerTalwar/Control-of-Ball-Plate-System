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
%simspan=[simspan1 simspan2];% Simulation Span
simspan = 0:0.01:20;
X0 = [0 0 0 0 0 0 0 0];% Initial Condition
amplitude=.04;% Radius of the circle

% Designing desired trajectory
desiredx1=zeros(1,size(simspan1,2));
desiredx2=amplitude*sin(simspan2-2.1);
%desiredx=[desiredx1 desiredx2];
desiredx=amplitude*sin(simspan);

desiredy1=(amplitude/5)*simspan1;
desiredy2=amplitude*cos(simspan2-2.1);
%desiredy=[desiredy1 desiredy2];
desiredy=amplitude*cos(simspan);
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
errorX0 = desired(2,1:8)-X0;
indvtspan = [0:0.001:0.01];
X=zeros(2001,8);
chosenRandomTime = 50 + randi(150);

for n=2:2000
    [A,B,K] = ballplateLQR(errorX0);
    [t,eX] = ode45(@(t,eX) DiffEqLQR(t,eX,A,B,K,n,chosenRandomTime),indvtspan,errorX0);
    X(n,:) = desired(n,1:8) - eX(11,:);
    torque(n-1,:) = -(K*(eX(1,:)'));
    errorX0 = desired(n+1,1:8) - X(n,:);
end

%%
%Starting Simulation
figure(1)
plot(X(1:2000,1),X(1:2000,5))
hold on
plot(desiredx,desiredy)
%axis('square');
title('Trajectory plot')
xlabel('X axis')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Y axis')
h_ylabel = get(gca,'XLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12);
legend('Actual Trajectory','Desired Trajectory')
grid on;

figure(3)
plot(simspan,X(:,5),'--',simspan,desiredy,'-.')
title('Tracking error of system using LQR controller')
xlabel('Time in secs')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Distance in m')
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
legend('Actual Y trajectory','Desired Y trajectory')
grid on;

figure(4)
plot(simspan(2:2000), torque)

figure(5)
plot(simspan,rad2deg(X(:,3)),simspan,rad2deg(X(:,7)))

%%

function deX = DiffEqLQR(t, eX,A0,B0,K0,n,chosenRandomTime)
if (n>=chosenRandomTime-1 && n<=chosenRandomTime+1)
    randomInput = rand(1);
else randomInput = 0;
end
deX = (A0-(B0*K0))*eX + randomInput;
end