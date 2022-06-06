%function [dx,uinstx,uinsty]=ballplateLQR(t,x,desired,tspan)
% m=mass of ball
% Ib=Moment of Inertia of Ball
% g=acceleration of gravity
% xerror=error in x coordinate
% xerrordot=rate of change of xerror
% uinstx=input to control x coordinate
% yerror=error in y coordinate
% yerrordot=rate of change of yerror
% uinsty=input to control y coordinate
% c = 1+(Ib/m*r*r) where r is the radius
% Kp proportional constant
% Kd = derivative constant
clear all
clc

X0 = [0.09 0 0 0 0.05 0 0 0]';
desired = [0.1;0;0;0;0.2;0;0;0];
errorX0 = desired-X0;
[A,B] = JacobianEvaluatorBPS(errorX0)

C = [1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0];

D = [0 0 ; 0 0];

Q = [1 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 0 0
    0 0 0 0 1 0 0 0
    0 0 0 0 0 1 0 0
    0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 1];

R = [0.01 0; 0 0.01];

K = lqr(A,B,Q,R);

%sys = ss((A-B*K), B, C, D);

%t = linspace(0,3,0.5);
tspan = [0: 0.01: 10];
%[Y, t, X] = initial(sys, X0, t);
[t,eX] = ode45(@(t,eX) odeFUN(eX,A,B,K),tspan,errorX0);

for n=1:length(t)
    X(n,:) = (desired')-eX(n,:);
end

figure(1)
plot(X(:,1),X(:,5))
hold on
%axis('square');
title('Trajectory plot')
xlabel('X axis')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Y axis')
h_ylabel = get(gca,'XLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12);
grid on;

figure(3)
plot(t,X(:,5),'--')
title('Tracking error of system using LQR controller')
xlabel('Time in secs')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Distance in m')
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
grid on;

%%
%getting control inputs (torque)
Torque=zeros(numel(t),2);
Torquex= Torque(:,1);
Torquey= Torque(:,2);
for ii=1:numel(t)
    Torque(ii,:) = -(K*X(ii,:)');
end

%%
simspan=linspace(0,10,1001);% simulation span
tstep= 0.01;% timestep
timelimit= 0.5;% time limit to reach target point
init=[.09;0;0;0;.05;0;0;0];% initial condition
balancepos=[0 0];% balance position

% Designing desired trajectory for regulator problem
desiredx=zeros(1,size(simspan,2));
desiredx(simspan<=timelimit)=((balancepos(1)-init(1))/timelimit).*simspan(simspan<=timelimit)+init(1);
desiredx(simspan>timelimit)=balancepos(1);
desiredvx=gradient(desiredx)/tstep;
desiredy=zeros(1,size(simspan,2));
desiredy(simspan<=timelimit)=((balancepos(2)-init(5))/timelimit).*simspan(simspan<=timelimit)+init(5);
desiredy(simspan>timelimit)=balancepos(2);
desiredvy=gradient(desiredy)/tstep;
%desired=[desiredx;desiredvx;desiredy;desiredvy;gradient(desiredvx)/tstep;gradient(desiredvy)/tstep];
%%
figure(1)
plot(Y(:,1),Y(:,2))
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

figure(2)
plot(t,rad2deg(X(:,3)),'--',t,rad2deg(X(:,7)),'-.')
title('Motor angle')
xlabel('time')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Angle')
h_ylabel = get(gca,'XLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12);
legend('Alpha','Beta')
grid on;

figure(4)
plot(t,Torque(:,1),'--',t,Torque(:,2),'-.')
title('Motor torque')
xlabel('time')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Torque')
h_ylabel = get(gca,'XLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12);
legend('Torque_x','Torque_y')
grid on;

figure(3)
plot(t,Y(:,1),'--')
title('Tracking error of system using LQR controller')
xlabel('Time in secs')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Distance in m')
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
% legend('Actual X trajectory','Desired X trajectory')
grid on;
%end

function dXdt = odeFUN(X,A0,B0,K0)
    dXdt = (A0-(B0*K0))*X;
end