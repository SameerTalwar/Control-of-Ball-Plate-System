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
Ib = 8e-6;
Ip = 0.045;
r = 0.02;
m = 0.05;
g=9.80;
c1=1+(Ib/(m*r*r));
c2 = m/(Ib+Ip);

C1=g/c1;
C2=g*c1;

A = [0 1 0 0 0 0 0 0;
    0 0 -C1 0 0 0 0 0;
    0 0 0 1 0 0 0 0;
    -C2 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 -C1 0;
    0 0 0 0 0 0 0 1;
    0 0 0 0 -C2 0 0 0];

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

Q = [1000 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 100 0 0 0 0 0;
    0 0 0 1 0 0 0 0
    0 0 0 0 1000 0 0 0
    0 0 0 0 0 1 0 0
    0 0 0 0 0 0 100 0
    0 0 0 0 0 0 0 1];

R = [0.01 0; 0 0.01];

K = lqr(A,B,Q,R)

B0 = [0 0;
    0 0; 
    0 0;
    1 0;
    0 0;
    0 0;
    0 0;
    0 1]; 

sys = ss((A-B*K), B, C, D);

X0 = [0.09 0 0 0 0.05 0 0 0]';
%t = linspace(0,3,0.5);
t = [0: 0.01: 10];
U = zeros(length(t),2);
randomTime = 0.0*randi(length(t));
%U((100*randomTime):(100*randomTime +3),:)=[20*rand(1),20*rand(1);20*rand(1),20*rand(1);20*rand(1),20*rand(1);20*rand(1),20*rand(1)];
[Y, t, X] = lsim(sys, U, t, X0);

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
figure(5)
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
plot(t,Y(:,1),'--',t,desiredx,'-.')
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