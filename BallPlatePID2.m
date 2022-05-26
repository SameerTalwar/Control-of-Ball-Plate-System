clear all
clc
Ib = 8e-6;
Ip = 0.045;
r = 0.02;
m = 0.05;
g=9.80;

%%

simspan=linspace(0,50,1000);% simulation span
tstep=50/1000;% timestep
xerrorint = 0;
yerrorint = 0;
timelimit=10;% time limit to reach target point
init=[.09;0;.05;0];% initial condition
balancepos=[0 0];% balance position

%%

opt=odeset('RelTol',1e-2,'AbsTol',1e-2);
[t,x]=ode45(@(t,x) ballplatePID2(t,x,xerrorint,yerrorint),simspan,init,opt);


%%
clc
uinstx=zeros(1,numel(t));
uinsty=uinstx;
for ii=1:numel(t)
    [~,uinstx(ii),uinsty(ii),xerrorint,yerrorint]=ballplatePID2(t(ii),x(ii,:),xerrorint,yerrorint);
end
%%

figure(1)
plot(t,x(:,1),'--',t,x(:,3),'-.')
title('Position with time')
xlabel('Time in secs')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Distance in m')
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
legend('Actual X trajectory','Actual Y trajectory')
grid on;

figure(2)
plot(x(:,1),x(:,3))
%axis('square');
title('Trajectory plot')
xlabel('X axis')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Y axis')
h_ylabel = get(gca,'XLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
grid on;

figure(3)
plot(t,rad2deg(uinstx),'--',t,rad2deg(uinsty))
title('Plate Angles')
legend('Motor X angle','Motor Y angle')
xlabel('Time in seconds')
ylabel('Angle in degrees')
set(gca,'FontSize',12)
grid on

%%

function [dx,uinstx,uinsty,xerrorint,yerrorint]=ballplatePID2(t,x,xerrorint,yerrorint)
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


Kp = 1;
Kd = 2;
Ki = 1;
c=1+2/5;
g=9.80;
dx=zeros(4,1);
dx(1)=x(2);
xerror = x(1);
xerrordot = x(2);
xerrorint = xerrorint + xerror;
C=g/c;
uinstx= Kp*(xerror)+Kd*(xerrordot) + Ki*(xerrorint);
dx(2)=-C*sin(uinstx);
dx(3)=x(4);
yerror= x(3);
yerrordot= x(4);
yerrorint = yerrorint + yerror;
uinsty= Kp*(yerror)+Kd*(yerrordot) + Ki*(yerrorint);
dx(4)=-C*sin(uinsty);
end