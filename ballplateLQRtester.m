%% 
% suppressing all warnings
clc
clear all
id='MATLAB:plot:IgnoreImaginaryXYPart';
warning('off',id)
%%
simspan=linspace(0,50,1000);% simulation span
tstep=50/1000;% timestep
timelimit=10;% time limit to reach target point
init=[.09;0;.05;0];% initial condition
balancepos=[0 0];% balance position

%%
% Designing desired trajectory for regulator problem
desiredx=zeros(1,size(simspan,2));
desiredx(simspan<=timelimit)=((balancepos(1)-init(1))/timelimit).*simspan(simspan<=timelimit)+init(1);
desiredx(simspan>timelimit)=balancepos(1);
desiredvx=gradient(desiredx)/tstep;
desiredy=zeros(1,size(simspan,2));
desiredy(simspan<=timelimit)=((balancepos(2)-init(3))/timelimit).*simspan(simspan<=timelimit)+init(3);
desiredy(simspan>timelimit)=balancepos(2);
desiredvy=gradient(desiredy)/tstep;
desired=[desiredx;desiredvx;desiredy;desiredvy;gradient(desiredvx)/tstep;gradient(desiredvy)/tstep];
%%
% Starting Simulation
clc
opt=odeset('RelTol',1e-2,'AbsTol',1e-2);
[t,x]=ode45(@(t,x) ballplateLQR(t,x,desired,simspan),simspan,init,opt);
%% Getting Control inputs
clc
uinstx=zeros(1,numel(t));
uinsty=uinstx;
DXX =zeros(4,numel(t));
UN1=uinstx;
UN2=uinstx;
for ii=1:numel(t)
    [~,uinstx(ii),uinsty(ii)]=ballplateLQR(t(ii),x(ii,:),desired,simspan);
end
%%
figure(1)
plot(t,x(:,1),'--',t,desiredx,'-.')
title('Tracking error of system using PID controller')
xlabel('Time in millisecs')
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',20);
ylabel('Distance in m')
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
% legend('Actual X trajectory','Desired X trajectory')
grid on;

figure(2)
plot(x(:,1),x(:,3))
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
set(gca,'FontSize',12)
legend('Actual Trajectory','Desired Trajectory')
grid on;
figure(3)
plot(t,rad2deg(uinstx),'--',t,rad2deg(uinsty))
title('Plate Angles')
legend('Motor X angle','Motor Y angle')
xlabel('Time in seconds')
ylabel('Angle in degrees')
set(gca,'FontSize',12)
grid on