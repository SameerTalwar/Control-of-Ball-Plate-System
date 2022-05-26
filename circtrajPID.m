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
xerrorint = 0;
yerrorint = 0;
%%
% Designing desired trajectory
desiredx1=zeros(1,size(simspan1,2));
desiredx2=amplitude*sin(simspan2-2.1);
desiredx=[desiredx1 desiredx2];

desiredy1=(amplitude/5)*simspan1;
desiredy2=amplitude*cos(simspan2-2.1);
desiredy=[desiredy1 desiredy2];
desiredvx=gradient(desiredx)/timestep;
desiredvy=gradient(desiredy)/timestep;

desired=[desiredx;desiredvx;desiredy;desiredvy;gradient(desiredvx)/timestep;gradient(desiredvy)/timestep];
figure(1)
plot(desiredx,desiredy);
title('Desired Trajectory')
axis('square')
xlabel('X-axis')
ylabel('Y-axis')
grid on

%%
%Starting Simulation
clc
opt=odeset('RelTol',1e-2,'AbsTol',1e-2);
[t,x]=ode45(@(t,x) ballplatePID(t,x,desired,simspan,xerrorint,yerrorint),simspan,init,opt);
%%
%% Getting Control inputs
clc
uinstx=zeros(1,numel(t));
uinsty=uinstx;
for ii=1:numel(t)
    
    [~,uinstx(ii),uinsty(ii),xerrorint,yerrorint]=ballplatePID(t(ii),x(ii,:),desired,simspan,xerrorint,yerrorint);
end

%%
figure(2)
plot(x(:,1),x(:,3),'-.')
hold on
plot(desiredx,desiredy,'--')
title('Performance evaluation between desired and actual trajectory')
axis('square')
xlabel('X-axis')
h_xlabel = get(gca,'XLabel')
set(h_xlabel,'FontSize',20);
ylabel('Y-axis')
h_ylabel = get(gca,'YLabel')
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
legend('Actual trajectory','Desired Trajectory')
grid on
%%
figure(3)
plot(t,rad2deg(uinsty),t,rad2deg(uinstx))
title('Plate angles')
axis('square')
xlabel('time in secs')
h_xlabel = get(gca,'XLabel')
set(h_xlabel,'FontSize',20);
ylabel('Angle in degrees')
h_ylabel = get(gca,'YLabel')
set(h_ylabel,'FontSize',20);
set(gca,'FontSize',12)
legend('Angle in y','Angle in x')


