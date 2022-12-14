function [dx,uinstx,uinsty]=ballplate1d(t,x,desired,tspan)
% m=mass of ball
% Ib=Moment of Inertia of Ball
% sliding surface= errordot + lambda*error
% g=acceleration of gravity
% Xcap= Nominal value of any generic variable X
% xerror=error in x coordinate
% xerrordot=rate of change of xerror
% uinstx=input to control x coordinate
% yerror=error in y coordinate
% yerrordot=rate of change of xerror
% uinsty=input to control y coordinate
% c = 1+(Ib/m*r*r) where r is the radius
% beta = coefficient of the control signal when the states are off the
% sliding surface
% sx is sliding surface for x coordinate
% sy is sliding surface for y coordinate

c=1+2/5;
lambda=6.4;
g=9.80;
gcap=9.8995;
dx=zeros(4,1);
dx(1)=x(2);
xerror=x(1)-interp1(tspan,desired(1,:),t);
xerrordot=x(2)-interp1(tspan,desired(2,:),t);
sx=xerrordot+lambda*xerror;
C=g/c;
Ccap=gcap/c;
beta=((1-C/Ccap)*(lambda*xerrordot -interp1(tspan,desired(5,:),t))+3.5*Ccap)/C;
m = (c*(lambda*xerrordot+beta*sign(sx)-interp1(tspan,desired(5,:),t))/g);

uinstx= m + m^3/6 ;
%+ 3*m^5/40 + 15*m^7/48;
dx(2)=-g*sin(uinstx)/c;
dx(3)=x(4);
yerror=x(3)-interp1(tspan,desired(3,:),t);
yerrordot=x(4)-interp1(tspan,desired(4,:),t);
sy=yerrordot+lambda*yerror;
gamma=((1-C/Ccap)*(lambda*xerrordot -interp1(tspan,desired(6,:),t))+3.5*Ccap)/C;
 v = (c*(lambda*yerrordot+gamma*sign(sy)-interp1(tspan,desired(6,:),t))/g);
uinsty= v+ v^3/6 ;
%+ 3*v^5/40 +15*v^7/48;
dx(4)=-g*sin(uinsty)/c;
end