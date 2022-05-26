function [dx,uinstx,uinsty,xerrorint,yerrorint]=ballplatePID(t,x,desired,tspan,xerrorint,yerrorint)
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


Kp = 01;
Kd = 2;
Ki = 1;
c=1+2/5;
g=9.80;
dx=zeros(4,1);
dx(1)=x(2);
xerror = x(1)-interp1(tspan,desired(1,:),t);
xerrordot = x(2)-interp1(tspan,desired(2,:),t);
xerrorint = xerrorint + xerror;
C=g/c;
uinstx= Kp*(xerror)+Kd*(xerrordot) + Ki*(xerrorint);
dx(2)=-C*sin(uinstx);
dx(3)=x(4);
yerror=x(3)-interp1(tspan,desired(3,:),t);
yerrordot=x(4)-interp1(tspan,desired(4,:),t);
yerrorint = yerrorint + yerror;
uinsty= Kp*(yerror)+Kd*(yerrordot) + Ki*(yerrorint);
dx(4)=-C*sin(uinsty);
end