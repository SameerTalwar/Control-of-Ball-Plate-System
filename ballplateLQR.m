function [dx,uinstx,uinsty]=ballplateLQR(t,x,desired,tspan)
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

c=1+2/5;
g=9.80;
C=g/c;

A = [0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0];
B = [0 0;
    -C 0;
    0 0
    0 -C];
rank(ctrb(A,B))
Q = [100 0 0 0;
    0 10 0 0;
    0 0 100 0;
    0 0 0 10];
R = [0.01 0;
    0 0.01];

K = lqr(A,B,Q,R);
%U = K*x;
%uinstx = U(1);
%uinsty = U(2);
dx=zeros(4,1);
dx(1)=x(2);
%dx(2)=-C*sin(uinstx);
dx(3)=x(4);
%dx(4)=-C*sin(uinsty);
end