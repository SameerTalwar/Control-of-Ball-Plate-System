function [A,B,K] = ballplateLQR(errorX0)

x = 0;
xdot = errorX0(2);
alpha = errorX0(3);
alphadot = errorX0(4);
y = 0;
ydot = errorX0(6);
beta = errorX0(7);
betadot = errorX0(8);

m = 0.05;
r = 0.02;
g = 9.8;
M = 0.5;
a = 0.2;
Ib = 2*m*r*r/5;
Ip = M*a*a/12;

c = 1 + (Ib/(m*r*r));
c1 = ((Ib + Ip)/m) + x^2;
c2 = ((Ib + Ip)/m) + y^2;
c3 = (Ib + Ip) + (m*(x^2)) + (m*(y^2));
C1 = g/(1+(Ib/(m*r*r)));
C2 = (g*m)/(Ib+Ip);

df_dx(1,1) = 0;
df_dx(1,2) = 1;
df_dx(1,3) = 0;
df_dx(1,4) = 0;
df_dx(1,5) = 0;
df_dx(1,6) = 0;
df_dx(1,7) = 0;
df_dx(1,8) = 0;
df_dx(2,1) = (alphadot^2)/c;
df_dx(2,2) = 0;
df_dx(2,3) = -(g*cos(alpha))/c;
df_dx(2,4) = ((2*m*x*alphadot)+(betadot*y))/c;
df_dx(2,5) = (alphadot*betadot)/c;
df_dx(2,6) = 0;
df_dx(2,7) = 0;
df_dx(2,8) = (alphadot*y)/c;
df_dx(3,1) = 0;
df_dx(3,2) = 0;
df_dx(3,3) = 0;
df_dx(3,4) = 1;
df_dx(3,5) = 0;
df_dx(3,6) = 0;
df_dx(3,7) = 0;
df_dx(3,8) = 0;
df_dx(4,1) = ((c3*((m*m*y*y*g*(cos(beta)-cos(alpha))) + (2*m*m*x*y*ydot*(alphadot-betadot)) + (m*m*y*y*xdot*alphadot) + (2*m*m*y*y*((ydot*betadot)-(xdot*alphadot))) - (Ib+Ip)*((m*g*cos(alpha))+(2*m*xdot*alphadot)+(m*ydot*betadot))))-(2*m*x)*((m*m*x*y*y*g*(cos(beta)-cos(alpha))) + (m*m*x*x*y*ydot*(alphadot-betadot)) + (m*m*y*y*xdot*((x*alphadot)-(y*betadot))) + (2*m*m*x*y*y*((ydot*betadot)-(xdot*alphadot))) - (Ib+Ip)*((m*g*x*cos(alpha))+(m*xdot*y*betadot)+(m*x*ydot*betadot)+(2*m*x*xdot*alphadot))))/((Ib+Ip)*c3*c3);
df_dx(4,2) = ((m*m*y*y*((x*alphadot)-(y*betadot))) - (2*m*m*x*y*y*alphadot) - (Ib+Ip)*((m*y*betadot)+(2*m*x*alphadot)))/((Ib+Ip)*c3);
df_dx(4,3) = ((m*m*x*y*y*g*sin(alpha)) + (Ib+Ip)*(m*g*x*sin(alpha)))/((Ib+Ip)*c3);
df_dx(4,4) = ((m*m*x*x*y*ydot)-(m*m*x*xdot*y*y))/((Ib+Ip)*c3);
df_dx(4,5) = ((c3*((2*m*m*x*y*g*(cos(beta)-cos(alpha))) + (m*m*x*x*ydot*(alphadot-betadot)) + (2*m*m*y*xdot*((x*alphadot)-(y*betadot))) - (m*m*y*y*xdot*betadot) + (4*m*m*x*y*((ydot*betadot)-(xdot*alphadot))) - (Ib+Ip)*(m*xdot*betadot))) - (2*m*y)*((m*m*x*y*y*g*(cos(beta)-cos(alpha))) + (m*m*x*x*y*ydot*(alphadot-betadot)) + (m*m*y*y*xdot*((x*alphadot)-(y*betadot))) + (2*m*m*x*y*y*((ydot*betadot)-(xdot*alphadot))) - (Ib+Ip)*((m*g*x*cos(alpha))+(m*xdot*y*betadot)+(m*x*ydot*betadot)+(2*m*x*xdot*alphadot))))/((Ib+Ip)*c3*c3);
df_dx(4,6) = ((m*m*x*x*y*(alphadot-betadot)) + (2*m*m*x*y*y*betadot) - (Ib+Ip)*(m*x*betadot))/((Ib+Ip)*c3);
df_dx(4,7) = -(m*m*x*y*y*g*sin(beta))/((Ib+Ip)*c3);
df_dx(4,8) = (-(m*m*x*x*y*ydot)-(m*m*xdot*y*y*y)+(2*m*m*x*y*y*ydot)-((Ib+Ip)*((m*xdot*y)+(m*x*ydot))))/((Ib+Ip)*c3);
df_dx(5,1) = 0;
df_dx(5,2) = 0;
df_dx(5,3) = 0;
df_dx(5,4) = 0;
df_dx(5,5) = 0;
df_dx(5,6) = 1;
df_dx(5,7) = 0;
df_dx(5,8) = 0;
df_dx(6,1) = (alphadot*betadot)/c;
df_dx(6,2) = 0;
df_dx(6,3) = 0;
df_dx(6,4) = (betadot*x)/c;
df_dx(6,5) = (betadot^2)/c;
df_dx(6,6) = 0;
df_dx(6,7) = -(g*cos(beta))/c;
df_dx(6,8) = ((2*m*y*betadot)+(alphadot*x))/c;
df_dx(7,1) = 0;
df_dx(7,2) = 0;
df_dx(7,3) = 0;
df_dx(7,4) = 0;
df_dx(7,5) = 0;
df_dx(7,6) = 0;
df_dx(7,7) = 0;
df_dx(7,8) = 1;
df_dx(8,1) = ((c3*((2*m*m*x*y*g*(cos(alpha)-cos(beta))) + (m*m*y*y*xdot*(betadot-alphadot)) + (2*m*m*x*ydot*((y*betadot)-(x*alphadot))) - (m*m*x*x*ydot*alphadot) + (4*m*m*y*x*((xdot*alphadot)-(ydot*betadot))) - (Ib+Ip)*(m*ydot*alphadot))) - (2*m*x)*((m*m*y*x*x*g*(cos(alpha)-cos(beta))) + (m*m*y*y*x*xdot*(betadot-alphadot)) + (m*m*x*x*ydot*((y*betadot)-(x*alphadot))) + (2*m*m*y*x*x*((xdot*alphadot)-(ydot*betadot))) - (Ib+Ip)*((m*g*y*cos(beta))+(m*ydot*x*alphadot)+(m*y*xdot*alphadot)+(2*m*y*ydot*betadot))))/((Ib+Ip)*c3*c3);
df_dx(8,2) = ((m*m*x*y*y*(betadot-alphadot)) + (2*m*m*x*x*y*alphadot) - (Ib+Ip)*(m*y*alphadot))/((Ib+Ip)*c3);
df_dx(8,3) = -(m*m*x*x*y*g*sin(alpha))/((Ib+Ip)*c3);
df_dx(8,4) = (-(m*m*x*y*y*xdot)-(m*m*ydot*x*x*x)+(2*m*m*y*x*x*xdot)-((Ib+Ip)*((m*xdot*y)+(m*x*ydot))))/((Ib+Ip)*c3);
df_dx(8,5) = ((c3*((m*m*x*x*g*(cos(alpha)-cos(beta))) + (2*m*m*x*y*xdot*(betadot-alphadot)) + (m*m*x*x*ydot*betadot) + (2*m*m*x*x*((xdot*alphadot)-(ydot*betadot))) - (Ib+Ip)*((m*g*cos(beta))+(2*m*ydot*betadot)+(m*xdot*alphadot))))-(2*m*y)*((m*m*x*x*y*g*(cos(alpha)-cos(beta))) + (m*m*y*y*x*xdot*(betadot-alphadot)) + (m*m*x*x*ydot*((y*betadot)-(x*alphadot))) + (2*m*m*y*x*x*((xdot*alphadot)-(ydot*betadot))) - (Ib+Ip)*((m*g*y*cos(beta))+(m*ydot*x*alphadot)+(m*y*xdot*alphadot)+(2*m*y*ydot*betadot))))/((Ib+Ip)*c3*c3);
df_dx(8,6) = ((m*m*x*x*((y*betadot)-(x*alphadot))) - (2*m*m*x*x*y*betadot) - (Ib+Ip)*((m*x*alphadot)+(2*m*y*betadot)))/((Ib+Ip)*c3);
df_dx(8,7) = ((m*m*x*x*y*g*sin(beta)) + (Ib+Ip)*(m*g*y*sin(beta)))/((Ib+Ip)*c3);
df_dx(8,8) = ((m*m*x*y*y*xdot)-(m*m*y*ydot*x*x))/((Ib+Ip)*c3);

B(1,1) = 0;
B(1,2) = 0;
B(2,1) = 0;
B(2,2) = 0;
B(3,1) = 0;
B(3,2) = 0;
B(4,1) = (m*c2)/((Ib+Ip)*c3);
B(4,2) = (-m*x*y)/((Ib+Ip)*c3);
B(5,1) = 0;
B(5,2) = 0;
B(6,1) = 0;
B(6,2) = 0;
B(7,1) = 0;
B(7,2) = 0;
B(8,1) = (-m*x*y)/((Ib+Ip)*c3);
B(8,2) = (m*c1)/((Ib+Ip)*c3);

Q = [1000000 0 0 0 0 0 0 0;
    0 0.01 0 0 0 0 0 0;
    0 0 0.01 0 0 0 0 0;
    0 0 0 0.01 0 0 0 0
    0 0 0 0 1000000 0 0 0
    0 0 0 0 0 0.01 0 0
    0 0 0 0 0 0 0.01 0
    0 0 0 0 0 0 0 0.01];

R = [10 0; 0 10];

A = [0 1 0 0 0 0 0 0;
    0 0 -C1 0 0 0 0 0;
    0 0 0 1 0 0 0 0;
    -C2 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 -C1 0;
    0 0 0 0 0 0 0 1;
    0 0 0 0 -C2 0 0 0];

K = lqr(A,B,Q,R);
end