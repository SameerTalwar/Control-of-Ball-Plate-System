% finding settling time for different cases

Kp = 1;
Ki = 1;
Kd = 2.0;

%% case 1 : PD control

sys1 = tf([Kd Kp],[1 Kd Kp]);
S1 = stepinfo(sys1)

%% case 2 : D control

sys2 = tf([Kd],[1 Kd]);
S2 = stepinfo(sys2)

%% case 3 : PID control

sys3 = tf([Kd Kp Ki],[1 Kd Kp Ki]);
S3 = stepinfo(sys3)
