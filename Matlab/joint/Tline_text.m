clc
clear;
q0 = 0; q1 =360;
v0 = 0; v1 =0;
vmax = 50; amax = 10;
[time, q, qd, qdd] = Tline(q0,q1,v0,v1,vmax,amax);
figure(1)
subplot(311)
plot(time,q,'r','LineWidth',1.5);
grid on;xlabel('time[s]');ylabel('position[mm]');
subplot(312)
plot(time,qd,'b','LineWidth',1.5);
grid on;xlabel('time[s]');ylabel('speed[mm/s]');
subplot(313)
plot(time,qdd,'g','LineWidth',1.5);
grid on;xlabel('time[s]');ylabel('acceleration[mm/s2]');