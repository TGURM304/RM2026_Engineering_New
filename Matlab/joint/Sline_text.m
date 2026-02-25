clc
clear;
% S曲线规划
% 边界条件
q0 = 0; q1 = 100;
v0 = 0; v1 = 0;
vmax = 10; amax = 10; jmax = 30;
sigma = sign(q1 - q0);
% 得到规划参数Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin
para = MySlinePara(q0, q1, v0, v1, vmax, amax, jmax);
T = para(1) + para(2) + para(3);
[time, q, qd, qdd,qddd] = Sline_compute( para(1), para(2), para(3), para(4), para(5), para(6), para(7), para(8), para(9), para(10), para(11), para(12), para(13), para(14), para(15), para(16),T);
figure(1)
subplot(5,1,1)
plot(time, q, 'r', 'LineWidth', 1.5)
grid on;xlabel('t(s)');ylabel('pos(mm)');
subplot(5,1,2)
plot(time, qd, 'b', 'LineWidth', 1.5)
grid on;xlabel('t(s)');ylabel('v(mm/s)');
subplot(5,1,3)
plot(time, qdd, 'g', 'LineWidth', 1.5)
grid on;xlabel('t(s)');ylabel('a(mm/s2)');
subplot(5,1,4)
plot(time, qddd, 'LineWidth', 1.5)
grid on;xlabel('t(s)');ylabel('&a(mmm/s3)');