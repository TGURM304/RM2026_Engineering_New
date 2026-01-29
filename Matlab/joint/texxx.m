clc;
clear;

%% 1. 参数设置
format short;

num = 50000;   % 采样点数

%% 2. 定义你的 MDH 机械臂 IRB4600
d2 = 14.64; a2 = 360; d4 = 250; a3 = -90;

L1 = Link([0 0 0 0],'modified');
L2 = Link([0 d2 0 pi/2],'modified');
L3 = Link([0 0 a2 0],'modified'); L3.offset = pi/2;
L4 = Link([0 d4 a3 pi/2],'modified');
L5 = Link([0 0 0 -pi/2],'modified');
L6 = Link([0 0 0 pi/2],'modified');

L1.qlim = [-150 150]*pi/180;
L2.qlim = [  64 140]*pi/180;
L3.qlim = [-145  19]*pi/180;
L4.qlim = [-180 180]*pi/180;
L5.qlim = [-120 120]*pi/180;
L6.qlim = [-180 180]*pi/180;

IRB4600 = SerialLink([L1 L2 L3 L4 L5 L6],'name','IRB4600');
IRB4600.base = transl(0,0,80);
IRB4600.tool = transl(0,0,170+250);

robot = IRB4600;  % 这里可以方便后续换机械臂

%% 3. 随机关节角采样
q = zeros(num,6);
for i = 1:6
    q(:,i) = robot.qlim(i,1) + rand(num,1)*(robot.qlim(i,2) - robot.qlim(i,1));
end
q(:,6) = 0; % 轴6对工作空间位置无影响

%% 4. 正运动学计算工作空间
tic;
T_cell = cell(num,1);
[T_cell{:,1}] = robot.fkine(q).t; % 返回末端 xyz
disp(['FK 运行时间: ', num2str(toc), ' s']);

%% 5. 绘制工作空间
t1 = clock;
figure('name','机械臂工作空间');
hold on;
plotopt = {'noraise','nowrist','nojaxes','delay',0};
robot.plot(zeros(1,6), plotopt{:});

figure_x = zeros(num,1);
figure_y = zeros(num,1);
figure_z = zeros(num,1);
for i = 1:num
    figure_x(i) = T_cell{i}(1);
    figure_y(i) = T_cell{i}(2);
    figure_z(i) = T_cell{i}(3);
end

plot3(figure_x, figure_y, figure_z, 'r.', 'MarkerSize', 3);
axis equal;
hold off;
disp(['绘图运行时间：', num2str(etime(clock,t1)), ' s']);

%% 6. 获取工作空间范围
Point_range = [
    min(figure_x) max(figure_x);
    min(figure_y) max(figure_y);
    min(figure_z) max(figure_z)
];
disp('工作空间范围 [min  max]：');
disp(Point_range);
