clc;
clear;

% 正解输 入关节角
q_input = [50.2,40.5,112.17,40.12,56.21,70.7];
%% 公式推导正解
% 连杆参数(没有的默认为零）
a2 = 0.350;
a3 = -0.090; % 未知
d2 = 0.0227; % 未知
d4 = 0.22571;

Base = transl(0.116, 0, 0.091+0.215);   % 基座偏移 100 mm
Tool = transl(0, 0, 0.150);   % 末端工具偏移 200 mm

T__ = my_fkine(q_input, Base, Tool, a2, a3, d2, d4);
%T__ = transl(-228,-110,110) * trotx(90) * troty(180);
disp("正解结果：")
disp(T__);

%L1 thi1(可)     0          0         0
%L2 0            d2(可)     0         phi2
%L3 thi3         0          a2        0
%L4 0            d4         a3(可)    phi4
%L5 0            0          0         phi5
%L6 0            0          0         phi6

%          thetai       di      ai-1     alphai-1
L1 = Link([0            0       0         0     ],'modified');
L2 = Link([0            d2      0         pi/2  ],'modified');
L3 = Link([0            0       a2        0     ],'modified');L3.offset = pi/2;
L4 = Link([0            d4      a3        pi/2  ],'modified');
L5 = Link([0            0       0        -pi/2  ],'modified');
L6 = Link([0            0       0         pi/2  ],'modified');

L1.qlim =[-150*pi/180, 150*pi/180];
L2.qlim =[  30*pi/180, 150*pi/180];
L3.qlim =[-150*pi/180, 150*pi/180];
L4.qlim =[-180*pi/180, 180*pi/180];
L5.qlim =[-120*pi/180, 120*pi/180];
L6.qlim =[-180*pi/180, 180*pi/180];

IRB4600=SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'IRB4600');
IRB4600.base = Base;
IRB4600.tool = Tool;

q0 = [0 30 30 0 0 0] * pi/180; % 合理初值
q_sol = IRB4600.ikcon(T__, q0);  % 所有关节参与求解

%disp('逆解（角度°）：');
%disp(rad2deg(q_sol));
%IRB4600.display();
IRB4600.teach;

%% 逆解函数
T_target = T__;
AllSloverTheta = cul(T_target, Base, Tool, a2, a3, d2, d4);

disp('逆解关节角（弧度）：');
for i = 1:size(AllSloverTheta,1)
    fprintf('解 %d: ', i);
    fprintf('%.2f ', rad2deg(AllSloverTheta(i,:)));
    fprintf('\n');
end

%% 验证
for i = 1:size(AllSloverTheta,1)
    q_rad = AllSloverTheta(i,:);
    q_deg = rad2deg(q_rad); 
    T_calc = my_fkine(q_deg, Base, Tool, a2, a3, d2, d4);
    fprintf('解 %d 的 FK 末端位姿误差:\n', i);
    disp(T__ - T_calc);
end

target_point1 = [-0.228, -0.110, 0.110];
target_point2 = [-0.228,  0.110, 0.110];

for i = 1:size(AllSloverTheta,1)
    figure(i);
    clf;
    IRB4600.plot(AllSloverTheta(i,:), ...
        'view', [135 25]);
    hold on;

    plot3(target_point1(1), target_point1(2), target_point1(3), ...
        'bo', 'MarkerSize', 10, 'LineWidth', 2);
    text(target_point1(1), target_point1(2), target_point1(3)+5, ...
        '目标点', 'Color', 'b', 'FontSize', 12);

    plot3(target_point2(1), target_point2(2), target_point2(3), ...
        'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(target_point2(1), target_point2(2), target_point2(3)+5, ...
        '目标点2', 'Color', 'r', 'FontSize', 12);

    q_deg = rad2deg(AllSloverTheta(i,:));
    sgtitle(sprintf( ...
        '逆解 %d   q = [%.1f  %.1f  %.1f  %.1f  %.1f  %.1f]°', ...
        i, q_deg));
end

