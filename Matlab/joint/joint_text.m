clc;
clear;

% 正解输 入关节角
q_input = [43.05,109.74,-101.11,57.14,-34.26,-75.06];
%% 公式推导正解
% 连杆参数(没有的默认为零）
a2 = 0.350;
a3 = -0.090; % 未知
d2 = 0.0227; % 未知
d4 = 0.22571;

Base = transl(0.116, 0, 0.091+0.215);   % 基座偏移 100 mm
Tool = transl(0, 0, 0.150);   % 末端工具偏移 200 mm

T__ = my_fkine(q_input, Base, Tool, a2, a3, d2, d4);
T_ = transl(0.264, 0.204, 0.554) * trotx(-100.8) * troty(17.9) * trotz(-104.1);
%T_ = transl(0.264, 0.204, 0.554) * rpy2tr(-104.1, 17.9, -100.8, 'deg');
disp("正解结果：")
disp(T__);
disp(T_);
disp(tr2rpy(T__, 'deg'));
disp(tr2rpy(T_, 'deg'));

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

L1.qlim =[-240.0 * pi / 180.0, 245.0 * pi / 180.0];
L2.qlim =[  67.0 * pi / 180.0, 135.0 * pi / 180.0];
L3.qlim =[-137.0 * pi / 180.0,  13.0 * pi / 180.0];
L4.qlim =[-175.0 * pi / 180.0, 115.0 * pi / 180.0];
L5.qlim =[ -80.0 * pi / 180.0,  82.0 * pi / 180.0];
L6.qlim =[ -pi               ,  pi ];

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

num_solutions = size(AllSloverTheta, 1);
All_T_matrices = zeros(4, 4, num_solutions);

for i = 1:num_solutions
    q_deg = rad2deg(AllSloverTheta(i,:));
    All_T_matrices(:,:,i) = my_fkine(q_deg, Base, Tool, a2, a3, d2, d4);
    disp(All_T_matrices(:,:,i));
end

T_lib = IRB4600.fkine(q_input * pi/180);
disp("库FK结果：")
disp(T_lib.T);   % .T 取出4x4矩阵

disp("我的FK结果：")
disp(T__);

disp(T_);
