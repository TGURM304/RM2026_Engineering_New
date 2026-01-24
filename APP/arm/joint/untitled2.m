%机器人工具箱测试
clc
clear;
% 输入关节角

a2 = 360;
a3 = -90; % 未知
d2 = 14.64; % 未知
d4 = 250;

q_(1)=150.1/180*pi;
q_(2)=-25.19/180*pi;
q_(3)=-35.07/180*pi;
q_(4)=-155.82/180*pi;
q_(5)=256.86/180*pi;
q_(6)=-1.51/180*pi;

q(1)=150.1;
q(2)=-25.19;
q(3)=-35.07;
q(4)=-155.82;
q(5)=256.86;
q(6)=-1.51;

%机器人工具箱建立机器人模型
%          thetai       di      ai-1     alphai-1
L1 = Link([0            0       0         0     ],'modified');
L2 = Link([0            d2      0         pi/2  ],'modified');
L3 = Link([0            0       a2        0     ],'modified');L3.offset = pi/2;
L4 = Link([0            d4      a3        pi/2  ],'modified');
L5 = Link([0            0       0        -pi/2  ],'modified');
L6 = Link([0            0       0         pi/2  ],'modified');

IRB4600=SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'IRB4600');

%vector = vector(q);
differential = jacobian(q, a2, a3, d2, d4);
%调用机器人工具箱的jacob0函数，求解雅克比矩阵
robot_data = IRB4600.jacob0(q_);

disp(differential);
disp(robot_data);