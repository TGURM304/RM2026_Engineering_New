% 六自由度机械臂运动的牛顿-欧拉递归逆动力学求解：
% 参数：各关节运动角度， 速度， 加速度（全部6*1矩阵）,刚体质量矩阵（1*6）
% 返回值：各关节力矩（6*1矩阵）
function tau = NewtonEuler(theta, theta_d, theta_dd, m, a2, a3, d2, d4)

th(1) = theta(1)*pi/180;
th(2) = theta(2)*pi/180;
th(3) = (theta(3) + 90)*pi/180;
th(4) = theta(4)*pi/180;
th(5) = theta(5)*pi/180;
th(6) = theta(6)*pi/180;

%base_link的各项初始值
w00 = [0; 0; 0]; v00 = [0; 0; 0]; w00d = [0; 0; 0]; v00d = [0; 0; 9.8];
z = [0; 0; 1];
 
% 各连杆质量
m1 = m(1); m2 = m(2); m3 = m(3); m4 = m(4); m5 = m(5); m6 = m(6);
% 惯性张量
e   =  1e-9;
I1 = [3851115.32 48716.4 23697.02; 48716.4 4514902.84 -65688.79; 23697.02 -65688.79 2352842.69] * e;
I2 = [783522.26 -328.72 -203486.02; -328.72 23937546.86 -41.73; -203486.02 -41.73 24004699.35] * e;
I3 = [10047985.38 3517941.27 1648446.23; 3517941.27 11295497.25 1882049.72; 1648446.23 1882049.72 11478405.51] * e;
I4 = [1633171.41 -9408.46 311.33; -9408.46 1554994.96 39837.17; 311.33 39837.17 798585.34] * e;
I5 = [412978.93 16348.19 -614.6; 16348.19 229625.28 -5848.72; -614.6 -5848.72 389801.96] * e;
I6 = [858165.56 -13104.78 20.93; -13104.78 600938.51 -1484.55; 20.93 -1484.55 502263.56] * e;
 
T01 = fun(0,      0,      0,   th(1));
T12 = fun(0,      pi/2,   d2,  th(2));
T23 = fun(a2,     0,      0,   th(3));
T34 = fun(a3,     pi/2,   d4,  th(4));
T45 = fun(0,     -pi/2,   0,   th(5));
T56 = fun(0,      pi/2,   0,   th(6));
 
% 各关节p及各link质心pc的距离(假设质心在几何中心)
p10 = T01(1:3,4);p21 = T12(1:3,4); p32 = T23(1:3,4);
p43 = T34(1:3,4); p54 = T45(1:3,4);  
p65 = T56(1:3,4); p76 = [0, 0, 0]';

pc11 = [0.00075; 0.00362; 0.08834];
pc22 = [0.18728; -0.00001; -0.05947];
pc33 = [-0.01792; -0.00668; 0.00728];
pc44 = [0.00044; 0.00614; 0.05728];
pc55 = [-0.00104; -0.02705; -0.00226];
pc66 = [0.00001; 0.0; 0.04709];
% 旋转矩阵
R01 = T01(1:3, 1:3); R12 = T12(1:3, 1:3); R23 = T23(1:3, 1:3);
R34 = T34(1:3, 1:3); R45 = T45(1:3, 1:3); R56 = T56(1:3, 1:3);
R10 = R01'; R21 = R12'; R32 = R23';
R43 = R34'; R54 = R45'; R65 = R56';
R67 = [1 0 0; 0 1 0; 0 0 1];
%内推 i: 0->5
% 连杆1到连杆6向外迭代
% i = 0
w11 = R10*w00 + theta_d(1)*z;
w11d = R10*w00d + cross(R10*w00, z*theta_d(1)) + theta_dd(1)*z;
v11d = R10*(cross(w00d, p10) + cross(w00, cross(w00, p10)) + v00d);
vc11d = cross(w11d, pc11) + cross(w11, cross(w11, pc11)) + v11d;
F11 = m1*vc11d;
N11 = I1*w11d + cross(w11, I1*w11); 
% i = 1
w22 = R21*w11 + theta_d(2)*z;
w22d = R21*w11d + cross(R21*w11, z*theta_d(2)) + theta_dd(2)*z;
v22d = R21*(cross(w11d, p21) + cross(w11, cross(w11, p21)) + v11d);
vc22d = cross(w22d, pc22) + cross(w22, cross(w22, pc22)) + v22d;
F22 = m2*vc22d;
N22 = I2*w22d + cross(w22, I2*w22);
% i = 2
w33 = R32*w22 + theta_d(3)*z;
w33d = R32*w22d + cross(R32*w22, z*theta_d(3)) + theta_dd(3)*z;
v33d = R32*(cross(w22d, p32) + cross(w22, cross(w22, p32)) + v22d);
vc33d = cross(w33d, pc33) + cross(w33, cross(w33, pc33)) + v33d;
F33 = m3*vc33d;
N33 = I3*w33d + cross(w33, I3*w33);
% i= 3
w44 = R43*w33 + theta_d(4)*z;
w44d = R43*w33d + cross(R43*w33, z*theta_d(4)) + theta_dd(4)*z;
v44d = R43*(cross(w33d, p43) + cross(w33, cross(w33, p43)) + v33d);
vc44d = cross(w44d, pc44) + cross(w44, cross(w44, pc44)) + v44d;
F44 = m4*vc44d;
N44 = I4*w44d + cross(w44, I4*w44);
% i = 4
w55 = R54*w44 + theta_d(5)*z;
w55d = R54*w44d + cross(R54*w44, z*theta_d(5)) + theta_dd(5)*z;
v55d = R54*(cross(w44d, p54) + cross(w44, cross(w44, p54)) + v44d);
vc55d = cross(w55d, pc55) + cross(w55, cross(w55, pc55)) + v55d;
F55 = m5*vc55d;
N55 = I5*w55d + cross(w55, I5*w55);
% i = 5
w66 = R65*w55 + theta_d(6)*z;
w66d = R65*w55d + cross(R65*w55, z*theta_d(6)) + theta_dd(6)*z;
v66d = R65*(cross(w55d, p65) + cross(w55, cross(w55, p65)) + v55d);
vc66d = cross(w66d, pc66) + cross(w66, cross(w66, pc66)) + v66d;
F66 = m6*vc66d;
N66 = I6*w66d + cross(w66, I6*w66);
 
% 外推: i: 6->1
% 连杆6到连杆1向内迭代
f77 = [0; 0; 0]; n77 = [0; 0; 0];
% i = 6
f66 = R67*f77 + F66;
n66 = N66 + R67*n77 + cross(pc66, F66) + cross(p76, R67*f77);
tau(6) = n66'*z;
% i = 5
f55 = R56*f66 + F55;
n55 = N55 + R56*n66 + cross(pc55, F55) + cross(p65, R56*f66);
tau(5) = n55'*z;
% i = 4
f44 = R45*f55 + F44;
n44 = N44 + R45*n55 + cross(pc44, F44) + cross(p54, R45*f55);
tau(4) = n44'*z;
% i = 3
f33 = R34*f44 + F33;
n33 = N33 + R34*n44 + cross(pc33, F33) + cross(p43, R34*f44);
tau(3) = n33'*z;
% i = 2
f22 = R23*f33 + F22;
n22 = N22 + R23*n33 + cross(pc22, F22) + cross(p32, R23*f33);
tau(2) = n22'*z;
% i =1
f11 = R12*f22 + F11;
n11 = N11 + R12*n22 + cross(pc11, F11) + cross(p21, R12*f22);
tau(1) = n11'*z;
end