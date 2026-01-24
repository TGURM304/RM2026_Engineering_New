clc;
clear;
%L1 thi1(可)     0          0         0
%L2 0            d2(可)     0         phi2
%L3 thi3         0          a2        0
%L4 0            d4         a3(可)    phi4
%L5 0            0          0         phi5
%L6 0            0          0         phi6

syms a2 a3 d2 d4 d6 Q1 Q2 Q3 Q4 Q5 Q6 real

% DH模型
T1 = fun(  0,       0,         0,      Q1);
T2 = fun(  0,   sym(pi)/2,    d2,      Q2);
T3 = fun( a2,       0,         0,      Q3);
T4 = fun( a3,   sym(pi)/2,    d4,      Q4);
T5 = fun(  0,  -sym(pi)/2,     0,      Q5);
T6 = fun(  0,   sym(pi)/2,     0,      Q6);

T15 = T1*T2*T3;
T15_inv = inv(T15);
T1_inv = inv(T1);
T56 = T5*T6;
T46 = T4*T56;
T23 = T2*T3;
T26 = T23*T46;
T16 = T1*T26;
simplify(T15)    % 化简后的符号公式
simplify(T46)
simplify(T6)