function T = my_fkine(q, Base, Tool, a2, a3, d2, d4)

    Q1 = q(1)*pi/180;
    Q2 = q(2)*pi/180;
    Q3 = (q(3) + 90)*pi/180;
    Q4 = q(4)*pi/180;
    Q5 = q(5)*pi/180;
    Q6 = q(6)*pi/180;

    A1 = fun(0,      0,      0,   Q1);
    A2 = fun(0,      pi/2,   d2,  Q2);
    A3 = fun(a2,     0,      0,   Q3);
    A4 = fun(a3,     pi/2,   d4,  Q4);
    A5 = fun(0,     -pi/2,   0,   Q5);
    A6 = fun(0,      pi/2,   0,   Q6);

    T = Base * (A1*A2*A3*A4*A5*A6) * Tool;
end
