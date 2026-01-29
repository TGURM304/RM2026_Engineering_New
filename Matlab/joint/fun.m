function A = fun(a, alpha, d, theta)
    % 改进 DH (MDH) 通用矩阵
    A = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
         0,0,0,1];
end
