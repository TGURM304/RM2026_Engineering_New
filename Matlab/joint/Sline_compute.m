% 计算位移，速度，加速度，加加速度
function [time, q, qd, qdd,qddd] = Sline_compute(Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vlim, amax, amin, alima, alimd, jmax, jmin,T)
i = 1; 
for t = 0: 0.001: T
   time(i) = 0.001*i;    
% 加速段
if (t >= 0 && t < Tj1)
    q(i) = q0 + v0*t + jmax*t^3/6;
    qd(i) = v0 + jmax*(t^2/2);
     qdd(i) = jmax*t;
     qddd(i) = jmax;
elseif (t >= Tj1 && t < Ta - Tj1)
    q(i) = q0 + v0*t +(alima/6)*(3*t^2 - 3*Tj1*t + Tj1^2);
    qd(i) = v0 + alima*(t - Tj1/2);
    qdd(i) = alima; 
    qddd(i) = 0;
elseif (t >= Ta - Tj1 && t < Ta)
    q(i) = q0 + (vlim + v0)*(Ta/2) - vlim*(Ta - t) - jmin*((Ta - t)^3/6);
    qd(i) = vlim + jmin*(power(Ta - t, 2)/2);
    qdd(i) = -jmin*(Ta - t);
    qddd(i) = jmin;
% 匀速段
elseif (t >= Ta && t < Ta + Tv)
    q(i) = q0 + (vlim + v0)*(Ta/2) + vlim*(t - Ta);
    qd(i) = vlim;
    qdd(i) = 0;
    qddd(i) = 0;
% 减速段
elseif (t >= Ta + Tv && t < T - Td + Tj2)
    q(i) = q1 - (vlim + v1)*(Td/2) + vlim*(t - T + Td) - jmax*(power(t - T + Td, 3)/6);
    qd(i) = vlim - jmax*(power(t - T + Td, 2)/2);
    qdd(i) = -jmax*(t - T + Td);
    qddd(i) = -jmax;
elseif (t >= T - Td + Tj2 && t < T - Tj2)
    q(i) = q1 - (vlim + v1)*(Td/2) + vlim*(t - T + Td) + (alimd/6)*(3*power(t - T + Td, 2) - 3*Tj2*(t - T + Td) + Tj2^2);
    qd(i) = vlim + alimd*(t - T + Td - Tj2/2);
    qdd(i) = alimd;
    qddd(i) = 0;
elseif (t >= T - Tj2 && t <= T)
    q(i) = q1 - v1*(T - t) - jmax*(power(T - t, 3)/6);
    qd(i) = v1 + jmax*(power(t - T, 2)/2);
    qdd(i) = -jmax*(T - t);
    qddd(i) = jmax;
end
i = i + 1;
end
end