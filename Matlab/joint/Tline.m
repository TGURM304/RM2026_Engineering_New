function [time, q, qd, qdd] = Tline(q0,q1,v0,v1,vmax,amax)
% input q0,q1,起始关节角度
% input vo,v1,起始关节速度
% inputvmax,amax 起始关节加速度
% output time,q,qd,qdd
 
v_temp = sqrt((2.0*amax*(q1-q0) + (v1^2 + v0^2)) / 2);
 
if(v_temp<vmax)
    vlin = v_temp;
else
    vlin = vmax;
end
 
Ta = (vlin-v0)/amax;
Sa = v0*Ta+amax*Ta^2/2;
 
Tv = (q1-q0-(vlin^2-v0^2)/(2*amax)-(v1^2-vlin^2)/(2*-amax))/vlin;
Sv = vlin*Tv;
 
Td = (vlin-v1)/amax;
Sd = vlin*Td - amax*Td^2/2;
 
T = Ta + Tv +Td;
%步长
td = 0.01;
k = 1;
for t = 0:td:T
    if(t >= 0 && t < Ta)
        time(k) = td *k;
        q(k) = q0 + v0*t + amax*t^2/2;
        qd(k) = v0 + amax*t;
        qdd(k) = amax;
    elseif(t >= Ta && t < Ta+Tv)
        time(k) = td *k;
        q(k) = q0 + Sa + vlin*(t - Ta);
        qd(k) = vlin;
        qdd(k) = 0;
    elseif(t >= Ta+Tv && t <= T)
        time(k) = td *k;
        q(k) = q0 + Sa + Sv + vlin*(t - Ta - Tv) - amax*power(t - Ta - Tv, 2)/2;
        qd(k) = vlin - amax*(t - Ta - Tv);
        qdd(k) = -amax;
    end
    k = k + 1;
end
end