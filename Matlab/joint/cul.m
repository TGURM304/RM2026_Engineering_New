function AllSloverTheta =cul(T__, Base, Tool, a2, a3, d2, d4)

coder.extrinsic('disp');

AllSloverTheta = zeros(8,6);

% 连杆参数(没有的默认为零）
c_a2 = a2;
c_a3 = a3;
c_d2 = d2;
c_d4 = d4;
T06 = Base \ T__ / Tool;

L = [-240.0 * pi / 180.0, 245.0 * pi / 180.0;
       67.0 * pi / 180.0, 135.0 * pi / 180.0;
     -137.0 * pi / 180.0,  13.0 * pi / 180.0;
     -175.0 * pi / 180.0, 115.0 * pi / 180.0;
      -80.0 * pi / 180.0,  82.0 * pi / 180.0;
      -pi               ,  pi ];

%输入的位姿数据
 nx=T06(1,1); ny=T06(2,1); nz=T06(3,1);
 ox=T06(1,2); oy=T06(2,2); oz=T06(3,2);
 ax=T06(1,3); ay=T06(2,3); az=T06(3,3);
 px=T06(1,4); py=T06(2,4); pz=T06(3,4);
 
 ForJudgment=px^2+py^2-c_d2^2;
 
    if ForJudgment<-1e-6
        disp('Out of workspace Unable to solve');
    else      
        if ForJudgment>=-1e-6&&ForJudgment<0
            ForJudgment=0;
        end
        %求解θ1
        theta1 = zeros(2,1);
        theta1(1) = atan2(py,px)-atan2(-c_d2,sqrt(ForJudgment));
        theta1(2) = atan2(py,px)-atan2(-c_d2,-sqrt(ForJudgment));

        %求解θ3
        theta3 = zeros(2,1);
        k_t = (px^2+py^2+pz^2-c_a2^2-c_a3^2-c_d2^2-c_d4^2)/(2*c_a2);
        theta3(1) = -atan2(c_a3,c_d4)+atan2(k_t,sqrt(c_a3^2+c_d4^2-k_t^2));
        theta3(2) = -atan2(c_a3,c_d4)+atan2(k_t,-sqrt(c_a3^2+c_d4^2-k_t^2));

        % 求解 θ2 θ4 θ5 θ6
        theta23 = zeros(4,1);
        theta2  = zeros(4,1);
        theta4  = zeros(4,1);
        theta5  = zeros(4,1);
        theta6  = zeros(4,1);

        id1 = [1 2 1 2];
        id3 = [1 1 2 2];
        for k = 1:4
            tem1 = theta1(id1(k));
            tem3 = theta3(id3(k));

            %求解θ2
            num = (c_a2*cos(tem3)+c_a3)*pz + (cos(tem1)*px + sin(tem1)*py)*(c_a2*sin(tem3) + c_d4);
            den = -(c_d4 + c_a2*sin(tem3))*pz + (cos(tem1)*px + sin(tem1)*py)*(c_a2*cos(tem3) + c_a3);

            theta23(k) = atan2(num, den);
            theta2(k)  = theta23(k) - tem3;
            
            %求解θ4
            tem23 = theta2(k) + tem3;

            tem4_1 = ax*sin(tem1) - ay*cos(tem1);
            tem4_2 = ax*cos(tem1)*cos(tem23) + ay*sin(tem1)*cos(tem23) + az*sin(tem23);
            if abs(tem4_1) < 1e-9 && abs(tem4_2) < 1e-9
                theta4(k) = 0;
                theta5(k) = 0;
                continue;
            else
                theta4(k) = atan2(tem4_1, tem4_2);
            end

            %求解θ5
            tem5_1 = ax*(cos(tem1)*cos(tem23)*cos(theta4(k)) + sin(tem1)*sin(theta4(k))) + ...
             ay*(sin(tem1)*cos(tem23)*cos(theta4(k)) - cos(tem1)*sin(theta4(k))) + ...
             az*(sin(tem23)*cos(theta4(k)));
            tem5_2 = ax*cos(tem1)*sin(tem23) + ay*sin(tem1)*sin(tem23) - az*cos(tem23);
            if abs(tem5_1) < 1e-9 && abs(tem5_2) < 1e-9
                theta5(k) = 0;
            else
                theta5(k) = atan2(tem5_1, tem5_2);
            end

            %求解θ6
            tem6_1 = -nx*(cos(tem1)*cos(tem23)*sin(theta4(k)) - sin(tem1)*cos(theta4(k))) - ...
                ny*(sin(tem1)*cos(tem23)*sin(theta4(k)) + cos(tem1)*cos(theta4(k))) - ...
                nz*sin(tem23)*sin(theta4(k));
            tem6_2 = nx*((cos(tem1)*cos(tem23)*cos(theta4(k)) + sin(tem1)*sin(theta4(k)))*cos(theta5(k)) - cos(tem1)*sin(tem23)*sin(theta5(k))) + ...
                ny*((sin(tem1)*cos(tem23)*cos(theta4(k)) - cos(tem1)*sin(theta4(k)))*cos(theta5(k)) - sin(tem1)*sin(tem23)*sin(theta5(k))) + ...
                nz*(sin(tem23)*cos(theta4(k))*cos(theta5(k)) + cos(tem23)*sin(theta5(k)));
            if abs(tem6_1) < 1e-9 && abs(tem6_2) < 1e-9
                theta6(k) = 0;
            else
                theta6(k) = atan2(tem6_1, tem6_2);
            end
        end

        theta1_8 = [theta1(id1) ; theta1(id1)];
        theta2_8 = [theta2       ; theta2      ];
        theta3_8 = [theta3(id3) ; theta3(id3)];

        theta4_8 = [theta4 ; theta4 + pi];
        theta5_8 = [theta5 ; -theta5];
        theta6_8 = [theta6 ; theta6 + pi];

        AllSolutions = [theta1_8 theta2_8 theta3_8 theta4_8 theta5_8 theta6_8];
        
        offset = [0, 0, pi/2, 0, 0, 0];
        AllSolutions = AllSolutions - offset;

        normalize = @(x) atan2(sin(x), cos(x));
        AllSolutions = normalize(AllSolutions);

        validIdx = true(8,1);
        for i=1:8
            for j=1:6
                if ~(AllSolutions(i,j) >= L(j,1) && AllSolutions(i,j) <= L(j,2))
                    validIdx(i)=false;
                    break;
                end
            end
        end

        AllSloverTheta = AllSolutions(validIdx, :);
    end
end
