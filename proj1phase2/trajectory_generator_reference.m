function s_des = trajectory_generator(t, path, h)
s_des = zeros(13,1);
persistent M;
persistent T;
persistent Px;
persistent Py;
persistent Pz;
R = 6;  %order
if nargin > 1 % pre-process can be done here (given waypoints). Pre-define the entire trajectory.

M = size(path,1) - 1; % num of segment
T = zeros(M+1,1); %time stamp
Qx = zeros(M*(R+1));
Qy = zeros(M*(R+1));
Qz = zeros(M*(R+1));

s = 0;
for i = 2:1:M+1
    s = s + sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2);
end
v = s/25;
for i = 2:1:M+1
    T(i) = T(i-1) + sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2)/v;
end

for m = 1:1:M
    for i = 4:1:R
        for j = 4:1:R
            Qx(i+1+(m-1)*(R+1),j+1+(m-1)*(R+1)) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*((T(m+1) - T(m))^(i+j-7))/(i+j-7);
        end
    end
end
for m = 1:1:M
    for i = 4:1:R
        for j = 4:1:R
            Qy(i+1+(m-1)*(R+1),j+1+(m-1)*(R+1)) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*((T(m+1) - T(m))^(i+j-7))/(i+j-7);
        end
    end
end
for m = 1:1:M
    for i = 4:1:R
        for j = 4:1:R
            Qz(i+1+(m-1)*(R+1),j+1+(m-1)*(R+1)) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*((T(m+1) - T(m))^(i+j-7))/(i+j-7);
        end
    end
end
A1x = zeros(2*M,M*(R+1));
d1x = zeros(2*M,1);
x = path(:,1);
A1y = zeros(2*M,M*(R+1));
d1y = zeros(2*M,1);
y = path(:,2);
A1z = zeros(2*M,M*(R+1));
d1z = zeros(2*M,1);
z = path(:,3);
%z = zeros(M+1,1);
%waypoint constrain
for m = 1:1:M
    for i = 0:1:R
        A1x(1+(m-1)*2,i+1+(m-1)*(R+1)) = (T(m) - T(m))^i;
        %A1x(1+(m-1)*2,i+1+(m-1)*(R+1)) = 333;
        d1x(1+(m-1)*2,1) = x(m);

        A1x(2+(m-1)*2,i+1+(m-1)*(R+1)) = (T(m+1) - T(m))^i;
        d1x(2+(m-1)*2,1) = x(m+1);
    end
end
for m = 1:1:M
    for i = 0:1:R
        A1y(1+(m-1)*2,i+1+(m-1)*(R+1)) = (T(m) - T(m))^i;
        d1y(1+(m-1)*2,1) = y(m);

        A1y(2+(m-1)*2,i+1+(m-1)*(R+1)) = (T(m+1) - T(m))^i;
        d1y(2+(m-1)*2,1) = y(m+1);
    end
end
for m = 1:1:M
    for i = 0:1:R
        A1z(1+(m-1)*2,i+1+(m-1)*(R+1)) = (T(m) - T(m))^i;
        d1z(1+(m-1)*2,1) = z(m);

        A1z(2+(m-1)*2,i+1+(m-1)*(R+1)) = (T(m+1) - T(m))^i;
        d1z(2+(m-1)*2,1) = z(m+1);
    end
end
%continues constrain
A2x = zeros(2*(M-1),M*(R+1));
d2x = zeros(2*(M-1),1);
for m = 1:1:M-1
    for k = 1:1:2
        for i = k:1:R
            l = i;
                A2x(k+(m-1)*2,i+1+(m-1)*(R+1)) = (factorial(i)/factorial(i-k))*(T(m+1) - T(m))^(i-k);
                A2x(k+(m-1)*2,l+1+(m)*(R+1)) = -(factorial(l)/factorial(l-k))*(T(m+1) - T(m+1))^(l-k);
                d2x(k+(m-1)*2,1) = 0;
        end
    end
end
A2y = zeros(2*(M-1),M*(R+1));
d2y = zeros(2*(M-1),1);
for m = 1:1:M-1
    for k = 1:1:2
        for i = k:1:R
            l =i;
                A2y(k+(m-1)*2,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m+1) - T(m))^(i-k);
                A2y(k+(m-1)*2,l+1+(m)*(R+1)) = -factorial(l)/factorial(l-k)*(T(m+1) - T(m+1))^(l-k);
                d2y(k+(m-1)*2,1) = 0;
        end
    end
end
A2z = zeros(2*(M-1),M*(R+1));
d2z = zeros(2*(M-1),1);
for m = 1:1:M-1
    for k = 1:1:2
        for i = k:1:R
            l =i;
                A2z(k+(m-1)*2,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m+1) - T(m))^(i-k);
                A2z(k+(m-1)*2,l+1+(m)*(R+1)) = -factorial(l)/factorial(l-k)*(T(m+1) - T(m+1))^(l-k);
                d2z(k+(m-1)*2,1) = 0;
        end
    end
end
%start final constrain
A3x = zeros(2,M*(R+1));
d3x = zeros(2,1);
A3y = zeros(2,M*(R+1));
d3y = zeros(2,1);
A3z = zeros(2,M*(R+1));
d3z = zeros(2,1);
for m = 1:1:M
    for k = 1:1:2
        for i = k:1:R
            if m ==1
                A3x(k,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m) - T(m))^(i-k);
                d3x(k,1) = 0;
            else if m == M
                A3x(k+2,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m+1) - T(m))^(i-k);
                d3x(k+2,1) = 0;
                end
            end
        end
    end
end
for m = 1:1:M
    for k = 1:1:2
        for i = k:1:R
            if m ==1
                A3y(k,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m) - T(m))^(i-k);
                d3y(k,1) = 0;
            else if m == M
                A3y(k+2,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m+1) - T(m))^(i-k);
                d3y(k+2,1) = 0;
                end
            end
        end
    end
end
for m = 1:1:M
    for k = 1:1:2
        for i = k:1:R
            if m ==1
                A3z(k,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m) - T(m))^(i-k);
                d3z(k,1) = 0;
            else if m == M
                A3z(k+2,i+1+(m-1)*(R+1)) = factorial(i)/factorial(i-k)*(T(m+1) - T(m))^(i-k);
                d3z(k+2,1) = 0;
                end
            end
        end
    end
end
Ax = [A1x;A2x;A3x];
dx = [d1x;d2x;d3x];
[Px,value]=quadprog(Qx,[],[],[],Ax,dx);
Ay = [A1y;A2y;A3y];
dy = [d1y;d2y;d3y];
[Py,value]=quadprog(Qy,[],[],[],Ay,dy);
m = 1;
m
Az = [A1z;A2z;A3z];
dz = [d1z;d2z;d3z];
[Pz,value]=quadprog(Qy,[],[],[],Ay,dz);
subplot(h);

    for t = T(1): 0.02:T(M+1)
         for j = 1:1:M
            if (t>=T(j)&&t<T(j+1))
                m = j;
                break;
            end
         end
        xx = 0;
        yy = 0;
        zz = 0;
        for i = 0:1:R
            xx = xx + Px(i+1+(m-1)*(R+1))*(t - T(m))^i;
            yy = yy + Py(i+1+(m-1)*(R+1))*(t - T(m))^i;
            zz = zz + Pz(i+1+(m-1)*(R+1))*(t - T(m))^i;
        end
		plot3(xx,yy,zz, 'r.');
		hold on;
	end

    scatter3(path(:,1),path(:,2),path(:,3));
    hold off;

else % output desired state along the trajectory here (given time).
    s_des(1) = 0;
    s_des(2) = 0;
    s_des(3) = 0;
    s_des(4) = 0;
    s_des(5) = 0;
    ax = 0;
    ay = 0;
    yaw0   = 0;
    pitch0 = 0;
    roll0  = 0;
    s_des(6)  = 0; %zdot
	Quat0  = R_to_quaternion(ypr_to_R([yaw0 pitch0 roll0])');
	s_des(7)  = Quat0(1);  %qw
	s_des(8)  = Quat0(2);  %qx
	s_des(9)  = Quat0(3);  %qy
	s_des(10) = Quat0(4);  %qz
	s_des(11) = 0;         %p
	s_des(12) = 0;         %q
	s_des(13) = 0;         %r
    for j = 1:1:M
        if (t>=T(j)&&t<T(j+1))
            m = j;
            break;
        end
    end
    for i = 0:1:R
        s_des(1) = s_des(1) + Px(i+1+(m-1)*(R+1))*(t - T(m))^i;
        s_des(2) = s_des(2) + Py(i+1+(m-1)*(R+1))*(t - T(m))^i;
        s_des(3) = s_des(3) + Pz(i+1+(m-1)*(R+1))*(t - T(m))^i;
        if i > 0
        s_des(4) = s_des(4) + i*Px(i+1+(m-1)*(R+1))*(t - T(m))^(i-1);
        s_des(5) = s_des(5) + i*Py(i+1+(m-1)*(R+1))*(t - T(m))^(i-1);
        s_des(6) = s_des(6) + i*Pz(i+1+(m-1)*(R+1))*(t - T(m))^(i-1);
        end
        if i>1
            ax = ax + i*(i-1)*Px(i+1+(m-1)*(R+1))*(t - T(m))^(i-2);
            ay = ay + i*(i-1)*Py(i+1+(m-1)*(R+1))*(t - T(m))^(i-2);
        end
    end
end
end
