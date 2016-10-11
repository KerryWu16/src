%%%% Sui Pang, Oct. 10th, 2016, ELEC 6910P, Project 1, phase 2
%%%% Optimization-based Trajectory Generation

%%% To solve a P vector contains M set of constants
%%% for a 7th degree polynomial
function s_des = trajectory_generator(t, path, h)
s_des = zeros(13,1);
% Global variables within this function
persistent M;
persistent T;
persistent Px;
persistent Py;
persistent Pz;
R = 8; % orders
global Q;
global A;
global dx;

if nargin > 1
    % local variables
    T_total = 25; % the curve takes 25 seconds to process
    M = size(path, 1) - 1; % number of segment, one less then the setpoints
    T_each = zeros(M+1, 1); % time period of each section
    T = zeros(M+1, 1); % accumulated time
    Q = zeros(M*R); % a set of all Qpresent state

    % Calculate the time periods
    l = 0; % total length
    for i = 2:1:M+1
        l = l + sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2);
    end
    v = l/T_total; % velocity
    for i = 2:1:M+1
        T_each(i) = sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2)/v;
    end
    for i=2:1:M+1
        T(i) = T(i-1) + T_each(i); % first term is zero
    end

    % Calculate the cost function and construct the Hessian matrix
    for m = 1:1:M
        for i = 4:1:R-1
            for j = 4:1:R-1
                Q(i+1 + (m-1)*R, j+1 + (m-1)*R) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*(T_each(m+1)^(i+j-7)) / (i+j-7);
            end
        end
    end

    % Build waypoints
    x   = path(:,1);
    y   = path(:,2);
    z   = path(:,3);
    % mapping matrix
    A1 = zeros(2*M    , M*R);
    A2 = zeros(2*(M-1), M*R);
    A3 = zeros(4      , M*R);

    % position derivative constraints
    d1x = zeros(2*M, 1);
    d1y = zeros(2*M, 1);
    d1z = zeros(2*M, 1);
    % continuity constraints
    d2x = zeros(2*(M-1), 1);
    d2y = zeros(2*(M-1), 1);
    d2z = zeros(2*(M-1), 1);
    % the start and final derivative constraints for velocity and acceleration
    d3x = zeros(4, 1);
    d3y = zeros(4, 1);
    d3z = zeros(4, 1);

    % position derivative constraints
    for m=1:M
        for i=1:R
            A1(2*m-1, i+(m-1)*R) = 0^(i-1);
            A1(2*m  , i+(m-1)*R) = T_each(m+1)^(i-1);
        end
        d1x(2*m-1, 1) = x(m);
        d1x(2*m  , 1) = x(m+1);
        d1y(2*m-1, 1) = y(m);
        d1y(2*m  , 1) = y(m+1);
        d1z(2*m-1, 1) = z(m);
        d1z(2*m  , 1) = z(m+1);
    end

    % continuity constraints
    for m=2:1:M
        for i=1:1:R-1 % velocity continuity constraints
            A2(2*m-3, i+1+(m-2)*R) =  (factorial(i)/factorial(i-1))*T_each(m)^(i-1);
            A2(2*m-3, i+1+(m-1)*R) = -(factorial(i)/factorial(i-1))*0^(i-1);
        end
        for i=2:1:R-1 % acceleration continuity constraints
            A2(2*m-2, i+1+(m-2)*R) =  (factorial(i)/factorial(i-2))*T_each(m)^(i-2);
            A2(2*m-2, i+1+(m-1)*R) = -(factorial(i)/factorial(i-2))*0^(i-2);
        end
    end

    % the start and final derivative constraints for velocity and acceleration
    for i=1:1:R-1 % velocity derivative constraints
        A3(1, i+1) =  (factorial(i)/factorial(i-1))*0^(i-1);
        A3(3, i+1+(M-1)*R) =  (factorial(i)/factorial(i-1))*T_each(M+1)^(i-1);
    end
    for i=2:1:R-1 % acceleration derivative constraints
        A3(2, i+1) =  (factorial(i)/factorial(i-2))*0^(i-2);
        A3(4, i+1+(M-1)*R) =  (factorial(i)/factorial(i-2))*T_each(M+1)^(i-2);
    end

    A  = [A1;A2;A3];
    dx = [d1x;d2x;d3x];
    dy = [d1y;d2y;d3y];
    dz = [d1z;d2z;d3z];

    % min 0.5 P'QP s.t. Aeq * x = deq
    Px = quadprog(Q,[],[],[],A,dx);
    Py = quadprog(Q,[],[],[],A,dy);
    Pz = quadprog(Q,[],[],[],A,dz);

else % output desired trajectory here (given time)
    s_des = zeros(13,1);
    yaw   = 0;
    pitch = 0;
    roll  = 0;
    quat = R_to_quaternion(ypr_to_R([yaw pitch roll])');
    s_des(7)  = quat(1);
  	s_des(8)  = quat(2);
  	s_des(9)  = quat(3);
  	s_des(10) = quat(4);
    % Detecting the stages
    for j=1:M
        if t >= T(j) && t < T(j+1)
            m = j; break; % break after the stage is identified
        end
    end

    for i=0:1:R-1
        s_des(1) = s_des(1) + Px(i+1+(m-1)*R)*(t-T(m))^i
        s_des(2) = s_des(2) + Py(i+1+(m-1)*R)*(t-T(m))^i;
        s_des(3) = s_des(3) + Pz(i+1+(m-1)*R)*(t-T(m))^i;
        if i > 0
            s_des(4) = s_des(4) + i*Px(i+1+(m-1)*R)*(t-T(m))^(i-1);
            s_des(5) = s_des(5) + i*Py(i+1+(m-1)*R)*(t-T(m))^(i-1);
            s_des(6) = s_des(6) + i*Pz(i+1+(m-1)*R)*(t-T(m))^(i-1);
        end
    end
end

end
