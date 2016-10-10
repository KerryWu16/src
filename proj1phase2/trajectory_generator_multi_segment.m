%%%% Sui Pang, Oct. 8th, 2016, ELEC 6910P, Project 1, phase 2
%%%% Smooth Multi-segment Trajectory Generation

function s_des = trajectory_generator(t, path, h)

% Global variables within this function
persistent N;
persistent T_cont;
persistent C;
persistent r_record;
persistent v_record;

if nargin > 1 % pre-process can be done here (given waypoints)
    % local variables
    R = 6; % orders
    T_total = 25; % if the curve takes 25 seconds to process
    N = size(path, 1) - 1; % number of segment, one less then the setpoints
    T = zeros(N+1, 1); % time duration of each section
    T_cont = zeros(N+1, 1); % time duration of each section
    C = zeros(N, R, 3);   % N set of polynomial constants
    x = path(:,1);
    y = path(:,2);
    z = path(:,3);

    % Calculate the time periods
    l = 0; % total length
    for i = 2:1:N+1
        l = l + sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2);
    end
    v = l/T_total; % velocity
    for i = 2:1:N+1
        T(i) = sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2)/v;
    end
    for i=1:N+1
        if i == 1
            T_cont(1) = T(1);
        else
            T_cont(i) = T_cont(i-1) + T(i);
        end
    end

    % trajectory planning
    Q = zeros(6);
    A = zeros(6,3);

    for m=2:N+1  % Strange parrallel processing kicked in
        Q = [ 0         0         0         0       0     1; ...
              T(m)^5    T(m)^4    T(m)^3    T(m)^2  T(m)  1; ...
              0         0         0         0       1     0; ...
              5*T(m)^4  4*T(m)^3  3*T(m)^2  2*T(m)  1     0; ...
              0         0         0         2       0     0; ...
              20*T(m)^3 12*T(m)^2 6*T(m)    2       0     0];
        A = [ x(m-1) x(m) 0 0 0 0; ...
              y(m-1) y(m) 0 0 0 0; ...
              z(m-1) z(m) 0 0 0 0; ]';
        C(m-1,:,:) = Q \ A; % Solution for QC = A
    end

    % Initialize the memory
    r_record = zeros(N,1);
    v_record = zeros(N,1);

else % output desired trajectory here (given time)
    s_des = zeros(13,1);
    yaw   = 0;
    pitch = 0;
    roll  = 0;
    quat = R_to_quaternion(ypr_to_R([yaw pitch roll])');
    s_des(7:10) = quat;

    for j=1:N
        if t >= T_cont(j) && t < T_cont(j+1)
            n = j; break; % break after the stage is identified
        end
    end

    % calculate three output in one line
    r_part =  C(n,1,:)*(t-T_cont(n))^5 + C(n,2,:)*(t-T_cont(n))^4 + C(n,3,:)*(t-T_cont(n))^3 + ...
              C(n,4,:)*(t-T_cont(n))^2 + C(n,5,:)*(t-T_cont(n))^1 + C(n,6,:);
    v_part =  5*C(n,1,:)*(t-T_cont(n))^4 + 4*C(n,2,:)*(t-T_cont(n))^3 + 3*C(n,3,:)*(t-T_cont(n))^2 + ...
              2*C(n,4,:)*(t-T_cont(n))^1 + 1*C(n,5,:) + 0;

    s_des(1) = r_part(1);
    s_des(2) = r_part(2);
    s_des(3) = r_part(3);
    s_des(4) = v_part(1);
    s_des(5) = v_part(2);
    s_des(6) = v_part(3);
end

end
