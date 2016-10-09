%%%% Sui Pang, Oct. 8th, 2016, ELEC 6910P, Project 1, phase 2
%%%% Smooth Multi-segment Trajectory Generation

function s_des = trajectory_generator(t, path, h)

persistent N;
persistent T;
persistent C;
persistent x;
persistent y;
persistent z;
persistent n;
R = 6; % orders

if nargin > 1 % pre-process can be done here (given waypoints)
  % local variables
  T_total = 25; % if the curve takes 25 seconds to process
  N = size(path, 1) - 1; % number of segment, one less then the setpoints
  T = zeros(N+1, 1); % time duration of each section
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

  % trajectory planning
  Q = zeros(6);
  A = zeros(6,3);

  for n=1:N+1  % Strange parrallel processing kicked in
      if n < N+1
        Q = [ 0         0         0         0       0     1; ...
              T(n)^5    T(n)^4    T(n)^3    T(n)^2  T(n)  1; ...
              0         0         0         0       1     0; ...
              5*T(n)^4  4*T(n)^3  3*T(n)^2  2*T(n)  1     0; ...
              0         0         0         2       0     0; ...
              20*T(n)^3 12*T(n)^2 6*T(n)    2       0     0];
        A = [ x(n) x(n+1) 0 0 0 0; ...
              y(n) y(n+1) 0 0 0 0; ...
              z(n) z(n+1) 0 0 0 0; ]';
      end

      if n>1
          C(n-1,:,:) = Q \ A
      end
  end
  n = 1;

else % output desired trajectory here (given time)
  s_des = zeros(13,1);
  ax = 0;
  ay = 0;
  yaw   = 0;
  pitch = 0;
  roll  = 0;
  quat = R_to_quaternion(ypr_to_R([yaw pitch roll])');
  s_des(7:10) = quat;

  for j=1:N
    if t >= T(j) && t < T(j+1)
      n = j; break; % really strange Matlab
    end
  end
  r_record = zeros(N);
  v_record = zeros(N);
  for k=1:n
    Cx(:,:) = C(:,:,1);
    x_r_part =  Cx(n,1)*(t-T(k))^5 + Cx(n,2)*(t-T(k))^4 + Cx(n,3)*(t-T(k))^3 + ...
                Cx(n,4)*(t-T(k))^2 + Cx(n,5)*(t-T(k))^1 + Cx(n,6)*(t-T(k));
    x_v_part =  5*Cx(n,1)*(t-T(k))^4 + 4*Cx(n,2)*(t-T(k))^3 + 3*Cx(n,3)*(t-T(k))^2 + ...
                2*Cx(n,4)*(t-T(k))^1 + Cx(n,5)*(t-T(k)) + 0;
    if k == 1
      r_record(k) = x_r_part;
      v_record(k) = x_v_part;
    else
      r_record(k) = r_record(k-1) + x_r_part;
      v_record(k) = v_record(k-1) + x_v_part;
    end
  end
  s_des(1) = r_record(n);
  s_des(2) = y(1);
  s_des(3) = z(1);
  s_des(4) = v_record(n);
  s_des(5) = 0;
  s_des(6) = 0;
end

end
