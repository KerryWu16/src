%%%% Sui Pang, Oct. 8th, 2016, ELEC 6910P, Project 1, phase 2
%%%% Optimization-based Trajectory Generation

function s_des = trajectory_generator(t, path, h)

R = 6; % orders

if nargin > 1 % pre-process can be done here (given waypoints)
  % local variables
  N = size(path, 1) - 1; % number of segment, one less then the setpoints
  T = zeros(N+1, 1); % time periods
  Qx = zeros(N*(R+1)); % a set of all Qpresent state
  Qy = zeros(N*(R+1));
  Qz = zeros(N*(R+1));


  l = 0; % totoal length
  for i = 2:1:N+1
    l = l + sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2);
  end
  v = l/25; % velocity
  for i = 2:1:N+1
    T(i) = T(i-1) + sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2 + (path(i,3) - path(i-1,3))^2)/v;
  end

  % Calculate the cost function and construct the Hessian matrix Q
  for n = 1:1:N
    for i = 4:1:R
      for j = 4:1:R
        Qx(i+1 + (n-1)*(R+1), j+1 + (n-1)*(R+1)) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*((T(n+1)-T(n))^(i+j-7)) / (i+j-7);
      end
    end
  end
  for n = 1:1:N
    for i = 4:1:R
      for j = 4:1:R
        Qy(i+1 + (n-1)*(R+1), j+1 + (n-1)*(R+1)) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*((T(n+1)-T(n))^(i+j-7)) / (i+j-7);
      end
    end
  end
  for n = 1:1:N
    for i = 4:1:R
      for j = 4:1:R
        Qz(i+1 + (n-1)*(R+1), j+1 + (n-1)*(R+1)) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*((T(n+1)-T(n))^(i+j-7)) / (i+j-7);
      end
    end
  end

  % Build waypoints
  x   = path(:,1);
  y   = path(:,2);
  z   = path(:,3);
  % waypoints constraints
  A1x = zeros(2*N, N*(R+1)); % mapping matrix
  d1x = zeros(2*N, 1); % vector contains the derivative values
  A1y = zeros(2*N, N*(R+1));
  d1y = zeros(2*N, 1);
  A1z = zeros(2*N, N*(R+1));
  d1z = zeros(2*N, 1);
  % continuity constraints
  A2x = zeros(2*(N-1), N*(R+1)); % mapping matrix
  d2x = zeros(2*(N-1), 1); % TODO: figure out the length
  A2y = zeros(2*(N-1), N*(R+1));
  d2y = zeros(2*(N-1), 1);
  A2z = zeros(2*(N-1), N*(R+1));
  d2z = zeros(2*(N-1), 1);
  % start and the final constraints
  A3x = zeros(2, N*(R+1));
  d3x = zeros(2, 1); % TODO: figure out the length
  A3y = zeros(2, N*(R+1));
  d3y = zeros(2, 1);
  A3z = zeros(2, N*(R+1));
  d3z = zeros(2, 1);

  % waypoints constraints
  for n=1:N
    for i=0:1:R
      A1x(1+(n-1)*2, i+1+(n-1)*(R+1)) = (T(n)   - T(n))^i; %TODO: check if its T(n-1)
      A1x(2+(n-1)*2, i+1+(n-1)*(R+1)) = (T(n+1) - T(n))^i;
      d1x(1+(n-1)*2, i+1+(n-1)*(R+1)) = x(n);
      d1x(2+(n-1)*2, i+1+(n-1)*(R+1)) = x(n+1);
    end
  end
  for n=1:N
    for i=0:1:R
      A1y(1+(n-1)*2, i+1+(n-1)*(R+1)) = (T(n)   - T(n))^i; %TODO: check if its T(n-1)
      A1y(2+(n-1)*2, i+1+(n-1)*(R+1)) = (T(n+1) - T(n))^i;
      d1y(1+(n-1)*2, i+1+(n-1)*(R+1)) = y(n);
      d1y(2+(n-1)*2, i+1+(n-1)*(R+1)) = y(n+1);
    end
  end
  for n=1:N
    for i=0:1:R
      A1z(1+(n-1)*2, i+1+(n-1)*(R+1)) = (T(n)   - T(n))^i; %TODO: check if its T(n-1)
      A1z(2+(n-1)*2, i+1+(n-1)*(R+1)) = (T(n+1) - T(n))^i;
      d1z(1+(n-1)*2, i+1+(n-1)*(R+1)) = z(n);
      d1z(2+(n-1)*2, i+1+(n-1)*(R+1)) = z(n+1);
    end
  end

  % % continuity constraints
  % for n=1:N-1
  %   for i=1:R
  %     A2x() = (factorial(i)/factorial(i-k))*(T(m+1) - T(m))^(i-k);
  %   end
  % end
  %
  % Ax = [A1x;A2x;A3x];
  % dx = [d1x;d2x;d3x];
  % Ay = [A1y;A2y;A3y];
  % dy = [d1y;d2y;d3y];
  % Az = [A1z;A2z;A3z];
  % dz = [d1z;d2z;d3z];
  %
  % % min 0.5 P'QP s.t. Aeq * x = deq
  % [Px, vx] = quadprog(Qx,[],[],[],Ax,dx);
  % [Py, vy] = quadprog(Qy,[],[],[],Ay,dy);
  % [Pz, vz] = quadprog(Qz,[],[],[],Az,dz);

else % output desired trajectory here (given time)
  s_des(1:13) = zeros(13,1);
  ax = 0;
  ay = 0;
  yaw   = 0;
  pitch = 0;
  roll  = 0;
  quat = R_to_quaternion(ypr_to_R([yaw pitch roll])');
  s_des(7:10) = quat;
  % for i=1:N
  %   if t>=T(i)&&t<T(i+1)
  %     n = i;
  %     break;
  %   end
  % end
  % for i=0:1:R
  %   s_des(1) = s_des(1) + Px(i+1+(n-1)*(R+1))*(t-T(n))^i;
  %
  % end
end

end
