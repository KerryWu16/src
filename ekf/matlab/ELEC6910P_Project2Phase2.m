syms x u n f(x,u,n)
x1 = sym('x1', [3,1]); % position
x2 = sym('x2', [3,1]); % orientation
x3 = sym('x3', [3,1]); % velocity
x4 = sym('x4', [3,1]); % gyro bias
x5 = sym('x5', [3,1]); % acceleromter bias
x = [x1; x2; x3; x4; x5];
am = sym('am', [3,1]); % input acceleration
wm = sym('wm', [3,1]); % input angular velocity
u = [am; wm];
na = sym('na', [3,1]); % linear acceleration noise
ng = sym('ng', [3,1]); % angular velocity noise
nba = sym('nba', [3,1]); % acceleration bias noise
nbg = sym('nbg', [3,1]); % gyro bias noise
n = [na; ng; nba; nbg];

g = 9.81;
G_x2 = [cos(x2(2)), 0, -cos(x2(1))*sin(x2(2)); ...
        0         , 1,  sin(x2(1)); ...
        sin(x2(2)), 0,  cos(x2(1))*cos(x2(2))];

% f(x,u,n) = [x3; ...
f = [x3; ...
     inv(G_x2)*(wm - x4 -ng); ...
     g + eul2rotm(x2, 'ZXY') * (am - x5 - na); ...
            nbg;...
            nba ]; %#ok<*MINV>
simplify(f)
At = simplify(jacobian(f, x));
Bt = simplify(jacobian(f, u));
Ut = simplify(jacobian(f, n));
