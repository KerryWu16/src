function [F, M] = controller(t, s, s_des)

%%%% Sui Pang, Sept. 28th, 2016, ELEC 6910P, Project 1, phase 1

global params
persistent phi_c_storage;
persistent theta_c_storage;
persistent error_sum;
phi_c_storage    = 0;
theta_c_storage  = 0;
error_sum = zeros(6,1);

m = params.mass;
g = params.grav;
I = params.I;

K_pz = 50.0;
K_dz = 80.0;

K_px = 6;
K_dx = 4.5;
K_py = 6;
K_dy = 4.5;

K_p_phi = 1000;
K_d_phi = 46;
K_p_theta = 1000;
K_d_theta = 46;
K_p_psi = 100;
K_d_psi = 60;


% 1. Z posiion control, independent
r_3_acc = s_des(13) + K_dz*(s_des(6)-s(6))+K_pz*(s_des(3)-s(3));

F = m*(g + r_3_acc);

% 2. Positin and Attitude control, cascaded
% Roll, pitch, yaw
[phi_des_ns,theta_des_ns,psi_des_ns] = RotToRPY_ZXY(QuatToRot(s_des(7:10)'));
[phi_ns,    theta_ns,    psi_ns    ] = RotToRPY_ZXY(QuatToRot(s(7:10)'));

r_1_acc = s_des(11) + K_dx*(s_des(4)-s(4)) + K_px*(s_des(1)-s(1));
r_2_acc = s_des(12) + K_dy*(s_des(5)-s(5)) + K_py*(s_des(2)-s(2));

phi_c_ns   = (r_1_acc * sin(psi_ns) -  r_2_acc * cos(psi_ns)) / g;
theta_c_ns = (r_1_acc * cos(psi_ns) +  r_2_acc * sin(psi_ns)) / g;
phi_c_ps   = phi_c_storage;
theta_c_ps = theta_c_storage;
phi_c_storage = phi_c_ns;
theta_c_storage = theta_c_ns;

% 3. attitude control

psi_angle_diff = psi_des_ns - psi_ns;
if psi_angle_diff >= pi
   psi_angle_diff = psi_angle_diff - 2 * pi;
elseif psi_angle_diff <= -pi
   psi_angle_diff = psi_angle_diff + 2 * pi;
end

phi_c_acc   = K_p_phi   * (phi_c_ns   - phi_ns)   + K_d_phi   * ((phi_c_ns - phi_c_ps)     - s(11));
theta_c_acc = K_p_theta * (theta_c_ns - theta_ns) + K_d_theta * ((theta_c_ns - theta_c_ps) - s(12));
psi_c_acc   = K_p_psi   * psi_angle_diff          + K_d_psi   * (s_des(13)                 - s(13));

% Output 2, roll pitch yaw output
M = I * [phi_c_acc , theta_c_acc, psi_c_acc]' + cross(s(11:13),  I * s(11:13));

% Error Analysis
RMS_X = sqrt(sum(s(1)-s_des(1)).^2);
RMS_Y = sqrt(sum(s(2)-s_des(2)).^2);
RMS_Z = sqrt(sum(s(3)-s_des(3)).^2);
RMS_X_v = sqrt(sum(s(4)-s_des(4)).^2);
RMS_Y_v = sqrt(sum(s(5)-s_des(5)).^2);
RMS_Z_v = sqrt(sum(s(6)-s_des(6)).^2);
        [RMS_X, RMS_Y, RMS_Z, RMS_X_v, RMS_Y_v, RMS_Z_v];

error_sum(1) = error_sum(1) + RMS_X;
error_sum(2) = error_sum(2) + RMS_Y;
error_sum(3) = error_sum(3) + RMS_Z;
error_sum(4) = error_sum(4) + RMS_X_v;
error_sum(5) = error_sum(5) + RMS_Y_v;
error_sum(6) = error_sum(6) + RMS_Z_v;
        error_sum';
end
