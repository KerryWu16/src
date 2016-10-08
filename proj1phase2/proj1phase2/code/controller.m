function [F, M] = controller(t, s, s_des)
% Note that the rotation (s(7:10)) is R^{body}_{world}.

global params %Ref to quadModel_readonly.m for the params

F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M

end
