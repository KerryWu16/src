% default type of Euler angle is ZYX
function y = eul2rotm(angle, type)
    phi = angle(1);
    theta = angle(2);
    psi = angle(3);
    
    rotx = [1           , 0         , 0         ; ...
            0           , cos(phi)  , sin(phi)  ;...
            0           , -sin(phi) , cos(phi)  ];
    roty = [cos(theta)  , 0         , -sin(theta);...
            0           , 1         , 0         ;...
            sin(theta)  , 0         , cos(theta)];
    rotz = [cos(psi)    , sin(psi)  , 0         ;...
            -sin(psi)   , cos(psi)  , 0         ;...
            0           , 0         , 1         ];
    
   %if exists(type)
        switch type 
            case 'ZXY'
                y = rotz * rotx * roty;
            case 'ZYX'
                y = rotz * roty * rotx;
            otherwise
                y = rotz * roty * rotx;
        end
    %end