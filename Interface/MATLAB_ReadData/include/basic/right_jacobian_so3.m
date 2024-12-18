function out = right_jacobian_so3(vec)
%RIGHT_JACOBIAN_SO3 이 함수의 요약 설명 위치
%   자세한 설명 위치

so3 = hat_so3(vec);

theta = norm2(vec);

nor = norm2(vec);
if NearZero(theta)
%     out = eye(3);
    
    out = eye(3)-0.5*so3;    %gtsam
else
    
    theta2 = theta*theta;
    sin_theta = sin(theta);
    s2 = sin(theta/2.0);
    one_minus_cos = 2.0*s2*s2;
    K = so3/theta;
    KK=K*K;
    a = one_minus_cos / theta;
    b = 1.0 - sin_theta / theta;
    
    out = eye(3) - a*K + b*KK;  %gtsam
    
    
%     out = eye(3) - (1-cos(nor))/(nor^2)*so3+(nor-sin(nor))/(norm2(vec.^3))*so3*so3;
end

