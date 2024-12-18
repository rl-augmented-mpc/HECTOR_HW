function out = inv_right_jacobian_so3(vec)
%INV_RIGHT_JACOBIAN_SO3 이 함수의 요약 설명 위치
%   자세한 설명 위치


theta = norm2(vec);


if NearZero(theta)
    out = eye(3);
else
    
    
    so3 = hat_so3(vec);
    
    theta2 = theta*theta;
    sin_theta = sin(theta);
    s2 = sin(theta/2.0);
    one_minus_cos = 2.0*s2*s2;
    K = so3/theta;
    KK=K*K;
    a = one_minus_cos / theta;
    b = 1.0 - sin_theta / theta;
    
    
    
%     out = eye(3) + 0.5*so3 + (1/(theta2) - (1+cos(theta))/(2* theta * sin_theta))*so3*so3;
    out = eye(3) + 0.5*so3 + (1/(theta2) + (1+cos(theta))/(2* theta * sin_theta))*so3*so3;
end
end

