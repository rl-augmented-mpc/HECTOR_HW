function out = expm_so3(so3)
%EXPM_SO3 이 함수의 요약 설명 위치
%   자세한 설명 위치
vec = vee_so3(so3);

theta = norm2(vec);
theta2 = theta*theta;
sin_theta = sin(theta);
s2 = sin(theta/2.0);
one_minus_cos = 2.0*s2*s2;
K = so3/theta;
KK=K*K;


if NearZero(theta)
%     out = eye(3);
    out = eye(3) + so3;   % gtsam method
else
out = eye(3) + sin_theta*K + one_minus_cos*KK;
end
end
