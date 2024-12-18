function out = inv_right_jacobian_so3_casadi(vec)
%INV_RIGHT_JACOBIAN_SO3 이 함수의 요약 설명 위치
%   자세한 설명 위치
nor = norm(vec);
out = eye(3) + 0.5*hat_so3(vec) + (1/(nor^2)+(1+cos(nor))/(2*nor*sin(nor)))*hat_so3(vec)*hat_so3(vec);
end

