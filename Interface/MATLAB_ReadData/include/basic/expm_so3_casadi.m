function out = expm_so3_casadi(so3)
%EXPM_SO3 �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
vec = vee_so3(so3);
nor = norm(vec);


% out = eye(3)+ sin(nor)/nor*so3 + (1-cos(nor))/(nor^2)*so3*so3;
out = eye(3)+ so3;
end